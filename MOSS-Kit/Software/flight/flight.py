#!/usr/bin/env python3
"""
flight.py
flight-side telemetry script.

Main idea
---------
- Build a telemetry record every 2 seconds (0.5 Hz).
- Log every record locally as JSONL (safety net — survives even if every
  radio packet is lost).
- Pack the record into a 22-byte binary payload via tm_schema.pack().
- Wrap that payload in a TM frame via protocol_tm.build_frame().
- Transmit the frame twice back-to-back (2x retransmit strategy).
- Run the camera in its own worker thread so subprocess capture never
  blocks the main telemetry loop.

"""

# =====================================================================
# STUDENT ORIENTATION — read this first
# ---------------------------------------------------------------------
# This program runs ON THE PAYLOAD (the Raspberry Pi that flies). Its job
# is a loop: read every sensor, write the full reading to the SD card,
# then send a compact copy to the ground over the radio. Repeat every 2s.
#
# Two rules the code follows everywhere:
#   1. NEVER let one broken part crash the flight. Every sensor/radio read
#      is wrapped so a failure is caught, recorded in the record's "errors",
#      and the loop keeps going. A dead sensor must not silence the others.
#   2. The SD card log (JSONL) is the source of truth. The radio is
#      best-effort — many packets are lost in flight — so the local log is
#      what you actually analyze afterward.
#
# The "(value, error)" return pattern you'll see everywhere is how rule #1
# works: a function hands back BOTH a result AND an error string (one is
# always None), so the caller can log the problem instead of crashing.
# =====================================================================

# -------------------------
# Standard libraries
# -------------------------
import json
import math
import queue
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

# -------------------------
# Hardware / sensor libraries
# -------------------------
# These only exist on the Raspberry Pi. board/busio (Adafruit Blinka) expose
# the Pi's I2C pins; the rest are drivers, one per sensor chip on the I2C bus:
#   BME280  -> temperature, pressure, humidity
#   LTR390  -> UV + ambient light
#   TSL2591 -> lux / infrared / visible light
#   ICM20948 (IMU) -> acceleration + rotation (gyro)
import board
import busio
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ltr390
import adafruit_tsl2591
from adafruit_icm20x import ICM20948

# UPS power monitoring (Waveshare UPS HAT E)
from smbus2 import SMBus

# LoRa driver
import sx126x

# Shared telemetry protocol (MUST match ground exactly)
from protocol_tm import build_frame as tm_build_frame

# Binary wire-format schema (MUST match ground exactly)
import tm_schema


# ===========================================================
# Constants
# ===========================================================

# ---- Telemetry pacing ----
TELEMETRY_HZ = 0.5           # 1 packet every 2 seconds
LOOP_PERIOD_S = 1.0 / TELEMETRY_HZ

# ---- UPS (Waveshare UPS HAT E) ----
UPS_I2C_ADDR   = 0x2D
LOW_BATT_PCT   = 20
CRIT_BATT_PCT  = 10
LOW_BATT_V     = 3.55

# ---- BME280 ----
# BME280 lives on either 0x76 or 0x77; we try 0x76 first.
BME280_PRIMARY_ADDR   = 0x76
BME280_FALLBACK_ADDR  = 0x77

# ---- IMU (ICM20948) ----
# Inertial Measurement Unit: accelerometer + gyroscope, on I2C address 0x68.
IMU_I2C_ADDR = 0x68

# Sea-level pressure used as fallback altitude reference if we can't
# capture a launch-pad reading at startup (in hPa).
DEFAULT_REF_PRESSURE_HPA = 1013.25

# ---- Camera ----
IMAGE_PERIOD_S       = 25 # capture cadence
IMG_W, IMG_H         = 3280, 2464
IMG_Q                = 85
CAMERA_TIMEOUT_S     = 8
CRIT_IMAGE_PERIOD_S  = 60 # slow capture when battery critical

# ---- LoRa ----
LORA_PORT          = "/dev/serial0"
LORA_FREQ_MHZ      = 915
LORA_SRC_ADDR      = 1
LORA_DEST_ADDR     = 65535
LORA_POWER_DBM     = 22
LORA_AIR_SPEED     = 2400
LORA_NET_ID        = 0
LORA_CRYPT         = 0
LORA_RSSI          = True
LORA_BUFFER_SIZE   = 240
LORA_BASE_FREQ_MHZ = 850 # 900MHz modules: offset = freq_mhz - 850

# 2x retransmit — send each frame twice back-to-back for resilience.
TX_COPIES          = 2
TX_GAP_S           = 0.05  # tiny gap between copies so UART isn't slammed

# ---- Storage ----
MAX_LOG_MB   = 200
MAX_IMAGE_MB = 500


# ==================================================================
# Time helpers
# ===================================================================
def utc_timestamp_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def sleep_to_rate(loop_start_s: float, period_s: float) -> None:
    """Sleep just long enough to keep the main loop at the target rate."""
    elapsed = time.time() - loop_start_s
    time.sleep(max(0.0, period_s - elapsed))


# ==================================================================
# Local logging (JSONL)
# ===================================================================
def append_jsonl(path: Path, record: Dict[str, Any]) -> Optional[str]:
    # JSONL = one complete JSON object per line. Appending a line at a time
    # means a power loss only risks the last line, not the whole file.
    # Returns None on success or an error string (never raises — keep flying).
    try:
        with path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        return None
    except Exception as e:
        return repr(e)


# ====================================================================
# Storage cleanup (delete oldest files when folder over the cap)
# =====================================================================
def cleanup_folder_to_mb(folder: Path, max_mb: int) -> int:
    if not folder.exists():
        return 0

    max_bytes = max_mb * 1024 * 1024
    files = [p for p in folder.rglob("*") if p.is_file()]
    files.sort(key=lambda p: p.stat().st_mtime)

    total = sum(p.stat().st_size for p in files)
    if total <= max_bytes:
        return 0

    deleted = 0
    for p in files:
        try:
            size = p.stat().st_size
            p.unlink()
            total -= size
            deleted += 1
            if total <= max_bytes:
                break
        except Exception:
            pass
    return deleted


# ======================================================================
# UPS helpers
# ======================================================================
def ups_read_u16(bus: SMBus, reg: int) -> int:
    # The battery gauge stores values across two 8-bit registers. Read the low
    # byte and high byte, then stitch them into one unsigned 16-bit number.
    lo = bus.read_byte_data(UPS_I2C_ADDR, reg)
    hi = bus.read_byte_data(UPS_I2C_ADDR, reg + 1)
    return (hi << 8) | lo


def ups_read_i16(bus: SMBus, reg: int) -> int:
    # Same as above but SIGNED: current is negative when discharging, so a
    # value with the top bit set is converted to its negative (two's-complement).
    v = ups_read_u16(bus, reg)
    return v - 65536 if v & 0x8000 else v


def read_ups_status(bus: SMBus) -> tuple[Dict[str, Any], Optional[str]]:
    try:
        batt_mv  = ups_read_u16(bus, 0x20)
        batt_ma  = ups_read_i16(bus, 0x22)
        batt_pct = ups_read_u16(bus, 0x24)
        rem_mah  = ups_read_u16(bus, 0x26)
        rem_min  = ups_read_u16(bus, 0x28)
        return {
            "battery_v": batt_mv / 1000.0,
            "battery_a": batt_ma / 1000.0,
            "battery_pct": batt_pct,
            "remaining_mah": rem_mah,
            "remaining_min": rem_min,
        }, None
    except Exception as e:
        return {}, repr(e)


def compute_battery_alerts(power: Dict[str, Any]) -> list[str]:
    alerts: list[str] = []
    pct = power.get("battery_pct")
    v   = power.get("battery_v")

    if isinstance(pct, (int, float)):
        if pct <= CRIT_BATT_PCT:
            alerts.append("BATT_CRIT_PCT")
        elif pct <= LOW_BATT_PCT:
            alerts.append("BATT_LOW_PCT")

    if isinstance(v, (int, float)) and v <= LOW_BATT_V:
        alerts.append("BATT_LOW_V")

    return alerts


# =======================================================================
# Pi health helpers (kept for JSONL log; not in radio packet)
# ========================================================================
def _run(cmd: list[str]) -> tuple[str, Optional[str]]:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip(), None
    except Exception as e:
        return "", repr(e)


def read_cpu_temp_c() -> tuple[Optional[float], Optional[str]]:
    out, err = _run(["vcgencmd", "measure_temp"])
    if err:
        return None, err
    try:
        return float(out.split("=")[1].split("'")[0]), None
    except Exception as e:
        return None, repr(e)


def read_disk_free_mb(path: Path) -> tuple[Optional[float], Optional[str]]:
    try:
        return shutil.disk_usage(str(path)).free / (1024 * 1024), None
    except Exception as e:
        return None, repr(e)


def read_pi_health(data_root: Path) -> tuple[Dict[str, Any], Dict[str, str]]:
    health: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    cpu_temp, err = read_cpu_temp_c()
    if err:
        errors["cpu_temp"] = err
    else:
        health["cpu_temp_c"] = cpu_temp

    disk_free, err = read_disk_free_mb(data_root)
    if err:
        errors["disk_free"] = err
    else:
        health["disk_free_mb"] = disk_free

    return health, errors


# ======================================================================
# Sensors (BME280 + LTR390 + TSL2591 + ICM20948)
# =======================================================================
def init_i2c_bus() -> busio.I2C:
    return busio.I2C(board.SCL, board.SDA)


def init_sensors(i2c: busio.I2C) -> Dict[str, Any]:
    """
    Initialize all I2C sensors. Each init is independent: one sensor failing
    does not stop the others. Init failures are recorded in
    sensors["_init_errors"] so downstream telemetry can surface the real
    reason a field shows up null.

    BME280 address is commonly 0x76 or 0x77. We try 0x76 first, then 0x77.
    """
    # Each sensor gets its own slot, defaulting to None. If a sensor fails to
    # initialize, its slot stays None and the rest still work — later reads
    # check "is this None?" before using it. Init errors collect in one dict.
    sensors: Dict[str, Any] = {
        "bme280_sensor": None,
        "ltr390_sensor": None,
        "tsl2591_sensor": None,
        "imu_sensor": None,
        "_init_errors": {},
    }
    init_errors: Dict[str, str] = {}

    # Each try/except below is independent on purpose: a missing or broken
    # sensor records its error and the next sensor still gets initialized.

    # --- BME280 (temp/pressure/humidity) ---
    # Try address 0x76 first, fall back to 0x77 (depends on the breakout).
    try:
        sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(
            i2c, address=BME280_PRIMARY_ADDR
        )
    except Exception as e76:
        try:
            sensors["bme280_sensor"] = adafruit_bme280.Adafruit_BME280_I2C(
                i2c, address=BME280_FALLBACK_ADDR
            )
        except Exception as e77:
            init_errors["bme280_init"] = (
                f"0x76 failed: {type(e76).__name__}: {e76}; "
                f"0x77 failed: {type(e77).__name__}: {e77}"
            )

    # --- LTR390 (UV / ambient light) ---
    try:
        sensors["ltr390_sensor"] = adafruit_ltr390.LTR390(i2c)
    except Exception as e:
        init_errors["ltr390_init"] = f"{type(e).__name__}: {e}"

    # --- TSL2591 (lux / IR / visible) ---
    try:
        sensors["tsl2591_sensor"] = adafruit_tsl2591.TSL2591(i2c)
    except Exception as e:
        init_errors["tsl2591_init"] = f"{type(e).__name__}: {e}"

    # --- IMU (ICM20948: accel + gyro) ---
    try:
        sensors["imu_sensor"] = ICM20948(i2c, address=IMU_I2C_ADDR)
    except Exception as e:
        init_errors["imu_init"] = f"{type(e).__name__}: {e}"

    sensors["_init_errors"] = init_errors
    return sensors


def read_bme280(sensor: Any) -> tuple[Dict[str, Any], Optional[str]]:
    """
    Read temperature, pressure, and humidity from the BME280.
    Altitude is computed later from pressure. Kept as its own function so
    the main loop can derive altitude from the pressure reading.
    """
    # The BME280 (with an "E") reads all three of these. Altitude is a DERIVED
    # value computed in the main loop, where the launch-pad reference pressure
    # is available — that's why it isn't returned here.
    try:
        return {
            "temperature_c": float(sensor.temperature),
            "pressure_hpa": float(sensor.pressure),
            "humidity_pct": float(sensor.humidity),
        }, None
    except Exception as e:
        return {}, repr(e)


def read_extra_sensors(sensors: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
    """
    Read the non-BME280 sensors (UV/ambient, lux/IR/visible, IMU).
    Returns (data, errors) where data is keyed by record section
    ("uv_light", "light", "imu"). BME280 is handled separately by
    read_bme280() because the main loop needs its pressure for altitude.
    """
    data: Dict[str, Any] = {}
    errors: Dict[str, str] = {}

    # Pull each sensor object; any that failed to init is None and is skipped.
    ltr390 = sensors.get("ltr390_sensor")
    tsl2591 = sensors.get("tsl2591_sensor")
    imu = sensors.get("imu_sensor")

    # UV + ambient light. Each block is guarded so one bad sensor doesn't
    # block the others (same "keep flying" rule as everywhere else).
    if ltr390 is not None:
        try:
            data["uv_light"] = {
                "uv_raw": float(ltr390.uvs),
                "ambient_light_raw": float(ltr390.light),
            }
        except Exception as e:
            errors["ltr390"] = repr(e)

    if tsl2591 is not None:
        try:
            data["light"] = {
                "lux": float(tsl2591.lux),
                "infrared": float(tsl2591.infrared),
                "visible": float(tsl2591.visible),
            }
        except Exception as e:
            errors["tsl2591"] = repr(e)

    # IMU: acceleration (m/s²) and rotation (rad/s), each as an (x, y, z) tuple.
    if imu is not None:
        try:
            accel_x, accel_y, accel_z = imu.acceleration
            gyro_x, gyro_y, gyro_z = imu.gyro
            data["imu"] = {
                "accel_mps2": [float(accel_x), float(accel_y), float(accel_z)],
                "gyro_rps": [float(gyro_x), float(gyro_y), float(gyro_z)],
            }
        except Exception as e:
            errors["icm20948"] = repr(e)

    return data, errors


def pressure_to_altitude(p_hpa: float, ref_p_hpa: float) -> float:
    """
    International barometric formula.

    Altitude (m) above the reference-pressure level. Use the launch-pad
    pressure as the reference so the returned value is altitude above
    launch site, not above mean sea level.
    """
    # Air pressure falls in a known way as you climb; this inverts that to
    # get height. The constants come from the standard atmosphere model.
    return 44330.0 * (1.0 - math.pow(p_hpa / ref_p_hpa, 0.1903))


def capture_reference_pressure(sensor: Any, samples: int = 10) -> tuple[float, Optional[str]]:
    """
    Average a few BMP280 readings at startup to use as the launch-pad
    reference. Returns the default 1013.25 hPa if reads fail.
    """
    # (The sensor is a BME280; "BMP280" in the line above is just a label
    # holdover.) Averaging several samples smooths out noise so "ground = 0 m"
    # is a stable baseline. If it can't read, fall back to standard sea-level
    # pressure — altitude then reads vs. sea level rather than the pad.
    if sensor is None:
        return DEFAULT_REF_PRESSURE_HPA, "no_sensor"

    readings = []
    for _ in range(samples):
        try:
            readings.append(float(sensor.pressure))
            time.sleep(0.1)
        except Exception:
            pass

    if not readings:
        return DEFAULT_REF_PRESSURE_HPA, "no_readings"

    return sum(readings) / len(readings), None


# ===================================================================
# Camera worker (subprocess in a background thread)
# ===================================================================
class CameraWorker:
    """
    Runs rpicam-still in a background thread so the main loop never blocks.

    Usage
    -----
    worker = CameraWorker(images_dir)
    worker.start()
    ...
    worker.request_capture(seq)         # non-blocking; main loop keeps going
    status = worker.snapshot_status()   # safe to read every loop
    ...
    worker.stop()
    """
    # WHY A THREAD: taking a photo takes a few seconds. Doing it in the main
    # loop would freeze telemetry during every capture, so the camera runs on
    # its own thread — the main loop just drops a request in a queue and moves on.

    def __init__(self, images_dir: Path) -> None:
        self.images_dir = images_dir
        # Mailbox of capture requests. maxsize=4: if the camera falls behind,
        # new requests are dropped rather than piling up without bound.
        self._queue: queue.Queue[int] = queue.Queue(maxsize=4)
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()   # thread-safe "please stop" flag
        self._lock = threading.Lock()         # guards the shared status dict

        # Status updated by the worker, read by the main loop.
        self._last_status: Dict[str, Any] = {
            "captured": False,
            "seq": None,
            "path": None,
            "bytes": None,
            "error": None,
        }

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="CameraWorker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def request_capture(self, seq: int) -> bool:
        """Returns False if the queue is full (capture skipped)."""
        try:
            self._queue.put_nowait(seq)
            return True
        except queue.Full:
            return False

    def snapshot_status(self) -> Dict[str, Any]:
        """Return a copy of the latest capture status (thread-safe)."""
        with self._lock:
            return dict(self._last_status)

    def _set_status(self, status: Dict[str, Any]) -> None:
        with self._lock:
            self._last_status = status

    def _run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                seq = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

            status = self._do_capture(seq)
            self._set_status(status)

    def _do_capture(self, seq: int) -> Dict[str, Any]:
        status: Dict[str, Any] = {
            "captured": False,
            "seq": seq,
            "path": None,
            "bytes": None,
            "error": None,
        }
        try:
            self.images_dir.mkdir(parents=True, exist_ok=True)
            img_path = self.images_dir / f"img_{seq:06d}.jpg"

            cmd = [
                "rpicam-still",
                "-n",
                "-t", "300",
                "--width", str(IMG_W),
                "--height", str(IMG_H),
                "-q", str(IMG_Q),
                "-o", str(img_path),
            ]

            # timeout kills a hung capture (the exact bug that lost all of
            # ATLAS-1's photos); check=True raises on a non-zero exit. Either
            # way it's caught below and logged, never crashing the flight.
            subprocess.run(
                cmd,
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=CAMERA_TIMEOUT_S,
            )

            status["path"] = str(img_path)
            status["bytes"] = int(img_path.stat().st_size)
            status["captured"] = True
        except Exception as e:
            status["error"] = repr(e)

        return status


# =====================================================================
# LoRa
# =====================================================================
def init_lora() -> tuple[Optional[Any], Optional[str]]:
    #validating frequency offset before touching radio
    try:
        freq_off = compute_freq_off(LORA_FREQ_MHZ)
    except ValueError as e:
        return None, f"freq_offsetOinvalid: {e}"
    try:
        lora = sx126x.sx126x(
            serial_num=LORA_PORT,
            freq=LORA_FREQ_MHZ,
            addr=LORA_SRC_ADDR,
            power=LORA_POWER_DBM,
            rssi=LORA_RSSI,
            air_speed=LORA_AIR_SPEED,
            net_id=LORA_NET_ID,
            buffer_size=LORA_BUFFER_SIZE,
            crypt=LORA_CRYPT,
            relay=False,
            lbt=False,
            wor=False,
        )
        return lora, None
    except Exception as e:
        return None, repr(e)


def compute_freq_off(freq_mhz: int) -> int:
    off = int(freq_mhz) - int(LORA_BASE_FREQ_MHZ)
    if not (0 <= off <= 255):
        raise ValueError(
            f"Frequency offset out of range: freq={freq_mhz} base={LORA_BASE_FREQ_MHZ} off={off}"
        )
    return off


def build_addressed_frame(dest_addr: int, payload: bytes) -> bytes:
    """Prepend the SX126x fixed-address header [DEST_H][DEST_L][FREQ_OFF]."""
    freq_off = compute_freq_off(LORA_FREQ_MHZ)
    return bytes([
        (dest_addr >> 8) & 0xFF,
        dest_addr & 0xFF,
        freq_off & 0xFF,
    ]) + payload


def transmit_record(lora: Any, record: Dict[str, Any]) -> Dict[str, Any]:
    """
    Pack the record into binary, wrap in TM frame, send TX_COPIES times.

    The 50-byte v2 binary payload still fits comfortably in a single TM
    frame (payload max 255), so we use frag_idx=0, frag_tot=1 every time
    (no fragmentation loop).

    Returns a status dict that gets attached to the JSONL record under
    "radio". Never raises — TX failure is recorded, not propagated.
    """
    # Each stage below is wrapped so a failure returns a status dict instead
    # of raising — transmitting must NEVER crash the flight loop.
    if lora is None:
        return {"enabled": False, "tx_success": False, "tx_error": "lora_none"}

    # ---- 1) tm_schema.pack → binary payload (v2 schema = 50 bytes) ----
    # (The "22 bytes" label is from the older v1 schema; the actual size is
    # whatever tm_schema.pack returns — reported as payload_bytes below.)
    try:
        binary_payload = tm_schema.pack(record)
    except Exception as e:
        return {"enabled": True, "tx_success": False, "tx_error": f"pack_failed:{e}"}

    # ---- 2) protocol_tm.build_frame → adds a 10-byte header (sync + IDs + CRC) ----
    # msg_id is the sequence number masked to 16 bits. frag_idx=0/frag_tot=1
    # because the payload fits in one frame (no splitting needed).
    msg_id = int(record.get("sequence_id") or 0) & 0xFFFF
    try:
        tm_frame = tm_build_frame(msg_id, 0, 1, binary_payload)
    except Exception as e:
        return {"enabled": True, "tx_success": False, "tx_error": f"tm_build_failed:{e}"}

    # ---- 3) prepend SX126x address header (who to send to + freq offset) ----
    try:
        radio_frame = build_addressed_frame(LORA_DEST_ADDR, tm_frame)
    except Exception as e:
        return {"enabled": True, "tx_success": False, "tx_error": f"addr_failed:{e}"}

    # ---- 4) Send the same frame TX_COPIES times back-to-back ----
    # Cheap insurance: if noise eats one copy, the other may survive. Even one
    # successful copy counts as tx_success.
    copies_sent = 0
    last_err: Optional[str] = None
    for _ in range(TX_COPIES):
        try:
            lora.send(radio_frame)
            copies_sent += 1
        except Exception as e:
            last_err = repr(e)
        time.sleep(TX_GAP_S)

    return {
        "enabled": True,
        "tx_success": copies_sent > 0,
        "tx_error": last_err if copies_sent == 0 else None,
        "msg_id": msg_id,
        "copies_sent": copies_sent,
        "copies_target": TX_COPIES,
        "payload_bytes": len(binary_payload),
        "frame_bytes": len(radio_frame),
    }


# =====================================================
# Run directories
# =====================================================
def create_run_directories(base_dir: Path) -> Dict[str, Path]:
    run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir    = base_dir / "runs" / run_id
    logs_dir   = run_dir / "logs"
    images_dir = run_dir / "images"
    logs_dir.mkdir(parents=True, exist_ok=True)
    images_dir.mkdir(parents=True, exist_ok=True)
    return {"run_dir": run_dir, "logs_dir": logs_dir, "images_dir": images_dir}


# =========================================================
# Main loop
# =========================================================
def main() -> None:
    base_dir = Path(__file__).resolve().parent
    dirs = create_run_directories(base_dir)
    telemetry_log_path = dirs["logs_dir"] / "telemetry.jsonl"

    # ---- Init sensors ----
    # init_sensors returns a dict of sensor objects (any that failed are None)
    # plus a dict of init errors we surface on every packet so a null field's
    # real cause is visible on the ground.
    i2c = init_i2c_bus()
    sensors = init_sensors(i2c)
    bme280 = sensors.get("bme280_sensor")
    sensor_init_errs = sensors.get("_init_errors") or {}

    # ---- Calibrate altitude reference (from BME280 pressure) ----
    ref_pressure_hpa, ref_err = capture_reference_pressure(bme280)
    print(f"Reference pressure: {ref_pressure_hpa:.2f} hPa (err={ref_err})")

    # ---- Init UPS ----
    try:
        ups_bus = SMBus(1)
    except Exception:
        ups_bus = None

    # ---- Init LoRa ----
    lora, lora_err = init_lora()
    freq_off = (LORA_FREQ_MHZ - LORA_BASE_FREQ_MHZ)
    print(f"Lora freq = {LORA_FREQ_MHZ} MHz base={LORA_BASE_FREQ_MHZ} MHz offset={freq_off} | err={lora_err}")
    if lora_err:
        print(f"LoRa init error: {lora_err}")

    # ---- Start camera worker ----
    camera = CameraWorker(dirs["images_dir"])
    camera.start()

    sequence_id = 0
    next_cleanup_s = time.time() + 60
    last_image_request_s = 0.0
    last_known_image_seq: Optional[int] = None

    try:
        while True:
            loop_start_s = time.time()
            sequence_id += 1

            # Every record starts with a UTC timestamp and a sequence number.
            record: Dict[str, Any] = {
                "timestamp_utc": utc_timestamp_iso(),
                "sequence_id": sequence_id,
            }

            # ---- Surface one-time sensor init errors in the JSONL stream ----
            # setdefault("errors", {}) creates the errors sub-dict on first use;
            # this same pattern collects every component's failures below.
            for k, v in sensor_init_errs.items():
                record.setdefault("errors", {})[k] = v

            # ---- BME280 (temperature, pressure, humidity → altitude) ----
            env: Dict[str, Any] = {}
            if bme280 is not None:
                env_data, env_err = read_bme280(bme280)
                env.update(env_data)
                if env_err:
                    record.setdefault("errors", {})["bme280"] = env_err

            # Compute altitude from pressure (above launch site)
            p_hpa = env.get("pressure_hpa")
            if isinstance(p_hpa, (int, float)) and p_hpa > 0:
                try:
                    env["altitude_m_est"] = pressure_to_altitude(p_hpa, ref_pressure_hpa)
                except Exception as e:
                    record.setdefault("errors", {})["altitude_calc"] = repr(e)

            record["environment"] = env

            # ---- UV/ambient, lux/IR/visible, IMU ----
            # These add "uv_light", "light", and "imu" sections to the record.
            # record.update() merges them in at the top level.
            extra_data, extra_errs = read_extra_sensors(sensors)
            record.update(extra_data)
            for k, v in extra_errs.items():
                record.setdefault("errors", {})[k] = v

            # ---- UPS / power ----
            if ups_bus is not None:
                power, ups_err = read_ups_status(ups_bus)
            else:
                power, ups_err = ({}, "ups_bus_init_failed")
            record["power"] = power
            record["alerts"] = compute_battery_alerts(power)
            if ups_err:
                record.setdefault("errors", {})["ups"] = ups_err

            # ---- Pi health (local log only, not transmitted) ----
            pi_health, pi_errs = read_pi_health(base_dir)
            record["pi_health"] = pi_health
            for k, v in pi_errs.items():
                record.setdefault("errors", {})[f"pi_{k}"] = v

            # ---- Camera (request capture; read worker's latest status) ----
            image_period_s = (
                CRIT_IMAGE_PERIOD_S
                if "BATT_CRIT_PCT" in record["alerts"]
                else IMAGE_PERIOD_S
            )
            now_s = loop_start_s
            if (now_s - last_image_request_s) >= image_period_s:
                if camera.request_capture(sequence_id):
                    last_image_request_s = now_s
                else:
                    record.setdefault("errors", {})["camera_queue"] = "queue_full"

            cam_status = camera.snapshot_status()
            record["image"] = cam_status
            if cam_status.get("error"):
                record.setdefault("errors", {})["camera"] = cam_status["error"]
            if cam_status.get("captured"):
                last_known_image_seq = cam_status.get("seq")

            # ---- LoRa TX (binary, 2x retransmit) ----
            # Pack + frame + send the compact binary copy; the returned status
            # (success, bytes sent, etc.) is stored so radio performance is
            # visible per packet in the log.
            record["radio"] = transmit_record(lora, record)

            # ---- Local JSONL log (safety net) ----
            # THE most important line: the full record hits the SD card whether
            # or not the radio worked. This file is the flight's source of truth.
            log_err = append_jsonl(telemetry_log_path, record)
            if log_err:
                record.setdefault("errors", {})["jsonl_log"] = log_err

            # ---- Periodic cleanup ----
            if time.time() >= next_cleanup_s:
                cleanup_folder_to_mb(dirs["logs_dir"], MAX_LOG_MB)
                cleanup_folder_to_mb(dirs["images_dir"], MAX_IMAGE_MB)
                next_cleanup_s = time.time() + 60

            # ---- Console status line ----
            # A one-line human-readable heartbeat, handy when watching over SSH
            # on the pad. Not part of the flight data itself.
            t_c     = env.get("temperature_c")
            rh      = env.get("humidity_pct")
            alt_m   = env.get("altitude_m_est")
            lux     = (record.get("light", {}) or {}).get("lux")
            batt_v  = power.get("battery_v")
            batt_pct= power.get("battery_pct")
            tx_ok   = record["radio"].get("tx_success")
            err_keys= list(record.get("errors", {}).keys())

            print(
                f"seq={sequence_id} t={record['timestamp_utc']} "
                f"T={t_c}C RH={rh}% alt={alt_m}m lux={lux} "
                f"bat={batt_v}V {batt_pct}% "
                f"img_seq={last_known_image_seq} tx={tx_ok} "
                f"errors={err_keys}"
            )

            sleep_to_rate(loop_start_s, LOOP_PERIOD_S)

    finally:
        camera.stop()
        try:
            if ups_bus is not None:
                ups_bus.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
