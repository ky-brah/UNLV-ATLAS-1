#!/usr/bin/env python3
"""
rx_to_latest.py — GROUND station receiver.

This is the mirror image of flight.py: it listens on the radio, rebuilds
each telemetry packet, and writes it to files the web dashboard reads.

The path each packet takes:
    radio bytes -> strip radio header -> find TM frames (CRC-checked)
    -> reassemble -> tm_schema.unpack() -> a record dict -> write to disk

Two files are produced:
    latest.json    the most recent reading (for the live dashboard)
    history.jsonl  every reading, one per line (for charts)

The flight side sends each packet twice for reliability, so the same
packet usually arrives twice; the TelemetryAccumulator drops the duplicate.
"""

import json
import os
import sys
import time
import fcntl
from pathlib import Path
from datetime import datetime, timezone
from typing import Any, Dict, Iterator, Optional

try:
    import sx126x  # type: ignore
    HAVE_RADIO = True
except Exception:
    sx126x = None  # type: ignore
    HAVE_RADIO = False

# Shared protocol + schema — MUST match flight exactly.
from protocol_tm import try_parse_one, HDR_LEN, MAGIC, VER  # noqa: F401
import tm_schema


# ----------------------------
# Single-instance lock
# ----------------------------
# Prevent multiple RX processes from fighting over /dev/serial0 and latest.json
LOCK_PATH = Path("/tmp/rx_to_latest.lock")
_lock_fd = open(LOCK_PATH, "w")
try:
    fcntl.flock(_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    print("rx_to_latest.py already running (lock held). Exiting.")
    sys.exit(0)


# ----------------------------
# RADIO CONFIG (match flight)
# ----------------------------
LORA_PORT = "/dev/serial0"
LORA_FREQ_MHZ = 915
LORA_ADDR = 1
LORA_POWER_DBM = 22
LORA_BUFFER_SIZE = 240
LORA_CRYPT = 0
LORA_RSSI = True
LORA_AIR_SPEED = 2400
LORA_NET_ID = 0
RX_TIMEOUT_S = 5.0

# ----------------------------
# Status + reassembly tuning
# ----------------------------
# "Contact" means: we have received ANY bytes recently (even if decode
# hasn't completed).
NO_CONTACT_AFTER_S = 30.0
HEARTBEAT_EVERY_S = 1.0

# Give fragments time to arrive out-of-order / with delays.
REASSEMBLY_TTL_S = 60.0

# Remember recently-decoded msg_ids so the 2x retransmit doesn't double-write.
# 256 covers a wide span of seq values; msg_id is only 16-bit and wraps, so
# this is a bounded ring, not unbounded growth.
DEDUP_WINDOW = 256

RADIO_SETTLE_S = 2.0
SERIAL_ERROR_BACKOFF_S = 0.5

# How often to write a "CONTACT (RX BYTES...)" status while receiving bytes.
CONTACT_RAW_EVERY_S = 1.0


# ----------------------------
# Files
# ----------------------------
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "logs"
LATEST_PATH = BASE_DIR / "latest.json"
HISTORY_PATH = LOG_DIR / "history.jsonl"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def utc_now() -> str:
    # Current UTC time as an ISO-8601 string, used to timestamp every status write.
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    """
    Atomic write (temp file then rename) so the web server never reads a
    half-written JSON.
    """
    # Write to a temp file, flush to disk, then rename over the target. Rename
    # is atomic, so a reader (the dashboard) always sees either the old whole
    # file or the new whole file — never a half-written one.
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    data = json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def append_history(obj: dict) -> None:
    """Append decoded records for charts/history."""
    # One JSON object per line (JSONL), so the dashboard can replay the flight.
    with open(HISTORY_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")


def flatten_for_dashboard(record: Dict[str, Any], meta: Optional[dict]) -> dict:
    """
    Map a nested tm_schema.unpack() record onto the FLAT field names the
    dashboard (index_l2.html) reads. latest.json / history.jsonl are a
    contract with the dashboard, so the shape is pinned here rather than
    letting the binary schema's internal layout leak into the UI.

    Dashboard field -> source:
        seq         <- sequence_id
        t_c         <- environment.temperature_c
        p_hpa       <- environment.pressure_hpa
        rh          <- environment.humidity_pct
        altitude_m  <- environment.altitude_m_est
        bv          <- power.battery_v
        bp          <- power.battery_pct
        uv_raw      <- uv_light.uv_raw
        ambient_raw <- uv_light.ambient_light_raw
        lux         <- light.lux
        ir          <- light.infrared
        vis         <- light.visible
        accel       <- imu.accel_mps2  ([x,y,z] and flat ax/ay/az)
        gyro        <- imu.gyro_rps    ([x,y,z] and flat gx/gy/gz)
        alerts      <- alerts (passthrough)

    A field whose sensor failed comes back as None from tm_schema.unpack()
    and is emitted as null; the dashboard formats null / missing keys as
    "—" on its own, so there's no harm in always emitting the key.

    v1 (22-byte) packets have no uv_light / light / imu blocks, so those
    fields fall back to None automatically via the `or {}` guards below.
    """
    env = record.get("environment") or {}
    power = record.get("power") or {}
    image = record.get("image") or {}
    uv_light = record.get("uv_light") or {}
    light = record.get("light") or {}
    imu = record.get("imu") or {}

    accel = imu.get("accel_mps2")  # [x, y, z] or None
    gyro = imu.get("gyro_rps")     # [x, y, z] or None

    def _axis(seq, i):
        if isinstance(seq, (list, tuple)) and len(seq) > i:
            return seq[i]
        return None

    return {
        # identity / timing
        "seq": record.get("sequence_id"),
        "sequence_id": record.get("sequence_id"),
        "ts": record.get("timestamp_utc"),
        "timestamp_utc": record.get("timestamp_utc"),

        # environment
        "t_c": env.get("temperature_c"),
        "p_hpa": env.get("pressure_hpa"),
        "rh": env.get("humidity_pct"),
        "altitude_m": env.get("altitude_m_est"),

        # battery
        "bv": power.get("battery_v"),
        "bp": power.get("battery_pct"),

        # UV / ambient (LTR390)
        "uv_raw": uv_light.get("uv_raw"),
        "ambient_raw": uv_light.get("ambient_light_raw"),

        # light (TSL2591)
        "lux": light.get("lux"),
        "ir": light.get("infrared"),
        "vis": light.get("visible"),

        # IMU (ICM20948) — both array and flat-axis forms
        "accel": accel,
        "gyro": gyro,
        "ax": _axis(accel, 0),
        "ay": _axis(accel, 1),
        "az": _axis(accel, 2),
        "gx": _axis(gyro, 0),
        "gy": _axis(gyro, 1),
        "gz": _axis(gyro, 2),

        # status
        "image_captured": bool(image.get("captured")),
        "alerts": record.get("alerts", []),

        # provenance
        "_radio": meta or None,
        "_flags_raw": record.get("_flags_raw"),
        "schema_version": record.get("schema_version"),
    }


def ensure_latest_initialized() -> None:
    """Ensure latest.json exists so the dashboard has something to load."""
    # On first run there's no data yet; write a placeholder so the dashboard
    # loads cleanly instead of erroring on a missing file.
    if (not LATEST_PATH.exists()) or LATEST_PATH.stat().st_size == 0:
        atomic_write_json(
            LATEST_PATH,
            {"status": "NO DATA YET", "_rx_utc": utc_now(), "_radio": None},
        )


# ==========================================================================================
# TelemetryAccumulator
# ==========================================================================================
class TelemetryAccumulator:
    """
    Owns the byte->frame->record decode pipeline and its state.

    Usage:
        acc = TelemetryAccumulator()
        for record in acc.feed(raw_bytes, now=time.time()):
            # record is a decoded telemetry dict from tm_schema.unpack()
            ...

    The class holds:
      - tm_stream: rolling byte buffer that TM frames are parsed out of.
      - pending:   partial multi-fragment messages keyed by msg_id.
      - seen:      recently-decoded msg_ids for dedup (2x retransmit).

    It does NOT touch files or the radio — feed() is pure decode so it can
    be unit-tested with handcrafted byte strings.
    """

    def __init__(
        self,
        reassembly_ttl_s: float = REASSEMBLY_TTL_S,
        dedup_window: int = DEDUP_WINDOW,
    ) -> None:
        self.reassembly_ttl_s = reassembly_ttl_s
        self.dedup_window = dedup_window

        self.tm_stream: bytes = b""
        # pending[msg_id] = {"t0": float, "tot": int, "parts": {frag_idx: bytes}}
        self.pending: Dict[int, Dict[str, Any]] = {}

        # Dedup ring: insertion-ordered set of recently decoded msg_ids.
        self._seen: Dict[int, None] = {}

        # Lightweight counters for status / debugging.
        self.frames_parsed = 0
        self.records_decoded = 0
        self.duplicates_dropped = 0
        self.unpack_failures = 0

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------
    def feed(self, chunk: bytes, now: Optional[float] = None) -> Iterator[Dict[str, Any]]:
        """
        Add bytes to the TM stream and yield every newly completed,
        non-duplicate telemetry record.
        """
        # This is the accumulator's main entry point: hand it raw bytes, and it
        # yields back any fully-decoded records those bytes completed.
        if now is None:
            now = time.time()

        self._expire_pending(now)

        if chunk:
            self.tm_stream += chunk

        # Parse out every frame currently in the stream. try_parse_one may
        # return None *and still consume bytes* (resyncing past garbage or a
        # CRC failure by sliding forward). So we don't stop on the first None
        # — we stop only when the stream stops shrinking, meaning what's left
        # is a genuine incomplete frame waiting for more bytes.
        while True:
            before = len(self.tm_stream)
            frame, rest = try_parse_one(self.tm_stream)
            self.tm_stream = rest

            if frame is not None:
                self.frames_parsed += 1
                record = self._handle_frame(frame, now)
                if record is not None:
                    yield record
                continue

            # No frame this pass. If the buffer didn't shrink, we're stuck on
            # an incomplete frame — leave it for the next feed().
            if len(self.tm_stream) >= before:
                break
            # Otherwise bytes were dropped (resync/CRC slide); keep trying.

    def pending_summary(self) -> str:
        """Human-readable in-progress reassembly state, e.g. '23:2/3 24:1/3'."""
        items = [
            f"{mid}:{len(st.get('parts', {}))}/{st.get('tot')}"
            for mid, st in self.pending.items()
        ]
        return " ".join(items) if items else "(empty)"

    @property
    def tm_stream_len(self) -> int:
        return len(self.tm_stream)

    # ----------------------------------------------------------------
    # Internals
    # ----------------------------------------------------------------
    def _expire_pending(self, now: float) -> None:
        """Drop partial messages that never completed within the TTL."""
        # Housekeeping: if some fragments of a message never arrived, don't keep
        # their partial data forever — discard once it's older than the TTL.
        for mid in list(self.pending.keys()):
            if (now - self.pending[mid].get("t0", now)) > self.reassembly_ttl_s:
                del self.pending[mid]

    def _seen_recently(self, msg_id: int) -> bool:
        # True if we've already decoded this msg_id (used to drop the 2x copy).
        return msg_id in self._seen

    def _mark_seen(self, msg_id: int) -> None:
        # Record a msg_id as decoded, keeping the memory bounded to a fixed window.
        self._seen[msg_id] = None
        # Bound the dedup memory: evict oldest insertions past the window.
        while len(self._seen) > self.dedup_window:
            oldest = next(iter(self._seen))
            del self._seen[oldest]

    def _handle_frame(self, frame: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        """
        Slot a parsed TM frame into reassembly state. When the message is
        complete, unpack it and return the decoded record (or None if it's a
        duplicate or fails to unpack).
        """
        msg_id = frame["msg_id"]
        frag_idx = frame["frag_idx"]
        frag_tot = frame["frag_tot"]
        frag_payload = frame["payload"]

        # If we've already emitted this msg_id, swallow the retransmit copy
        # without rebuilding it.
        if self._seen_recently(msg_id):
            self.duplicates_dropped += 1
            return None

        st = self.pending.get(msg_id)
        if st is None or st.get("tot") != frag_tot:
            st = {"t0": now, "tot": frag_tot, "parts": {}}
            self.pending[msg_id] = st

        st["parts"][frag_idx] = frag_payload

        # Not all fragments here yet.
        if len(st["parts"]) != st["tot"]:
            return None

        # Complete — assemble in fragment order and decode.
        full = b"".join(st["parts"][i] for i in range(st["tot"]))
        del self.pending[msg_id]

        try:
            record = tm_schema.unpack(full)
        except Exception as e:
            self.unpack_failures += 1
            print(f"unpack failed for msg_id={msg_id}: {e} | bytes={full.hex()}")
            return None

        # Decoded OK — mark seen so the second copy is ignored.
        self._mark_seen(msg_id)
        self.records_decoded += 1
        return record


# ================================================================================================
# Main loop
# ================================================================================================
def main() -> None:
    # Ground-station entry point: set up the radio, then loop forever reading
    # bursts, decoding them, and writing records to disk. If the radio library
    # isn't present it falls back to a DEMO mode so the dashboard still runs.
    ensure_latest_initialized()

    if not HAVE_RADIO:
        print("sx126x not available -> DEMO mode (no radio).")
        while True:
            atomic_write_json(
                LATEST_PATH,
                {"status": "DEMO / NO RADIO", "_rx_utc": utc_now(), "_radio": None},
            )
            time.sleep(1.0)

    print("Starting LoRa RX… (binary TM)")
    print(f"Waiting {RADIO_SETTLE_S:.1f}s for LoRa radio/UART to settle…")
    time.sleep(RADIO_SETTLE_S)

    lora = sx126x.sx126x(
        serial_num=LORA_PORT,
        freq=LORA_FREQ_MHZ,
        addr=LORA_ADDR,
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

    acc = TelemetryAccumulator()

    last_rx_bytes_time = 0.0
    last_heartbeat = 0.0
    last_contact_write = 0.0
    last_good_rx_utc: Optional[str] = None

    print("RX running… waiting for telemetry.")

    while True:
        # 1) Read a burst from radio/UART.
        try:
            pkt = lora.recv_packet(timeout_s=RX_TIMEOUT_S)
        except Exception as e:
            print("Serial RX error, retrying:", e)
            time.sleep(SERIAL_ERROR_BACKOFF_S)
            continue

        now = time.time()

        # 2) No bytes this cycle -> heartbeat status.
        if pkt is None:
            if (now - last_heartbeat) >= HEARTBEAT_EVERY_S:
                since_rx = now - last_rx_bytes_time
                status = (
                    "CONTACT (RX BYTES, decode pending)"
                    if since_rx < NO_CONTACT_AFTER_S
                    else "NO CONTACT"
                )
                atomic_write_json(
                    LATEST_PATH,
                    {
                        "status": status,
                        "_rx_utc": utc_now(),
                        "last_good_rx_utc": last_good_rx_utc,
                        "pending": acc.pending_summary(),
                        "tm_stream_len": acc.tm_stream_len,
                        "_radio": None,
                    },
                )
                last_heartbeat = now
            continue

        # Normalize pkt to raw bytes.
        raw = pkt[0] if isinstance(pkt, (tuple, list)) and pkt else pkt
        raw = bytes(raw)

        last_rx_bytes_time = now
        last_good_rx_utc = utc_now()

        # 3) Periodic contact-only update so the dashboard shows "contact".
        if (now - last_contact_write) >= CONTACT_RAW_EVERY_S:
            atomic_write_json(
                LATEST_PATH,
                {
                    "status": "CONTACT (RX BYTES, decode pending)",
                    "_rx_utc": utc_now(),
                    "last_good_rx_utc": last_good_rx_utc,
                    "rx_len": len(raw),
                    "rx_hex": raw[:16].hex(),
                    "pending": acc.pending_summary(),
                    "tm_stream_len": acc.tm_stream_len,
                    "_radio": None,
                },
            )
            last_contact_write = now
            last_heartbeat = now

        # 4) Pull SX126x meta + payload, but don't corrupt TM frames:
        # if raw already begins with MAGIC, feed raw straight to the TM parser.
        meta: Dict[str, Any] = {}
        payload = b""
        try:
            meta, payload = lora.parse_packet(raw)
            payload = payload or b""
        except Exception:
            meta, payload = {}, b""

        if raw.startswith(MAGIC):
            chunk = raw
        elif payload.startswith(MAGIC):
            chunk = payload
        else:
            chunk = payload if payload else raw

        if not chunk:
            continue

        # 5) Feed the accumulator; write out every completed record.
        for record in acc.feed(chunk, now=now):
            flat = flatten_for_dashboard(record, meta)
            flat["_rx_utc"] = utc_now()

            atomic_write_json(LATEST_PATH, flat)
            append_history(flat)

            last_heartbeat = now

            seq = flat.get("seq")
            rssi = meta.get("packet_rssi_dbm") if meta else None
            print(f"RX OK (TM) seq={seq} rssi={rssi}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nRX stopped.")
