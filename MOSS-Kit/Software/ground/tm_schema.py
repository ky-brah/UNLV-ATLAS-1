# tm_schema.py
# UNLVCUBE1 — Binary telemetry schema (single source of truth)
#
# Both flight-side (flight.py) and ground-side (rx_to_latest.py) MUST
# import pack() / unpack() from this module. Nothing else.
#
# If this file changes, BOTH sides must be updated together. The
# SCHEMA_VERSION byte at the start of every packet exists so old
# captures stay decodable after a schema bump.

import struct
from datetime import datetime, timezone
from typing import Any, Dict


# -------------------------
# Schema version
# -------------------------
# Bump this whenever the field layout changes. Old captures stay decodable
# because unpack() reads the version byte first and dispatches accordingly.
#
# v1: temp, pressure, humidity, altitude, battery V/pct, flags  (22 bytes)
# v2: everything in v1 PLUS UV, ambient light, lux/IR/visible, IMU accel+gyro
SCHEMA_VERSION = 2


# ==================================================================================
# Wire format (version 1) — UNCHANGED. Kept so old captures stay decodable.
# ==================================================================================
# Big-endian (>) for network byte order — matches protocol_tm.py.
#
# Field           | Type   | Bytes | Encoding
# ----------------|--------|-------|----------------------------------------
# schema_version  | uint8  |   1   | currently 1
# seq             | uint32 |   4   | sequence_id from flight.py
# ts_unix         | uint32 |   4   | seconds since 1970 UTC
# temp_c          | int16  |   2   | temperature_c × 100 (signed)
# pressure_hpa    | uint16 |   2   | pressure_hpa × 10
# humidity_pct    | uint8  |   1   | 0..100
# altitude_m      | uint32 |   4   | altitude in meters above launch site
# battery_v       | uint16 |   2   | battery_v × 100
# battery_pct     | uint8  |   1   | 0..100
# flags           | uint8  |   1   | bitfield, see below
# ----------------|--------|-------|----------------------------------------
# total                       22 bytes

V1_FORMAT = ">B I I h H B I H B B"
V1_LEN = struct.calcsize(V1_FORMAT)  # 22


# ==================================================================================
# Wire format (version 2) — current. Superset of v1.
# ==================================================================================
# The first 10 fields are byte-for-byte identical to v1 (same order, same
# encoding) except the leading version byte is 2. Everything after `flags`
# is new. This keeps the layout easy to reason about and lets you eyeball
# a v2 packet as "a v1 packet with a tail".
#
# Field           | Type   | Bytes | Encoding
# ----------------|--------|-------|----------------------------------------
# schema_version  | uint8  |   1   | currently 2
# seq             | uint32 |   4   | sequence_id
# ts_unix         | uint32 |   4   | seconds since 1970 UTC
# temp_c          | int16  |   2   | temperature_c × 100 (signed)
# pressure_hpa    | uint16 |   2   | pressure_hpa × 10
# humidity_pct    | uint8  |   1   | 0..100
# altitude_m      | uint32 |   4   | altitude m above launch site
# battery_v       | uint16 |   2   | battery_v × 100
# battery_pct     | uint8  |   1   | 0..100
# flags           | uint8  |   1   | bitfield, see below
# --- new in v2 ----------------------------------------------------------
# uv_raw          | uint32 |   4   | LTR390 uvs (raw counts)
# ambient_raw     | uint32 |   4   | LTR390 light/ALS (raw counts)
# lux             | uint32 |   4   | TSL2591 lux × 10
# ir_raw          | uint16 |   2   | TSL2591 infrared (raw counts)
# vis_raw         | uint16 |   2   | TSL2591 visible (raw counts)
# accel_x         | int16  |   2   | m/s² × 100 (signed)
# accel_y         | int16  |   2   | m/s² × 100 (signed)
# accel_z         | int16  |   2   | m/s² × 100 (signed)
# gyro_x          | int16  |   2   | rad/s × 100 (signed)
# gyro_y          | int16  |   2   | rad/s × 100 (signed)
# gyro_z          | int16  |   2   | rad/s × 100 (signed)
# ----------------|--------|-------|----------------------------------------
# total                       50 bytes

# Python concatenates adjacent string literals, so this is one format string:
# the v1 layout, then the 5 light fields, then the 6 IMU axes. Kept split into
# three visual groups so it maps cleanly onto the table above.
V2_FORMAT = ">B I I h H B I H B B" "I I I H H" "h h h h h h"
V2_LEN = struct.calcsize(V2_FORMAT)  # 50


# -------------------------
# Flags bitfield (shared by v1 and v2)
# -------------------------
# Bit | Name              | Meaning
# ----|-------------------|-------------------------------
# 0   | IMAGE_CAPTURED    | record.image.captured == True
# 1   | BATT_LOW_PCT      | "BATT_LOW_PCT" in alerts
# 2   | BATT_CRIT_PCT     | "BATT_CRIT_PCT" in alerts
# 3   | BATT_LOW_V        | "BATT_LOW_V" in alerts
# 4   | CAMERA_FAULT      | "CAMERA_FAULT" in alerts
# 5-7 | (reserved)        | always 0; reserved for future use

FLAG_IMAGE_CAPTURED = 1 << 0
FLAG_BATT_LOW_PCT   = 1 << 1
FLAG_BATT_CRIT_PCT  = 1 << 2
FLAG_BATT_LOW_V     = 1 << 3
FLAG_CAMERA_FAULT   = 1 << 4


# -------------------------
# Sentinel values
# -------------------------
# When a field is missing or sensor failed, we still need to send something.
# These sentinels are picked so they're obviously "no data" on the ground
# rather than looking like real readings.
TEMP_SENTINEL_RAW       = -32768   # int16 min  → unpacks to -327.68 °C
PRESSURE_SENTINEL_RAW   = 0         # uint16     → 0.0 hPa (impossible)
HUMIDITY_SENTINEL_RAW   = 255       # uint8      → 255 % (impossible)
ALTITUDE_SENTINEL_RAW   = 0xFFFFFFFF
BATT_V_SENTINEL_RAW     = 0         # uint16     → 0.00 V
BATT_PCT_SENTINEL_RAW   = 255       # uint8      → 255 % (impossible)

# v2 sentinels
UV_SENTINEL_RAW         = 0xFFFFFFFF  # uint32
AMBIENT_SENTINEL_RAW    = 0xFFFFFFFF  # uint32
LUX_SENTINEL_RAW        = 0xFFFFFFFF  # uint32 (lux × 10)
IR_SENTINEL_RAW         = 0xFFFF      # uint16
VIS_SENTINEL_RAW        = 0xFFFF      # uint16
IMU_SENTINEL_RAW        = -32768      # int16 min, used for every accel/gyro axis


# -------------------------
# Helpers
# -------------------------
def _iso_to_unix(iso_str: Any) -> int:
    """
    Convert an ISO-8601 UTC timestamp like '2026-03-27T21:13:00.123+00:00'
    to a unix timestamp (seconds since 1970). Returns 0 if input is invalid.
    """
    # A plain integer of seconds is 4 bytes on the wire; the text form is ~29.
    if not isinstance(iso_str, str):
        return 0
    try:
        dt = datetime.fromisoformat(iso_str)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        return int(dt.timestamp())
    except Exception:
        return 0


def _unix_to_iso(ts_unix: int) -> str:
    """Inverse of _iso_to_unix. Returns ISO-8601 with millisecond precision."""
    # Used on the ground to turn the packed integer time back into readable text.
    try:
        dt = datetime.fromtimestamp(ts_unix, tz=timezone.utc)
        return dt.isoformat(timespec="milliseconds")
    except Exception:
        return ""


def _safe_get(d: Any, key: str, default: Any = None) -> Any:
    """dict.get() that survives None/missing intermediate dicts."""
    # Records may be missing whole sub-dicts if a sensor failed, so we can't
    # assume record["environment"] etc. exist.
    if not isinstance(d, dict):
        return default
    return d.get(key, default)


def _scale_uint(value: Any, scale: float, hi: int, sentinel: int) -> int:
    """
    Coerce value to round(value * scale) as an unsigned int clamped to [0, hi].
    Returns sentinel if value is None or non-numeric. Negative values clamp to 0.
    """
    # Shared encoder for all unsigned fields: multiply by the scale (e.g. ×10),
    # round to an int, and keep it inside the field's byte range.
    if value is None:
        return sentinel
    try:
        v = int(round(float(value) * scale))
    except (TypeError, ValueError):
        return sentinel
    if v < 0:
        return 0
    if v > hi:
        return hi
    return v


def _scale_int16(value: Any, scale: float, sentinel: int) -> int:
    """
    Coerce value to round(value * scale) as a signed int16 clamped to
    [-32767, 32767] (leaving -32768 as the sentinel). Returns sentinel if
    value is None or non-numeric.
    """
    # Signed version of the above, for values that can go negative (temperature,
    # accel, gyro). -32768 is reserved as the "no data" sentinel.
    if value is None:
        return sentinel
    try:
        v = int(round(float(value) * scale))
    except (TypeError, ValueError):
        return sentinel
    if v < -32767:
        return -32767
    if v > 32767:
        return 32767
    return v


def _imu_axis(seq: Any, idx: int) -> Any:
    """Pull one axis out of an [x, y, z] list, surviving None/short lists."""
    if not isinstance(seq, (list, tuple)) or len(seq) <= idx:
        return None
    return seq[idx]


# ==================================================================================
# Pack: called in the flight script. Always emits the current SCHEMA_VERSION (v2).
# ==================================================================================
def pack(record: Dict[str, Any]) -> bytes:
    """
    Convert a flight.py telemetry record dict to wire-format bytes (v2).

    Input shape (only the fields we encode are required; missing fields
    become sentinels):
        {
            "timestamp_utc": "2026-03-27T21:13:00.123+00:00",
            "sequence_id": 1418,
            "environment": {
                "temperature_c": 34.9,
                "pressure_hpa": 974.8,
                "humidity_pct": 42.1,
                "altitude_m_est": 0,
            },
            "uv_light": {
                "uv_raw": 12.0,
                "ambient_light_raw": 3400.0,
            },
            "light": {
                "lux": 812.5,
                "infrared": 230.0,
                "visible": 1900.0,
            },
            "imu": {
                "accel_mps2": [0.1, -0.2, 9.79],
                "gyro_rps": [0.01, 0.0, -0.02],
            },
            "power": {"battery_v": 4.05, "battery_pct": 87},
            "image": {"captured": True},
            "alerts": ["BATT_LOW_PCT", ...],
        }

    Returns: exactly V2_LEN (50) bytes.
    """
    env      = _safe_get(record, "environment", {}) or {}
    uv_light = _safe_get(record, "uv_light", {}) or {}
    light    = _safe_get(record, "light", {}) or {}
    imu      = _safe_get(record, "imu", {}) or {}
    power    = _safe_get(record, "power", {}) or {}
    image    = _safe_get(record, "image", {}) or {}
    alerts   = _safe_get(record, "alerts", []) or []

    # ---- seq / timestamp ----
    seq = _scale_uint(_safe_get(record, "sequence_id"), 1, 0xFFFFFFFF, 0)
    ts_unix = _iso_to_unix(_safe_get(record, "timestamp_utc"))

    # ---- temperature: float °C → int16 (×100) ----
    temp_raw = _scale_int16(_safe_get(env, "temperature_c"), 100, TEMP_SENTINEL_RAW)

    # ---- pressure: float hPa → uint16 (×10) ----
    pressure_raw = _scale_uint(_safe_get(env, "pressure_hpa"), 10, 0xFFFF, PRESSURE_SENTINEL_RAW)

    # ---- humidity: float % → uint8 (0..100) ----
    raw_humidity = _safe_get(env, "humidity_pct")
    if raw_humidity is None:
        humidity_raw = HUMIDITY_SENTINEL_RAW
    else:
        try:
            humidity_raw = max(0, min(100, int(round(float(raw_humidity)))))
        except (TypeError, ValueError):
            humidity_raw = HUMIDITY_SENTINEL_RAW

    # ---- altitude: float m → uint32 (leave 0xFFFFFFFF as sentinel) ----
    raw_altitude = _safe_get(env, "altitude_m_est")
    if raw_altitude is None:
        altitude_raw = ALTITUDE_SENTINEL_RAW
    else:
        try:
            a = int(round(float(raw_altitude)))
            if a < 0:
                a = 0  # pre-launch readings can be slightly negative; clamp
            elif a > 0xFFFFFFFE:
                a = 0xFFFFFFFE
            altitude_raw = a
        except (TypeError, ValueError):
            altitude_raw = ALTITUDE_SENTINEL_RAW

    # ---- battery ----
    bv_raw = _scale_uint(_safe_get(power, "battery_v"), 100, 0xFFFF, BATT_V_SENTINEL_RAW)
    raw_bp = _safe_get(power, "battery_pct")
    if raw_bp is None:
        bp_raw = BATT_PCT_SENTINEL_RAW
    else:
        try:
            bp_raw = max(0, min(100, int(round(float(raw_bp)))))
        except (TypeError, ValueError):
            bp_raw = BATT_PCT_SENTINEL_RAW

    # ---- flags ----
    flags = 0
    if _safe_get(image, "captured"):
        flags |= FLAG_IMAGE_CAPTURED
    if "BATT_LOW_PCT" in alerts:
        flags |= FLAG_BATT_LOW_PCT
    if "BATT_CRIT_PCT" in alerts:
        flags |= FLAG_BATT_CRIT_PCT
    if "BATT_LOW_V" in alerts:
        flags |= FLAG_BATT_LOW_V
    if "CAMERA_FAULT" in alerts:
        flags |= FLAG_CAMERA_FAULT

    # ---- UV / ambient (LTR390): raw counts → uint32 ----
    uv_raw      = _scale_uint(_safe_get(uv_light, "uv_raw"), 1, 0xFFFFFFFE, UV_SENTINEL_RAW)
    ambient_raw = _scale_uint(_safe_get(uv_light, "ambient_light_raw"), 1, 0xFFFFFFFE, AMBIENT_SENTINEL_RAW)

    # ---- TSL2591: lux ×10 → uint32, IR/visible raw → uint16 ----
    lux_raw = _scale_uint(_safe_get(light, "lux"), 10, 0xFFFFFFFE, LUX_SENTINEL_RAW)
    ir_raw  = _scale_uint(_safe_get(light, "infrared"), 1, 0xFFFE, IR_SENTINEL_RAW)
    vis_raw = _scale_uint(_safe_get(light, "visible"), 1, 0xFFFE, VIS_SENTINEL_RAW)

    # ---- IMU: accel/gyro float → int16 (×100) ----
    accel = _safe_get(imu, "accel_mps2")
    gyro  = _safe_get(imu, "gyro_rps")
    ax = _scale_int16(_imu_axis(accel, 0), 100, IMU_SENTINEL_RAW)
    ay = _scale_int16(_imu_axis(accel, 1), 100, IMU_SENTINEL_RAW)
    az = _scale_int16(_imu_axis(accel, 2), 100, IMU_SENTINEL_RAW)
    gx = _scale_int16(_imu_axis(gyro, 0), 100, IMU_SENTINEL_RAW)
    gy = _scale_int16(_imu_axis(gyro, 1), 100, IMU_SENTINEL_RAW)
    gz = _scale_int16(_imu_axis(gyro, 2), 100, IMU_SENTINEL_RAW)

    return struct.pack(
        V2_FORMAT,
        SCHEMA_VERSION,
        seq,
        ts_unix,
        temp_raw,
        pressure_raw,
        humidity_raw,
        altitude_raw,
        bv_raw,
        bp_raw,
        flags,
        uv_raw,
        ambient_raw,
        lux_raw,
        ir_raw,
        vis_raw,
        ax, ay, az,
        gx, gy, gz,
    )


# ==================================================================================
# Unpack: called on the ground. Dispatches on the version byte.
# ==================================================================================
def unpack(blob: bytes) -> Dict[str, Any]:
    """
    Convert wire-format bytes back to a dict shaped like flight.py records.
    Sentinel values become None (so "no data" is obvious downstream).

    Raises ValueError on malformed input — callers should catch it and
    log the offending bytes for debugging.
    """
    if not isinstance(blob, (bytes, bytearray)):
        raise ValueError(f"unpack expected bytes, got {type(blob).__name__}")

    if len(blob) < 1:
        raise ValueError("unpack: empty buffer")

    version = blob[0]

    if version == 1:
        return _unpack_v1(bytes(blob))
    if version == 2:
        return _unpack_v2(bytes(blob))

    raise ValueError(f"unpack: unknown schema_version={version}")


def _unpack_v1(blob: bytes) -> Dict[str, Any]:
    # Decode a 22-byte v1 packet (old captures). Inverse of the v1 layout:
    # unpack raw ints, undo the scaling, turn sentinels back into None.
    if len(blob) != V1_LEN:
        raise ValueError(f"unpack v1: expected {V1_LEN} bytes, got {len(blob)}")

    (
        version,
        seq,
        ts_unix,
        temp_raw,
        pressure_raw,
        humidity_raw,
        altitude_raw,
        bv_raw,
        bp_raw,
        flags,
    ) = struct.unpack(V1_FORMAT, blob)

    temp_c       = None if temp_raw     == TEMP_SENTINEL_RAW      else temp_raw / 100.0
    pressure_hpa = None if pressure_raw == PRESSURE_SENTINEL_RAW  else pressure_raw / 10.0
    humidity_pct = None if humidity_raw == HUMIDITY_SENTINEL_RAW  else int(humidity_raw)
    altitude_m   = None if altitude_raw == ALTITUDE_SENTINEL_RAW  else int(altitude_raw)
    battery_v    = None if bv_raw       == BATT_V_SENTINEL_RAW    else bv_raw / 100.0
    battery_pct  = None if bp_raw       == BATT_PCT_SENTINEL_RAW  else int(bp_raw)

    alerts = _decode_alert_flags(flags)

    return {
        "schema_version": int(version),
        "sequence_id": int(seq),
        "timestamp_utc": _unix_to_iso(int(ts_unix)),
        "environment": {
            "temperature_c": temp_c,
            "pressure_hpa": pressure_hpa,
            "humidity_pct": humidity_pct,
            "altitude_m_est": altitude_m,
        },
        "power": {
            "battery_v": battery_v,
            "battery_pct": battery_pct,
        },
        "image": {
            "captured": bool(flags & FLAG_IMAGE_CAPTURED),
        },
        "alerts": alerts,
        "_flags_raw": int(flags),
    }


def _unpack_v2(blob: bytes) -> Dict[str, Any]:
    # Decode a 50-byte v2 packet (current). Same as v1 for the first ten fields,
    # then the added light and IMU values.
    if len(blob) != V2_LEN:
        raise ValueError(f"unpack v2: expected {V2_LEN} bytes, got {len(blob)}")

    (
        version,
        seq,
        ts_unix,
        temp_raw,
        pressure_raw,
        humidity_raw,
        altitude_raw,
        bv_raw,
        bp_raw,
        flags,
        uv_raw,
        ambient_raw,
        lux_raw,
        ir_raw,
        vis_raw,
        ax, ay, az,
        gx, gy, gz,
    ) = struct.unpack(V2_FORMAT, blob)

    # ---- shared v1 fields ----
    temp_c       = None if temp_raw     == TEMP_SENTINEL_RAW      else temp_raw / 100.0
    pressure_hpa = None if pressure_raw == PRESSURE_SENTINEL_RAW  else pressure_raw / 10.0
    humidity_pct = None if humidity_raw == HUMIDITY_SENTINEL_RAW  else int(humidity_raw)
    altitude_m   = None if altitude_raw == ALTITUDE_SENTINEL_RAW  else int(altitude_raw)
    battery_v    = None if bv_raw       == BATT_V_SENTINEL_RAW    else bv_raw / 100.0
    battery_pct  = None if bp_raw       == BATT_PCT_SENTINEL_RAW  else int(bp_raw)

    # ---- v2 additions ----
    uv_raw_v       = None if uv_raw      == UV_SENTINEL_RAW       else int(uv_raw)
    ambient_raw_v  = None if ambient_raw == AMBIENT_SENTINEL_RAW  else int(ambient_raw)
    lux            = None if lux_raw     == LUX_SENTINEL_RAW      else lux_raw / 10.0
    infrared       = None if ir_raw      == IR_SENTINEL_RAW       else int(ir_raw)
    visible        = None if vis_raw     == VIS_SENTINEL_RAW      else int(vis_raw)

    def _imu(v: int):
        return None if v == IMU_SENTINEL_RAW else v / 100.0

    accel = [_imu(ax), _imu(ay), _imu(az)]
    gyro  = [_imu(gx), _imu(gy), _imu(gz)]
    # If every axis came back as sentinel, report the whole block as missing.
    accel_out = None if all(c is None for c in accel) else accel
    gyro_out  = None if all(c is None for c in gyro) else gyro

    alerts = _decode_alert_flags(flags)

    return {
        "schema_version": int(version),
        "sequence_id": int(seq),
        "timestamp_utc": _unix_to_iso(int(ts_unix)),
        "environment": {
            "temperature_c": temp_c,
            "pressure_hpa": pressure_hpa,
            "humidity_pct": humidity_pct,
            "altitude_m_est": altitude_m,
        },
        "uv_light": {
            "uv_raw": uv_raw_v,
            "ambient_light_raw": ambient_raw_v,
        },
        "light": {
            "lux": lux,
            "infrared": infrared,
            "visible": visible,
        },
        "imu": {
            "accel_mps2": accel_out,
            "gyro_rps": gyro_out,
        },
        "power": {
            "battery_v": battery_v,
            "battery_pct": battery_pct,
        },
        "image": {
            "captured": bool(flags & FLAG_IMAGE_CAPTURED),
        },
        "alerts": alerts,
        "_flags_raw": int(flags),
    }


def _decode_alert_flags(flags: int) -> list:
    # Turn the flags byte back into the human-readable alert list (tests each bit).
    alerts = []
    if flags & FLAG_BATT_LOW_PCT:  alerts.append("BATT_LOW_PCT")
    if flags & FLAG_BATT_CRIT_PCT: alerts.append("BATT_CRIT_PCT")
    if flags & FLAG_BATT_LOW_V:    alerts.append("BATT_LOW_V")
    if flags & FLAG_CAMERA_FAULT:  alerts.append("CAMERA_FAULT")
    return alerts
