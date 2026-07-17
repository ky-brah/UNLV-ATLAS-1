#!/usr/bin/env python3
"""
test_flight_smoke.py — preflight check for flight.py.

Run this on the flight Pi BEFORE launch day. It catches the boring
integration problems (missing library, mismatched packet size, broken
framing) while you can still fix them at a desk.

A "smoke test" checks that the thing turns on without catching fire.
It doesn't prove your data is correct — only that nothing is obviously
broken end to end.

What it checks, most important first:
  1. flight.py imports cleanly            (catches missing libraries)
  2. tm_schema.pack() gives the right size (catches schema mismatches)
  3. A record survives the full radio path:
         record -> pack -> build_frame -> try_parse_one -> unpack
  4. The JSONL log writes parseable JSON
  5. main() can loop a few times without crashing, even with sensors
     or radio missing (errors get logged; crashes do not)

What it does NOT check — these need real hardware:
  - Whether the BME280 readings are sensible
  - Whether the UPS HAT reports true battery values
  - Whether LoRa frames physically transmit
  - Whether the camera makes valid JPEGs (use test_camera_worker_real.py)

Run:
    python3 test_flight_smoke.py

Exit code is 0 if every test passed, 1 if any failed — so you can spot
a bad result at a glance.
"""

import json
import sys
import time
import threading
from pathlib import Path

# Make flight.py importable from this directory
sys.path.insert(0, str(Path(__file__).resolve().parent))


# ----------------------------------------------------------------------
# Test 1: imports
# ----------------------------------------------------------------------
def test_imports():
    """
    The cheapest, most useful test: if a library is missing on this Pi,
    everything else is meaningless. Run this first and stop if it fails.
    """
    print("\n" + "=" * 70)
    print("TEST 1: flight.py imports cleanly")
    print("=" * 70)
    try:
        import flight  # noqa: F401
        import tm_schema  # noqa: F401
        import protocol_tm  # noqa: F401
        print("  PASS: all imports succeeded")
        return True
    except Exception as e:
        print(f"  FAIL: import error -> {type(e).__name__}: {e}")
        return False


# ----------------------------------------------------------------------
# Test 2: pack() produces the size the current schema says it should
# ----------------------------------------------------------------------
def test_pack_size():
    """
    A packet's size is a contract: the ground station reads a fixed
    number of bytes. If pack() returns a different length than the
    schema declares, decoding breaks silently on the ground.

    We ask tm_schema what the current version's length should be rather
    than hard-coding a number here, so this test stays correct when the
    schema is revised. (v1 was 22 bytes; v2 is 50.)
    """
    print("\n" + "=" * 70)
    print("TEST 2: tm_schema.pack() produces the expected packet size")
    print("=" * 70)
    import tm_schema

    # The expected length for whatever schema version is current.
    expected = tm_schema.V2_LEN if tm_schema.SCHEMA_VERSION == 2 else tm_schema.V1_LEN

    # A realistic record — the same shape flight.py builds each loop.
    record = {
        "timestamp_utc": "2026-05-20T18:30:00.000+00:00",
        "sequence_id": 12345,
        "environment": {
            "temperature_c": 22.5,
            "pressure_hpa": 1012.3,
            "humidity_pct": 41,
            "altitude_m_est": 150,
        },
        "power": {
            "battery_v": 4.05,
            "battery_pct": 87,
        },
        "image": {"captured": True},
        "alerts": ["BATT_LOW_PCT"],
    }

    blob = tm_schema.pack(record)
    print(f"  schema version: v{tm_schema.SCHEMA_VERSION} (expects {expected} bytes)")
    print(f"  pack() returned {len(blob)} bytes")
    if len(blob) == expected:
        print(f"  PASS: payload is exactly {expected} bytes")
        return True
    else:
        print(f"  FAIL: expected {expected} bytes, got {len(blob)}")
        return False


# ----------------------------------------------------------------------
# Test 3: round-trip (pack -> frame -> parse -> unpack)
# ----------------------------------------------------------------------
def test_round_trip():
    """
    The most valuable test here: it simulates the entire radio path in
    memory, with no radio. If a record survives pack -> frame -> parse
    -> unpack with its fields intact, the flight and ground sides agree
    on the format.

    Values are deliberately extreme (very cold, low pressure, high
    altitude) because that's what a real balloon sees near burst — a
    schema that only works at room temperature is a schema that fails
    at 30 km.
    """
    print("\n" + "=" * 70)
    print("TEST 3: Record survives full radio round-trip")
    print("=" * 70)
    import tm_schema
    from protocol_tm import build_frame, try_parse_one

    original = {
        "timestamp_utc": "2026-05-20T18:30:00.000+00:00",
        "sequence_id": 999,
        "environment": {
            "temperature_c": -45.3,
            "pressure_hpa": 250.7,
            "humidity_pct": 12,
            "altitude_m_est": 15800,
        },
        "power": {
            "battery_v": 3.78,
            "battery_pct": 42,
        },
        "image": {"captured": False},
        "alerts": ["CAMERA_FAULT"],
    }

    # 1. Pack the record down to its binary payload.
    payload = tm_schema.pack(original)

    # 2. Wrap it in a TM frame. The frame adds a 10-byte header
    #    (sync bytes, ids, CRC) on top of the payload.
    frame = build_frame(msg_id=original["sequence_id"], frag_idx=0, frag_tot=1, payload=payload)
    expected_frame = len(payload) + 10
    print(f"  payload: {len(payload)} bytes, frame: {len(frame)} bytes (expected {expected_frame})")

    # 3. Now pretend to be the ground station: find and verify the frame.
    parsed, remaining = try_parse_one(frame)
    if parsed is None:
        print("  FAIL: try_parse_one could not decode the frame")
        return False

    # 4. Turn the bytes back into a record.
    decoded = tm_schema.unpack(parsed["payload"])

    # 5. Compare the fields that matter. Temperature and pressure are
    #    compared rounded because packing scales them to integers — a
    #    tiny loss of precision is expected and fine.
    checks = [
        ("sequence_id", original["sequence_id"], decoded["sequence_id"]),
        ("temperature_c (approx)",
            round(original["environment"]["temperature_c"], 1),
            round(decoded["environment"]["temperature_c"], 1)),
        ("pressure_hpa (approx)",
            round(original["environment"]["pressure_hpa"], 1),
            round(decoded["environment"]["pressure_hpa"], 1)),
        ("altitude_m_est",
            original["environment"]["altitude_m_est"],
            decoded["environment"]["altitude_m_est"]),
        ("battery_pct", original["power"]["battery_pct"], decoded["power"]["battery_pct"]),
        ("camera_fault_flag",
            "CAMERA_FAULT" in original["alerts"],
            "CAMERA_FAULT" in decoded["alerts"]),
    ]

    all_pass = True
    for name, want, got in checks:
        match = want == got
        marker = "OK " if match else "BAD"
        print(f"  [{marker}] {name}: want={want} got={got}")
        if not match:
            all_pass = False

    if all_pass:
        print("  PASS: round-trip preserved all key fields")
    else:
        print("  FAIL: round-trip lost or corrupted fields")
    return all_pass


# ----------------------------------------------------------------------
# Test 4: JSONL append produces parseable output
# ----------------------------------------------------------------------
def test_jsonl_log():
    """
    The SD card log is the source of truth for a flight — the radio only
    carries a lossy copy. If a line of the log is malformed, that reading
    is gone for good. This writes a few records to /tmp and reads them
    back to prove the log format holds.
    """
    print("\n" + "=" * 70)
    print("TEST 4: JSONL append writes parseable records")
    print("=" * 70)
    import flight

    log_path = Path("/tmp/smoke_test.jsonl")
    if log_path.exists():
        log_path.unlink()

    test_records = [
        {"sequence_id": 1, "timestamp_utc": "2026-05-20T18:30:00.000+00:00", "data": "first"},
        {"sequence_id": 2, "timestamp_utc": "2026-05-20T18:30:01.000+00:00", "data": "second"},
        {"sequence_id": 3, "timestamp_utc": "2026-05-20T18:30:02.000+00:00", "nested": {"a": 1}},
    ]

    for r in test_records:
        err = flight.append_jsonl(log_path, r)
        if err:
            print(f"  FAIL: append_jsonl returned error: {err}")
            return False

    # Read it back
    with log_path.open() as f:
        lines = f.readlines()

    if len(lines) != len(test_records):
        print(f"  FAIL: expected {len(test_records)} lines, got {len(lines)}")
        return False

    for i, line in enumerate(lines):
        try:
            parsed = json.loads(line)
            if parsed["sequence_id"] != test_records[i]["sequence_id"]:
                print(f"  FAIL: line {i} sequence_id mismatch")
                return False
        except json.JSONDecodeError as e:
            print(f"  FAIL: line {i} is not valid JSON: {e}")
            return False

    print(f"  PASS: wrote and parsed {len(lines)} records")
    log_path.unlink()
    return True


# ----------------------------------------------------------------------
# Test 5: main loop runs a few iterations without crashing
# ----------------------------------------------------------------------
def test_main_loop_runs():
    """
    Proves flight.py's core promise: a missing sensor or radio must not
    stop the loop. We run main() on a background thread for a few
    seconds and only ask one question — did it crash?

    Errors printed during this test are EXPECTED if hardware isn't
    attached. A crash is not.
    """
    print("\n" + "=" * 70)
    print("TEST 5: main() loop runs for 6 seconds without crashing")
    print("=" * 70)
    print("  (sensors and radio may not be present — errors are OK, crashes are not)")

    import flight

    # Run main() in a background thread, kill it after a few seconds
    crashed = {"yes": False, "error": None}

    def runner():
        try:
            flight.main()
        except KeyboardInterrupt:
            pass
        except SystemExit:
            pass
        except Exception as e:
            crashed["yes"] = True
            crashed["error"] = f"{type(e).__name__}: {e}"

    t = threading.Thread(target=runner, daemon=True)
    t.start()

    # Let it run a few iterations. At 0.5 Hz that's about 3 loop iterations.
    time.sleep(6.0)

    # The main loop runs forever — we just need it to not have crashed.
    if crashed["yes"]:
        print(f"  FAIL: main() crashed -> {crashed['error']}")
        return False

    if not t.is_alive():
        print("  FAIL: main() returned/exited unexpectedly")
        return False

    print("  PASS: main() ran for 6s without crashing")
    print("  NOTE: thread will keep running until process exits (daemon)")
    return True


# ----------------------------------------------------------------------
# Runner
# ----------------------------------------------------------------------
if __name__ == "__main__":
    results = []

    results.append(("imports", test_imports()))
    if not results[-1][1]:
        print("\nSKIPPING remaining tests because imports failed.")
        sys.exit(1)

    results.append(("pack_size", test_pack_size()))
    results.append(("round_trip", test_round_trip()))
    results.append(("jsonl_log", test_jsonl_log()))
    results.append(("main_loop", test_main_loop_runs()))

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    for name, passed in results:
        marker = "PASS" if passed else "FAIL"
        print(f"  [{marker}] {name}")

    total = len(results)
    passed_count = sum(1 for _, p in results if p)
    print(f"\n{passed_count}/{total} tests passed\n")

    sys.exit(0 if passed_count == total else 1)
