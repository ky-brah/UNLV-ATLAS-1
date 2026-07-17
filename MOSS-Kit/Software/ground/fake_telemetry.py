#!/usr/bin/env python3
"""
fake_telemetry.py
UNLVCUBE1 — Feed pretend telemetry to the dashboard (ground pi), with NO radio.

Why this exists
---------------
Before a launch (or while building the ground station) you want to know
the dashboard actually works: that nginx serves the files, that the
charts draw, and that numbers update live. This script writes the same
two files the real receiver writes — latest.json and logs/history.jsonl
— but fills them with realistic made-up numbers. Open the dashboard and
you should see it come alive.

It touches ONLY those two files. No radio, no sudo, no system settings,
so it cannot harm your Pi or your setup.

How to run (on the ground Pi, in the folder with your dashboard files):
    python3 fake_telemetry.py

Then open the dashboard in a browser and watch the charts move.
Press Ctrl+C to stop.

The field names below (t_c, rh, p_hpa, lux, uv_raw, ambient_raw, bv, bp)
match what index.html reads. If you ever rename a field in the
dashboard, rename it here too so the two stay in step.
"""

import json
import math
import os
import random
import time
from datetime import datetime, timezone
from pathlib import Path

# =====================================================================
# SETTINGS — the only part you might want to change
# =====================================================================
# How many seconds between fake packets. 2.0 matches the dashboard's
# 2-second refresh, so the graphs update smoothly.
SEND_INTERVAL_S = 2.0

# How many past packets to pre-fill so the charts aren't empty on load.
SEED_COUNT = 50
# =====================================================================


# ---------------------------------------------------------------------
# File locations — found automatically, nothing to edit.
# ---------------------------------------------------------------------
# This script assumes it lives in the ground folder next to latest.json
# (the same place rx_to_latest.py runs). It works out that folder from
# its own location, so you never have to hard-code a path.
GROUND_DIR   = Path(__file__).resolve().parent
LATEST_PATH  = GROUND_DIR / "latest.json"
LOG_DIR      = GROUND_DIR / "logs"
HISTORY_PATH = LOG_DIR / "history.jsonl"


def utc_now_iso() -> str:
    # Timestamp in the same UTC ISO format the real receiver uses.
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, obj: dict) -> None:
    # Write to a temp file then rename over the target, so the dashboard
    # never reads a half-written file (rename is instant/atomic).
    tmp = path.with_name(f"{path.name}.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":")) + "\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def make_packet(seq: int) -> dict:
    """
    Build one realistic-looking telemetry record.

    Uses gentle sine waves plus a little randomness so the values drift
    the way a real flight would, instead of sitting still. Field names
    match exactly what the dashboard reads.
    """
    t_c   = round(22.0 + 3.0 * math.sin(seq * 0.10) + random.uniform(-0.3, 0.3), 2)
    rh    = round(45.0 + 5.0 * math.cos(seq * 0.08) + random.uniform(-0.5, 0.5), 1)
    p_hpa = round(1013.0 - seq * 0.05 + random.uniform(-0.2, 0.2), 2)
    lux   = round(max(0.0, 800 + 200 * math.sin(seq * 0.20) + random.uniform(-20, 20)), 0)

    # UV and ambient are the two "raw" light fields the dashboard shows.
    uv_raw      = round(max(0.0, 120 + 60 * math.sin(seq * 0.15) + random.uniform(-8, 8)), 0)
    ambient_raw = round(max(0.0, 900 + 150 * math.cos(seq * 0.12) + random.uniform(-15, 15)), 0)

    # Altitude climbs early then eases, roughly like an ascent.
    altitude_m = round(max(0.0, 150 + seq * 12 + random.uniform(-5, 5)), 0)

    # Battery slowly drains.
    bv = round(3.95 - seq * 0.0005 + random.uniform(-0.01, 0.01), 2)
    bp = max(0, round(88 - seq * 0.02 + random.uniform(-0.5, 0.5)))

    ts = utc_now_iso()

    return {
        "seq": seq,
        "ts": ts,
        "_rx_utc": ts,
        "schema_version": 2,
        "t_c": t_c,
        "rh": rh,
        "p_hpa": p_hpa,
        "altitude_m": altitude_m,
        "lux": lux,
        "uv_raw": uv_raw,
        "ambient_raw": ambient_raw,
        "bv": bv,
        "bp": bp,
        "image_captured": (seq % 12 == 0),   # pretend a photo every ~12 packets
        "alerts": [],
        "run_id": "fake-test",
    }


def seed_history(n: int) -> None:
    # Pre-fill history.jsonl so the charts already have a curve on load.
    print(f">>> Seeding {n} past packets into {HISTORY_PATH.name} ...")
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    with open(HISTORY_PATH, "w", encoding="utf-8") as f:
        for i in range(n):
            f.write(json.dumps(make_packet(i), separators=(",", ":")) + "\n")
    print("    done.")


def main() -> None:
    print("=" * 60)
    print("  UNLVCUBE1 — fake telemetry (dashboard test, no radio)")
    print("=" * 60)
    print(f"  Folder:       {GROUND_DIR}")
    print(f"  latest.json:  {LATEST_PATH}")
    print(f"  history:      {HISTORY_PATH}")
    print()

    # A friendly check: if latest.json's folder has no dashboard, the user
    # is probably running this in the wrong place. We don't stop — writing
    # the files is harmless — but we say so.
    if not (GROUND_DIR / "static" / "index.html").exists():
        print("  NOTE: no static/index.html here. If the dashboard doesn't")
        print("        update, you may be running this outside the ground folder.")
        print()

    seed_history(SEED_COUNT)

    print()
    print(f">>> Writing a fake packet every {SEND_INTERVAL_S:g}s. Ctrl+C to stop.")
    print()

    seq = SEED_COUNT
    try:
        while True:
            pkt = make_packet(seq)
            atomic_write_json(LATEST_PATH, pkt)
            with open(HISTORY_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(pkt, separators=(",", ":")) + "\n")

            print(f"  seq={seq:4d}  T={pkt['t_c']:5.1f}C  RH={pkt['rh']:4.1f}%  "
                  f"P={pkt['p_hpa']:7.1f}hPa  bat={pkt['bv']:.2f}V {pkt['bp']}%")

            seq += 1
            time.sleep(SEND_INTERVAL_S)

    except KeyboardInterrupt:
        print()
        print(">>> Stopped.")
        print("    If the charts moved in the browser, the dashboard works.")
        print("    If they stayed blank, check that nginx is running and that")
        print("    you opened the dashboard's address in the browser.")


if __name__ == "__main__":
    main()
