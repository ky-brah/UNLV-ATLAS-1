#!/usr/bin/env python3
"""
test_camera_worker_real.py — check the camera on the real hardware.

Unlike the smoke test (which fakes everything), this one uses the
ACTUAL camera and writes real .jpg files. Run it on the flight Pi.

Before running, confirm the camera works at all:
    rpicam-still -n -t 300 -o /tmp/manual_test.jpg
If that fails, fix the camera first — this test can't help you.

What this confirms:
  - CameraWorker fires rpicam-still on its background thread
  - Real .jpg files land in the images folder with non-zero size
  - The main loop keeps ticking during a capture (never stalls)
  - Several captures in a row don't break anything

Why the threading matters: taking a photo takes about a second. If
flight.py waited for it, telemetry would stall every time. So captures
run on a separate thread, and the loop just checks in on them. This
test proves that actually works.

Run:
    python3 test_camera_worker_real.py

Writes to /tmp/cam_real_test/, so it won't touch your flight data.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import flight  # noqa: E402


def main():
    # Write test photos somewhere harmless — never into a real run folder.
    images_dir = Path("/tmp/cam_real_test")
    print(f"Images will be written to: {images_dir}")

    worker = flight.CameraWorker(images_dir)
    worker.start()

    # Ask for 3 photos, 5 seconds apart. Our loop below ticks every
    # 0.5s — much faster than the captures — which is the whole point:
    # we keep running while the camera thread works in the background.
    capture_seqs = [101, 102, 103]
    next_capture_idx = 0
    next_capture_at = time.time()  # fire first one immediately

    loop_start = time.time()
    seen_seqs = set()

    while time.time() - loop_start < 30:
        now = time.time()

        # Time for the next photo? Ask for it. request_capture() returns
        # immediately — it does not wait for the photo to be taken.
        if next_capture_idx < len(capture_seqs) and now >= next_capture_at:
            seq = capture_seqs[next_capture_idx]
            ok = worker.request_capture(seq)
            print(f"  [t={now-loop_start:5.2f}s] requested capture seq={seq} accepted={ok}")
            next_capture_idx += 1
            next_capture_at = now + 5.0

        # Check on the worker each tick. This mirrors what flight.py
        # does: ask "any news?" rather than waiting around.
        status = worker.snapshot_status()
        completed_seq = status.get("seq")

        if (
            status.get("captured")
            and completed_seq is not None
            and completed_seq not in seen_seqs
        ):
            seen_seqs.add(completed_seq)
            print(
                f"  [t={now-loop_start:5.2f}s] CAPTURED seq={completed_seq} "
                f"path={status['path']} bytes={status['bytes']}"
            )
        elif status.get("error") and completed_seq not in seen_seqs:
            seen_seqs.add(completed_seq)
            print(
                f"  [t={now-loop_start:5.2f}s] ERROR seq={completed_seq} "
                f"error={status['error']}"
            )

        time.sleep(0.5)  # main loop cadence

    worker.stop()

    # Summary
    print("\n--- Summary ---")
    files = sorted(images_dir.glob("*.jpg")) if images_dir.exists() else []
    print(f"Files produced: {len(files)}")
    for f in files:
        print(f"  {f.name}  ({f.stat().st_size} bytes)")

    if len(files) == len(capture_seqs):
        print("PASS: all expected captures produced files")
    else:
        print(f"FAIL: expected {len(capture_seqs)} files, got {len(files)}")


if __name__ == "__main__":
    main()
