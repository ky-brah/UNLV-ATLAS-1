# MOSS Kit

A hands-on kit for building, flying, and recovering a real telemetry payload — the hardware, the code, and the docs that tie them together. Built by students, for students, as part of UNLV's ATLAS-1 high-altitude balloon platform.

## What's here

The folders in this directory are already arranged in the exact structure each Raspberry Pi expects. Don't rearrange them — the scripts write files relative to their own location, and the ground station's nginx setup depends on this layout.

-  Flight Pi (payload). The main flight program, shared pipeline modules, LoRa radio driver, and preflight tests. A `runs/` folder gets created at runtime, one timestamped folder per launch.
- ground — Ground Pi. The receiver, the dashboard it serves, the shared modules, and optional test scripts for checking things work with no radio attached.
- `MOSS-docs.html` — Full setup documentation. Open it in any browser.

## Getting the files onto your Pis

Download the whole thing as a ZIP straight from GitHub rather than cloning or copying files one at a time:

You can either get the whole repository, or simply the software folder.

The ZIP preserves the folder structure exactly as it appears here, which is why this is the recommended way to get started.

## The most important rule

`tm_schema.py` and `protocol_tm.py` must be **byte-identical copies** on both Pis. They aren't shared over the network — each Pi keeps its own copy. If the two drift apart, the ground station can't decode anything the flight Pi sends.

Because the ZIP gives both Pis the same starting copies, downloading fresh is the simplest way to guarantee they match. To verify later:

```bash
sha256sum ~/hab/tm_schema.py ~/hab/protocol_tm.py       # flight Pi
sha256sum ~/ground/tm_schema.py ~/ground/protocol_tm.py # ground Pi
```
Note: The above file paths /hab may not be accurate to yours

If the fingerprints match, the files are the same.

## Step-by-step guide

**Open `MOSS-docs.html` in a browser** for the full walkthrough. It covers everything below in detail, with commands you can copy.

The recommended order:

1. **Main setup** — do this on both Pis. OS, connection, interfaces, Python. Four steps, once per Pi. Nothing else works until this is done.
2. **Ground station** — build and test this first. Install nginx, set the folder path, then prove the dashboard works using fake data. No radio needed, nothing to wire.
3. **Flight Pi** — the payload. Wire the sensors, confirm the bus sees each one, install the drivers, then run it. With the ground station already working, you can watch real telemetry arrive immediately.
4. **Field hotspot** — last of all. Only once both Pis work on ordinary Wi-Fi. This is purely for the launch site, where no network exists.

Ground before flight is deliberate: the ground station can be fully tested on a desk, so when you power up the payload, anything that goes wrong is on the flight side rather than a mystery split between the two.

## Hardware

| Component | Role |

| Raspberry Pi 3B+ | Flight computer |
| Sensor suite (BME280, LTR390, TSL2591, ICM20948) | Environmental + motion data |
| Raspberry Pi Camera v2 | Imaging |

## Notes

- Paths in the docs use `unlvcube1` as the username (`/home/unlvcube1/hab/`). Yours will read `/home/<your-username>/` with whatever name you chose in Raspberry Pi Imager.
- Folder names (`hab`, `ground`) are examples. The structure is what matters.
- Wi-Fi never reaches the balloon in flight. It's only for the ground: starting scripts before launch and pulling data after recovery.

Questions, bugs, or ideas? Reach us at *contact TBD*.
