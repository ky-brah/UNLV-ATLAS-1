"""
sx126x.py

Demo-safe driver for Waveshare/Ebyte SX126x UART LoRa modules when
M0/M1 are strapped with jumpers (Normal Mode).

- Keeps original API
- Skips self.set() at init
- Forces Normal Mode (M0=0, M1=0)
- Adds burst-safe recv_packet() to avoid partial UART slices

Context: the SX126x is configured over UART by toggling two mode pins
(M0/M1). This patched build assumes the radios were ALREADY configured
identically by hand and the pins are jumpered, so it never reconfigures
at runtime — it just opens the serial port and sends/receives bytes.
"""

import time
from typing import Any, Dict, Optional, Tuple

import RPi.GPIO as GPIO
import serial


class sx126x:
    # GPIO pins (BCM numbering) wired to the module's mode-select pins.
    M0 = 22
    M1 = 27

    def __init__(
        self,
        serial_num: str = "/dev/serial0",
        freq: int = 915,
        addr: int = 1,
        power: int = 22,
        rssi: bool = False,
        air_speed: int = 2400,
        net_id: int = 0,
        buffer_size: int = 240,
        crypt: int = 0,
        relay: bool = False,
        lbt: bool = False,
        wor: bool = False,
    ) -> None:
        # Store the config. NOTE: in this patched build most of these are
        # recorded but NOT applied to the module (see the skipped set() below);
        # they must already match the physical radio's jumper/manual setup.
        self.serial_num = serial_num
        self.start_freq = int(freq)
        self.addr = int(addr)
        self.power = int(power)
        self.rssi = bool(rssi)
        self.air_speed = int(air_speed)
        self.net_id = int(net_id)
        self.buffer_size = int(buffer_size)
        self.crypt = int(crypt)
        self.relay = bool(relay)
        self.lbt = bool(lbt)
        self.wor = bool(wor)

        # GPIO init (safe even if not used)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)

        # Force NORMAL mode (M0=0, M1=0) — the mode where the module actually
        # sends and receives data (as opposed to its configuration mode).
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)

        # Open the UART. 9600 baud is the module's default config-pin baud;
        # timeout=0 makes reads non-blocking so recv_packet controls timing.
        self.ser = serial.Serial(
            self.serial_num,
            9600,
            timeout=0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        # IMPORTANT: Skip self.set(...) / runtime configuration
        # Assumes both radios were pre-configured to same settings.

    # -------------------------------------------------
    # TX
    # -------------------------------------------------
    def send(self, data: bytes) -> None:
        # Transmitting is just writing bytes to the UART — the module takes
        # care of the actual radio transmission.
        if isinstance(data, str):
            data = data.encode("utf-8")
        self.ser.write(data)

    # -------------------------------------------------
    # Helpers
    # -------------------------------------------------
    def max_app_payload_bytes(self, header_len: int = 3) -> int:
        """
        Maximum application payload size accounting for UART header bytes.
        """
        return max(0, int(self.buffer_size) - int(header_len))

    # -------------------------------------------------
    # RX helpers
    # -------------------------------------------------
    def recv_packet(self, timeout_s: float = 0.5) -> Optional[bytes]:
        """
        Read one UART *burst* (closest thing to a full packet) instead of an arbitrary slice.

        Behavior:
        - Wait up to timeout_s for the first byte.
        - Once bytes arrive, keep reading and accumulating.
        - Stop when the UART has been idle for a short gap (IDLE_GAP_S).

        This greatly reduces the chance that higher-level code feeds partial packet fragments
        into parse_packet(), which assumes packet-aligned data.
        """
        # The problem this solves: a naive read grabs whatever bytes happen to
        # be in the buffer right now, which is often HALF a packet. Instead we
        # read until the line goes quiet, treating that quiet gap as the packet
        # boundary — so a whole frame arrives in one piece.
        deadline = time.time() + timeout_s
        buf = bytearray()

        # 30ms of no new bytes -> consider the burst complete
        IDLE_GAP_S = 0.03
        # Safety cap: don't let one read grow unbounded
        MAX_BURST = int(getattr(self, "buffer_size", 240))

        # Phase 1: wait (up to timeout) for the first byte to show up.
        while time.time() < deadline:
            waiting = self.ser.inWaiting()
            if waiting > 0:
                # Read what is currently available
                chunk = self.ser.read(waiting)
                if chunk:
                    buf.extend(chunk)

                # Phase 2: keep collecting until the line is idle for IDLE_GAP_S,
                # which marks the end of this burst.
                idle_start = time.time()
                while True:
                    time.sleep(0.005)
                    waiting2 = self.ser.inWaiting()
                    if waiting2 > 0:
                        chunk2 = self.ser.read(waiting2)
                        if chunk2:
                            buf.extend(chunk2)
                        idle_start = time.time()  # got data, reset the idle timer
                        if len(buf) >= MAX_BURST:
                            break
                    else:
                        if (time.time() - idle_start) >= IDLE_GAP_S:
                            break

                return bytes(buf) if buf else None

            # Nothing in the buffer yet — wait a beat before checking again.
            # 50ms (vs 5ms) cuts idle CPU ~10x on a Pi 3B+ with no RX impact.
            time.sleep(0.05)

        return None  # timed out with no bytes

    def parse_packet(self, pkt: bytes) -> Tuple[Dict[str, Any], bytes]:
        """
        Parse module header:

        - First 2 bytes: src address (big-endian)
        - Next 1 byte: freq offset
        - Optional last 1 byte: RSSI (if self.rssi True)
        - Payload in the middle
        """
        # The module prepends a small routing header to each received packet.
        # This strips it off, returning the metadata plus the raw payload
        # (which is the TM frame that protocol_tm.try_parse_one then decodes).
        meta: Dict[str, Any] = {}
        if not pkt or len(pkt) < 3:
            return meta, b""

        src_addr = (pkt[0] << 8) | pkt[1]
        freq_off = pkt[2]
        freq_mhz = int(self.start_freq + freq_off)

        meta.update({"src_addr": src_addr, "freq_mhz": freq_mhz, "raw_len": len(pkt)})

        # When RSSI reporting is on, the module appends a signal-strength byte
        # at the very end — pull it off and convert to dBm.
        if self.rssi and len(pkt) >= 4:
            rssi_raw = pkt[-1]
            meta["packet_rssi_dbm"] = -(256 - int(rssi_raw))
            payload = pkt[3:-1]
        else:
            payload = pkt[3:]

        return meta, payload
