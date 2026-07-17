# protocol_tm.py
#
# The framing layer for the radio link. A "frame" is the payload wrapped
# in a small header so the ground station can (a) find where a packet
# starts in a noisy byte stream and (b) confirm it arrived intact.
# This file must be an identical copy on the flight Pi and the ground Pi.
import struct

# Every frame starts with these two bytes. The ground scans for them to
# lock onto the start of a packet — this is the "sync" marker.
MAGIC = b"TM"
VER = 1

# MAGIC(2), VER(1), MSG_ID(u16), FRAG_IDX(u8), FRAG_TOT(u8), PAYLEN(u8), CRC16(u16)
# struct format string: ">" = big-endian (network byte order).
HDR_FMT = ">2sB H B B B H"
HDR_LEN = struct.calcsize(HDR_FMT)

def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    # A checksum over the header + payload. The sender computes it and
    # stores it in the frame; the receiver recomputes it and compares.
    # If they differ, the packet was corrupted in transit and is dropped.
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def build_frame(msg_id: int, frag_idx: int, frag_tot: int, payload: bytes) -> bytes:
    """
    Build a single TM frame:
      [MAGIC 'TM'][VER][msg_id][frag_idx][frag_tot][paylen][crc16][payload]
    """
    # frag_idx / frag_tot support splitting a large payload across several
    # frames. Our 22-byte telemetry fits in one, so flight.py always sends
    # frag_idx=0, frag_tot=1 — but the framing supports more.
    if not (1 <= frag_tot <= 255):
        raise ValueError("frag_tot out of range")
    if not (0 <= frag_idx < frag_tot):
        raise ValueError("frag_idx out of range")
    if len(payload) > 255:
        raise ValueError("payload too long (max 255)")

    # CRC is computed over the header (minus the CRC field itself) plus the
    # payload, then the full frame is packed with the CRC included.
    header_wo_crc = struct.pack(">B H B B B", VER, msg_id & 0xFFFF, frag_idx, frag_tot, len(payload))
    crc = crc16_ccitt(header_wo_crc + payload)
    return struct.pack(HDR_FMT, MAGIC, VER, msg_id & 0xFFFF, frag_idx, frag_tot, len(payload), crc) + payload

def try_parse_one(buf: bytes):
    """
    Stream parser:
      - Finds MAGIC anywhere (resync)
      - Parses one complete TM frame if available
      - Verifies CRC16
    Returns: (frame_dict, remaining_buf) or (None, buf) if incomplete/invalid.

    frame_dict: {"msg_id": int, "frag_idx": int, "frag_tot": int, "payload": bytes}
    """
    # Radio bytes arrive as a messy stream, not neat packets. This pulls at
    # most ONE valid frame off the front and hands back the leftover bytes
    # to try again later. Returning (None, buf) means "not enough yet."
    if len(buf) < HDR_LEN:
        return None, buf

    # Resync: skip any garbage before the first MAGIC marker.
    i = buf.find(MAGIC)
    if i == -1:
        return None, b""
    if i > 0:
        buf = buf[i:]
        if len(buf) < HDR_LEN:
            return None, buf

    magic, ver, msg_id, frag_idx, frag_tot, paylen, crc = struct.unpack(HDR_FMT, buf[:HDR_LEN])
    # A MAGIC match can happen by chance in noise; if the version byte is
    # wrong, slide forward one byte and keep hunting.
    if magic != MAGIC or ver != VER:
        return None, buf[1:]  # slide forward and keep searching

    need = HDR_LEN + paylen
    if len(buf) < need:
        return None, buf  # full payload hasn't arrived yet

    payload = buf[HDR_LEN:need]

    # Recompute the CRC and compare. Mismatch = corrupted or a false MAGIC
    # match, so slide forward and keep searching rather than trusting it.
    header_wo_crc = struct.pack(">B H B B B", ver, msg_id, frag_idx, frag_tot, paylen)
    crc_calc = crc16_ccitt(header_wo_crc + payload)
    if crc_calc != crc:
        return None, buf[1:]  # CRC mismatch, slide forward

    frame = {
        "msg_id": msg_id,
        "frag_idx": frag_idx,
        "frag_tot": frag_tot,
        "payload": payload,
    }
    # Hand back the good frame plus everything after it for the next call.
    return frame, buf[need:]
