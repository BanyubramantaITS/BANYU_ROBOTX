#!/usr/bin/env python3
"""
Emit 10 Hz zero-delta VISION_POSITION_DELTA over MAVLink2 for GPS-denied, DVL-less POR runs.

ArduSub requires EKF3 PV_AidingMode != AID_NONE (position_ok() == true) both to enter
GUIDED mode (ModeGuided::init) and to stream LOCAL_POSITION_NED (/mavros/local_position/pose).
Without GPS or a hardware DVL, setting EKF3 source set 2 (POSXY=NONE, VELXY=EXTNAV,
POSZ=BARO, YAW=COMPASS) and sending a 10 Hz zero-delta VISION_POSITION_DELTA stream
on port 14552 transitions EKF3 into AID_RELATIVE so barometer depth and IMU/compass
heading are published continuously on /mavros/local_position/pose for visual servoing.

Uses pure Python standard library (socket + struct) so no pymavlink virtualenv is required.
Self-check: `./nogps_odom_beacon.py --self-check`
"""

import argparse
import socket
import struct
import sys
import time

MSG_ID_VISION_POSITION_DELTA = 11011
CRC_EXTRA_VISION_POSITION_DELTA = 106


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_vision_position_delta(
    time_usec: int,
    time_delta_usec: int,
    angle_delta=(0.0, 0.0, 0.0),
    position_delta=(0.0, 0.0, 0.0),
    confidence: float = 50.0,
    seq: int = 0,
    sysid: int = 1,
    compid: int = 197,
) -> bytes:
    payload = struct.pack(
        "<QQ3f3ff",
        int(time_usec),
        int(time_delta_usec),
        float(angle_delta[0]),
        float(angle_delta[1]),
        float(angle_delta[2]),
        float(position_delta[0]),
        float(position_delta[1]),
        float(position_delta[2]),
        float(confidence),
    )
    # Trim trailing zero bytes per MAVLink 2 payload truncation rules only when needed;
    # confidence=50.0 (0x42480000) has non-zero high byte so full 44-byte payload is preserved.
    while len(payload) > 1 and payload[-1] == 0:
        payload = payload[:-1]

    header = struct.pack(
        "<BBBBBBB",
        0xFD,
        len(payload),
        0,
        0,
        seq & 0xFF,
        sysid & 0xFF,
        compid & 0xFF,
    ) + struct.pack("<I", MSG_ID_VISION_POSITION_DELTA)[:3]

    crc = x25_crc(header[1:] + payload + bytes([CRC_EXTRA_VISION_POSITION_DELTA]))
    return header + payload + struct.pack("<H", crc)


def parse_udpin(endpoint: str):
    addr = endpoint
    for prefix in ("udpin:", "udp:"):
        if addr.startswith(prefix):
            addr = addr[len(prefix) :]
    host, port_s = addr.rsplit(":", 1)
    return host, int(port_s)


def self_check():
    pkt = pack_vision_position_delta(
        10_000_000, 100_000, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 50.0, seq=0
    )
    expected = (
        "fd2c00000001c5032b008096980000000000a08601000000000000000000000000"
        "0000000000000000000000000000000000000048422091"
    )
    assert pkt.hex() == expected, f"got {pkt.hex()}"
    assert parse_udpin("udpin:127.0.0.1:14552") == ("127.0.0.1", 14552)
    print("nogps_odom_beacon: self-check OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mavlink", default="udpin:127.0.0.1:14552")
    ap.add_argument("--rate-hz", type=float, default=10.0)
    ap.add_argument("--confidence", type=float, default=50.0)
    ap.add_argument("--self-check", action="store_true")
    args, _ = ap.parse_known_args()

    if args.self_check:
        self_check()
        return 0

    host, port = parse_udpin(args.mavlink)
    rate_hz = max(5.0, float(args.rate_hz))
    period_s = 1.0 / rate_hz

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(1.0)
    print(f"[nogps_odom_beacon] listening for MAVLink on {host}:{port} ...", flush=True)

    peer = None
    while peer is None:
        try:
            data, addr = sock.recvfrom(2048)
            if data and data[0] in (0xFD, 0xFE):
                peer = addr
        except socket.timeout:
            continue

    print(
        f"[nogps_odom_beacon] peer {peer[0]}:{peer[1]} detected; "
        f"streaming VISION_POSITION_DELTA at {rate_hz:.1f} Hz",
        flush=True,
    )

    sock.setblocking(False)
    t0 = time.monotonic()
    prev = t0
    seq = 0
    try:
        while True:
            time.sleep(period_s)
            while True:
                try:
                    _, addr = sock.recvfrom(2048)
                    peer = addr
                except BlockingIOError:
                    break
            now = time.monotonic()
            dt = max(1e-3, now - prev)
            prev = now
            pkt = pack_vision_position_delta(
                int((now - t0 + 1.0) * 1_000_000),
                int(dt * 1_000_000),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                args.confidence,
                seq=seq,
            )
            seq = (seq + 1) & 0xFF
            sock.sendto(pkt, peer)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    if "--self-check" in sys.argv:
        self_check()
        sys.exit(0)
    sys.exit(main())
