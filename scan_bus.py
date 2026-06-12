#!/usr/bin/env python3
"""Raw Dynamixel Protocol 2.0 bus scanner — no ROS, no dynamixel SDK.

Sends a PING to every ID at each baud rate and reports who answers,
including the error byte (bit 0x80 = hardware-alert flag). Use this to
debug "no status packet" failures without the ros2_control driver in the
way. The launch file must NOT be running (it owns the serial port).

Inside the robotis_hand container:
    apt-get update && apt-get install -y python3-serial   # once
    python3 /root/ros2_ws/src/robotis_hand/scan_bus.py
    python3 /root/ros2_ws/src/robotis_hand/scan_bus.py --bauds 4000000 --ids 100-130
"""
import argparse
import time

try:
    import serial
except ImportError:
    raise SystemExit("pyserial missing — run: apt-get update && apt-get install -y python3-serial")


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ping_packet(dxl_id: int) -> bytes:
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x03, 0x00, 0x01])
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def parse_status(buf: bytes):
    """Find a status packet in buf; return (id, err, model) or None."""
    i = buf.find(bytes([0xFF, 0xFF, 0xFD, 0x00]))
    if i < 0 or len(buf) < i + 9:
        return None
    dxl_id = buf[i + 4]
    length = buf[i + 5] | (buf[i + 6] << 8)
    if buf[i + 7] != 0x55:          # not a status instruction
        return None
    err = buf[i + 8]
    model = None
    if length >= 7 and len(buf) >= i + 11:
        model = buf[i + 9] | (buf[i + 10] << 8)
    return dxl_id, err, model


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--bauds", default="4000000,57600,1000000,2000000,3000000")
    ap.add_argument("--ids", default="0-252", help="e.g. 100-130 or 0-252")
    ap.add_argument("--timeout", type=float, default=0.02, help="per-ping reply wait (s)")
    args = ap.parse_args()

    lo, hi = (int(x) for x in args.ids.split("-"))
    ids = range(lo, hi + 1)

    for baud in (int(b) for b in args.bauds.split(",")):
        print(f"\n=== scanning {args.port} @ {baud} baud, IDs {lo}-{hi} ===")
        try:
            port = serial.Serial(args.port, baudrate=baud, timeout=args.timeout)
        except Exception as e:
            print(f"  cannot open port at {baud}: {e}")
            continue
        found = 0
        for dxl_id in ids:
            port.reset_input_buffer()
            port.write(ping_packet(dxl_id))
            port.flush()
            time.sleep(args.timeout)
            buf = port.read(port.in_waiting or 16)
            if not buf:
                continue
            parsed = parse_status(buf)
            if parsed:
                rid, err, model = parsed
                alert = " [ALERT flag set]" if err & 0x80 else ""
                concrete = f" err=0x{err:02X}" if err & 0x7F else ""
                print(f"  ID {rid:3d} responds  model={model}{concrete}{alert}")
                found += 1
            else:
                print(f"  ID {dxl_id:3d}: got {len(buf)} bytes but unparseable: {buf.hex()}")
        port.close()
        print(f"  -> {found} device(s) responded at {baud}")
        if found:
            break   # right baud found; no need to try others


if __name__ == "__main__":
    main()
