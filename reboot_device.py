#!/usr/bin/env python3
"""Raw Dynamixel Protocol 2.0 REBOOT — clears a latched hardware-alert flag.

The HX5 palm controller (ID 110) latches a hardware-alert flag (error bit 0x80)
that survives power cycles. On launch, dynamixel_hardware_interface's on_init
tries to Reboot it to clear the alert, but that version of the SDK segfaults in
rxPacket when a reboot isn't cleanly acked. Rebooting the device here first —
with correct post-reboot timing — clears the alert so init has nothing to do.

The ros2_control launch must NOT be running (it owns the serial port).

Inside the robotis_hand container:
    apt-get install -y python3-serial   # once, if not already
    python3 /root/ros2_ws/src/robotis_hand/reboot_device.py            # reboots 110
    python3 /root/ros2_ws/src/robotis_hand/reboot_device.py --ids 110-130
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


def _packet(dxl_id: int, instr: int) -> bytes:
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x03, 0x00, instr])
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def reboot_packet(dxl_id: int) -> bytes:
    return _packet(dxl_id, 0x08)   # Protocol 2.0 REBOOT instruction


def ping_packet(dxl_id: int) -> bytes:
    return _packet(dxl_id, 0x01)


def find_status(buf: bytes):
    """Return (id, err) of the first status packet in buf, or None."""
    i = buf.find(bytes([0xFF, 0xFF, 0xFD, 0x00]))
    if i < 0 or len(buf) < i + 9 or buf[i + 7] != 0x55:
        return None
    return buf[i + 4], buf[i + 8]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=4000000)
    ap.add_argument("--ids", default="110", help="single ID or range, e.g. 110 or 110-130")
    ap.add_argument("--settle", type=float, default=0.3, help="post-reboot wait (s)")
    args = ap.parse_args()

    if "-" in args.ids:
        lo, hi = (int(x) for x in args.ids.split("-"))
        ids = range(lo, hi + 1)
    else:
        ids = [int(args.ids)]

    port = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    for dxl_id in ids:
        port.reset_input_buffer()
        port.write(reboot_packet(dxl_id))
        port.flush()
        time.sleep(args.settle)               # let the controller reboot
        port.read(port.in_waiting or 16)      # drain any reboot status

        # confirm it came back and report the (hopefully cleared) error byte
        port.reset_input_buffer()
        port.write(ping_packet(dxl_id))
        port.flush()
        time.sleep(0.05)
        st = find_status(port.read(port.in_waiting or 16))
        if st is None:
            print(f"  ID {dxl_id:3d}: rebooted, NO ping reply (still settling?)")
        else:
            rid, err = st
            flag = " [ALERT STILL SET]" if err & 0x80 else " alert clear"
            print(f"  ID {rid:3d}: back up, err=0x{err:02X}{flag}")
    port.close()


if __name__ == "__main__":
    main()
