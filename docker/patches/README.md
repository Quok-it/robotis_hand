# HX5 alertfix image (required before `docker compose up`)

`docker/docker-compose.yml` pins `robotis/robotis-hand:0.3.0-hx5-alertfix`,
which is NOT on Docker Hub — build it locally once per machine:

```bash
./build_alertfix_image.sh
```

This pulls the stock multi-arch `0.3.0` image (works on x86 dev boxes and the
G1's arm64 PC2 alike), applies two source patches, rebuilds `dynamixel_sdk` +
`dynamixel_hardware_interface`, and bakes in `python3-zmq` for `zmq_bridge.py`
(the XRT teleop bridge, see repo root).

Patches:
- `dynamixel-alertfix.patch` — the stale latched hardware-alert flag on the HX5
  hub controller (ID 110) is treated as non-fatal at init.
- `sdk-rxpacket-overflow.patch` — **required on arm64 (the G1's PC2).** The
  DynamixelSDK's `reboot()`/`ping()`/etc. size their status buffer at 11–14
  bytes, but `rxPacket` can read up to `RXPACKET_MAX_LEN` (1024) when bus noise
  during a reboot is misparsed as a long packet. That overruns the stack
  buffer: harmless on x86, but a guard-page `SIGSEGV` on arm64. Without this
  patch, on-robot bring-up crashes mid-init (`Segmentation fault ... in
  Protocol2PacketHandler::rxPacket`) and `/joint_states` never publishes. The
  fix sizes those buffers to `RXPACKET_MAX_LEN`.

## Quickstart on the G1 (PC2)

```bash
git clone -b jazzy https://github.com/Quok-it/robotis_hand.git
cd robotis_hand/docker
./patches/build_alertfix_image.sh
./container.sh start          # or: docker compose up -d
docker exec -it robotis_hand bash
# inside, terminal 1: right-hand stack (real hardware on /dev/ttyUSB0)
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py
# inside, terminal 2: XRT bridge (ZMQ :6700 cmd / :6701 feedback)
python3 /root/ros2_ws/src/robotis_hand/zmq_bridge.py
```
