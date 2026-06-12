# HX5 alertfix image (required before `docker compose up`)

`docker/docker-compose.yml` pins `robotis/robotis-hand:0.3.0-hx5-alertfix`,
which is NOT on Docker Hub — build it locally once per machine:

```bash
./build_alertfix_image.sh
```

This pulls the stock multi-arch `0.3.0` image (works on x86 dev boxes and the
G1's arm64 PC2 alike), applies `dynamixel-alertfix.patch` (stale latched alert
flag on the HX5 controller is non-fatal at init), rebuilds
`dynamixel_hardware_interface`, and bakes in `python3-zmq` for `zmq_bridge.py`
(the XRT teleop bridge, see repo root).

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
