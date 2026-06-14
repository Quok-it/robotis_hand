#!/bin/bash
# Build robotis/robotis-hand:0.3.0-hx5-alertfix for the LOCAL architecture.
#
# Starts from the stock multi-arch 0.3.0 image and bakes in everything needed to
# run the hand from XRT on the G1's arm64 PC2:
#   - dynamixel-alertfix.patch    : HX5 stale latched alert flag is non-fatal
#   - sdk-rxpacket-overflow.patch : arm64 rxPacket overflow/underflow crash fix
#   - THIS fork's robotis_hand    : xacro port_name forwarding + launch port
#                                   auto-detect + zmq_bridge.py (replaces the
#                                   upstream robotis_hand cloned into the image)
#   - python3-zmq                 : needed by zmq_bridge.py
# Building on PC2 produces native arm64 binaries; a prebuilt image from an x86
# box cannot be copied over.
#
# Usage: ./build_alertfix_image.sh [output-tag]
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$DIR/../.." && pwd)"          # the robotis_hand fork checkout
BASE_IMAGE="robotis/robotis-hand:0.3.0"
OUT_TAG="${1:-robotis/robotis-hand:0.3.0-hx5-alertfix}"

# skip the pull when the base image is already present (e.g. docker load'ed
# from a tarball — save/load strips registry digests, so pull would re-download)
if ! docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; then
    docker pull "$BASE_IMAGE"
fi
docker rm -f hx5-alertfix-build 2>/dev/null || true

docker run --name hx5-alertfix-build \
    -v "$DIR/dynamixel-alertfix.patch:/tmp/dynamixel-alertfix.patch:ro" \
    -v "$DIR/sdk-rxpacket-overflow.patch:/tmp/sdk-rxpacket-overflow.patch:ro" \
    -v "$REPO_ROOT":/tmp/fork:ro \
    "$BASE_IMAGE" bash -ec '
        # Replace the upstream robotis_hand clone with this fork (xacro/launch/
        # bridge fixes) so the INSTALLED packages carry them — runtime uses
        # install/share, not src, so the fork must be built into the image.
        rm -rf /root/ros2_ws/src/robotis_hand
        mkdir -p /root/ros2_ws/src/robotis_hand
        cp -a /tmp/fork/. /root/ros2_ws/src/robotis_hand/
        rm -rf /root/ros2_ws/src/robotis_hand/.git

        cd /root/ros2_ws/src/dynamixel_hardware_interface
        patch -p1 --forward < /tmp/dynamixel-alertfix.patch
        # SDK fix: single-command status buffers (reboot/ping/...) overflow on
        # arm64 when rxPacket misparses a length, and read() is called with a
        # negative (huge) length on bus noise. Both crash the Jetson; harmless on
        # x86. This is what crashed on-robot bring-up.
        cd /root/ros2_ws/src/DynamixelSDK
        patch -p1 --forward < /tmp/sdk-rxpacket-overflow.patch

        source /opt/ros/jazzy/setup.bash
        cd /root/ros2_ws
        colcon build --symlink-install \
            --packages-select dynamixel_sdk dynamixel_hardware_interface \
                             robotis_hand_description robotis_hand_bringup \
            --cmake-args -DCMAKE_BUILD_TYPE=Release

        # Fail the build if the SDK overflow fix did not compile into the loaded
        # library (catches patching the wrong source copy).
        if ! strings /root/ros2_ws/install/dynamixel_sdk/lib/libdynamixel_sdk.so \
             | grep -q DXL_RXPACKET_OVERFLOW_FIX_V2; then
            echo "ERROR: SDK overflow fix marker missing from built libdynamixel_sdk.so" >&2
            exit 1
        fi
        # Fail the build if the installed xacro does not forward port_name (the
        # bug that pinned the hardware to /dev/ttyUSB0 regardless of the arg).
        if ! grep -Rq "arg port_name" \
             /root/ros2_ws/install/robotis_hand_description/share; then
            echo "ERROR: installed xacro does not forward port_name" >&2
            exit 1
        fi
        echo "OK: SDK overflow fix + xacro port_name forwarding present"

        apt-get update
        apt-get install -y --no-install-recommends python3-zmq
        rm -rf /var/lib/apt/lists/*
    '

docker commit hx5-alertfix-build "$OUT_TAG"
docker rm hx5-alertfix-build
echo "Built $OUT_TAG for $(docker image inspect "$OUT_TAG" --format '{{.Architecture}}')"
