#!/bin/bash
# Build robotis/robotis-hand:0.3.0-hx5-alertfix for the LOCAL architecture.
#
# The stock 0.3.0 dynamixel_hardware_interface treats the HX5 controller's
# stale latched alert flag (0x80, no concrete error register) as fatal at
# init. This script pulls the multi-arch stock image, applies
# dynamixel-alertfix.patch, rebuilds that one package, installs python3-zmq
# (needed by zmq_bridge.py), and commits the result under the tag that
# docker-compose.yml pins. Run it once per machine (e.g. on the G1's PC2,
# which is arm64 — images cannot be copied from an x86 dev box).
#
# Usage: ./build_alertfix_image.sh [output-tag]
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
    "$BASE_IMAGE" bash -ec '
        cd /root/ros2_ws/src/dynamixel_hardware_interface
        patch -p1 --forward < /tmp/dynamixel-alertfix.patch
        # SDK fix: single-command status buffers (reboot/ping/...) are sized 11/14
        # but rxPacket can read up to RXPACKET_MAX_LEN on a misparsed length, which
        # overflows the stack buffer and SIGSEGVs on arm64 (guard page). Harmless on
        # x86, fatal on the Jetson — this is what crashed on-robot bring-up.
        cd /root/ros2_ws/src/DynamixelSDK
        patch -p1 --forward < /tmp/sdk-rxpacket-overflow.patch
        source /opt/ros/jazzy/setup.bash
        cd /root/ros2_ws
        colcon build --symlink-install \
            --packages-select dynamixel_sdk dynamixel_hardware_interface \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
        # Fail the build if the SDK overflow fix did not actually compile into the
        # library that gets loaded at runtime (catches patching the wrong copy).
        if ! strings /root/ros2_ws/install/dynamixel_sdk/lib/libdynamixel_sdk.so \
             | grep -q DXL_RXPACKET_OVERFLOW_FIX_V2; then
            echo "ERROR: SDK overflow fix marker missing from built libdynamixel_sdk.so" >&2
            exit 1
        fi
        echo "OK: SDK overflow fix present in built libdynamixel_sdk.so"
        apt-get update
        apt-get install -y --no-install-recommends python3-zmq
        rm -rf /var/lib/apt/lists/*
    '

docker commit hx5-alertfix-build "$OUT_TAG"
docker rm hx5-alertfix-build
echo "Built $OUT_TAG for $(docker image inspect "$OUT_TAG" --format '{{.Architecture}}')"
