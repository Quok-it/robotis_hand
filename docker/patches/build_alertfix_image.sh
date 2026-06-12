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

docker pull "$BASE_IMAGE"
docker rm -f hx5-alertfix-build 2>/dev/null || true

docker run --name hx5-alertfix-build \
    -v "$DIR/dynamixel-alertfix.patch:/tmp/dynamixel-alertfix.patch:ro" \
    "$BASE_IMAGE" bash -ec '
        cd /root/ros2_ws/src/dynamixel_hardware_interface
        patch -p1 --forward < /tmp/dynamixel-alertfix.patch
        source /opt/ros/jazzy/setup.bash
        cd /root/ros2_ws
        colcon build --symlink-install --packages-select dynamixel_hardware_interface \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
        apt-get update
        apt-get install -y --no-install-recommends python3-zmq
        rm -rf /var/lib/apt/lists/*
    '

docker commit hx5-alertfix-build "$OUT_TAG"
docker rm hx5-alertfix-build
echo "Built $OUT_TAG for $(docker image inspect "$OUT_TAG" --format '{{.Architecture}}')"
