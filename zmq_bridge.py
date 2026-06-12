#!/usr/bin/env python3
"""ZMQ <-> ROS 2 bridge for XRT waldo teleop of ROBOTIS HX5-D20 hands.

Runs INSIDE the robotis_hand container, next to the ros2_control stack
(hx5_d20_{left,right}.launch.py must be running). Speaks the wire contract
from unitree_xr_teleoperate's waldo_rt_robotis.py:

  ZMQ SUB bind :6700  cmd in   40 float32 radians [left 20 | right 20]
  ZMQ PUB bind :6701  state out 172 float32
      [l_present, r_present | l_q20 l_dq20 l_press45 | r_q20 r_dq20 r_press45]

Presence is detected from /joint_states (finger_{l,r}_joint1) and latched.
Each cmd frame becomes a one-point JointTrajectory with a short
time_from_start so the 100 Hz JTC interpolates instead of stepping.

Usage (inside the container, after the launch file):
    apt-get install python3-zmq   # once (pip not available in container)
    python3 /root/ros2_ws/src/robotis_hand/zmq_bridge.py
"""
import argparse
import threading

import numpy as np
import zmq

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_interfaces.msg import HandPressures

NUM_JOINTS = 20
NUM_PRESSURE = 45                       # 5 sensors x 9 taxels
SIDE_LEN = 2 * NUM_JOINTS + NUM_PRESSURE
FB_FRAME_LEN = 2 + 2 * SIDE_LEN         # 172

SIDES = ('left', 'right')
CMD_TOPIC = '/leader/joint_trajectory_command_broadcaster_{side}_hand/joint_trajectory'
PRESSURE_TOPIC = '/{side}_hand/finger_pressures'


class XrtZmqBridge(Node):

    def __init__(self, cmd_port, fb_port, rate, traj_time):
        super().__init__('xrt_zmq_bridge')
        self.traj_time = traj_time
        self.joint_names = {
            s: [f'finger_{s[0]}_joint{i}' for i in range(1, 21)] for s in SIDES
        }
        self.lock = threading.Lock()
        self.present = {s: False for s in SIDES}
        self.q = {s: np.zeros(NUM_JOINTS, dtype=np.float32) for s in SIDES}
        self.dq = {s: np.zeros(NUM_JOINTS, dtype=np.float32) for s in SIDES}
        self.press = {s: np.zeros(NUM_PRESSURE, dtype=np.float32) for s in SIDES}

        self.traj_pubs = {
            s: self.create_publisher(JointTrajectory, CMD_TOPIC.format(side=s), 10)
            for s in SIDES
        }
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint_states, 10)
        for s in SIDES:
            self.create_subscription(
                HandPressures, PRESSURE_TOPIC.format(side=s),
                lambda msg, side=s: self.on_pressures(side, msg), 10)

        self.ctx = zmq.Context()
        self.fb_pub = self.ctx.socket(zmq.PUB)
        self.fb_pub.bind(f'tcp://*:{fb_port}')
        self.cmd_port = cmd_port
        self._cmd_thread = threading.Thread(target=self.cmd_loop, daemon=True)
        self._cmd_thread.start()
        self.create_timer(1.0 / rate, self.publish_feedback)
        self.get_logger().info(
            f'bridge up: cmd SUB :{cmd_port}, feedback PUB :{fb_port}')

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        with self.lock:
            for s in SIDES:
                names = self.joint_names[s]
                if not self.present[s]:
                    if names[0] in pos:
                        self.present[s] = True
                        self.get_logger().info(f'{s} hand detected in /joint_states')
                    else:
                        continue
                for i, n in enumerate(names):
                    if n in pos:
                        self.q[s][i] = pos[n]
                        self.dq[s][i] = vel.get(n, 0.0)

    def on_pressures(self, side, msg):
        flat = np.array(
            [v for sensor in msg.sensors for v in sensor.pressure_values],
            dtype=np.float32)
        if flat.shape[0] == NUM_PRESSURE:
            with self.lock:
                self.press[side][:] = flat

    def cmd_loop(self):
        sub = self.ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.SUBSCRIBE, b'')
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.bind(f'tcp://*:{self.cmd_port}')
        while rclpy.ok():
            if not sub.poll(timeout=100):
                continue
            arr = np.frombuffer(sub.recv(), dtype=np.float32)
            if arr.shape[0] != 2 * NUM_JOINTS:
                continue
            halves = {'left': arr[:NUM_JOINTS], 'right': arr[NUM_JOINTS:]}
            for s in SIDES:
                with self.lock:
                    if not self.present[s]:
                        continue
                traj = JointTrajectory()
                traj.joint_names = self.joint_names[s]
                pt = JointTrajectoryPoint()
                pt.positions = [float(v) for v in halves[s]]
                pt.time_from_start = Duration(
                    sec=0, nanosec=int(self.traj_time * 1e9))
                traj.points = [pt]
                self.traj_pubs[s].publish(traj)

    def publish_feedback(self):
        frame = np.zeros(FB_FRAME_LEN, dtype=np.float32)
        with self.lock:
            frame[0] = 1.0 if self.present['left'] else 0.0
            frame[1] = 1.0 if self.present['right'] else 0.0
            for k, s in enumerate(SIDES):
                base = 2 + k * SIDE_LEN
                frame[base:base + NUM_JOINTS] = self.q[s]
                frame[base + NUM_JOINTS:base + 2 * NUM_JOINTS] = self.dq[s]
                frame[base + 2 * NUM_JOINTS:base + SIDE_LEN] = self.press[s]
        self.fb_pub.send(frame.tobytes())


def main():
    parser = argparse.ArgumentParser(description='XRT ZMQ bridge for HX5-D20')
    parser.add_argument('--cmd-port', type=int, default=6700)
    parser.add_argument('--fb-port', type=int, default=6701)
    parser.add_argument('--rate', type=float, default=100.0)
    parser.add_argument('--traj-time', type=float, default=0.03,
                        help='time_from_start for each one-point trajectory (s)')
    args = parser.parse_args()

    rclpy.init()
    node = XrtZmqBridge(args.cmd_port, args.fb_port, args.rate, args.traj_time)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
