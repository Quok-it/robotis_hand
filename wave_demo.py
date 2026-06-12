#!/usr/bin/env python3
"""Wave demo for the ROBOTIS HX5-D20 right hand.

Cycles the fingers fist -> open until Ctrl+C.
Run inside the container (controller must be running, see README):
    python3 /root/ros2_ws/src/robotis_hand/wave_demo.py
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# The hand_r_controller command topic (remapped in hx5_d20_right.launch.py)
TOPIC = '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory'

JOINT_NAMES = [f'finger_r_joint{i}' for i in range(1, 21)]

# Joint layout per finger group: spread, curl (knuckle), mid, tip.
# Values kept at ~60% of URDF limits; thumb (joints 1-4) stays clear of the palm.
FIST = [0.0, 0.0, 0.6, 0.6,
        0.0, 1.2, 0.9, 0.9,
        0.0, 1.2, 0.9, 0.9,
        0.0, 1.2, 0.9, 0.9,
        0.0, 1.2, 0.9, 0.9]
OPEN = [0.0] * 20

CLOSE_TIME_S = 2   # seconds to close the fist
OPEN_TIME_S = 4    # seconds (from start) to be open again
PAUSE_S = 1.0      # pause between cycles


def make_trajectory() -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    for positions, t in ((FIST, CLOSE_TIME_S), (OPEN, OPEN_TIME_S)):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=t)
        traj.points.append(point)
    return traj


def main():
    rclpy.init()
    node = Node('wave_demo')
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)

    node.get_logger().info('Waiting for the hand controller to subscribe...')
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info('Controller connected. Waving until Ctrl+C...')

    try:
        cycle = 0
        while rclpy.ok():
            cycle += 1
            node.get_logger().info(f'Cycle {cycle}: fist -> open')
            pub.publish(make_trajectory())
            # let the trajectory play out before sending the next one
            end = node.get_clock().now().nanoseconds + int((OPEN_TIME_S + PAUSE_S) * 1e9)
            while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Stopped. Hand stays in its last pose.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
