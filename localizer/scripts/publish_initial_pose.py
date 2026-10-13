#!/usr/bin/env python3
"""ROS 2 port of FAST_LIO_LOCALIZATION's publish_initial_pose.py."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("roll", type=float)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("publish_initial_pose")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.position.z = args.z
    qx, qy, qz, qw = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    node.get_logger().info(
        "Initial Pose: %.3f %.3f %.3f %.3f %.3f %.3f"
        % (args.x, args.y, args.z, args.yaw, args.pitch, args.roll)
    )
    # Match the ROS 1 utility: allow discovery before sending the one-shot pose.
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    publisher.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
