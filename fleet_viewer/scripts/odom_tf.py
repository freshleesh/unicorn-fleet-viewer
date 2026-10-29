#!/usr/bin/env python3
"""Turn each car's /<car>/odom into a TF map -> <car>, for views that follow a car.

Stamped with this PC's clock (the car clocks are not synced with the viewer).
Run it with /tf remapped to a private topic so viewers on different PCs don't mix their TFs:
  odom_tf.py nuc1 nuc5 --ros-args -r /tf:=/driver_nuc1/tf
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:] or ['nuc1', 'nuc4', 'nuc5']
    rclpy.init()
    node = rclpy.create_node('odom_tf')
    tf = TransformBroadcaster(node)

    def callback(car):
        def cb(msg):
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = car
            p = msg.pose.pose.position
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = msg.pose.pose.orientation
            tf.sendTransform(t)
        return cb

    for car in args:
        node.create_subscription(Odometry, f'/{car}/odom', callback(car), qos_profile_sensor_data)
    node.get_logger().info(f'map -> {", ".join(args)}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
