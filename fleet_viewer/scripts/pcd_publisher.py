#!/usr/bin/env python3
"""Publish a binary PCD (x y z [intensity], float32) once as a latched PointCloud2.
usage: pcd_publisher.py <file.pcd> [topic=/map_cloud] [frame=map] [z_min z_max]   (z band: keep only those heights)"""
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode().strip()
            if line.startswith('#') or not line:
                continue
            key, *vals = line.split()
            header[key] = vals
            if key == 'DATA':
                break
        if header['DATA'][0] != 'binary' or set(header['TYPE']) != {'F'} or set(header['SIZE']) != {'4'}:
            raise ValueError('only binary float32 PCD is supported')
        n = int(header['POINTS'][0])
        fields = header['FIELDS']
        data = np.frombuffer(f.read(n * 4 * len(fields)), np.float32).reshape(n, len(fields))
    return fields, data


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)     # launch appends --ros-args ...
    path = argv[1]
    topic = argv[2] if len(argv) > 2 else '/map_cloud'
    frame = argv[3] if len(argv) > 3 else 'map'
    fields, data = read_pcd(path)
    if len(argv) > 5:
        z = data[:, fields.index('z')]
        data = np.ascontiguousarray(data[(z >= float(argv[4])) & (z <= float(argv[5]))])
    rclpy.init()
    node = rclpy.create_node('pcd_publisher')
    pub = node.create_publisher(PointCloud2, topic,
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height, msg.width = 1, data.shape[0]
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(fields)]
    msg.point_step = 4 * len(fields)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = data.tobytes()
    pub.publish(msg)
    node.get_logger().info(f'{path}: {msg.width} points -> {topic} (latched)')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
