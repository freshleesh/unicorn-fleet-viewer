#!/usr/bin/env python3
"""Fleet relay: bridge one car's private ROS domain to the shared fleet domain.

The car stack runs unchanged on its own (localhost-only) domain. This node joins
both domains and copies a few topics across, prefixed with the car name:

  private  ->  shared                       shared  ->  private
  /car_state/odom            /<car>/odom (30 Hz)       /<car>/joy          /joy
  /vesc/ackermann_cmd (mux out)                  /<car>/initialpose  /initialpose
                             /<car>/drive       /<other>/odom       /fleet/<other>/odom
                                                /<car>/max_speed    /fleet/max_speed (latched)
  /oak/rgb/image_raw/compressed
                             /<car>/image/compressed
  /vesc/sensors/core         /<car>/vesc/core
  /kiss_loc/scan_aligned     /<car>/scan     (live scan in the map frame)
  /planner/path              /<car>/path
  (from odom)                /<car>/marker   (car body + name, frame "map")

usage: fleet_relay.py --car nuc1 --private 11 --shared 1 [--color 1,0.3,0.1]
"""
import argparse
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage, Joy, PointCloud2
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from vesc_msgs.msg import VescStateStamped

FLEET = ('nuc1', 'nuc4', 'nuc5')
QOS = QoSProfile(depth=10)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
IMG_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)


def make_node(name, domain):
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(name, context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    return node, ex


class Relay:
    def __init__(self, car, private, shared, color):
        self.car = car
        self.color = color
        self.priv, ex_p = make_node(f'{car}_relay_private', private)
        self.pub, ex_s = make_node(f'{car}_relay', shared)
        self.executors = [ex_p, ex_s]
        self.odom_period = 1.0 / 30.0      # kiss publishes at IMU rate (~200 Hz); 30 Hz is plenty over WiFi
        self.last_odom = 0.0

        p = f'/{car}'
        # private -> shared (raw bytes, no re-serialization)
        self._raw(AckermannDriveStamped, '/vesc/ackermann_cmd', f'{p}/drive', QOS)
        self._raw(CompressedImage, '/oak/rgb/image_raw/compressed', f'{p}/image/compressed', IMG_QOS)
        self._raw(VescStateStamped, '/vesc/sensors/core', f'{p}/vesc/core', QOS)
        self._raw(PointCloud2, '/kiss_loc/scan_aligned', f'{p}/scan', IMG_QOS)     # ~220 KB/s, map frame
        self.odom_pub = self.pub.create_publisher(Odometry, f'{p}/odom', QOS)
        self.marker_pub = self.pub.create_publisher(MarkerArray, f'{p}/marker', QOS)
        self.priv.create_subscription(Odometry, '/car_state/odom', self._odom_cb, QOS)
        # shared -> private
        joy_pub = self.priv.create_publisher(Joy, '/joy', QOS)
        self.pub.create_subscription(Joy, f'{p}/joy', joy_pub.publish, QOS, raw=True)
        pose_pub = self.priv.create_publisher(PoseWithCovarianceStamped, '/initialpose', QOS)
        self.pub.create_subscription(PoseWithCovarianceStamped, f'{p}/initialpose', pose_pub.publish, QOS, raw=True)
        for other in FLEET:                     # the other cars' poses, for the ring planner
            if other != car:
                opub = self.priv.create_publisher(Odometry, f'/fleet/{other}/odom', QOS)
                self.pub.create_subscription(Odometry, f'/{other}/odom', opub.publish, QOS, raw=True)
        self._raw(Path, '/planner/path', f'{p}/path', IMG_QOS)
        cap_pub = self.priv.create_publisher(Float32, '/fleet/max_speed', LATCHED)   # panel speed cap
        self.pub.create_subscription(Float32, f'{p}/max_speed', cap_pub.publish, LATCHED, raw=True)

    def _raw(self, msg_type, src, dst, qos):
        out = self.pub.create_publisher(msg_type, dst, qos)
        self.priv.create_subscription(msg_type, src, out.publish, qos, raw=True)

    def _odom_cb(self, msg):
        now = time.monotonic()
        if now - self.last_odom < self.odom_period:
            return
        self.last_odom = now
        self.odom_pub.publish(msg)
        body = Marker()
        body.header.frame_id = 'map'          # stamp 0 = latest: car clocks are not synced with the viewer
        body.ns, body.id, body.type = self.car, 0, Marker.CUBE
        body.pose = msg.pose.pose
        q = body.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        body.pose.position.x += 0.17 * math.cos(yaw)    # base_link is the rear axle; centre the box
        body.pose.position.y += 0.17 * math.sin(yaw)
        body.scale.x, body.scale.y, body.scale.z = 0.55, 0.30, 0.15
        body.color.r, body.color.g, body.color.b, body.color.a = *self.color, 0.9
        label = Marker()
        label.header.frame_id = 'map'
        label.ns, label.id, label.type = self.car, 1, Marker.TEXT_VIEW_FACING
        label.pose.position.x = msg.pose.pose.position.x
        label.pose.position.y = msg.pose.pose.position.y
        label.pose.position.z = 0.5
        label.scale.z = 0.2
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = self.car
        self.marker_pub.publish(MarkerArray(markers=[body, label]))

    def spin(self):
        threads = [threading.Thread(target=ex.spin, daemon=True) for ex in self.executors]
        for t in threads:
            t.start()
        for t in threads:
            t.join()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', required=True)
    ap.add_argument('--private', type=int, required=True)
    ap.add_argument('--shared', type=int, default=1)
    ap.add_argument('--color', default='1,0.4,0.1')
    a = ap.parse_args()
    Relay(a.car, a.private, a.shared, tuple(float(c) for c in a.color.split(','))).spin()
