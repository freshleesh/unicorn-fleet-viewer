#!/usr/bin/env python3
"""Mapping view relay: /livox/lidar on the car's private domain -> /<car>/livox/lidar on the shared domain.

Best effort, depth 1, every Nth frame (default 2 -> ~5 Hz) so Wi-Fi isn't flooded; the bag records the full
rate locally on the car.  usage: lidar_relay.py --car nuc4 --private 14 --shared 1 [--every 2]
"""
import argparse
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

IN_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
OUT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def make_node(name, domain):
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(name, context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    return node, ex


ap = argparse.ArgumentParser()
ap.add_argument('--car', required=True)
ap.add_argument('--private', type=int, required=True)
ap.add_argument('--shared', type=int, default=1)
ap.add_argument('--every', type=int, default=2)
a = ap.parse_args()

priv, ex_p = make_node(f'{a.car}_lidar_relay_private', a.private)
pub, ex_s = make_node(f'{a.car}_lidar_relay', a.shared)
out = pub.create_publisher(PointCloud2, f'/{a.car}/livox/lidar', OUT_QOS)
n = [0]


def cb(raw):
    n[0] += 1
    if n[0] % a.every == 0:
        out.publish(raw)


priv.create_subscription(PointCloud2, '/livox/lidar', cb, IN_QOS, raw=True)
threads = [threading.Thread(target=ex.spin, daemon=True) for ex in (ex_p, ex_s)]
for t in threads:
    t.start()
for t in threads:
    t.join()
