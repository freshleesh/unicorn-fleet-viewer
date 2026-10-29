#!/usr/bin/env python3
"""Ring planner: drive round a closed track forever, avoiding the other cars.

Every replan (5 Hz) runs A* from the car to the track centreline point one third of a
lap ahead. The track is one-way: a move may not point against the local track direction,
so the search can never pick "turn round and go the other way" when the road ahead is
busy. Cell cost = 1 + wall term + opponent term + "stay near the previous path" term.
Pure pursuit follows the latest path at 50 Hz. A speed planner runs on every new path: the speed
cap from the autodrive panel is the top speed, each point is limited by its curvature
(lateral acceleration a_lat), then accelerating out of corners is limited to a_acc and braking
into them (or to a stop behind a car that blocks the whole road) to a_dec.
Each replan keeps the start of the previous path (the next ~0.4 s of driving) and runs A* from
its end, so near-equal alternatives cannot flip right in front of the car. Pure pursuit steers
from the pose predicted `steer_delay` s ahead (servo/motor lag), otherwise it oscillates at speed.
No path -> keep the last one for 0.5 s, then speed 0.

Runs on the car (private domain):
  in : /car_state/odom, /fleet/<other>/odom, /fleet/max_speed (from fleet_relay), map yaml (2D track grid)
  out: /vesc/high_level/ackermann_cmd (simple_mux autodrive input; press RB to hand over),
       /planner/path (nav_msgs/Path, relayed to /<car>/path)
"""
import heapq
import math
import os
import sys
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

# ---------------------------------------------------------------- planning core (no ROS)


class Grid:
    """Planning grid at `res` m from the map's signed distance field (m, + = outside track)."""

    def __init__(self, field, res=0.10):
        step = max(1, int(round(res / field.res)))
        self.res = field.res * step
        # sample cell centres of the coarse grid; row 0 = max y (image convention)
        self.sdf = field.sdf[step // 2::step, step // 2::step].astype(np.float64)
        self.h, self.w = self.sdf.shape
        self.ox, self.oy = field.ox, field.oy
        self.top = field.oy + field.h * field.res        # y of the image's top edge
        rows, cols = np.mgrid[0:self.h, 0:self.w]
        self.cx = self.ox + (cols + 0.5) * self.res      # cell-centre world coords
        self.cy = self.top - (rows + 0.5) * self.res
        self.free = self.sdf <= 0.0
        loop = centreline_loop(field.sdf <= 0.0)                     # ordered (row, col) on the fine grid
        self.centre = np.stack([field.ox + (loop[:, 1] + 0.5) * field.res,
                                self.top - (loop[:, 0] + 0.5) * field.res], 1)
        d = np.roll(self.centre, -3, 0) - np.roll(self.centre, 3, 0)   # tangent, loop order
        self.tangent = d / np.linalg.norm(d, axis=1, keepdims=True)
        self.tree = cKDTree(self.centre)
        _, idx = self.tree.query(np.stack([self.cx.ravel(), self.cy.ravel()], 1))
        self.cell_tangent = self.tangent[idx].reshape(self.h, self.w, 2)  # track direction at each cell

    def cell(self, x, y):
        return int(math.floor((self.top - y) / self.res)), int(math.floor((x - self.ox) / self.res))

    def inside(self, rc):
        return 0 <= rc[0] < self.h and 0 <= rc[1] < self.w

    def world(self, rc):
        return self.cx[rc], self.cy[rc]


class Planner:
    def __init__(self, grid, r_hard=0.10, w_wall=3.0, k_wall=0.25, hard_cost=50.0,
                 opp_half_len=0.43, opp_half_wid=0.30, opp_hard=200.0, opp_soft=5.0, k_opp=0.6,
                 w_prev=0.3, d_prev=0.3, horizon=1 / 3, back_tol=0.3, start_out=0.2):
        self.g = grid
        self.p = dict(opp_half_len=opp_half_len, opp_half_wid=opp_half_wid, opp_hard=opp_hard,
                      opp_soft=opp_soft, k_opp=k_opp, w_prev=w_prev, d_prev=d_prev,
                      horizon=horizon, back_tol=back_tol, start_out=start_out)
        self.dir = None                    # +1 / -1: driving along / against the centreline order
        d_in = -grid.sdf
        wall = np.where(d_in < r_hard, hard_cost, w_wall * np.exp(-(d_in - r_hard) / k_wall))
        self.base = np.where(grid.free, wall, np.inf)
        self.prev_mask = None

    # --- cost terms
    def opponent_cost(self, opponents):
        """opponents: list of (x, y, yaw, v). Oriented ellipse around the body, stretched ahead by v*0.3 s."""
        c = np.zeros_like(self.base)
        a0, b = self.p['opp_half_len'], self.p['opp_half_wid']
        for x, y, yaw, v in opponents:
            ahead = max(0.0, v) * 0.3
            a = a0 + ahead / 2
            cx = x + (0.17 + ahead / 2) * math.cos(yaw)   # base_link = rear axle; body centre 0.17 ahead
            cy = y + (0.17 + ahead / 2) * math.sin(yaw)
            dx, dy = self.g.cx - cx, self.g.cy - cy
            u = (dx * math.cos(yaw) + dy * math.sin(yaw)) / a
            w = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / b
            e = np.sqrt(u * u + w * w)
            c += np.where(e < 1.0, self.p['opp_hard'], self.p['opp_soft'] * np.exp(-(e - 1.0) / self.p['k_opp']))
        return c

    def prev_cost(self):
        if self.prev_mask is None:
            return 0.0
        d = distance_transform_edt(~self.prev_mask) * self.g.res
        return self.p['w_prev'] * np.minimum(d / self.p['d_prev'], 1.0)

    def direction(self, i, yaw):
        """+1 if the car faces along the centreline order at point i, else -1.
        Sticky while the car is roughly sideways, so it cannot flip back and forth."""
        dot = self.g.tangent[i] @ (math.cos(yaw), math.sin(yaw))
        if self.dir is None or abs(dot) > 0.5:
            self.dir = 1 if dot >= 0.0 else -1
        return self.dir

    # --- search
    def plan(self, x, y, yaw, opponents=()):
        """World path [(x, y), ...] from the car to `horizon` of a lap ahead, or None (car off the track)."""
        g = self.g
        start = g.cell(x, y)
        if not g.inside(start) or g.sdf[start] > self.p['start_out']:
            return None                    # off the track: wrong localization or really out -> give up
        cost = self.base + self.opponent_cost(opponents) + self.prev_cost()
        cost[start] = min(cost[start], self.base.max()) if np.isinf(cost[start]) else cost[start]
        n = len(g.centre)
        i0 = int(g.tree.query((x, y))[1])
        sgn = self.direction(i0, yaw)
        goal = g.cell(*g.centre[(i0 + sgn * int(n * self.p['horizon'])) % n])
        tan = (sgn * g.cell_tangent).tolist()
        back = -self.p['back_tol']

        cost_l = cost.tolist()
        h, w = g.h, g.w
        gr, gc = goal
        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        best = {start: 0.0}
        parent = {}
        openq = [(0.0, 0.0, start)]
        while openq:
            _, d, cur = heapq.heappop(openq)
            if cur == goal:
                break
            if d > best.get(cur, math.inf):
                continue
            r, c = cur
            for dr, dc, step in nbrs:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                cc = cost_l[nr][nc]
                if cc == math.inf:
                    continue
                tx, ty = tan[r][c]         # one-way: no move against the track direction
                if (dc * tx - dr * ty) < back * step:     # col = +x, row = -y
                    continue
                if dr and dc and (cost_l[r][nc] == math.inf or cost_l[nr][c] == math.inf):
                    continue                       # no corner cutting through walls
                nd = d + step * (1.0 + cc)
                if nd < best.get((nr, nc), math.inf):
                    best[(nr, nc)] = nd
                    parent[(nr, nc)] = cur
                    hh = math.hypot(nr - gr, nc - gc)
                    heapq.heappush(openq, (nd + hh, nd, (nr, nc)))
        if goal not in parent:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        cells.reverse()
        mask = np.zeros((h, w), bool)
        for rc in cells:
            mask[rc] = True
        self.prev_mask = mask
        pts = np.array([g.world(rc) for rc in cells])
        pts[0] = (x, y)
        return smooth(pts)

    def replan(self, x, y, yaw, v, opponents=(), prev=None, keep_time=0.4, keep_min=0.5, max_off=0.3):
        """plan() with the start of the previous path kept: the part the car drives in the next
        `keep_time` s (at least `keep_min` m) is reused as is and A* starts at its end. Near-equal
        alternatives (hug the inside / swing wide) then cannot flip right in front of the car."""
        if prev is not None and len(prev) > 4:
            near = nearest(prev, x, y)
            if math.hypot(prev[near, 0] - x, prev[near, 1] - y) < max_off:
                j = near + int(max(keep_min, v * keep_time) / 0.05)
                if j < len(prev) - 2:
                    sx, sy = prev[j]
                    syaw = math.atan2(prev[j + 1, 1] - prev[j - 1, 1], prev[j + 1, 0] - prev[j - 1, 0])
                    tail = self.plan(sx, sy, syaw, opponents)
                    if tail is not None:
                        return np.vstack([prev[near:j], tail])
        return self.plan(x, y, yaw, opponents)

    def blocked_ahead(self, path, opponents):
        """Arc length along `path` to the first point inside an opponent's hard footprint (or None)."""
        if not opponents or path is None:
            return None
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
        a, b = self.p['opp_half_len'], self.p['opp_half_wid']
        hit = np.zeros(len(path), bool)
        for x, y, yaw, v in opponents:
            ahead = max(0.0, v) * 0.3
            aa = a + ahead / 2
            cx = x + (0.17 + ahead / 2) * math.cos(yaw)
            cy = y + (0.17 + ahead / 2) * math.sin(yaw)
            dx, dy = path[:, 0] - cx, path[:, 1] - cy
            u = (dx * math.cos(yaw) + dy * math.sin(yaw)) / aa
            w = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / b
            hit |= u * u + w * w < 1.0
        idx = np.flatnonzero(hit)
        return float(s[idx[0]]) if len(idx) else None


def centreline_loop(free):
    """Ordered (row, col) pixels of the ring's centreline: skeleton, spurs pruned, walked once round."""
    sk = skeletonize(free)
    offs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def nbrs(r, c):
        return [(r + dr, c + dc) for dr, dc in offs
                if 0 <= r + dr < sk.shape[0] and 0 <= c + dc < sk.shape[1] and sk[r + dr, c + dc]]

    pruned = True
    while pruned:                          # strip dead-end branches until only the loop is left
        pruned = False
        for r, c in zip(*np.nonzero(sk)):
            if len(nbrs(r, c)) <= 1:
                sk[r, c] = False
                pruned = True
    pts = list(zip(*np.nonzero(sk)))
    if not pts:
        raise ValueError('track map has no closed loop')
    order, seen = [pts[0]], {pts[0]}
    while True:
        r, c = order[-1]
        nxt = [p for p in nbrs(r, c) if p not in seen]
        if not nxt:
            break
        nxt.sort(key=lambda p: abs(p[0] - r) + abs(p[1] - c))   # 4-neighbours first: follow the line
        order.append(nxt[0])
        seen.add(nxt[0])
    if len(order) < 0.9 * len(pts):
        raise ValueError(f'centreline walk covered {len(order)}/{len(pts)} pixels: not a simple ring')
    return np.array(order)


def smooth(pts, window=5, spacing=0.05):
    """Moving-average the grid staircase, then resample at `spacing` m."""
    if len(pts) > window:
        k = np.ones(window) / window
        pad = window // 2
        xs = np.convolve(np.pad(pts[:, 0], pad, mode='edge'), k, 'valid')
        ys = np.convolve(np.pad(pts[:, 1], pad, mode='edge'), k, 'valid')
        pts = np.stack([xs, ys], 1)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    t = np.arange(0.0, s[-1], spacing)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], 1)


def curvature(path, span=0.8, spacing=0.05):
    """Signed curvature [1/m] at every path point from the circle through the points `span` m
    before and after it (a wide stencil, so the smoothed grid staircase does not read as corners)."""
    k = max(1, int(round(span / spacing)))
    i = np.arange(len(path))
    a, b, c = path[np.clip(i - k, 0, len(path) - 1)], path, path[np.clip(i + k, 0, len(path) - 1)]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    den = (np.hypot(*(b - a).T) * np.hypot(*(c - b).T) * np.hypot(*(c - a).T))
    return np.where(den > 1e-9, 2.0 * cross / np.maximum(den, 1e-9), 0.0)


def speed_profile(path, cap, a_lat, a_acc, a_dec, v_floor=0.5, stop_at=None, spacing=0.05,
                  span=0.8, smooth_len=0.5):
    """Speed [m/s] at every path point: lateral-acceleration limit from the curvature, capped at `cap`,
    then a forward pass (accelerate at most a_acc out of corners) and a backward pass (brake at most
    a_dec into them, and to 0 at arc length `stop_at` when the road ahead is blocked)."""
    kappa = np.abs(curvature(path, span, spacing))
    if smooth_len > 0 and len(kappa) > 2:              # average out what is left of the grid staircase
        k = max(1, int(round(smooth_len / spacing)))
        kappa = np.convolve(np.pad(kappa, k // 2, mode='edge'), np.ones(k) / k, 'valid')[:len(path)]
    v = np.minimum(cap, np.sqrt(a_lat / np.maximum(kappa, 1e-6)))
    v = np.maximum(v, min(v_floor, cap))           # the stencil never sees a real 0-radius corner
    if stop_at is not None:
        v[int(max(0.0, stop_at) / spacing):] = 0.0
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * spacing))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * spacing))
    return v


def predict(x, y, yaw, v, yaw_rate, dt, steps=5):
    """Pose `dt` s ahead at constant speed v and yaw rate: where the car will be when a command
    sent now takes effect (actuation lag). Use the measured yaw rate (gyro): predicting with the
    last steering command feeds every command back into the next one and the steering chatters."""
    h = dt / steps
    for _ in range(steps):
        x += v * math.cos(yaw) * h
        y += v * math.sin(yaw) * h
        yaw += yaw_rate * h
    return x, y, yaw


def nearest(path, x, y):
    """Index of the path point nearest to the car, searched in the first half (the path ends far ahead)."""
    d = np.hypot(path[:, 0] - x, path[:, 1] - y)
    return int(np.argmin(d[: max(1, len(path) // 2)]))


def pure_pursuit(path, x, y, yaw, lookahead, wheelbase, near=None):
    """Steering angle toward the point `lookahead` m ahead along the path from its nearest point."""
    near = nearest(path, x, y) if near is None else near
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path[near:], axis=0).T))])
    j = near + int(min(np.searchsorted(s, lookahead), len(s) - 1))
    tx, ty = path[j]
    alpha = math.atan2(ty - y, tx - x) - yaw
    ld = max(math.hypot(tx - x, ty - y), 1e-3)
    return math.atan2(2.0 * wheelbase * math.sin(alpha), ld)


# ---------------------------------------------------------------- ROS node


def main():
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32
    from controller.geofence import TrackField, resolve_map_yaml

    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    class RingPlanner(Node):
        def __init__(self):
            super().__init__('ring_planner')
            P = lambda n, d: self.declare_parameter(n, d).value
            self.speed = float(P('speed', 2.0))                 # until the panel's cap arrives on /fleet/max_speed
            self.max_steer = float(P('max_steer', 0.4))
            self.wheelbase = float(P('wheelbase', 0.33))
            self.lookahead_gain = float(P('lookahead_gain', 0.35))  # lookahead = gain*v + 0.3, clamped 0.6..2.5 m
            self.steer_delay = float(P('steer_delay', 0.2))      # s: pure pursuit steers from where the car will be then
            self.keep_path = float(P('keep_path', 0.5))           # s to keep the last path when a replan fails
            self.pose_timeout = float(P('pose_timeout', 0.3))
            self.opp_timeout = float(P('opp_timeout', 0.5))
            self.follow_gap = float(P('follow_gap', 0.5))      # stop this far before a blocking car [m]
            # speed planner: the panel cap is the top speed, corners and blocking cars slow the car down
            self.a_lat = float(P('a_lat', 3.3))              # max lateral acceleration in corners [m/s^2] (car 1 grips ~3.6 on the expo floor)
            self.a_acc = float(P('a_acc', 3.0))              # max acceleration out of corners [m/s^2]
            self.a_dec = float(P('a_dec', 4.0))              # max braking into corners / behind a car [m/s^2]
            self.preview = float(P('speed_preview', 0.25))   # s: command the speed this far ahead (actuation lag)
            others = P('opponents', ['nuc1', 'nuc4', 'nuc5'])
            me = os.uname().nodename
            path = resolve_map_yaml(P('map', 'nuc4_map_1029_1905'))
            self.planner = Planner(Grid(TrackField(path), float(P('plan_res', 0.10))))
            self.get_logger().info(f'map {path}, grid {self.planner.g.w}x{self.planner.g.h} @ {self.planner.g.res} m, '
                                   f'speed {self.speed} m/s')

            self.pose = None                 # (x, y, yaw), monotonic time
            self.v_meas = 0.0                # measured speed from /car_state/odom [m/s]
            self.yaw_rate = (0.0, 0.0)       # gyro z [rad/s] (lidar IMU, mounted level with base_link), monotonic time
            self.opp = {}                    # name -> ((x, y, yaw, v), monotonic time)
            self.path = None
            self.profile = None              # speed at every path point (speed_profile)
            self.path_time = 0.0
            self.block = None
            self.last_steer = 0.0
            self.last_speed = 0.0
            self.create_subscription(Odometry, '/car_state/odom', self._odom_cb, 10)
            self.create_subscription(Imu, P('imu_topic', '/livox/imu'), self._imu_cb, qos_profile_sensor_data)
            for o in others:
                if o != me:
                    self.create_subscription(Odometry, f'/fleet/{o}/odom', lambda m, o=o: self._opp_cb(o, m), 10)
            self.create_subscription(Float32, '/fleet/max_speed', self._speed_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/ackermann_cmd', 10)
            self.path_pub = self.create_publisher(Path, '/planner/path', 1)
            self.create_timer(0.2, self._replan)
            self.create_timer(0.02, self._control)

        def _speed_cb(self, m):
            self.speed = float(m.data)
            self.get_logger().info(f'speed cap {self.speed:.1f} m/s')

        def _odom_cb(self, m):
            p = m.pose.pose
            self.pose = ((p.position.x, p.position.y, yaw_of(p.orientation)), time.monotonic())
            self.v_meas = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

        def _imu_cb(self, m):
            w, _ = self.yaw_rate
            self.yaw_rate = (0.7 * w + 0.3 * m.angular_velocity.z, time.monotonic())   # ~15 ms low-pass at 200 Hz

        def _opp_cb(self, name, m):
            p = m.pose.pose
            v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
            self.opp[name] = ((p.position.x, p.position.y, yaw_of(p.orientation), v), time.monotonic())

        def _fresh(self):
            now = time.monotonic()
            pose = self.pose[0] if self.pose and now - self.pose[1] < self.pose_timeout else None
            opps = [s for s, t in self.opp.values() if now - t < self.opp_timeout]
            return pose, opps

        def _replan(self):
            pose, opps = self._fresh()
            if pose is None:
                self.path = None
                return
            t0 = time.monotonic()
            path = self.planner.replan(*pose, self.v_meas, opps, prev=self.path)
            dt = time.monotonic() - t0
            if path is None:
                self.get_logger().warn('no path (car off the track?)', throttle_duration_sec=2.0)
                if t0 - self.path_time > self.keep_path:
                    self.path = None           # -> speed 0
                return
            self.block = self.planner.blocked_ahead(path, opps)
            stop_at = None if self.block is None else self.block - self.follow_gap
            self.profile = speed_profile(path, self.speed, self.a_lat, self.a_acc, self.a_dec, stop_at=stop_at)
            self.path, self.path_time = path, t0
            if dt > 0.15:
                self.get_logger().warn(f'slow replan {dt*1000:.0f} ms', throttle_duration_sec=5.0)
            if path is not None:
                msg = Path()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                for x, y in path[::2]:
                    ps = PoseStamped()
                    ps.header = msg.header
                    ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                    ps.pose.orientation.w = 1.0
                    msg.poses.append(ps)
                self.path_pub.publish(msg)

        def _control(self):
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            pose, _ = self._fresh()
            if pose is not None and self.path is not None and len(self.path) > 2:
                path, profile = self.path, self.profile
                # servo + motor lag: steer from the pose the car reaches by the time this command acts
                w, tw = self.yaw_rate
                if time.monotonic() - tw > 0.1:                  # no IMU: fall back to the commanded steering
                    w = self.v_meas / self.wheelbase * math.tan(self.last_steer)
                pose = predict(*pose, self.v_meas, w, self.steer_delay)
                near = nearest(path, pose[0], pose[1])
                # planned speed a little ahead (the car reacts late), never above the current panel cap
                j = min(len(path) - 1, near + int(self.v_meas * self.preview / 0.05))
                v = min(float(profile[j]), self.speed)
                # each replan brings a slightly different profile: rate-limit the command so it does not
                # jitter, and ramp up from the measured speed (standstill, or after a hand-over)
                dt = 0.02
                v = min(v, self.last_speed + self.a_acc * dt, self.v_meas + self.a_acc * 0.5)
                v = max(v, self.last_speed - 2.0 * self.a_dec * dt)
                lookahead = min(2.5, max(0.6, self.lookahead_gain * v + 0.3))
                steer = pure_pursuit(path, *pose, lookahead, self.wheelbase, near)
                self.last_steer = max(-self.max_steer, min(self.max_steer, steer))
                if self.block is not None:                    # no room to pass: follow
                    v = min(v, max(0.0, 1.5 * (self.block - self.follow_gap)))
                cmd.drive.speed = v
            self.last_speed = cmd.drive.speed               # 0 when stopped (no pose / no path)
            cmd.drive.steering_angle = self.last_steer      # stopping keeps the last steering
            self.cmd_pub.publish(cmd)

    rclpy.init()
    node = RingPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
