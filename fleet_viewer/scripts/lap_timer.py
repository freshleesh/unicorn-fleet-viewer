#!/usr/bin/env python3
"""Race control for an N-lap race on the ring track. Run once (main PC).

Laps are counted on the track centreline, not on a line test: each car's position is projected
to the centreline and its forward progress is integrated, so wobbling on the line never counts
twice, driving backwards never counts, and a lap needs a full lap of forward progress.
A car lined up just behind the start line at GO crosses it first without completing a lap.
Lap times are this PC's receive times of /<car>/odom (car clocks are not synced), with the
crossing moment interpolated between two odom messages.

  in : /<car>/odom, /race/start_line (PoseStamped: where + driving direction, from the
       "Start Line" RViz tool; per-car overrides: /race/<car>/start_line), /race/command (String: "start <laps>" | "reset")
  out: /race/state (String, JSON, 10 Hz), /race/marker (MarkerArray: chequered line + direction)

  lap_timer.py --ros-args -p map:=<track.yaml>
The start line is saved to ~/.ros/race_start_line.yaml and restored on the next start.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

CARS = ('nuc1', 'nuc4', 'nuc5')
COUNTDOWN = 3.0          # s from START to GO
JUMP = 1.5               # m of progress in one odom step = relocalisation, not driving
STALE = 1.0              # s without odom: car is not on track
LINE_FILE = os.path.expanduser('~/.ros/race_start_line.yaml')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


# ---------------------------------------------------------------- track geometry (no ROS)

def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:                      # magic, width, height, maxval (comments allowed)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    if fields[0] != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) maps are supported')
    w, h, maxval = int(fields[1]), int(fields[2]), int(fields[3])
    return np.frombuffer(data[pos + 1:pos + 1 + w * h], np.uint8).reshape(h, w).astype(np.float64) / maxval


def centreline_loop(free):
    """Ordered (row, col) pixels of the ring's centreline (same as the car's ring_planner)."""
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
        nxt.sort(key=lambda p: abs(p[0] - r) + abs(p[1] - c))
        order.append(nxt[0])
        seen.add(nxt[0])
    if len(order) < 0.9 * len(pts):
        raise ValueError(f'centreline walk covered {len(order)}/{len(pts)} pixels: not a simple ring')
    return np.array(order)


class Track:
    """Closed centreline resampled every 5 cm: points, arc length, tangents, half widths."""

    def __init__(self, map_yaml, spacing=0.05):
        meta = yaml.safe_load(open(map_yaml))
        img = read_pgm(os.path.join(os.path.dirname(map_yaml), meta['image']))
        occ = img if meta.get('negate', 0) else 1.0 - img
        free = occ < meta.get('free_thresh', 0.196)
        res = meta['resolution']
        ox, oy = meta['origin'][:2]
        top = oy + free.shape[0] * res
        loop = centreline_loop(free)
        pts = np.stack([ox + (loop[:, 1] + 0.5) * res, top - (loop[:, 0] + 0.5) * res], 1)
        k, pad = np.ones(5) / 5, 2                             # smooth the pixel staircase (wraps)
        pts = np.stack([np.convolve(np.r_[pts[-pad:, i], pts[:, i], pts[:pad, i]], k, 'valid')
                        for i in range(2)], 1)
        closed = np.vstack([pts, pts[:1]])
        s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(closed, axis=0).T))]
        self.length = s[-1]
        t = np.arange(0.0, self.length, spacing)
        self.pts = np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], 1)
        self.s = t
        d = np.roll(self.pts, -2, 0) - np.roll(self.pts, 2, 0)
        self.tangent = d / np.linalg.norm(d, axis=1, keepdims=True)
        edt = distance_transform_edt(free) * res
        rows = np.clip(((top - self.pts[:, 1]) / res).astype(int), 0, free.shape[0] - 1)
        cols = np.clip(((self.pts[:, 0] - ox) / res).astype(int), 0, free.shape[1] - 1)
        self.half_width = edt[rows, cols]
        self.tree = cKDTree(self.pts)

    def project(self, x, y, near=None, window=2.0):
        """Index of the closest centreline point; around `near` (+-window m) when given, so a
        car never snaps to the other side of a thin wall."""
        if near is not None:
            n = len(self.pts)
            w = int(window / (self.s[1] - self.s[0]))
            idx = (near + np.arange(-w, w + 1)) % n
            return int(idx[np.argmin(np.hypot(self.pts[idx, 0] - x, self.pts[idx, 1] - y))])
        return int(self.tree.query([x, y])[1])


# ---------------------------------------------------------------- race logic (no ROS)

class CarRace:
    def __init__(self, distance, go_time, laps_total):
        self.d = distance          # race distance [m]: 0 at the start line, lap k done at k*L
        self.laps_total = laps_total
        self.laps = 0
        self.lap_start = go_time
        self.last = None
        self.best = None
        self.finished = False
        self.total = None
        self.finish_order = None


class Race:
    """Pure race bookkeeping: feed it every car's forward progress, read the standings."""

    def __init__(self, length):
        self.L = length
        self.phase = 'idle'
        self.laps_total = 0
        self.go_time = None
        self.cars = {}
        self.finishers = 0

    def start(self, laps, now):
        self.phase, self.laps_total, self.go_time = 'countdown', laps, now + COUNTDOWN
        self.cars, self.finishers = {}, 0

    def reset(self):
        self.phase, self.cars, self.go_time, self.finishers = 'idle', {}, None, 0

    def go(self, offsets):
        """offsets: car -> forward distance past the start line now, in [0, L)."""
        self.phase = 'running'
        for car, r in offsets.items():
            # just behind the line (second half of the lap): the first crossing is the start, not a lap
            d = r - self.L if r > self.L / 2 else r
            self.cars[car] = CarRace(d, self.go_time, self.laps_total)

    def advance(self, car, ds, t_prev, t):
        c = self.cars.get(car)
        if c is None or self.phase != 'running':
            return
        d_prev, c.d = c.d, c.d + ds
        while not c.finished and c.d >= (c.laps + 1) * self.L:
            target = (c.laps + 1) * self.L
            frac = (target - d_prev) / (c.d - d_prev) if c.d > d_prev else 1.0
            t_cross = t_prev + frac * (t - t_prev)
            lap = t_cross - c.lap_start
            c.laps += 1
            c.last = lap
            c.best = lap if c.best is None else min(c.best, lap)
            c.lap_start = t_cross
            if c.laps >= self.laps_total:
                c.finished = True
                c.total = t_cross - self.go_time
                self.finishers += 1
                c.finish_order = self.finishers
        if self.cars and all(x.finished for x in self.cars.values()):
            self.phase = 'finished'

    def standings(self):
        order = sorted(self.cars, key=lambda k: (self.cars[k].finish_order or 99, -self.cars[k].d))
        return {car: i + 1 for i, car in enumerate(order)}


# ---------------------------------------------------------------- ROS node

class LapTimer:
    def __init__(self):
        self.node = rclpy.create_node('lap_timer')
        map_yaml = os.path.expanduser(self.node.declare_parameter('map', '').value)
        self.track = Track(map_yaml)
        self.race = Race(self.track.length)
        self.log = self.node.get_logger()
        self.log.info(f'{map_yaml}: centreline {self.track.length:.1f} m')
        self.lines = {}
        self.map_yaml = os.path.realpath(map_yaml)
        self.cars = {}                       # car -> dict(idx, s, t)
        self.state_pub = self.node.create_publisher(String, '/race/state', 10)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/race/marker', LATCHED)
        for car in CARS:
            self.node.create_subscription(Odometry, f'/{car}/odom',
                                          lambda m, c=car: self.on_odom(c, m), qos_profile_sensor_data)
        self.node.create_subscription(PoseStamped, '/race/start_line', self.on_line, 10)
        for car in CARS:
            self.node.create_subscription(PoseStamped, f'/race/{car}/start_line',
                                          lambda m, c=car: self.on_line(m, c), 10)
        self.node.create_subscription(String, '/race/command', self.on_command, 10)
        self.node.create_timer(0.1, self.tick)
        if os.path.isfile(LINE_FILE):
            try:
                with open(LINE_FILE) as f:
                    saved = yaml.safe_load(f) or {}
                if 'lines' in saved:
                    if saved.get('map') == self.map_yaml:
                        for car, p in saved['lines'].items():
                            if car in (*CARS, 'shared'):
                                self.set_line(p['x'], p['y'], p['yaw'], car, save=False)
                    else:
                        self.log.warn('saved start lines belong to a different map; set new lines')
                elif all(k in saved for k in ('x', 'y', 'yaw')):
                    self.set_line(saved['x'], saved['y'], saved['yaw'], save=False)
            except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as e:
                self.log.warn(f'could not restore start lines: {e}')
        self.publish_marker()

    # start lines -----------------------------------------------------------
    def line_for(self, car):
        # Once individual lines are set, only explicitly configured cars participate.
        if any(c in self.lines for c in CARS):
            return self.lines.get(car)
        return self.lines.get('shared')

    def save_lines(self):
        os.makedirs(os.path.dirname(LINE_FILE), exist_ok=True)
        payload = {'map': self.map_yaml, 'lines': {
            car: {k: line[k] for k in ('x', 'y', 'yaw')} for car, line in self.lines.items()}}
        tmp = LINE_FILE + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump(payload, f)
        os.replace(tmp, LINE_FILE)

    def set_line(self, x, y, yaw, car='shared', save=True):
        if car not in (*CARS, 'shared') or not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('invalid start line')
        i = self.track.project(x, y)
        tx, ty = self.track.tangent[i]
        direction = 1 if tx * math.cos(yaw) + ty * math.sin(yaw) >= 0 else -1
        self.lines[car] = dict(idx=i, direction=direction, x=float(x), y=float(y), yaw=float(yaw))
        if save:
            self.save_lines()
        self.log.info(f'{car} start line at {self.track.s[i]:.1f} m, direction {direction:+d}')
        self.publish_marker()

    def on_line(self, msg, car='shared'):
        if self.race.phase in ('countdown', 'running'):
            self.log.warn('race in progress: start line not moved; RESET first')
            return
        if msg.header.frame_id != 'map':
            self.log.warn('start line must be in map frame')
            return
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            self.set_line(msg.pose.position.x, msg.pose.position.y, yaw, car)
        except ValueError as e:
            self.log.warn(str(e))

    def offset(self, car, idx):
        line = self.line_for(car)
        return (line['direction'] * (self.track.s[idx] - self.track.s[line['idx']])) % self.track.length

    # cars ------------------------------------------------------------------
    def on_odom(self, car, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        prev = self.cars.get(car)
        fresh = prev is not None and now - prev['t'] < STALE
        idx = self.track.project(p.x, p.y, prev['idx'] if fresh else None)
        self.cars[car] = {'idx': idx, 't': now}
        if not fresh:
            return
        line = self.line_for(car)
        if line is None:
            return
        L = self.track.length
        ds = (line['direction'] * (self.track.s[idx] - self.track.s[prev['idx']]) + L / 2) % L - L / 2
        if abs(ds) > JUMP:                   # relocalised (Pose tool): not driving
            return
        self.race.advance(car, ds, prev['t'], now)

    def on_command(self, msg):
        words = msg.data.split()
        if not words:
            return
        if words[0] == 'start':
            if self.race.phase in ('countdown', 'running'):
                self.log.warn('race already active; RESET first')
                return
            if not any(self.line_for(c) for c in CARS):
                self.log.warn('set at least one start line before START')
                return
            try:
                laps = int(words[1]) if len(words) > 1 else 3
            except ValueError:
                self.log.warn('invalid lap count')
                return
            self.race.start(max(1, laps), time.monotonic())
            self.log.info(f'race: {laps} laps, GO in {COUNTDOWN:.0f} s')
        elif words[0] == 'clear_lines':
            if self.race.phase in ('countdown', 'running'):
                self.log.warn('RESET before clearing start lines')
                return
            self.lines.clear()
            self.save_lines()
            self.publish_marker()
        elif words[0] == 'reset':
            self.race.reset()
            self.log.info('race reset')

    def tick(self):
        now = time.monotonic()
        r = self.race
        if r.phase == 'countdown' and now >= r.go_time:
            on_track = {c: self.offset(c, v['idx']) for c, v in self.cars.items()
                        if now - v['t'] < STALE and self.line_for(c) is not None}
            if not on_track:
                r.reset()
                self.log.warn('race cancelled: no cars with a start line and fresh odometry')
            else:
                r.go(on_track)
            self.log.info(f'GO: {", ".join(on_track) or "no cars on track"}')
        pos = r.standings()
        cars = {}
        for car in CARS:
            st = {'on_track': car in self.cars and now - self.cars[car]['t'] < STALE,
                  'line_set': self.line_for(car) is not None}
            c = r.cars.get(car)
            if c is not None:
                st.update(in_race=True, pos=pos[car], lap=c.laps, laps_total=r.laps_total,
                          lap_time=None if c.finished else now - c.lap_start,
                          last=c.last, best=c.best, finished=c.finished, total=c.total,
                          progress=max(0.0, (c.d % r.L) / r.L) if not c.finished else 1.0)
            else:
                st['in_race'] = False
            cars[car] = st
        state = {
            'phase': r.phase, 'laps_total': r.laps_total, 'line_set': bool(self.lines),
            'start_mode': 'individual' if any(c in self.lines for c in CARS) else 'shared',
            'countdown': max(0.0, r.go_time - now) if r.phase == 'countdown' else None,
            'clock': now - r.go_time if r.phase == 'running' else None,
            'cars': cars, 'n_in_race': len(r.cars),
        }
        self.state_pub.publish(String(data=json.dumps(state)))

    # marker ----------------------------------------------------------------
    def publish_marker(self):
        legacy_label = Marker()
        legacy_label.header.frame_id = 'map'
        legacy_label.ns, legacy_label.id, legacy_label.action = 'start_line', 2, Marker.DELETE
        markers = [legacy_label]
        active = {c: self.lines[c] for c in CARS if c in self.lines}
        if not active and 'shared' in self.lines:
            active = {'shared': self.lines['shared']}
        # Delete only inactive IDs; active IDs are replaced by their ADD markers.
        for car in ('shared', *CARS):
            if car in active:
                continue
            for mid in range(3):
                m = Marker()
                m.header.frame_id = 'map'
                m.ns, m.id, m.action = f'start_line_{car}', mid, Marker.DELETE
                markers.append(m)
        for car, line in active.items():
            markers.extend(self.line_markers(car, line))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def line_markers(self, car, line):
        tr, i = self.track, line['idx']
        (x, y), (tx, ty) = tr.pts[i], tr.tangent[i] * line['direction']
        ns = f'start_line_{car}'
        color = {'nuc1': (1., .39, .1), 'nuc4': (.2, .59, 1.),
                 'nuc5': (.29, 1., .29), 'shared': (1., .85, 0.)}[car]
        nx, ny = -ty, tx
        hw = max(tr.half_width[i], 0.3)
        sq = 0.1
        chk = Marker()
        chk.header.frame_id = 'map'
        chk.ns, chk.id, chk.type = ns, 0, Marker.CUBE_LIST
        chk.pose.orientation.w = 1.0
        chk.scale.x = chk.scale.y = sq
        chk.scale.z = 0.01
        n = int(2 * hw / sq)
        for row in range(2):
            for k in range(n):
                off = -hw + (k + 0.5) * sq
                along = (row - 0.5) * sq
                chk.points.append(Point(x=float(x + nx * off + tx * along), y=float(y + ny * off + ty * along), z=0.02))
                v = 1.0 if (k + row) % 2 else 0.0
                chk.colors.append(ColorRGBA(r=v, g=v, b=v, a=1.0))
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.ns, arrow.id, arrow.type = ns, 1, Marker.ARROW
        arrow.points = [Point(x=float(x + tx * 0.15), y=float(y + ty * 0.15), z=0.03),
                        Point(x=float(x + tx * 0.75), y=float(y + ty * 0.75), z=0.03)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.15, 0.15
        arrow.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        # Explicitly remove labels retained by RViz from earlier versions.
        label = Marker()
        label.header.frame_id = 'map'
        label.ns, label.id, label.action = ns, 2, Marker.DELETE
        return [chk, arrow, label]


def main():
    rclpy.init()
    lt = LapTimer()
    try:
        rclpy.spin(lt.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
