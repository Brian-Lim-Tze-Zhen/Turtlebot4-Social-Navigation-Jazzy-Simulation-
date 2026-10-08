#!/usr/bin/env python3
"""bend_in_window.py <window_m> <bag dir> [...] - does the robot turn when the plan's bend enters the controller window?

For each /plan message during the approach, the plan is cut to its first <window_m> metres of arc length from
the robot (as MPPI's prune_distance does); "bend visible" = some point of that part is more than 0.10 m off
the robot's current lateral position. Prints the gap (walker x - robot x, ground truth) when the bend first
becomes visible, and the gap when the robot's heading first exceeds 10 deg towards its pass side
("turn onset"). 8 Oct 2026.
"""
import math, os, sys
from bisect import bisect_left
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray

W = float(sys.argv[1])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


for bag in sys.argv[2:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(bag, 'data')), rosbag2_py.ConverterOptions('', ''))
    rob, wal, plans = [], [], []
    while r.has_next():
        tp, d, _ = r.read_next()
        if tp == '/sim_ground_truth_pose':
            m = deserialize_message(d, Odometry); p = m.pose.pose
            rob.append((st(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
        elif tp == '/person_ground_truth':
            m = deserialize_message(d, PoseArray)
            if m.poses:
                wal.append((st(m.header), m.poses[0].position.x, m.poses[0].position.y))
        elif tp == '/plan':
            m = deserialize_message(d, Path)
            plans.append((st(m.header), [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
    rob.sort(); wal.sort(); plans.sort(key=lambda x: x[0])
    rt = [x[0] for x in rob]; wt = [x[0] for x in wal]
    at = lambda s, ts, t: s[min(max(bisect_left(ts, t), 0), len(s) - 1)]
    best = min(rob, key=lambda x: math.hypot(at(wal, wt, x[0])[1] - x[1], at(wal, wt, x[0])[2] - x[2]))
    side = 1.0 if best[2] > 0 else -1.0
    g_bend = None
    for t, pts in plans:
        if t > best[0] or not pts:
            continue
        rb = at(rob, rt, t); gap = at(wal, wt, t)[1] - rb[1]
        if gap > 9.0:
            continue
        s, prev, off = 0.0, pts[0], 0.0
        for q in pts:
            s += math.hypot(q[0] - prev[0], q[1] - prev[1]); prev = q
            if s > W:
                break
            off = max(off, abs(q[1] - rb[2]))
        if off > 0.10:
            g_bend = gap
            break
    g_turn = None
    for x in rob:
        if x[0] > best[0]:
            break
        gap = at(wal, wt, x[0])[1] - x[1]
        if gap < 9.0 and math.degrees(math.atan2(math.sin(x[3]), math.cos(x[3]))) * side > 10.0:
            g_turn = gap
            break
    f = lambda v: '  -  ' if v is None else f'{v:5.2f}'
    print(f"{os.path.basename(bag):28s} {'L' if side > 0 else 'R'}  bend in first {W:.1f} m of plan at gap {f(g_bend)} m, "
          f"turn onset (>10 deg) at gap {f(g_turn)} m")
