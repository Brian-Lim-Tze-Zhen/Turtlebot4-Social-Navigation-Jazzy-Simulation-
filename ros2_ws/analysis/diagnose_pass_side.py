#!/usr/bin/env python3
"""diagnose_pass_side.py <bag dir> [...] - why are left passes closer than right passes? (8 Oct 2026)

Head-on corridor trials (run_headon_F_trial.sh). Everything is put on the axis "gap" = walker x - robot x
(ground truth, /person_ground_truth and /sim_ground_truth_pose, sim time). Per trial it prints:
  - pass side (sign of the robot's y at the closest point) and the closest centre distance
  - start state when the robot starts to move (first |vx| > 0.05): y, yaw error from the corridor axis, gap
  - gap at: first published person (/predicted_person_positions), the critic's "lane rule now follows",
    "pass side fixed" (from logs/<name>/nav2.log, wall -> sim through /clock)
  - at gaps 7, 6, 5, 4, 3, 2, 1.5 m: the robot's lateral offset TOWARDS its pass side (m), its heading
    towards the pass side (deg), the latest global plan's offset towards the pass side at the walker's x
    (m; negative = plan on the other side), and the latest lane_block cloud: points / mean y towards the side
and then the mean of each column over the left and over the right passes.
Outputs to stdout; save with tee.
"""
import math, os, re, sys, statistics as st
from bisect import bisect_left, bisect_right
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

GAPS = [7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.5]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def run(bag):
    name = os.path.basename(bag.rstrip('/'))
    ws = os.path.dirname(os.path.dirname(os.path.abspath(bag)))
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(bag, 'data')), rosbag2_py.ConverterOptions('', ''))
    clock, robot, walker, plans, blocks, cmd, pers = [], [], [], [], [], [], []
    while r.has_next():
        tp, d, tn = r.read_next()
        if tp == '/clock':
            c = deserialize_message(d, Clock).clock
            clock.append((tn * 1e-9, c.sec + c.nanosec * 1e-9))
        elif tp == '/sim_ground_truth_pose':
            m = deserialize_message(d, Odometry)
            p = m.pose.pose
            robot.append((stamp(m.header), p.position.x, p.position.y, yaw_of(p.orientation)))
        elif tp == '/person_ground_truth':
            m = deserialize_message(d, PoseArray)
            if m.poses:
                walker.append((stamp(m.header), m.poses[0].position.x, m.poses[0].position.y))
        elif tp == '/plan':
            m = deserialize_message(d, Path)
            plans.append((stamp(m.header), [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
        elif tp == '/social_critic/lane_block':
            m = deserialize_message(d, PointCloud2)
            pts = [(p[0], p[1]) for p in point_cloud2.read_points(m, field_names=('x', 'y'), skip_nans=True)]
            blocks.append((stamp(m.header), pts))
        elif tp == '/cmd_vel':
            pass
        elif tp == '/predicted_person_positions':
            pers.append(tn * 1e-9)
    clock.sort()
    cw = [c[0] for c in clock]

    def w2s(w):
        i = min(max(bisect_left(cw, w), 0), len(cw) - 1)
        return clock[i][1]
    robot.sort(); walker.sort(); plans.sort(key=lambda x: x[0]); blocks.sort(key=lambda x: x[0])
    rt = [x[0] for x in robot]; wt = [x[0] for x in walker]

    def at(series, ts, t):
        i = min(max(bisect_left(ts, t), 0), len(series) - 1)
        return series[i]

    def gap_at(t):
        return at(walker, wt, t)[1] - at(robot, rt, t)[1]
    # walker only valid once it moves; closest point
    best = None
    for (t, x, y, _) in robot:
        w = at(walker, wt, t)
        dd = math.hypot(w[1] - x, w[2] - y)
        if best is None or dd < best[0]:
            best = (dd, t, x, y)
    side = 1.0 if best[3] > 0 else -1.0
    # start of motion: from ground truth speed
    t_move = None
    for a, b in zip(robot, robot[1:]):
        if b[0] > a[0] and math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0]) > 0.05:
            t_move = a[0]
            break
    s_state = at(robot, rt, t_move)
    # log events
    ev = {}
    pat = re.compile(r'\[(\d+\.\d+)\] \[controller_server\]: SocialCritic: (lane rule now follows|track \d+ pass side fixed)')
    for line in open(os.path.join(ws, 'logs', name, 'nav2.log'), errors='ignore'):
        m = pat.search(line)
        if m:
            k = 'lane' if m.group(2).startswith('lane') else 'fixed'
            ev.setdefault(k, w2s(float(m.group(1))))
    t_det = w2s(min(pers)) if pers else None

    # time when the gap reaches g (gap decreases monotonically during the approach)
    t_g = {}
    for (t, x, y, _) in robot:
        if t < best[1]:
            g = gap_at(t)
            for G in GAPS:
                if G not in t_g and g <= G:
                    t_g[G] = t
    out = {'name': name, 'side': side, 'dmin': best[0],
           'y0': s_state[2] * side, 'yaw0': math.degrees(s_state[3] - math.pi if s_state[3] > 0 else s_state[3] + math.pi) * side,
           'gap0': gap_at(t_move),
           'gap_det': gap_at(t_det) if t_det else float('nan'),
           'gap_lane': gap_at(ev['lane']) if 'lane' in ev else float('nan'),
           'gap_fixed': gap_at(ev['fixed']) if 'fixed' in ev else float('nan')}
    pt = [p[0] for p in plans]; bt = [b[0] for b in blocks]
    for G in GAPS:
        t = t_g.get(G)
        if t is None:
            continue
        rb = at(robot, rt, t); wk = at(walker, wt, t)
        out[f'lat{G}'] = rb[2] * side
        # heading towards the pass side; robot drives towards +x, yaw 0
        out[f'hdg{G}'] = math.degrees(math.atan2(math.sin(rb[3]), math.cos(rb[3]))) * side
        i = bisect_right(pt, t) - 1
        if i >= 0 and plans[i][1]:
            near = [q for q in plans[i][1] if abs(q[0] - wk[1]) < 0.3]
            out[f'plan{G}'] = (st.mean(q[1] for q in near) - wk[2]) * side if near else float('nan')
        j = bisect_right(bt, t) - 1
        if j >= 0:
            pts = blocks[j][1]
            out[f'blkn{G}'] = len(pts)
            out[f'blky{G}'] = st.mean(p[1] for p in pts) * side if pts else float('nan')
    return out


rows = [run(b) for b in sys.argv[1:]]
f = lambda v: 'nan' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:+.2f}'
print('start of motion (towards the pass side): y0 m, yaw0 deg; gaps (m) at: start, first person, lane, side fixed')
for o in rows:
    print(f"{o['name'][-7:]:>8s} {'L' if o['side'] > 0 else 'R'} dmin {o['dmin']:.3f} | y0 {f(o['y0'])} yaw0 {f(o['yaw0'])} | "
          f"gap start {o['gap0']:.2f} det {f(o['gap_det'])} lane {f(o['gap_lane'])} fixed {f(o['gap_fixed'])}")
print('\nper gap: robot lateral towards pass side (m) / heading towards pass side (deg) / plan offset at walker towards side (m) / lane_block points, mean y towards side')
for o in rows:
    cells = []
    for G in GAPS:
        cells.append(f"{G:g}:{f(o.get(f'lat{G}'))}/{f(o.get(f'hdg{G}'))}/{f(o.get(f'plan{G}'))}/{o.get(f'blkn{G}', '-')},{f(o.get(f'blky{G}'))}")
    print(f"{o['name'][-7:]:>8s} {'L' if o['side'] > 0 else 'R'}  " + '  '.join(cells))
print('\nmeans: left passes vs right passes')
for k in ['dmin', 'y0', 'yaw0', 'gap0', 'gap_det', 'gap_lane', 'gap_fixed'] + \
         [f'{p}{G}' for p in ('lat', 'hdg', 'plan', 'blkn', 'blky') for G in GAPS]:
    vals = {}
    for s_, lab in ((1.0, 'L'), (-1.0, 'R')):
        v = [o[k] for o in rows if o['side'] == s_ and k in o and not (isinstance(o[k], float) and math.isnan(o[k]))]
        vals[lab] = f'{st.mean(v):+.2f} (n={len(v)})' if v else '-'
    print(f'  {k:9s} L {vals["L"]:>16s}   R {vals["R"]:>16s}')
