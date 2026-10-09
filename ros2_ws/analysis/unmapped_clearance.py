#!/usr/bin/env python3
"""unmapped_clearance.py - the robot-run clearance measure for simulation bags (9 Oct 2026).

Same measure as humble_client workspace/pytest/analysis/side_20261009 (summarise_runs.py / pair_run.py) for the robot:
for every scan, the nearest LiDAR return within 2.5 m (from the sensor) whose point is NOT on the static map (3-cell
margin, as blocked_person_beep_node_front.py); the run value is the minimum over
  - head-on: points ahead of the start (x > spawn x + 0.5 m; the dock the robot starts on is not on the map),
  - pair:    points within 1.5 m before and 1.0 m after the pair (x of the /social_groups members).
Robot pose from TF map->odom plus /odom, as on the robot (AMCL, not ground truth). Simulation robots drive along +x,
so x/y are already the hallway frame.

usage (in the container): python3 analysis/unmapped_clearance.py bags/<name> [bags/<name> ...]
"""
import sys, math, re, statistics as st
import numpy as np, yaml
from PIL import Image
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def load_map(path):
    M = yaml.safe_load(open(path)); import os
    img = np.array(Image.open(os.path.join(os.path.dirname(path), M['image'])))
    occ = img < 100 if not M.get('negate', 0) else img > 155
    return occ, M['resolution'], M['origin'][0], M['origin'][1]

def run(bag):
    notes = open(bag + '/run_notes.txt').read()
    occ, res, ox0, oy0 = load_map(re.search(r'^Map: (\S+)', notes, re.M).group(1))
    spawn_x = float(re.search(r'Robot spawn: \(([-0-9.]+)', notes).group(1))
    rows = occ.shape[0]
    def occupied(x, y, m=3):
        cx = int((x - ox0) / res); cy = rows - 1 - int((y - oy0) / res)
        a = occ[max(cy - m, 0):cy + m + 1, max(cx - m, 0):cx + m + 1]
        return a.size > 0 and bool(a.any())
    rd = rosbag2_py.SequentialReader()
    import os
    uri = bag + '/data' if os.path.isdir(bag + '/data') else bag   # head-on bags keep the recording in data/
    rd.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rd.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', '/odom', '/scan', '/social_groups']))
    m2o = None; robot = None; static = {}; near = []; members = []
    while rd.has_next():
        t, d, ts = rd.read_next()
        if t in ('/tf', '/tf_static'):
            for tr in deserialize_message(d, TFMessage).transforms:
                q = tr.transform.rotation; yaw = 2 * math.atan2(q.z, q.w)
                if tr.header.frame_id.endswith('map') and tr.child_frame_id.endswith('odom'):
                    m2o = (tr.transform.translation.x, tr.transform.translation.y, yaw)
                elif t == '/tf_static':
                    static[tr.child_frame_id] = (tr.header.frame_id, tr.transform.translation.x, tr.transform.translation.y, yaw)
        elif t == '/odom' and m2o:
            m = deserialize_message(d, Odometry); p = m.pose.pose.position; q = m.pose.pose.orientation; ox, oy, oth = m2o
            robot = (ox + p.x * math.cos(oth) - p.y * math.sin(oth), oy + p.x * math.sin(oth) + p.y * math.cos(oth),
                     oth + 2 * math.atan2(q.z, q.w))
        elif t == '/social_groups':
            f = deserialize_message(d, String).data.split(',')
            members += [tuple(float(v) for v in p.split(';')) for p in f[9].split('|')]
        elif t == '/scan' and robot:
            m = deserialize_message(d, LaserScan)
            # sensor pose: chain of static transforms from the scan frame up to base_link
            fx, fy, fyaw, fr = 0.0, 0.0, 0.0, m.header.frame_id
            for _ in range(6):
                if fr not in static or fr.endswith('base_link'):
                    break
                par, tx, ty, tyaw = static[fr]
                fx, fy = tx + fx * math.cos(tyaw) - fy * math.sin(tyaw), ty + fx * math.sin(tyaw) + fy * math.cos(tyaw)
                fyaw += tyaw; fr = par
            rx, ry, ryaw = robot
            lx = rx + fx * math.cos(ryaw) - fy * math.sin(ryaw); ly = ry + fx * math.sin(ryaw) + fy * math.cos(ryaw)
            th = ryaw + fyaw
            rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
            ok = np.isfinite(rg) & (rg > max(0.15, m.range_min)) & (rg < 2.5)
            for r_, an in sorted(zip(rg[ok], a[ok])):
                x = lx + r_ * math.cos(th + an); y = ly + r_ * math.sin(th + an)
                if not occupied(x, y):
                    near.append((float(r_), x, y)); break
    if members:
        px = st.median(p[0] for p in members)
        cand = [n for n in near if px - 1.5 <= n[1] <= px + 1.0]; kind = f'near the pair (x {px:.2f})'
    else:
        cand = [n for n in near if n[1] > spawn_x + 0.5]; kind = 'ahead of the start'
    c = min(cand) if cand else None
    return bag.split('/')[-1], c, kind

vals = {}
for b in sys.argv[1:]:
    name, c, kind = run(b.rstrip('/'))
    if c:
        print(f'{name:34s} nearest return not on the map {kind}: {c[0]:.3f} m at ({c[1]:.2f}, {c[2]:+.2f})')
        vals.setdefault(re.sub(r'_trial.*', '', name), []).append(c[0])
    else:
        print(f'{name:34s} no unmapped return found ({kind})')
for k, v in vals.items():
    print(f'{k}: n {len(v)}  mean {st.mean(v):.3f}  sd {st.stdev(v) if len(v) > 1 else 0:.3f}  min {min(v):.3f}  max {max(v):.3f}')
