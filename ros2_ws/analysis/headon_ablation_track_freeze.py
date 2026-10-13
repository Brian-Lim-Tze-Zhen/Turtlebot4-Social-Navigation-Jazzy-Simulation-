#!/usr/bin/env python3
# ----------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Explains the single low trials of the A-E ablation. When the robot turns away, the walker leaves
#   the camera image at about 3 m and the predictor republishes its last estimate unchanged (coasting)
#   until the LiDAR node takes over. headon_ablation_rerun_detection.py counts messages and does not see
#   this, because the repeats keep arriving. This script finds, per trial, the longest stretch in which
#   /fused_person_positions repeats the same position while the walker is 8 m to 0.8 m away, and prints
#   the frozen estimate, the robot's lateral position then, and the controller command (/cmd_vel_nav)
#   from the start of the freeze until 1 s after it.
#   Run in the simulation image:  python3 headon_ablation_track_freeze.py <bag folder pattern> ...
#     e.g. python3 headon_ablation_track_freeze.py '/ws/bags/[b-e]_*_rr_t0*' '/ws/bags/[d-e]_*_on_t0*'
#   Times: bag receive time (wall) is converted to simulation time through the /odom header stamps.
# ----------------------------------------------------------------------------------------------------
import sys, os, glob, math, bisect, csv
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyse_avoidance as aa
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

md = {}
for f in ("headon_ablation_AE_results.csv", "headon_ablation_rerun_results.csv"):
    for r in csv.DictReader(open(os.path.join(HERE, f))): md[r["trial"]] = float(r["min_distance"])

def near(tr, t):
    ts = [r[0] for r in tr]; i = min(bisect.bisect_left(ts, t), len(tr) - 1); return tr[i]

print("trial | min dist (m) | longest freeze (s, sim) | gap at its start (m) | robot y then (m) | frozen y (m) | frozen vx, vy (m/s) | min cmd v (m/s) | most negative cmd w (rad/s)")
for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        n = os.path.basename(d)
        if "_1.2_" in n: continue
        data = aa.read_bag(os.path.join(d, "data"))
        rob = aa.robot_path_in_map(data["/odom"], aa.extract_map_to_odom(data["/tf"] + data["/tf_static"]))
        per = aa.person_track(data["/person_ground_truth"])
        ck = [(t * 1e-9, aa.stamp_to_sec(m.header.stamp)) for t, m in data["/odom"]]; ckw = [c[0] for c in ck]
        def sim(tw):
            i = min(max(bisect.bisect_left(ckw, tw), 1), len(ck) - 1); (w0, s0), (w1, s1) = ck[i - 1], ck[i]
            return s0 + (s1 - s0) * (tw - w0) / (w1 - w0) if w1 > w0 else s0
        gap = lambda t: math.hypot(near(rob, t)[1] - near(per, t)[1], near(rob, t)[2] - near(per, t)[2])
        rd = SequentialReader(); rd.open(StorageOptions(uri=os.path.join(d, "data"), storage_id="mcap"), ConverterOptions("", ""))
        types = {t.name: t.type for t in rd.get_all_topics_and_types()}
        if "/fused_person_positions" not in types:
            print(f"{n} | {md.get(n, float('nan')):.3f} | /fused_person_positions not recorded"); continue
        CMD = get_message(types["/cmd_vel_nav"]); fus, cmd = [], []
        while rd.has_next():
            tp, dat, s = rd.read_next()
            if tp == "/fused_person_positions":
                f = deserialize_message(dat, String).data.split("\n")[0].split(",")
                if len(f) >= 8: fus.append((sim(s * 1e-9), f[2], f[3], float(f[4]), float(f[5])))
            elif tp == "/cmd_vel_nav":
                m = deserialize_message(dat, CMD); tw = m.twist if hasattr(m, "twist") else m
                cmd.append((sim(s * 1e-9), tw.linear.x, tw.angular.z))
        tmin = min(rob, key=lambda r: gap(r[0]))[0]
        app = [x for x in fus if x[0] < tmin and 0.8 < gap(x[0]) < 8]
        best, i = (0.0, None), 0
        while i < len(app):
            j = i
            while j + 1 < len(app) and app[j + 1][1:3] == app[i][1:3]: j += 1
            if j > i:
                dur = (app[j + 1][0] if j + 1 < len(app) else app[j][0]) - app[i][0]
                if dur > best[0]: best = (dur, i)
            i = j + 1
        if best[1] is None:
            print(f"{n} | {md.get(n, float('nan')):.3f} | no repeated estimate"); continue
        a = app[best[1]]; w = [c for c in cmd if a[0] <= c[0] <= a[0] + best[0] + 1.0]
        print(f"{n} | {md.get(n, float('nan')):.3f} | {best[0]:.2f} | {gap(a[0]):.2f} | {near(rob, a[0])[2]:+.2f} | {float(a[2]):+.3f} | {a[3]:+.2f}, {a[4]:+.2f} | "
              f"{min(c[1] for c in w):+.2f} | {min(c[2] for c in w):+.2f}", flush=True)
