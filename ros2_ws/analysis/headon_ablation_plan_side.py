#!/usr/bin/env python3
# ----------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Explains low trials of the A-E ablation by the side of the global plan. With the person cloud the
#   plan bends around the predicted walker; when the walker estimate jumps sideways the plan can flip to
#   the other side for a while. The robot passes on its right in every trial, so a plan on the left
#   (y > +0.3 m at its widest point between robot and walker) is the wrong side.
#   For each trial the script lists the side changes of /plan (with the robot-walker gap at each change)
#   and the metres of approach (gap) during which the plan was on the wrong side while the gap was
#   between 6.5 m and 3.0 m, the stretch in which the robot builds its lateral offset.
#   Run in the simulation image:  python3 headon_ablation_plan_side.py <bag folder pattern> ...
#   Times: bag receive time is converted to simulation time through the /odom header stamps.
# ----------------------------------------------------------------------------------------------------
import sys, os, glob, math, bisect, csv
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyse_avoidance as aa
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LO, HI, BENT = 3.0, 6.5, 0.3
md = {}
for f in ("headon_ablation_AE_results.csv", "headon_ablation_rerun_results.csv"):
    for r in csv.DictReader(open(os.path.join(HERE, f))): md[r["trial"]] = (float(r["min_distance"]), float(r["dev_at_min"]))
def near(tr, t):
    ts = [r[0] for r in tr]; i = min(bisect.bisect_left(ts, t), len(tr) - 1); return tr[i]

print(f"trial | min dist (m) | robot side at closest approach | plan side changes (side@gap m) | wrong-side plan between {HI} and {LO} m gap (m of approach)")
seen = set()
for pat in sys.argv[1:]:
    for d in sorted(glob.glob(pat)):
        n = os.path.basename(d)
        if "_1.2_" in n or n in seen: continue
        seen.add(n)
        data = aa.read_bag(os.path.join(d, "data"))
        rob = aa.robot_path_in_map(data["/odom"], aa.extract_map_to_odom(data["/tf"] + data["/tf_static"]))
        per = aa.person_track(data["/person_ground_truth"])
        ck = [(t * 1e-9, aa.stamp_to_sec(m.header.stamp)) for t, m in data["/odom"]]; ckw = [c[0] for c in ck]
        def sim(tw):
            i = min(max(bisect.bisect_left(ckw, tw), 1), len(ck) - 1); (w0, s0), (w1, s1) = ck[i - 1], ck[i]
            return s0 + (s1 - s0) * (tw - w0) / (w1 - w0) if w1 > w0 else s0
        rd = SequentialReader(); rd.open(StorageOptions(uri=os.path.join(d, "data"), storage_id="mcap"), ConverterOptions("", ""))
        ty = {t.name: t.type for t in rd.get_all_topics_and_types()}
        plan = []
        while rd.has_next():
            tp, dat, s = rd.read_next()
            if tp == "/plan":
                m = deserialize_message(dat, get_message(ty[tp])); plan.append((sim(s * 1e-9), [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        gap = lambda t: math.hypot(near(rob, t)[1] - near(per, t)[1], near(rob, t)[2] - near(per, t)[2])
        tmin = min(rob, key=lambda r: gap(r[0]))[0]
        seq = []                                   # (gap, side) for every plan message before the closest approach
        for t, pts in plan:
            if t >= tmin: break
            rx, px = near(rob, t)[1], near(per, t)[1]
            seg = [q for q in pts if rx < q[0] < px]
            if not seg: continue
            q = max(seg, key=lambda q: abs(q[1]))
            seq.append((gap(t), "-" if abs(q[1]) < BENT else ("L" if q[1] > 0 else "R")))
        changes = []
        for g, sd in seq:
            if sd != "-" and (not changes or changes[-1][0] != sd): changes.append((sd, g))
        wrong = 0.0                                 # metres of gap closed while the plan was on the left, inside [LO, HI]
        for (g0, s0), (g1, _) in zip(seq, seq[1:]):
            if s0 == "L":
                a, b = min(g0, HI), max(g1, LO)
                if a > b: wrong += a - b
        mind, dev = md.get(n, (float("nan"), float("nan")))
        print(f"{n} | {mind:.3f} | {'right' if dev < 0 else 'left'} | {' -> '.join(f'{s}@{g:.1f}' for s, g in changes) or 'never bent'} | {wrong:.2f}", flush=True)
