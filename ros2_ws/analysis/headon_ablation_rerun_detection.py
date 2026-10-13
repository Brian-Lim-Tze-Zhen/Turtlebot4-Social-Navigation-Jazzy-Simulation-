#!/usr/bin/env python3
# ----------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   For every trial of the ablation re-run (and, for comparison, the August trials) it shows how well
#   the walker was tracked during the approach: the robot-walker distance when the predictor published
#   the walker for the first time after the walker started, the longest pause between two predictor
#   messages while the walker was 8 m to 1 m away, and the number of messages in that window.
#   It is used to explain single low distances in the re-run.
#   Run in the simulation image:  python3 headon_ablation_rerun_detection.py <bags folder> [out.txt]
#   Reads:  <bags folder>/[a-e]_*_t0*/data      Writes: the text given as second argument
# ----------------------------------------------------------------------------------------------------
import sys, os, glob, math, csv
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import analyse_avoidance as aa
import bisect
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
def read_strings(path, topic):
    rd = SequentialReader(); rd.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("", "")); res = []
    while rd.has_next():
        tp_, dat, st = rd.read_next()
        if tp_ == topic: res.append((st, deserialize_message(dat, String).data))
    return res
BAGS = sys.argv[1]; out = []
def P(s): print(s, flush=True); out.append(s)
md = {}
for f in ("headon_ablation_AE_results.csv", "headon_ablation_rerun_results.csv"):
    for r in csv.DictReader(open(os.path.join(HERE, f))): md[r["trial"]] = r
P("trial | min distance (m) | commit distance (m) | first predictor message at gap (m) | predictor messages 8 m to 1 m | longest pause (s) | gap at the start of that pause (m)")
for d in sorted(glob.glob(os.path.join(BAGS, "[a-e]_*_t0*"))):
    name = os.path.basename(d)
    if "_1.2_" in name: continue
    data = aa.read_bag(os.path.join(d, "data"))
    m2o = aa.extract_map_to_odom(data["/tf"] + data["/tf_static"])
    robot = aa.robot_path_in_map(data["/odom"], m2o); per = aa.person_track(data["/person_ground_truth"])
    if not robot or not per: continue
    x0 = per[0][1]; tp = next((t for t, x, y in per if abs(x - x0) > 0.05), per[0][0])
    at = lambda tr, t: min(tr, key=lambda r: abs(r[0] - t))
    gap = lambda t: math.hypot(at(robot, t)[1] - at(per, t)[1], at(robot, t)[2] - at(per, t)[2])
    tmin = min(per, key=lambda r: gap(r[0]))[0]
    # The predictor message has no time stamp: its recording time is turned into simulation time
    # through the /odom messages (recording time -> stamp), so that it fits the two tracks.
    od = [(st, aa.stamp_to_sec(m.header.stamp)) for st, m in data["/odom"]]; keys = [o[0] for o in od]
    simt = lambda st: od[min(len(od) - 1, bisect.bisect_left(keys, st))][1]
    pred = [simt(st) for st, txt in read_strings(os.path.join(d, "data"), "/predicted_person_positions") if txt.strip()]
    pred = [t for t in pred if tp <= t <= tmin]
    win = [t for t in pred if 1.0 <= gap(t) <= 8.0]
    pause, gp = 0.0, float("nan")
    for a, b in zip(win, win[1:]):
        if b - a > pause: pause, gp = b - a, gap(a)
    r = md.get(name, {})
    P(f"{name} | {float(r.get('min_distance') or 'nan'):.3f} | {float(r.get('commit_dist') or 'nan'):.2f} | {(gap(pred[0]) if pred else float('nan')):.2f} | {len(win)} | {pause:.2f} | {gp:.2f}")
if len(sys.argv) > 2: open(sys.argv[2], "w").write("\n".join(out) + "\n")
