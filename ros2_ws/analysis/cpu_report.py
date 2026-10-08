#!/usr/bin/env python3
"""cpu_report.py <cpu.csv> <bag dir> - CPU load per node during one head-on trial (8 Oct 2026).

Input: the CSV of cpu_sampler.py (1 s samples, cpu in % of ONE core) and the trial's bag folder.
Window: from the goal being accepted (mtime of logs/<name>/goal_accepted) to the goal result
(mtime of bags/<name>/goal_result.txt), wall time.
Groups:
  perception  - our perception, zone and beep nodes (run on the laptop on the robot as well)
  nav2        - Nav2 servers incl. both costmaps and the MPPI controller with SocialCritic (laptop)
  sim-only    - Gazebo, bridges and the simulated Create 3 base nodes (on the robot: none / on the base)
  recorder    - ros2 bag record (measurement only)
  desktop     - editor, shell, desktop, this session
  other       - anything else (mostly launch / ros2 CLI processes)
Prints mean and 95th percentile per node and per group (% of one core; the machine has N cores),
the whole-machine load, the real-time factor of the trial from /clock, and perception values
scaled to real time (divided by the RTF), because the simulated camera publishes in sim time.
"""
import csv, os, sys, statistics as st, collections
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
import psutil

csv_path, bag = sys.argv[1], sys.argv[2].rstrip('/')
name = os.path.basename(bag)
ws = os.path.dirname(os.path.dirname(os.path.abspath(bag)))
t0 = os.path.getmtime(os.path.join(ws, 'logs', name, 'goal_accepted'))
t1 = os.path.getmtime(os.path.join(bag, 'goal_result.txt'))

SIM_BASE = ("motion_control", "sensors_node", "pose_republisher_node", "wheel_status_publisher", "ui_mgr",
            "ir_intensity_vector_publisher", "robot_state", "hazards_vector_publisher", "kidnap_estimator_publisher",
            "mock_publisher", "interface_buttons_node", "dock", "ros_gz", "gz", "ruby")
DESKTOP = ("code", "claude", "gnome-shell", "Xorg", "Xwayland", "pipewire", "gnome", "firefox", "chromium", "snap")


def group(g, n):
    if g in ("perception", "nav2", "recorder", "_total"):
        return g
    if g == "gazebo" or any(n.startswith(k) for k in SIM_BASE):
        return "sim-only"
    if any(n.startswith(k) for k in DESKTOP):
        return "desktop"
    return "other"


samples = collections.defaultdict(lambda: collections.defaultdict(float))   # wall -> key -> cpu
for r in csv.DictReader(open(csv_path)):
    w = float(r['wall'])
    if not (t0 <= w <= t1):
        continue
    G = group(r['group'], r['node'])
    c = float(r['cpu'])
    if G == "_total":
        samples[w]["_total"] = c
        continue
    samples[w]["G:" + G] += c
    if G in ("perception", "nav2"):
        samples[w]["N:" + G + ":" + r['node']] += c
walls = sorted(samples)
keys = sorted({k for s in samples.values() for k in s})


def stats(k):
    v = [samples[w].get(k, 0.0) for w in walls]
    v.sort()
    return st.mean(v), v[int(0.95 * (len(v) - 1))]


# real-time factor in the window, from /clock
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=os.path.join(bag, 'data')), rosbag2_py.ConverterOptions('', ''))
clk = []
while r.has_next():
    tp, d, tn = r.read_next()
    if tp == '/clock':
        c = deserialize_message(d, Clock).clock
        w = tn * 1e-9
        if t0 <= w <= t1:
            clk.append((w, c.sec + c.nanosec * 1e-9))
rtf = (clk[-1][1] - clk[0][1]) / (clk[-1][0] - clk[0][0])

ncore = psutil.cpu_count()
print(f"{name}: window {t1 - t0:.0f} s wall (goal accepted -> result), {len(walls)} samples of 1 s, "
      f"real-time factor {rtf:.2f}, machine {ncore} cores")
print("values in % of ONE core (100 = one core fully busy); mean / 95th percentile")
print("\nper group")
for k in [k for k in keys if k.startswith("G:")]:
    m, p = stats(k)
    print(f"  {k[2:]:12s} {m:6.1f} / {p:6.1f}   = {m / ncore:5.1f} % of the machine")
m, p = stats("_total")
print(f"  {'machine':12s} {m:6.1f} / {p:6.1f}   (% of all {ncore} cores)")
for G in ("perception", "nav2"):
    print(f"\n{G} nodes")
    for k in [k for k in keys if k.startswith("N:" + G + ":")]:
        m, p = stats(k)
        extra = f"   real-time estimate {m / rtf:6.1f}" if G == "perception" else ""
        print(f"  {k.split(':', 2)[2]:28s} {m:6.1f} / {p:6.1f}{extra}")
mp, _ = stats("G:perception")
mn, _ = stats("G:nav2")
print(f"\nlaptop share on the robot (perception + Nav2): {mp + mn:.1f} % of one core measured, "
      f"{mp / rtf + mn:.1f} with perception scaled to real time "
      f"= {(mp / rtf + mn) / ncore:.1f} % of the {ncore}-core machine")
