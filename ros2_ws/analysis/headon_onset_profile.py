#!/usr/bin/env python3
"""headon_onset_profile.py - map y (m) and heading off the start-goal line (deg) of the robot at given robot-walker gaps.
The walker walks along y = 0. Usage: python3 headon_onset_profile.py <bag dir> ..."""
import sys, os, math, bisect
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import headon_ablation_onset as H
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseArray
from tf2_msgs.msg import TFMessage
GAPS = [10, 9, 8, 7, 6, 5, 4, 3, 2]
def load(bag):
    uri = bag + "/data" if os.path.exists(bag + "/data/metadata.yaml") else bag
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    tf_mo, odom, walker = [], [], []
    while r.has_next():
        tp, d, t = r.read_next(); t *= 1e-9
        if tp == "/tf":
            for tr in deserialize_message(d, TFMessage).transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    a = tr.transform; tf_mo.append((t, a.translation.x, a.translation.y, H.yaw(a.rotation)))
        elif tp == "/odom":
            m = deserialize_message(d, Odometry).pose.pose
            odom.append((t, m.position.x, m.position.y, H.yaw(m.orientation)))
        elif tp == "/person_ground_truth":
            m = deserialize_message(d, PoseArray)
            if m.poses: walker.append((t, m.poses[0].position.x, m.poses[0].position.y))
    tmo = [x[0] for x in tf_mo]
    def robot(o):
        t, ox, oy, oth = o
        j = max(0, bisect.bisect_right(tmo, t) - 1); _, tx, ty, th = tf_mo[j]
        return tx + ox*math.cos(th) - oy*math.sin(th), ty + ox*math.sin(th) + oy*math.cos(th), oth + th
    tw = [w[0] for w in walker]
    def wpos(t):
        i = bisect.bisect_left(tw, t)
        if i <= 0: return walker[0][1:]
        if i >= len(walker): return walker[-1][1:]
        (t0,x0,y0),(t1,x1,y1) = walker[i-1], walker[i]; f=(t-t0)/(t1-t0) if t1>t0 else 0
        return x0+f*(x1-x0), y0+f*(y1-y0)
    rs, re_ = robot(odom[0]), robot(odom[-1])
    ux, uy = re_[0]-rs[0], re_[1]-rs[1]; L = math.hypot(ux, uy); ux, uy = ux/L, uy/L
    w0 = walker[0]
    t_start = next(w[0] for w in walker if math.hypot(w[1]-w0[1], w[2]-w0[2]) > 0.2)
    out = {}; gi = 0
    for o in odom:
        if o[0] < t_start: continue
        x, y, th = robot(o); w = wpos(o[0])
        g = math.hypot(x-w[0], y-w[1])
        while gi < len(GAPS) and g <= GAPS[gi]:
            lat = y
            hd = math.degrees(math.atan2(math.sin(th - math.atan2(uy, ux)), math.cos(th - math.atan2(uy, ux))))
            out[GAPS[gi]] = (lat, hd); gi += 1
    return out
print(f"{'bag':36s}" + "".join(f"  gap {g:>4}m lat/hdg" for g in GAPS))
for bag in sys.argv[1:]:
    o = load(bag)
    print(f"{os.path.basename(bag):36s}" + "".join(f"  {o[g][0]:+5.2f} {o[g][1]:+5.0f}°     " if g in o else "        -          " for g in GAPS), flush=True)
