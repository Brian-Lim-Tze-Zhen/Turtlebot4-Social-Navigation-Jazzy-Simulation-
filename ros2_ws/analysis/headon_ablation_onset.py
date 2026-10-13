#!/usr/bin/env python3
"""onset_check.py - when does the robot start to avoid the head-on walker, and which component reacts first?

For each bag of the head-on ablation (A-E), the gap between robot and walker (centres, ground truth for the walker,
odom + AMCL map->odom for the robot, as in analyse_avoidance.py) at three events:
  plan   first /plan that leaves the robot-walker line by more than PLAN_DEV m between the robot and the walker
  turn   first /cmd_vel_nav with |wz| > WZ_ON rad/s held for HOLD s (controller starts the manoeuvre)
  commit robot more than 0.20 m off its start->goal line (the commit distance of the thesis)
Only events after the walker has started to move and while the walker is still ahead of the robot count.
Usage: python3 onset_check.py <bag dir> ..."""
import sys, os, math, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseArray, TwistStamped
from tf2_msgs.msg import TFMessage

PLAN_DEV, WZ_ON, HOLD, COMMIT = 0.30, 0.15, 0.5, 0.20

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def run(bag):
    uri = bag + "/data" if os.path.exists(bag + "/data/metadata.yaml") else bag
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    tf_mo, odom, walker, plans, cmds = [], [], [], [], []
    while r.has_next():
        tp, d, t = r.read_next()
        t *= 1e-9
        if tp == "/tf":
            for tr in deserialize_message(d, TFMessage).transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    a = tr.transform
                    tf_mo.append((t, a.translation.x, a.translation.y, yaw(a.rotation)))
        elif tp == "/odom":
            m = deserialize_message(d, Odometry).pose.pose
            odom.append((t, m.position.x, m.position.y))
        elif tp == "/person_ground_truth":
            m = deserialize_message(d, PoseArray)
            if m.poses:
                walker.append((t, m.poses[0].position.x, m.poses[0].position.y))
        elif tp == "/plan":
            m = deserialize_message(d, Path)
            plans.append((t, [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif tp == "/cmd_vel_nav":
            cmds.append((t, deserialize_message(d, TwistStamped).twist.angular.z))
    if not (tf_mo and odom and walker):
        return None
    tmo = [x[0] for x in tf_mo]

    def robot(t):
        i = max(0, min(len(odom) - 1, bisect.bisect_left([o[0] for o in odom], t)))
        _, ox, oy = odom[i]
        j = max(0, bisect.bisect_right(tmo, t) - 1)
        _, tx, ty, th = tf_mo[j]
        return tx + ox * math.cos(th) - oy * math.sin(th), ty + ox * math.sin(th) + oy * math.cos(th)

    tw = [w[0] for w in walker]

    def wpos(t):
        i = bisect.bisect_left(tw, t)
        if i <= 0: return walker[0][1:]
        if i >= len(walker): return walker[-1][1:]
        (t0, x0, y0), (t1, x1, y1) = walker[i - 1], walker[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0)

    # walker starts moving: first sample 0.2 m away from its first position
    w0 = walker[0]
    t_start = next((w[0] for w in walker if math.hypot(w[1] - w0[1], w[2] - w0[2]) > 0.2), None)
    if t_start is None:
        return None
    # robot start->goal line: first and last odom positions in map
    rs, re_ = robot(odom[0][0]), robot(odom[-1][0])
    ux, uy = re_[0] - rs[0], re_[1] - rs[1]
    L = math.hypot(ux, uy); ux, uy = ux / L, uy / L
    lat = lambda p: abs(-(p[0] - rs[0]) * uy + (p[1] - rs[1]) * ux)

    def gap(t):
        p, w = robot(t), wpos(t)
        return math.hypot(p[0] - w[0], p[1] - w[1]), (w[0] - p[0]) * ux + (w[1] - p[1]) * uy

    ev = {}
    for t, pts in plans:
        if t < t_start: continue
        g, ahead = gap(t)
        if ahead < 0.5: break
        p = robot(t); w = wpos(t)
        seg = [q for q in pts if 0.0 <= (q[0] - p[0]) * ux + (q[1] - p[1]) * uy <= ahead]
        if seg and max(lat(q) for q in seg) > PLAN_DEV:
            ev["plan"] = g; break
    on = None
    for t, wz in cmds:
        if t < t_start: continue
        g, ahead = gap(t)
        if ahead < 0.5: break
        if abs(wz) > WZ_ON:
            on = on if on is not None else t
            if t - on >= HOLD:
                ev["turn"] = gap(on)[0]; break
        else:
            on = None
    for t, ox, oy in odom:
        if t < t_start: continue
        g, ahead = gap(t)
        if ahead < 0.0: break
        if lat(robot(t)) > COMMIT:
            ev["commit"] = g; break
    return ev

if __name__ == "__main__":
    for bag in sys.argv[1:]:
        ev = run(bag)
        f = lambda k: f"{ev[k]:5.2f}" if ev and k in ev else "   - "
        print(f"{os.path.basename(bag.rstrip('/')):42s} plan {f('plan')}  turn {f('turn')}  commit {f('commit')}", flush=True)
