import sys, os, bisect, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseArray
for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    W, C = [], []
    while r.has_next():
        tp, d, t = r.read_next(); t *= 1e-9
        if tp == "/person_ground_truth":
            m = deserialize_message(d, PoseArray)
            if m.poses: W.append((t, m.poses[0].position.x, m.poses[0].position.y))
        elif tp == "/predicted_person_cloud":
            m = deserialize_message(d, PointCloud2)
            pts = [(p[0], p[1]) for p in point_cloud2.read_points(m, field_names=("x", "y"), skip_nans=True)]
            if pts: C.append((t, m.header.frame_id, pts))
    tw = [w[0] for w in W]
    frames = set(c[1] for c in C)
    ahead = []
    for t, fr, pts in C:
        i = min(len(W) - 1, bisect.bisect_left(tw, t)); wx = W[i][1]
        if wx < 2.0 or wx > 8.5: continue   # walker moving toward -x, robot far
        xs = [p[0] for p in pts]
        ahead.append((wx, wx - min(xs), max(xs) - wx, min(p[1] for p in pts), max(p[1] for p in pts)))
    if ahead:
        a = sorted(ahead)[len(ahead)//2]
        print(f"{os.path.basename(os.path.dirname(bag)) if os.path.basename(bag) == 'data' else os.path.basename(bag)} frames {frames} n {len(ahead)}  median-sample walker x {a[0]:.2f}: cloud reaches {a[1]:.2f} m ahead, {a[2]:.2f} m behind, y {a[3]:+.2f}..{a[4]:+.2f}; max ahead over run {max(x[1] for x in ahead):.2f}")
