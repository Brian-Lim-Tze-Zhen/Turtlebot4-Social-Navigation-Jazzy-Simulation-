#!/usr/bin/env python3
"""trace_jump.py <bag data dir> <KF track id> - where does a position jump of a person track enter? (8 Oct 2026)

Lists, around the robot critic's lane start for <KF track id> (from /rosout), the three stages of one person:
  KF   /predicted_person_positions  id,conf,x,y,vx,vy,...           (what the critic sees)
  ID   /person_positions_fused      stable_id,conf,x,y,...,source,cam_id   (identity node output)
  RAY  /camera_ray_clusters         cam_id,conf,map_x,map_y          (camera-ray node output, identity input)
For ID and RAY the step length to the previous message of the same stable id / cam id is printed, so a jump can
be located: present in RAY -> created by the ray node; only in ID -> created by the identity node (re-attach,
coast, hold); only in KF -> filter.
"""
import math, re, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rcl_interfaces.msg import Log

bag, tid = sys.argv[1], int(sys.argv[2])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
kf, fused, ray, t_lane = [], [], [], None
while r.has_next():
    tp, d, tn = r.read_next()
    t = tn * 1e-9
    if tp == '/predicted_person_positions':
        f = deserialize_message(d, String).data.split(',')
        kf.append((t, int(float(f[0])), float(f[2]), float(f[3])))
    elif tp == '/person_positions_fused':
        f = deserialize_message(d, String).data.split(',')
        fused.append((t, int(float(f[0])), float(f[2]), float(f[3]), f[11] if len(f) > 11 else '', f[12] if len(f) > 12 else ''))
    elif tp == '/camera_ray_clusters':
        f = deserialize_message(d, String).data.split(',')
        ray.append((t, int(float(f[0])), float(f[2]), float(f[3])))
    elif tp == '/rosout' and t_lane is None:
        if re.search(rf'follows track {tid} \(', deserialize_message(d, Log).msg):
            t_lane = t
print(f'{bag.split("/")[-2]}  KF track {tid}, robot lane start at bag t={t_lane:.2f}')
W0, W1 = -1.6, 0.15
sids = sorted({s for (t, s, *_) in fused if s == tid})
print(f'  identity stable ids equal to the KF id: {sids or "none"}')
cams = sorted({c for (t, s, x, y, src, c) in fused if s == tid and t_lane + W0 <= t <= t_lane + W1 and c not in ('', '-1')})
print(f'  camera ids feeding stable {tid} in the window: {cams}')
rows = []
for (t, i, x, y) in kf:
    if i == tid and t_lane + W0 <= t <= t_lane + W1:
        rows.append((t, 'KF ', f'id {i}', x, y, ''))
last = {}
for (t, s, x, y, src, c) in fused:
    if s == tid and t_lane + W0 - 2 <= t <= t_lane + W1:
        st = math.hypot(x - last[s][0], y - last[s][1]) if s in last else 0.0
        last[s] = (x, y)
        if t >= t_lane + W0:
            rows.append((t, 'ID ', f'sid {s} {src} cam {c}', x, y, f'{st:.2f}'))
lastc = {}
for (t, c, x, y) in ray:
    if str(c) in cams and t_lane + W0 - 2 <= t <= t_lane + W1:
        st = math.hypot(x - lastc[c][0], y - lastc[c][1]) if c in lastc else 0.0
        lastc[c] = (x, y)
        if t >= t_lane + W0:
            rows.append((t, 'RAY', f'cam {c}', x, y, f'{st:.2f}'))
print('   t-lane  stage  who                              x       y     step')
for (t, stg, who, x, y, st) in sorted(rows):
    print(f'  {t - t_lane:+6.2f}  {stg}  {who:32s} {x:7.2f} {y:7.2f}  {st}')
