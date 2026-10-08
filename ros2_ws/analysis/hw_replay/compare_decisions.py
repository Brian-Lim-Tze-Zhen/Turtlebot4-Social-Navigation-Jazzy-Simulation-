#!/usr/bin/env python3
"""compare_decisions.py <bag data dir> <controller.log> - the robot critic's decisions (/rosout in the bag) next
to the replayed critic's decisions (8 Oct 2026). Lines kept: lane rule now follows, pass side fixed, jumped,
inherits the lane. Times are seconds after the first person message (robot: bag time of the /rosout message;
replay: the log stamp, which is wall time of the replay, converted through the bag start; both rounded)."""
import re, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log

KEYS = ('lane rule now follows', 'pass side fixed', 'jumped', 'inherits the lane', 'side hold:', 'measured against')
bag, log = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
robot, t_person = [], None
while r.has_next():
    tp, d, tn = r.read_next()
    if tp == '/predicted_person_positions' and t_person is None:
        t_person = tn * 1e-9
    if tp == '/rosout':
        m = deserialize_message(d, Log)
        if 'Critic' in m.msg and any(k in m.msg for k in KEYS[:4]):
            robot.append((tn * 1e-9, re.sub(r'^Social\w*Critic: ', '', m.msg)))
rep = []
pat = re.compile(r'\[(\d+\.\d+)\] \[controller_server\]: SocialCritic: (.*)')
for line in open(log, errors='ignore'):
    mm = pat.search(line)
    if mm and any(k in mm.group(2) for k in KEYS):
        rep.append((float(mm.group(1)), mm.group(2)))
print('ROBOT (bag /rosout), t after first person message:')
for t, m in robot:
    print(f'  {t - t_person:+7.2f} s  {m}')
print('REPLAY (this config):')
setup = [m for _, m in rep if any(k in m for k in KEYS[4:])]
for m in setup:
    print(f'  setup    {m}')
dec = [(t, m) for t, m in rep if not any(k in m for k in KEYS[4:])]
t0 = dec[0][0] if dec else 0.0
for t, m in dec:
    print(f'  {t - t0:+7.2f} s* {m}')
print('  (* replay times relative to its first decision; the replay runs on bag time, its log stamps are wall time)')
