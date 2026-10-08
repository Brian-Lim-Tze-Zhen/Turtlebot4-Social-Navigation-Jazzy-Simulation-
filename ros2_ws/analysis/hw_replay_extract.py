#!/usr/bin/env python3
"""hw_replay_extract.py <hw bag folder> <out dir> - small copy of a robot bag for the critic replay (8 Oct 2026).

Copies only the topics the replay needs from the robot's sqlite3 bag (<hw bag folder>/bag/bag_0.db3 or
<hw bag folder>/bag_0.db3) into <out dir>/data/bag_0.db3 with a matching metadata.yaml; the original is
read-only. Kept: TF, static TF, odom, scan, global plan, goal pose, person positions, groups, /rosout
(the robot critic's own decisions, used as the reference). Needs no ROS (plain sqlite3).
"""
import os, shutil, sqlite3, sys, yaml

KEEP = ['/turtlebot4/tf', '/turtlebot4/tf_static', '/turtlebot4/odom', '/turtlebot4/scan', '/turtlebot4/plan',
        '/turtlebot4/goal_pose', '/predicted_person_positions', '/social_groups', '/rosout']
src, out = sys.argv[1].rstrip('/'), sys.argv[2].rstrip('/')
sd = os.path.join(src, 'bag') if os.path.isdir(os.path.join(src, 'bag')) else src
db_in = [f for f in os.listdir(sd) if f.endswith('.db3')]
assert len(db_in) == 1, db_in
meta = yaml.safe_load(open(os.path.join(sd, 'metadata.yaml')))
os.makedirs(os.path.join(out, 'data'), exist_ok=True)
dst = os.path.join(out, 'data', 'bag_0.db3')
if os.path.exists(dst):
    os.remove(dst)
conn = sqlite3.connect(dst)
conn.execute("ATTACH DATABASE ? AS src", ('file:' + os.path.join(sd, db_in[0]) + '?mode=ro',))
for (sql,) in conn.execute("SELECT sql FROM src.sqlite_master WHERE type='table' AND sql IS NOT NULL").fetchall():
    conn.execute(sql)
ids = [r[0] for r in conn.execute(
    f"SELECT id FROM src.topics WHERE name IN ({','.join('?' * len(KEEP))})", KEEP)]
q = ','.join('?' * len(ids))
conn.execute(f"INSERT INTO topics SELECT * FROM src.topics WHERE id IN ({q})", ids)
conn.execute(f"INSERT INTO messages SELECT * FROM src.messages WHERE topic_id IN ({q})", ids)
for t in ('schema', 'metadata'):
    try:
        conn.execute(f"INSERT INTO {t} SELECT * FROM src.{t}")
    except sqlite3.OperationalError:
        pass
conn.commit()
counts = dict(conn.execute("SELECT t.name, count(m.id) FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.name"))
conn.execute("DETACH DATABASE src")
conn.close()
info = meta['rosbag2_bagfile_information']
info['relative_file_paths'] = ['bag_0.db3']
info['topics_with_message_count'] = [t for t in info['topics_with_message_count'] if t['topic_metadata']['name'] in KEEP]
for t in info['topics_with_message_count']:
    t['message_count'] = counts.get(t['topic_metadata']['name'], 0)
info['message_count'] = sum(t['message_count'] for t in info['topics_with_message_count'])
for f in info.get('files', []):
    f['path'] = 'bag_0.db3'
    f['message_count'] = info['message_count']
info['files'] = info.get('files', [])[:1]
yaml.safe_dump(meta, open(os.path.join(out, 'data', 'metadata.yaml'), 'w'), sort_keys=False)
with open(os.path.join(out, 'source.txt'), 'w') as fh:
    fh.write(f"extracted from {os.path.abspath(src)} by hw_replay_extract.py; topics: {', '.join(KEEP)}\n")
print(f"{os.path.basename(src)}: {info['message_count']} messages, {os.path.getsize(dst) / 1e6:.1f} MB")
