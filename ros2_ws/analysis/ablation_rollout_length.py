#!/usr/bin/env python3
"""ablation_rollout_length.py - number of poses per /optimal_trajectory message (= MPPI time_steps) in each bag.
Usage: python3 ablation_rollout_length.py <bag dir> ..."""
import sys, os, collections, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for bag in sys.argv[1:]:
    uri = bag + "/data"
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=["/optimal_trajectory"]))
    T = get_message(types["/optimal_trajectory"]); c = collections.Counter()
    while r.has_next():
        _, d, _ = r.read_next(); m = deserialize_message(d, T)
        n = len(m.poses) if hasattr(m, "poses") else len(m.points)
        c[n] += 1
    print(f"{bag:42s} {types['/optimal_trajectory']:28s} poses per msg {dict(c.most_common(2))}", flush=True)
