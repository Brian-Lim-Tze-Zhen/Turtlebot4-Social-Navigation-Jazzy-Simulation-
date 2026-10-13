#!/usr/bin/env python3
"""member_lanes_rec20.py - lane starts for members of the standing pair (11 Oct 2026).
For every valid pair bag of the rec20 sets, counts the critic lines "lane rule now follows" in logs/<run>/nav2.log.
usage: python3 analysis/member_lanes_rec20.py   (run from ros2_ws)"""
import glob, os
for s in ("conv_F_rec20gs_wide", "conv_F_rec20gs_narrow", "conv_F_rec20ng_wide", "conv_F_rec20ng_narrow"):
    runs = sorted(os.path.basename(b) for b in glob.glob(f"bags/{s}_trial*"))
    if s == "conv_F_rec20ng_narrow": runs = [r for r in runs if r != "conv_F_rec20ng_narrow_trial02"]  # no zone map, see logs/rec20noghost_summary.txt
    hits = []
    for r in runs:
        n = open(f"logs/{r}/nav2.log", errors="ignore").read().count("lane rule now follows")
        if n: hits.append(f"{r} ({n})")
    print(f"{s:24s} runs {len(runs):2d}  runs with a member lane {len(hits)}  {', '.join(hits)}")
