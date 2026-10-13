#!/usr/bin/env python3
# --------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   For each head-on run of the final configuration in simulation, finds how far ahead the walker was
#   when the critic started the lane ("lane rule now follows track ... (X m ahead)" in the controller
#   log), and compares it with the distance at which the walker was first detected.
#   Used in the thesis: Section 8.3 and Table 8.1 (lane started 6.06 +/- 0.30 m before the walker)
#   Reads:  logs/headon_rec20gs_trial*/nav2.log, analysis/headon_rec20gs_n10_report.txt
#   Writes: printout, saved as analysis/headon_lane_start_report.txt
# --------------------------------------------------------------------------------------------------
"""usage (from ros2_ws): python3 analysis/headon_lane_start.py"""
import glob, os, re, statistics as st
det = [float(x) for x in re.findall(r"first detection range\s+([\d.]+)", open("analysis/headon_rec20gs_n10_report.txt").read())][:10]
runs = sorted(glob.glob("logs/headon_rec20gs_trial*/nav2.log"))
lane = []
for log in runs:
    m = re.search(r"lane rule now follows track \S+ \(([\d.]+) m ahead\)", open(log, errors="ignore").read())
    lane.append(float(m.group(1)))
    print(f"{os.path.basename(os.path.dirname(log)):26s} lane start {lane[-1]:.2f} m ahead")
d = [a - b for a, b in zip(det, lane)]
print(f"lane start      {st.mean(lane):.2f} +/- {st.stdev(lane):.2f} m ({min(lane):.2f} - {max(lane):.2f}), n = {len(lane)}")
print(f"first detection {st.mean(det):.2f} +/- {st.stdev(det):.2f} m")
print(f"detection - lane start {st.mean(d):.2f} +/- {st.stdev(d):.2f} m ({min(d):.2f} - {max(d):.2f})")
