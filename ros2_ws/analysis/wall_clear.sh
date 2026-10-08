#!/bin/bash
# wall_clear.sh <bag name> - largest |y| of the robot (ground truth) and wall clearance in the 2.5 m corridor
# (inner wall faces at +/-1.25 m, robot radius 0.189 m). 8 Oct 2026. Run inside the container.
cd /root/thesis_social_navigation_ws && python3 -c "
import csv; r=list(csv.DictReader(open('bags/$1/trajectory.csv'))); m=max(abs(float(x['y_ref'])) for x in r); print(f'max |y| {m:.3f} m, wall clearance {1.25-m-0.189:.3f} m')"
