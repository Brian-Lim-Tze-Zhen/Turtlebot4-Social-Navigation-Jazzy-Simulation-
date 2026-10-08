#!/bin/bash
# plan_conflict.sh <bag name> - global plan offset at the walker towards the robot's pass side at gaps 4/3/2 m
# (map frame, from diagnose_pass_side.py); negative = plan on the other side than the pass = critic/planner conflict.
cd /root/thesis_social_navigation_ws && source /opt/ros/jazzy/setup.bash
python3 analysis/diagnose_pass_side.py "bags/$1" 2>/dev/null | awk '/^per gap/{p=1;next} /^means/{p=0} p && NF>2 {split($5,a,"/"); split($6,c,"/"); split($7,d,"/"); s=a[3]+0; t=c[3]+0; u=d[3]+0; printf "plan@4/3/2 m: %s / %s / %s   %s\n", a[3], c[3], d[3], (s<0||t<0||u<0)?"PLAN ON OTHER SIDE":"plan on pass side"}'
