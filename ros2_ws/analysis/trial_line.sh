#!/bin/bash
# trial_line.sh <bag name> - one summary line for a head-on trial (8 Oct 2026). Run inside the container.
# Appends to analysis/pathside30_lines.txt:
# name | min centre | surface | side | commit gap | stop/spin/reverse | provisional->fixed side (offset) | plans L/R | noghost
cd /root/thesis_social_navigation_ws && source /opt/ros/jazzy/setup.bash
N=$1
A=$(python3 analysis/analyse_headon_F.py "bags/$N" 2>&1)
g() { echo "$A" | grep -m1 "$1" | sed "s|.*$1 *||"; }
T=$(python3 analysis/hwsync_pass_timeline.py "bags/$N" 2>/dev/null)
prov=$(echo "$T" | grep -m1 "lane rule now follows" | grep -o "keeps [A-Z]*" | cut -d' ' -f2)
fixed=$(echo "$T" | grep -m1 "pass side fixed" | grep -o "keeps [A-Z]*" | cut -d' ' -f2)
off=$(echo "$T" | grep -m1 "pass side fixed" | grep -o "offset [-+0-9.]* m" | cut -d' ' -f2)
ng=$(echo "$T" | grep -m1 "NOGHOST: dropped" | sed 's/.*event *\([-+0-9.]*\) s.*/drop \1 s/')
P=$(python3 analysis/plan_side_check.py "bags/$N" 2>&1 | grep -m1 "$N" | sed 's/.*wrong side (+y) \([0-9]*\), right (-y) \([0-9]*\).*/\1\/\2/')
L="$N | $(g 'min centre distance' | cut -d' ' -f1) m | surf $(g 'min surface clearance' | cut -d' ' -f1) | $(g 'robot passed on its' | cut -d' ' -f1) | commit $(g 'gap when committed (>0.2 m)' | cut -d' ' -f1) | s/s/r $(g 'stopped / spin / reverse' | tr -d ' ') | $prov->$fixed ($off) | plans L/R $P | ${ng:-no drop} | goal $(g 'goal reached')"
echo "$L" | tee -a "${OUT:-analysis/pathside30_lines.txt}"
