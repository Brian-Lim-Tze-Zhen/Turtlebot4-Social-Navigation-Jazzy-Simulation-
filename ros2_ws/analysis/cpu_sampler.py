#!/usr/bin/env python3
"""cpu_sampler.py <out.csv> [period_s] - CPU use per process during a trial (8 Oct 2026).

Runs on the HOST (sees the processes of the container). Every period_s (default 1.0) it writes one
row per process that used CPU: wall time, pid, group, node, cpu (% of ONE core, so 100 = one full
core), rss_mb; plus one row per sample with group "_total" = whole machine in % of all cores.
Groups: perception (our perception/zone/beep nodes), nav2 (Nav2 servers incl. the costmaps and the
MPPI controller with SocialCritic), gazebo (gz sim + bridges: simulation only), recorder (ros2 bag),
other. Stop with Ctrl-C / SIGTERM.
"""
import csv, os, signal, sys, time
import psutil

OUT = sys.argv[1]
PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
PERCEPTION = ("yolo_leg_detector", "camera_ray_person", "camera_ray_identity", "person_marker_publisher",
              "human_kf_predictor", "predicted_person_cloud", "social_group_detector", "social_zone_costmap",
              "blocked_person_beep", "beep_retry", "track_dropout_relay")
NAV2 = ("controller_server", "planner_server", "bt_navigator", "behavior_server", "amcl", "map_server",
        "collision_monitor", "velocity_smoother", "smoother_server", "waypoint_follower", "lifecycle_manager",
        "route_server", "docking_server", "opennav_docking", "filter_mask_server", "costmap_filter_info",
        "component_container")
GAZEBO = ("gz sim", "gz-sim", "ruby", "parameter_bridge", "ros_gz", "create3", "turtlebot4_node", "robot_state_publisher",
          "joint_state_publisher", "hmi", "diffdrive", "undock", "set_pose", "move_person", "headon_send_goal")


def classify(p):
    try:
        cmd = " ".join(p.cmdline())
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return None, None
    for k in PERCEPTION:
        if k in cmd:
            return "perception", k
    for k in NAV2:
        if k in cmd:
            return "nav2", k
    if "ros2 bag record" in cmd or "rosbag2" in cmd or ("ros2" in cmd and " bag " in cmd):
        return "recorder", "ros2 bag record"
    for k in GAZEBO:
        if k in cmd:
            return "gazebo", k.strip()
    return "other", (p.info.get("name") or "?")


stop = False
def _stop(*_):
    global stop
    stop = True
signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT, _stop)

procs = {}
psutil.cpu_percent(None)
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["wall", "pid", "group", "node", "cpu", "rss_mb"])
    while not stop:
        time.sleep(PERIOD)
        now = time.time()
        seen = set()
        for p in psutil.process_iter(["pid", "name"]):
            seen.add(p.pid)
            if p.pid not in procs:
                procs[p.pid] = p
                try:
                    p.cpu_percent(None)          # prime; first value is meaningless
                except psutil.Error:
                    pass
                continue
            try:
                c = procs[p.pid].cpu_percent(None)
                if c <= 0.0:
                    continue
                g, n = classify(procs[p.pid])
                if g is None:
                    continue
                w.writerow([f"{now:.2f}", p.pid, g, n, f"{c:.1f}", f"{procs[p.pid].memory_info().rss / 1e6:.0f}"])
            except psutil.Error:
                pass
        for pid in list(procs):
            if pid not in seen:
                del procs[pid]
        w.writerow([f"{now:.2f}", 0, "_total", "machine", f"{psutil.cpu_percent(None):.1f}", 0])
        fh.flush()
