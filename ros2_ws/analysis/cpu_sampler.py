#!/usr/bin/env python3
"""cpu_sampler.py <out.csv> <seconds>: CPU use per process once per second from /proc (utime+stime),
in percent of one core; writes time, pid, name, cpu_percent. Run inside the container that runs the trial."""
import os, sys, time
out, dur = sys.argv[1], float(sys.argv[2])
hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
def name_of(pid):
    try:
        cmd = open(f"/proc/{pid}/cmdline","rb").read().split(b"\0")
        cmd = [c.decode(errors="ignore") for c in cmd if c]
    except Exception:
        return None
    if not cmd: return None
    joined = " ".join(cmd)
    for c in cmd:
        if c.endswith(".py"): return os.path.basename(c)
    for key in ("__node:=",):
        for c in cmd:
            if key in c: return c.split(key)[1]
    b = os.path.basename(cmd[0])
    if b in ("python3","bash","sh","timeout","sleep","grep","sed","awk","tail","ros2"):
        if b=="ros2" and len(cmd)>2: return "ros2 "+cmd[1]+" "+cmd[2]
        return None
    if "gz" in b and "sim" in joined: return "gz sim " + ("server" if "-s" in cmd else "gui/other")
    return b
def ticks():
    d = {}
    for p in os.listdir("/proc"):
        if not p.isdigit(): continue
        try:
            f = open(f"/proc/{p}/stat").read().rsplit(")",1)[1].split()
            d[int(p)] = int(f[11]) + int(f[12])
        except Exception:
            pass
    return d
prev = ticks(); t0 = time.time()
with open(out, "w") as fo:
    fo.write("t,pid,name,cpu_percent\n")
    while time.time() - t0 < dur:
        time.sleep(1.0)
        cur = ticks(); now = time.time()
        for pid, tk in cur.items():
            if pid in prev:
                d = tk - prev[pid]
                if d > 0:
                    n = name_of(pid)
                    if n: fo.write(f"{now-t0:.1f},{pid},{n},{100.0*d/hz:.1f}\n")
        fo.flush(); prev = cur
