#!/usr/bin/env python3
# ----------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Report of the re-run of the head-on ablation A to E (13 Oct 2026, bags <condition>_rr_tNN).
#   For every trial it reads the proof files written at the trial (launched Nav2 file, live controller
#   parameters, person cloud node on or off, goal result) and the bag (same metrics and the same code
#   as for the August trials, analyse_avoidance.py), then gives mean, SD, minimum and maximum per
#   condition next to the August values, and exact two-sided Mann-Whitney tests.
#   Run in the simulation image:  python3 headon_ablation_rerun_report.py <bags folder>
#   Reads:  <bags folder>/[a-e]_*_rr_t*/ and analysis/headon_ablation_AE_results.csv (August)
#   Writes: analysis/headon_ablation_rerun_report.txt and headon_ablation_rerun_results.csv (stdout + --out)
# ----------------------------------------------------------------------------------------------------
import sys, os, glob, csv, math, re, statistics, itertools, hashlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyse_avoidance as aa

BAGS = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else None
CONDS = [("A", "a_nolayer"), ("B", "b_critweight20"), ("C", "c_critweight10"),
         ("D", "d_critweight10_socialcritic_on"), ("E", "e_critweight20_socialcritic_on")]
METRICS = ["min_distance", "max_lateral_dev", "path_ratio", "commit_dist", "dev_at_min"]
lines = []
def P(s=""):
    print(s, flush=True); lines.append(s)

def mwu_exact(x, y):
    """Exact two-sided Mann-Whitney p value by enumeration (no ties assumed beyond mid-ranks)."""
    allv = sorted(x + y); n, m = len(x), len(y)
    def U(a, b): return sum((1.0 if i > j else 0.5 if i == j else 0.0) for i in a for j in b)
    u_obs = U(x, y); mid = n * m / 2.0; dev = abs(u_obs - mid); cnt = tot = 0
    for idx in itertools.combinations(range(n + m), n):
        s = set(idx); a = [allv[i] for i in idx]; b = [allv[i] for i in range(n + m) if i not in s]
        tot += 1
        if abs(U(a, b) - mid) >= dev - 1e-12: cnt += 1
    return u_obs, cnt / tot

def note(folder, key):
    try:
        for l in open(os.path.join(folder, "run_notes.txt")):
            if l.startswith(key): return l[len(key):].strip()
    except OSError: pass
    return "?"

def start_protocol(data):
    m2o = aa.extract_map_to_odom(data["/tf"] + data["/tf_static"])
    robot = aa.robot_path_in_map(data["/odom"], m2o); per = aa.person_track(data["/person_ground_truth"])
    def first_move(tr, thr=0.05):
        x0, y0 = tr[0][1], tr[0][2]
        for t, x, y in tr:
            if math.hypot(x - x0, y - y0) > thr: return t
    tr, tp = first_move(robot), first_move(per)
    at = lambda track, t: min(track, key=lambda r: abs(r[0] - t))
    rp = at(robot, tp)
    return robot[0][1:], robot[-1][1:], per[0][1:], tp - tr, math.hypot(rp[1] - at(per, tp)[1], rp[2] - at(per, tp)[2])

aug = {}
for r in csv.DictReader(open(os.path.join(HERE, "headon_ablation_AE_results.csv"))):
    for c, pre in CONDS:
        if re.fullmatch(pre + r"_t0\d", r["trial"]): aug.setdefault(c, []).append({k: float(r[k]) for k in METRICS})

P("Head-on ablation A-E, re-run of 13 Oct 2026 with the August state restored (workspace ablation_ws)")
P("Script: analysis/headon_ablation_rerun_report.py; metrics by analysis/analyse_avoidance.py, as for the August bags")
P()
P("1. Proof of the condition of every trial (from the files written at the trial)")
P("trial | launched Nav2 file (md5) | cloud node | live obstacle weight | live SocialCritic | cloud msgs in bag | fused msgs | goal | died")
rows = {}; csvrows = []; prot = []
for c, pre in CONDS:
    for d in sorted(glob.glob(os.path.join(BAGS, pre + "_rr_t*"))):
        name = os.path.basename(d)
        cfg = os.path.join(d, "config_used.yaml")
        md5 = hashlib.md5(open(cfg, "rb").read()).hexdigest()[:8] if os.path.exists(cfg) else "?"
        cfgname = os.path.basename(open(os.path.join(d, "config_path.txt")).read().strip())
        data = aa.read_bag(os.path.join(d, "data"))
        import yaml
        cnt = {t["topic_metadata"]["name"]: t["message_count"] for t in yaml.safe_load(open(os.path.join(d, "data", "metadata.yaml")))["rosbag2_bagfile_information"]["topics_with_message_count"]}
        ncloud = cnt.get("/predicted_person_cloud", 0); nfused = cnt.get("/fused_person_positions", 0)
        goal = note(d, "Goal result:").replace("Goal finished with status: ", "")
        P(f"{name} | {cfgname} ({md5}) | {note(d, 'Person cloud node started:').split()[0]} | "
          f"{note(d, 'Live controller, ObstaclesCritic.critical_weight:').split()[0]} | {note(d, 'Live controller, SocialCritic.enabled:').split()[0]} | "
          f"{ncloud} | {nfused} | {goal} | {note(d, 'Processes that died before the end:')}")
        rows.setdefault(c, []).append((name, data))
P()

# metrics through the script's own command-line path, so the numbers are computed exactly as in August
import subprocess, tempfile
tmp = tempfile.mktemp(suffix=".csv")
bags = [os.path.join(BAGS, n, "data") for c, _ in CONDS for n, _d in rows.get(c, [])]
subprocess.run([sys.executable, os.path.join(HERE, "analyse_avoidance.py"), *bags, "--csv", tmp], check=True, stdout=subprocess.DEVNULL)
new = {}; allrows = list(csv.DictReader(open(tmp)))
for r in allrows:
    for c, pre in CONDS:
        if r["trial"].startswith(pre + "_rr_t") and r["min_distance"] not in ("", "nan"): new.setdefault(c, []).append({**{k: float(r[k] or "nan") for k in METRICS}, "trial": r["trial"]})
if OUT:
    with open(OUT.replace("_report.txt", "_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=allrows[0].keys()); w.writeheader(); w.writerows(allrows)

P("2. Start of every trial (map frame)")
for c, _ in CONDS:
    for name, data in rows.get(c, []):
        try:
            rs, re_, ps, dt, gap = start_protocol(data); prot.append((rs, re_, dt, gap))
            P(f"{name}: robot start ({rs[0]:.2f},{rs[1]:.2f}) end ({re_[0]:.2f},{re_[1]:.2f}); walker from ({ps[0]:.2f},{ps[1]:.2f}), starts {dt:+.1f} s after the robot, gap then {gap:.2f} m")
        except Exception as e: P(f"{name}: start not readable ({e})")
if prot:
    f = lambda v: f"{statistics.mean(v):.2f} +/- {statistics.pstdev(v):.2f} (min {min(v):.2f}, max {max(v):.2f})"
    P(f"all {len(prot)} trials: robot start x {f([p[0][0] for p in prot])}, y {f([p[0][1] for p in prot])}; walker starts after robot (s) {f([p[2] for p in prot])}; gap (m) {f([p[3] for p in prot])}")
    P("August (30 trials): robot start x -1.88 +/- 0.03, y 0.03 +/- 0.03; walker starts after robot 2.38 +/- 1.04 s (min -0.21, max 6.23); gap 8.68 +/- 1.02 m")
P()
P("3. Results per condition (mean, SD with n-1, min, max), re-run and August")
for c, pre in CONDS:
    P(f"{c} ({pre})  re-run n={len(new.get(c, []))}, August n={len(aug.get(c, []))}")
    P("  per trial min_distance (re-run): " + ", ".join(f"{r['min_distance']:.3f}" for r in new.get(c, [])))
    P("  per trial min_distance (August): " + ", ".join(f"{r['min_distance']:.3f}" for r in aug.get(c, [])))
    for k in METRICS:
        s = f"  {k:16s}"
        for lab, src in (("re-run", new), ("August", aug)):
            v = [r[k] for r in src.get(c, []) if not math.isnan(r[k])]
            miss = len(src.get(c, [])) - len(v)
            if len(v) > 1: s += f"  {lab}: {statistics.mean(v):7.3f} +/- {statistics.stdev(v):.3f} ({min(v):.3f} - {max(v):.3f})" + (f" [n={len(v)}, {miss} without value]" if miss else "")
        P(s)
P()
P("4. Exact two-sided Mann-Whitney tests on min_distance")
md = lambda src, c: [r["min_distance"] for r in src.get(c, [])]
for a, b in (("A", "B"), ("B", "C"), ("C", "D"), ("B", "E"), ("D", "E")):
    if md(new, a) and md(new, b):
        u, p = mwu_exact(md(new, a), md(new, b)); P(f"re-run {a} vs {b}: U = {u:.1f}, p = {p:.4f}")
for c, _ in CONDS:
    if md(new, c) and md(aug, c):
        u, p = mwu_exact(md(new, c), md(aug, c)); P(f"condition {c}, re-run vs August: U = {u:.1f}, p = {p:.4f}")
if OUT: open(OUT, "w").write("\n".join(lines) + "\n")
