#!/usr/bin/env python3
# ----------------------------------------------------------------------------------------------------
# What this script is for (plain words):
#   Turns the per-trial lines of headon_ablation_onset*.txt and headon_onset_profile*.txt into the
#   per-condition numbers quoted in Sections 4.1.2 and 7.2: where the plan bent, where the robot began
#   to turn, the commit distance, and the heading off the line at 5 m and 3 m.
#   Run on the host:  python3 ablation_onset_summary.py <onset.txt> <profile.txt> [rr]
#   With "rr" only the re-run trials (*_rr_t*) are used, otherwise only the first-run trials.
# ----------------------------------------------------------------------------------------------------
import sys, re, statistics as st
on_f, pr_f = sys.argv[1], sys.argv[2]; rr = sys.argv[3:] == ["rr"]
COND = {"a_nolayer": "A", "b_critweight20": "B", "c_critweight10": "C", "d_critweight10_socialcritic_on": "D", "e_critweight20_socialcritic_on": "E"}
def cond(name):
    m = re.match(r"^([a-e]_[a-z0-9_]+?)_(rr_)?t0\d", name)
    if not m or bool(m.group(2)) != rr or "_1.2_" in name: return None
    return COND.get(m.group(1))
def ms(v): return f"{st.mean(v):.2f} +/- {st.stdev(v):.2f} (min {min(v):.2f}, max {max(v):.2f}, n={len(v)})" if len(v) > 1 else str(v)
on = {}
for l in open(on_f):
    f = l.split()
    if len(f) >= 7 and f[1] == "plan" and cond(f[0]):
        val = lambda s: None if s == "-" else float(s)
        on.setdefault(cond(f[0]), []).append((f[0], val(f[2]), val(f[4]), val(f[6])))
GAPS = None; pr = {}
for l in open(pr_f):
    if l.startswith("bag"): GAPS = [int(g) for g in re.findall(r"gap\s+(\d+)m", l)]; continue
    f = l.split()
    if f and cond(f[0]) and GAPS:
        vals = re.findall(r"([+-]\d+\.\d+)\s+([+-]\d+)°|        -          ", l[36:])
        row = {}
        for g, m in zip(GAPS, re.finditer(r"([+-]\d+\.\d+)\s+([+-]\d+)°|\s{8}-\s{10}", l[36:])):
            if m.group(1): row[g] = (float(m.group(1)), abs(int(m.group(2))))
        pr.setdefault(cond(f[0]), []).append((f[0], row))
print("Trials:", "re-run (*_rr_t*)" if rr else "first run")
for c in "ABCDE":
    r = on.get(c, [])
    print(f"\n{c}: n = {len(r)}")
    for k, i in (("plan bent at gap (m)", 1), ("turn began at gap (m)", 2), ("commit (0.20 m off line) at gap (m)", 3)):
        v = [x[i] for x in r if x[i] is not None]
        print(f"  {k}: {ms(v)}; missing {len(r) - len(v)}" if v else f"  {k}: none reached")
    for g in (5, 3, 2):
        v = [row[g][1] for _, row in pr.get(c, []) if g in row]
        if v: print(f"  heading off the line at {g} m (deg): {ms(v)}")
    print("  per trial: " + "; ".join(f"{n.split('_t0')[-1]}: plan {p} turn {t} commit {m}" for n, p, t, m in r))
