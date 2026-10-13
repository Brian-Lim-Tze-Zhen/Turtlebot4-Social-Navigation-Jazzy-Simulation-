#!/usr/bin/env python3
"""compare_chain.py <chain.csv> - replayed perception vs the robot's recorded perception (8 Oct 2026).
For each stage (ray, fused, predicted) pairs every recorded message with the replayed message of the same id
closest in time (within 0.15 s) and reports how many pair up and the position difference."""
import csv, math, sys, collections
rows = list(csv.DictReader(open(sys.argv[1])))
st = {'/camera_ray_clusters': '/bag_ray', '/person_positions_fused': '/bag_fused', '/predicted_person_positions': '/bag_pred'}
for rep, rec in st.items():
    R = collections.defaultdict(list)
    for r in rows:
        if r['topic'] == rep:
            f = r['data'].split(','); R[f[0]].append((float(r['t']), float(f[2]), float(f[3])))
    n = ok = 0; dif = []
    ids_rec, ids_rep = set(), set(R)
    for r in rows:
        if r['topic'] != rec:
            continue
        f = r['data'].split(','); t = float(r['t']); i = f[0]; ids_rec.add(i); n += 1
        c = min(R.get(i, []), key=lambda q: abs(q[0] - t), default=None)
        if c and abs(c[0] - t) <= 0.15:
            ok += 1; dif.append(math.hypot(c[1] - float(f[2]), c[2] - float(f[3])))
    dif.sort()
    q = lambda p: dif[int(p * (len(dif) - 1))] if dif else float('nan')
    print(f'{rep:28s} recorded {n:4d}  paired {ok:4d} ({100*ok/max(1,n):.0f} %)  position diff median {q(.5):.3f} m, 90% {q(.9):.3f}, max {dif[-1] if dif else float("nan"):.3f}'
          f'   ids recorded {sorted(ids_rec, key=lambda x: float(x))}  replayed {sorted(ids_rep, key=lambda x: float(x))}')
# --- id-free comparison: nearest replayed message in time AND position (ids differ when the robot node ran before the bag)
print('id-free pairing (same time within 0.15 s, nearest position):')
for rep, rec in list(st.items())[1:]:
    Rt = sorted((float(r['t']), r['data'].split(',')) for r in rows if r['topic'] == rep)
    import bisect
    ts = [x[0] for x in Rt]
    dif, idmap = [], collections.Counter()
    for r in rows:
        if r['topic'] != rec:
            continue
        f = r['data'].split(','); t = float(r['t'])
        lo, hi = bisect.bisect_left(ts, t - 0.15), bisect.bisect_right(ts, t + 0.15)
        c = min(Rt[lo:hi], key=lambda q: math.hypot(float(q[1][2]) - float(f[2]), float(q[1][3]) - float(f[3])), default=None)
        if c:
            dif.append(math.hypot(float(c[1][2]) - float(f[2]), float(c[1][3]) - float(f[3]))); idmap[(f[0], c[1][0])] += 1
    dif.sort(); q = lambda p: dif[int(p * (len(dif) - 1))] if dif else float('nan')
    print(f'  {rep:28s} paired {len(dif)}  diff median {q(.5):.3f}, 90% {q(.9):.3f}   id map (recorded->replayed: count) {dict(idmap.most_common(8))}')
