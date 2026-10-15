"""Results chart for the READMEs: light and dark PNG.

Data: ros2_ws/analysis/headon_F_corridor_n10_report.txt, headon_hwreq_z5_corr_n10_report.txt,
headon_rec20gs_n10_report.txt (min centre distance), unmapped_clearance_rec20gs_report.txt (simulation),
humble_client/workspace/pytest/analysis/side_20261009/compare_days_precise_rec20gs_12oct_result.txt (robot).
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HEADON = [
    ("Baseline\n(person cloud only)", [0.034, 0.177, 0.029, 0.042, 0.008, 0.158, 0.008, 0.144, 0.164, 0.077], False),
    ("Lane rules", [0.891, 0.845, 0.804, 0.828, 0.804, 0.864, 0.878, 0.799, 0.901, 0.792], False),
    ("Final configuration", [0.742, 0.681, 0.756, 0.679, 0.839, 0.696, 0.791, 0.812, 0.719, 0.858], True),
]
SIM = {
    "Head-on walker": [0.517, 0.473, 0.527, 0.461, 0.617, 0.475, 0.570, 0.590, 0.493, 0.647],
    "Pair, wide": [0.712, 0.765, 0.736, 0.779, 0.809, 0.779, 0.751, 0.806, 0.748, 0.763],
    "Pair, narrow": [0.570, 0.615, 0.639, 0.608, 0.620, 0.583, 0.587, 0.623, 0.608, 0.586],
}
ROBOT = {
    "Head-on walker": [0.584, 0.688, 0.894, 0.588, 0.868, 0.670, 0.802, 0.498, 0.646, 0.447],
    "Pair, wide": [0.804, 0.828, 0.772, 0.820, 0.798, 0.752, 0.760, 0.768, 0.766, 0.588],
    "Pair, narrow": [0.490, 0.560, 0.592, 0.568, 0.530, 0.473, 0.584, 0.612, 0.552, 0.596],
}

THEMES = {
    "light": dict(surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", muted="#898781", grid="#e1e0d9",
                  axis="#c3c2b7", s1="#2a78d6", s2="#eb6834"),
    "dark": dict(surface="#1a1a19", ink="#f0efec", ink2="#c3c2b7", muted="#898781", grid="#2c2c2a",
                 axis="#383835", s1="#3987e5", s2="#d95926"),
}


def draw(mode, out):
    t = THEMES[mode]
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10.5})
    fig, (a, b) = plt.subplots(1, 2, figsize=(12.4, 4.6), facecolor=t["surface"],
                               gridspec_kw=dict(wspace=0.42))
    rng = np.random.default_rng(3)
    for ax in (a, b):
        ax.set_facecolor(t["surface"])
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color(t["axis"])
        ax.tick_params(colors=t["ink2"], length=0)
        ax.xaxis.grid(True, color=t["grid"], linewidth=0.8)
        ax.set_axisbelow(True)

    # A: head-on, closest distance between centres, simulation
    for i, (name, vals, emph) in enumerate(HEADON):
        y = len(HEADON) - 1 - i
        c = t["s1"] if emph else t["muted"]
        jit = rng.uniform(-0.13, 0.13, len(vals))
        a.scatter(vals, y + jit, s=58, color=c, edgecolors=t["surface"], linewidths=1.6, zorder=3)
        m = float(np.mean(vals))
        a.plot([m, m], [y - 0.28, y + 0.28], color=t["ink"], linewidth=2, zorder=4)
        a.text(m, y + 0.34, f"{m:.2f}", ha="center", va="bottom", color=t["ink"], fontsize=10)
    for x, label in ((0.44, "contact 0.44 m"), (0.80, "target 0.8 m")):
        a.axvline(x, color=t["ink2"], linewidth=1, zorder=2)
        a.text(x + 0.012, len(HEADON) - 0.32, label, ha="left", va="bottom", color=t["ink2"], fontsize=9.5)
    a.set_yticks(range(len(HEADON)))
    a.set_yticklabels([h[0] for h in reversed(HEADON)], color=t["ink"])
    a.set_xlim(-0.02, 1.0)
    a.set_ylim(-0.6, len(HEADON) - 0.05)
    a.set_xlabel("closest distance between centres (m)", color=t["ink2"])
    a.set_title("Head-on walker in simulation, 10 trials each", loc="left", color=t["ink"], fontsize=12, pad=26)

    # B: final configuration, simulation vs robot, same measure
    cases = list(SIM)
    for i, case in enumerate(cases):
        y = len(cases) - 1 - i
        for off, data, c in ((0.17, SIM, t["s1"]), (-0.17, ROBOT, t["s2"])):
            vals = data[case]
            jit = rng.uniform(-0.07, 0.07, len(vals))
            b.scatter(vals, y + off + jit, s=58, color=c, edgecolors=t["surface"], linewidths=1.6, zorder=3)
            m = float(np.mean(vals))
            b.plot([m, m], [y + off - 0.15, y + off + 0.15], color=t["ink"], linewidth=2, zorder=4)
    # direct labels on the top case, legend for the rest
    top = len(cases) - 1
    b.text(max(SIM[cases[0]]) + 0.03, top + 0.17, "simulation", va="center", color=t["ink2"], fontsize=9.5)
    b.text(max(ROBOT[cases[0]]) + 0.03, top - 0.17, "robot", va="center", color=t["ink2"], fontsize=9.5)
    h1 = b.scatter([], [], s=58, color=t["s1"], label="simulation")
    h2 = b.scatter([], [], s=58, color=t["s2"], label="robot")
    leg = b.legend(handles=[h1, h2], loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=2, frameon=False,
                   fontsize=9.5, handletextpad=0.3, columnspacing=1.2, borderaxespad=0.2)
    for txt in leg.get_texts():
        txt.set_color(t["ink2"])
    b.set_yticks(range(len(cases)))
    b.set_yticklabels(list(reversed(cases)), color=t["ink"])
    b.set_xlim(0.4, 1.0)
    b.set_ylim(-0.55, len(cases) - 0.45)
    b.set_xlabel("nearest LiDAR return not on the map (m)", color=t["ink2"])
    b.set_title("Final configuration, simulation and robot, 10 runs each", loc="left", color=t["ink"],
                fontsize=12, pad=26)

    fig.text(0.01, 0.005, "Dots: single runs. Bars: mean.", color=t["muted"], fontsize=9)
    fig.savefig(out, dpi=160, bbox_inches="tight", facecolor=t["surface"])


if __name__ == "__main__":
    draw("light", sys.argv[1])
    draw("dark", sys.argv[2])
