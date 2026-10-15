"""Analysis charts for the READMEs, each in a light and a dark version.

  python3 make_results_chart.py <output folder>

writes headon_light.png, headon_dark.png, sim_vs_robot_light.png, sim_vs_robot_dark.png.

Data (10 runs each):
  head-on, simulation: min centre distance in ros2_ws/analysis/headon_F_corridor_n10_report.txt,
    headon_hwreq_z5_corr_n10_report.txt, headon_rec20gs_n10_report.txt
  simulation vs robot: ros2_ws/analysis/unmapped_clearance_rec20gs_report.txt and, in the robot repository,
    humble_client/workspace/pytest/analysis/side_20261009/compare_days_precise_rec20gs_12oct_result.txt
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HEADON = [
    ("No lane rules", [0.034, 0.177, 0.029, 0.042, 0.008, 0.158, 0.008, 0.144, 0.164, 0.077], False),
    ("Lane rules", [0.891, 0.845, 0.804, 0.828, 0.804, 0.864, 0.878, 0.799, 0.901, 0.792], False),
    ("Final configuration", [0.742, 0.681, 0.756, 0.679, 0.839, 0.696, 0.791, 0.812, 0.719, 0.858], True),
]
SIM = {
    "Walker coming towards the robot": [0.517, 0.473, 0.527, 0.461, 0.617, 0.475, 0.570, 0.590, 0.493, 0.647],
    "Standing pair, wide hallway": [0.712, 0.765, 0.736, 0.779, 0.809, 0.779, 0.751, 0.806, 0.748, 0.763],
    "Standing pair, narrow section": [0.570, 0.615, 0.639, 0.608, 0.620, 0.583, 0.587, 0.623, 0.608, 0.586],
}
ROBOT = {
    "Walker coming towards the robot": [0.584, 0.688, 0.894, 0.588, 0.868, 0.670, 0.802, 0.498, 0.646, 0.447],
    "Standing pair, wide hallway": [0.804, 0.828, 0.772, 0.820, 0.798, 0.752, 0.760, 0.768, 0.766, 0.588],
    "Standing pair, narrow section": [0.490, 0.560, 0.592, 0.568, 0.530, 0.473, 0.584, 0.612, 0.552, 0.596],
}

THEMES = {
    "light": dict(surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", muted="#898781", grid="#e1e0d9",
                  axis="#c3c2b7", rest="#c3c2b7", s1="#2a78d6", s2="#eb6834"),
    "dark": dict(surface="#1a1a19", ink="#f0efec", ink2="#c3c2b7", muted="#898781", grid="#2c2c2a",
                 axis="#383835", rest="#52514e", s1="#3987e5", s2="#d95926"),
}


def style(ax, t):
    ax.set_facecolor(t["surface"])
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(t["axis"])
    ax.tick_params(colors=t["ink2"], length=0, labelsize=10.5)
    ax.xaxis.grid(True, color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)


def titles(fig, t, title, subtitle):
    fig.text(0.012, 0.97, title, color=t["ink"], fontsize=13.5, weight="bold", va="top")
    fig.text(0.012, 0.885, subtitle, color=t["ink2"], fontsize=10.5, va="top")


def headon(mode, out):
    t = THEMES[mode]
    fig, ax = plt.subplots(figsize=(9.2, 3.7), facecolor=t["surface"])
    fig.subplots_adjust(left=0.2, right=0.97, top=0.74, bottom=0.17)
    style(ax, t)
    for i, (name, vals, final) in enumerate(HEADON):
        y = len(HEADON) - 1 - i
        m = float(np.mean(vals))
        ax.barh(y, m, height=0.52, color=t["s1"] if final else t["rest"], zorder=3)
        note = "  robot touches the walker" if m < 0.44 else ""
        ax.text(m + 0.012, y, f"{m:.2f} m{note}", va="center", color=t["ink"], fontsize=11, zorder=4,
                bbox=dict(facecolor=t["surface"], edgecolor="none", pad=1.5))
    ax.axvline(0.44, color=t["ink2"], linewidth=1.2, zorder=2)
    ax.text(0.452, len(HEADON) - 0.45, "touching (0.44 m)", color=t["ink2"], fontsize=9.5, va="bottom")
    ax.set_yticks(range(len(HEADON)))
    ax.set_yticklabels([h[0] for h in reversed(HEADON)], color=t["ink"], fontsize=11)
    ax.set_xlim(0, 1.05)
    ax.set_ylim(-0.5, len(HEADON) - 0.15)
    ax.set_xlabel("distance between the centres of robot and person (m)", color=t["ink2"], fontsize=10)
    titles(fig, t, "With the lane rules, the robot moves aside for a walker",
           "Simulation, 2.5 m corridor, walker at 1.2 m/s. Closest distance, mean of 10 trials.")
    fig.savefig(out, dpi=160, facecolor=t["surface"])
    plt.close(fig)


def sim_vs_robot(mode, out):
    t = THEMES[mode]
    fig, ax = plt.subplots(figsize=(9.2, 3.7), facecolor=t["surface"])
    fig.subplots_adjust(left=0.29, right=0.97, top=0.74, bottom=0.17)
    style(ax, t)
    cases = list(SIM)
    for i, case in enumerate(cases):
        y = len(cases) - 1 - i
        ms, mr = float(np.mean(SIM[case])), float(np.mean(ROBOT[case]))
        lo, hi = sorted([(ms, "simulation"), (mr, "robot")])
        same = abs(ms - mr) < 0.01
        if same:   # equal means: one above the other, so neither dot hides the other
            for m, c, dy in ((ms, t["s1"], 0.13), (mr, t["s2"], -0.13)):
                ax.scatter([m], [y + dy], s=150, color=c, edgecolors=t["surface"], linewidths=2, zorder=3)
            ax.text(max(ms, mr) + 0.014, y, f"{ms:.2f}, the same", va="center", color=t["ink"],
                    fontsize=11)
        else:
            ax.plot([ms, mr], [y, y], color=t["axis"], linewidth=2, zorder=2)
            for m, c in ((ms, t["s1"]), (mr, t["s2"])):
                ax.scatter([m], [y], s=150, color=c, edgecolors=t["surface"], linewidths=2, zorder=3)
            ax.text(lo[0] - 0.012, y, f"{lo[0]:.2f}", va="center", ha="right", color=t["ink"], fontsize=11)
            ax.text(hi[0] + 0.012, y, f"{hi[0]:.2f}", va="center", ha="left", color=t["ink"], fontsize=11)
    h1 = ax.scatter([], [], s=110, color=t["s1"], label="simulation")
    h2 = ax.scatter([], [], s=110, color=t["s2"], label="robot")
    leg = ax.legend(handles=[h1, h2], loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, frameon=False,
                    fontsize=10.5, handletextpad=0.3, columnspacing=1.2, borderaxespad=0.1)
    for txt in leg.get_texts():
        txt.set_color(t["ink2"])
    ax.set_yticks(range(len(cases)))
    ax.set_yticklabels(list(reversed(cases)), color=t["ink"], fontsize=11)
    ax.set_xlim(0.4, 0.9)
    ax.set_ylim(-0.5, len(cases) - 0.5)
    ax.set_xlabel("distance from the LiDAR to the nearest person (m)", color=t["ink2"], fontsize=10)
    titles(fig, t, "The robot matches the simulation for the pair, and passes the walker wider",
           "Final configuration, measured the same way on both. Closest distance, mean of 10 runs.")
    fig.savefig(out, dpi=160, facecolor=t["surface"])
    plt.close(fig)


if __name__ == "__main__":
    plt.rcParams["font.family"] = "DejaVu Sans"
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    for mode in ("light", "dark"):
        headon(mode, os.path.join(d, f"headon_{mode}.png"))
        sim_vs_robot(mode, os.path.join(d, f"sim_vs_robot_{mode}.png"))
