"""
analyze_data.py  --  hypothesis tests + figures from the batch runs

Reads:   data_outputs/conflicts.csv, summary_*.csv, vehicles_*.csv   (from run_batch.py + parse_ssm.py)
Writes:  data_outputs/safety_results.png   (TTC distribution + critical conflicts per run)
         data_outputs/tti_matrix.png       (speed vs TTI, one dot per vehicle, dynamic runs)
         data_outputs/stats_summary.txt    (every number for the paper)
         data_outputs/per_run_metrics.csv  (one row per run)

Only FREE-FLOW conflicts are used (closest approach while the left-turner is crossing and the oncoming car
is moving >= 8 m/s). See parse_ssm.py for why.

H1: mean TTC is higher under the sensor-actuated controller than under static FYA
H2: critical conflicts per run are lower under the controller
Unit of analysis = one simulation run. Baseline and dynamic runs share a seed, so they are matched pairs.

CRIT_METRIC: which measure defines a "critical" conflict.
  "TTC"  the original definition in the notes (TTC <= 1.5 s). In this model it produces ~zero events, so H2 has
         nothing to compare.
  "PET"  post-encroachment time <= 1.5 s (the other surrogate measure in the notes).
Both, plus TTC <= 3 s, are always reported in stats_summary.txt.
"""
import glob
import os
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from scipy import stats

OUT = "data_outputs"
CRIT = 1.5              # s
CRIT_METRIC = "PET"     # "PET" or "TTC"  (see docstring)
TAU = 6.0               # s   controller threshold used in the runs (only for the TTI figure label)

THEME = {
    "bg": "#0B0D12", "panel": "#11141B", "grid": "#232834", "text": "#E6E8EE",
    "muted": "#8B93A7", "base": "#FBBF24", "dyn": "#5EEAD4", "hazard": "#FB7185",
}
COL = {"baseline": THEME["base"], "dynamic": THEME["dyn"], "protected": "#C4A1FF"}
LABEL = {"baseline": "Static FYA", "dynamic": "Sensor-actuated", "protected": "Protected-only"}


def apply_theme():
    installed = {f.name for f in font_manager.fontManager.ttflist}
    font = next((f for f in ["Inter", "SF Pro Display", "Helvetica Neue", "Arial"] if f in installed), "DejaVu Sans")
    plt.rcParams.update({
        "font.family": font, "figure.facecolor": THEME["bg"], "axes.facecolor": THEME["bg"],
        "axes.edgecolor": THEME["grid"], "axes.labelcolor": THEME["muted"],
        "xtick.color": THEME["muted"], "ytick.color": THEME["muted"], "text.color": THEME["text"],
        "savefig.facecolor": THEME["bg"],
    })


def style_axes(ax):
    ax.grid(True, color=THEME["grid"], linewidth=0.7, alpha=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.tick_params(length=0, labelsize=9.5)


def stat_card(fig, x, y, w, h, value, label, color):
    ax = fig.add_axes([x, y, w, h])
    ax.set_axis_off()
    ax.add_patch(Rectangle((0, 0), 1, 1, transform=ax.transAxes, facecolor=THEME["panel"],
                           edgecolor=THEME["grid"], linewidth=1))
    ax.text(0.07, 0.58, value, transform=ax.transAxes, fontsize=16, fontweight="bold", color=color, va="center")
    ax.text(0.07, 0.20, label.upper(), transform=ax.transAxes, fontsize=7.2, color=THEME["muted"], va="center")


def header(fig, title, subtitle):
    fig.text(0.07, 0.94, title, fontsize=19, fontweight="bold", va="center")
    fig.text(0.07, 0.90, subtitle, fontsize=11, color=THEME["muted"], va="center")


def glow(ax, x, y, color, size, alpha=0.85, z=3):
    ax.scatter(x, y, s=size * 5, color=color, alpha=0.06, linewidths=0, zorder=z)
    ax.scatter(x, y, s=size * 2, color=color, alpha=0.12, linewidths=0, zorder=z + 1)
    ax.scatter(x, y, s=size, color=color, alpha=alpha, linewidths=0, zorder=z + 2)


def fp(p):
    """Format a p-value for the paper."""
    return "n/a" if p != p else ("< 0.0001" if p < 0.0001 else f"= {p:.4f}")


def dz(a, b):
    """Cohen's d_z for paired data (mean difference / sd of differences)."""
    d = np.asarray(a) - np.asarray(b)
    return d.mean() / d.std(ddof=1) if len(d) > 1 and d.std(ddof=1) > 0 else float("nan")


def load_runs():
    runs = []
    for p in glob.glob(os.path.join(OUT, "summary_*_*.csv")):
        m = re.match(r"summary_(baseline|dynamic|protected)_(\d+)\.csv", os.path.basename(p))
        if m:
            s = pd.read_csv(p).iloc[0].to_dict()
            s.update(mode=m.group(1), seed=int(m.group(2)))
            runs.append(s)
    return pd.DataFrame(runs)


def per_run_table(conf, runs):
    f = conf[conf["free_flow"] == 1]
    g = f.groupby(["mode", "seed"])
    t = pd.DataFrame({
        "mean_ttc": g["TTC"].mean(),
        "n_conf": g.size(),
        "ttc15": g["TTC"].apply(lambda s: (s <= 1.5).sum()),
        "ttc3": g["TTC"].apply(lambda s: (s <= 3.0).sum()),
        "pet15": g["PET"].apply(lambda s: (s <= 1.5).sum()),
    }).reset_index()
    out = runs.merge(t, how="left", on=["mode", "seed"])
    for c in ("n_conf", "ttc15", "ttc3", "pet15"):
        out[c] = out[c].fillna(0)                     # a run with no such conflict counts as 0
    return out


def paired(per_run, col, a="baseline", c="dynamic"):
    """Values of condition a and condition c for the seeds present in both."""
    if col not in per_run.columns:
        return pd.Series(dtype=float), pd.Series(dtype=float)
    b = per_run[per_run["mode"] == a].set_index("seed")[col]
    d = per_run[per_run["mode"] == c].set_index("seed")[col]
    both = b.index.intersection(d.index)
    return b.loc[both], d.loc[both]


def test_block(per_run, col, higher_is_better, two_sided=False, a="baseline", c="dynamic"):
    b, d = paired(per_run, col, a, c)
    ok = b.notna() & d.notna()
    b, d = b[ok], d[ok]
    if len(b) < 2:
        return dict(n=len(b), base=float("nan"), dyn=float("nan"), t=float("nan"), p_t=float("nan"),
                    p_w=float("nan"), dz=float("nan"))
    alt = "two-sided" if two_sided else ("greater" if higher_is_better else "less")
    t, p_t = stats.ttest_rel(d, b, alternative=alt)
    try:
        _, p_w = stats.wilcoxon(d, b, alternative=alt)
    except ValueError:                                 # all differences are zero
        p_w = float("nan")
    return dict(n=len(b), base=b.mean(), dyn=d.mean(), t=t, p_t=p_t, p_w=p_w, dz=dz(d, b))


def results_figure(conf, per_run, st_h1, st_h2, crit_label, crit_col):
    fig = plt.figure(figsize=(12, 7.2), dpi=100)
    header(fig, "Dynamic Intersection Controller",
           "Left-turn conflicts with free-flowing oncoming traffic at the blind crest: static FYA vs. sensor-actuated")
    cards = [
        (f"{st_h1['base']:.2f} s", "Mean TTC · static FYA", THEME["base"]),
        (f"{st_h1['dyn']:.2f} s", "Mean TTC · sensor-actuated", THEME["dyn"]),
        (f"{st_h2['base']:.1f}", f"{crit_label} / run · static", THEME["base"]),
        (f"{st_h2['dyn']:.1f}", f"{crit_label} / run · sensor", THEME["dyn"]),
    ]
    cw, gap, x0 = 0.205, 0.02, 0.07
    for i, (v, l, c) in enumerate(cards):
        stat_card(fig, x0 + i * (cw + gap), 0.78, cw, 0.085, v, l, c)

    ax1 = fig.add_axes([0.07, 0.15, 0.42, 0.56])
    f = conf[conf["free_flow"] == 1]
    for mode in ("baseline", "dynamic"):
        v = np.sort(f.loc[f["mode"] == mode, "TTC"].dropna().values)
        v = v[v <= 6.0]
        if len(v):
            ax1.step(v, np.arange(1, len(v) + 1) / len(v), where="post", color=COL[mode], linewidth=2.2, label=LABEL[mode])
    ax1.axvline(CRIT, color=THEME["hazard"], linestyle=(0, (5, 4)), linewidth=1.3)
    ax1.text(CRIT + 0.05, 0.97, f"{CRIT}s", color=THEME["hazard"], fontsize=9, transform=ax1.get_xaxis_transform(), va="top")
    ax1.set_xlim(1, 6)
    ax1.set_xlabel("Time-to-collision of each conflict (s)", fontsize=10.5, labelpad=8)
    ax1.set_ylabel("Share of conflicts at or below TTC", fontsize=10.5, labelpad=8)
    ax1.set_title("Distribution of conflict severity", loc="left", fontsize=11, color=THEME["text"], pad=10)
    style_axes(ax1)
    ax1.legend(frameon=False, loc="lower right", fontsize=9.5)

    ax2 = fig.add_axes([0.58, 0.15, 0.37, 0.56])
    b, d = paired(per_run, crit_col)
    for s in b.index:
        ax2.plot([0, 1], [b[s], d[s]], color=THEME["grid"], linewidth=0.8, zorder=1)
    rng = np.random.default_rng(1)
    glow(ax2, rng.uniform(-0.04, 0.04, len(b)), b.values, COL["baseline"], 26)
    glow(ax2, 1 + rng.uniform(-0.04, 0.04, len(d)), d.values, COL["dynamic"], 26)
    for i, vals in enumerate((b, d)):
        ax2.hlines(vals.mean(), i - 0.22, i + 0.22, color=COL["baseline" if i == 0 else "dynamic"], linewidth=2.8, zorder=6)
    ax2.set_xticks([0, 1])
    ax2.set_xticklabels([LABEL["baseline"], LABEL["dynamic"]], fontsize=10.5)
    ax2.set_xlim(-0.5, 1.5)
    ax2.set_ylabel(f"Conflicts with {crit_label} per run", fontsize=10.5, labelpad=8)
    ax2.set_title("Critical conflicts per run (lines join matching seeds)", loc="left", fontsize=11, color=THEME["text"], pad=10)
    style_axes(ax2)

    note = (f"H1  paired t = {st_h1['t']:.2f}, p {fp(st_h1['p_t'])}, d_z = {st_h1['dz']:.2f}      "
            f"H2  Wilcoxon p {fp(st_h2['p_w'])}, paired t p {fp(st_h2['p_t'])}      n = {st_h2['n']} paired runs")
    fig.text(0.07, 0.03, note, fontsize=9, color=THEME["muted"])
    fig.savefig(os.path.join(OUT, "safety_results.png"), dpi=300)
    plt.close(fig)


def tti_figure():
    files = glob.glob(os.path.join(OUT, "vehicles_dynamic_*.csv"))
    if not files:
        return
    df = pd.concat((pd.read_csv(f) for f in files), ignore_index=True)
    fig = plt.figure(figsize=(12, 7.2), dpi=100)
    header(fig, "Speed vs. time-to-intersection at the upstream sensor",
           "One dot per vehicle, first detection · dynamic runs · log scale")
    ax = fig.add_axes([0.07, 0.11, 0.89, 0.72])
    ok, hit = df[df["Override_Triggered"] == 0], df[df["Override_Triggered"] == 1]
    ax.axhspan(0.5, TAU, color=THEME["hazard"], alpha=0.07, zorder=1)
    glow(ax, ok["Speed_MPH"], ok["TTI"], THEME["dyn"], 14, 0.55)
    glow(ax, hit["Speed_MPH"], hit["TTI"], THEME["hazard"], 20, 0.85)
    ax.axhline(TAU, color=THEME["base"], linestyle=(0, (6, 4)), linewidth=1.4, zorder=2)
    ax.set_yscale("log")
    ax.set_ylim(0.5, max(df["TTI"].max() * 1.2, 20))
    ax.text(ax.get_xlim()[0] + 0.5, TAU * 1.06, f"τ = {TAU}s override threshold", color=THEME["base"], fontsize=9)
    ax.set_xlabel("Oncoming approach speed at the sensor (MPH)", fontsize=10.5, labelpad=8)
    ax.set_ylabel("Time-to-intersection (s, log)", fontsize=10.5, labelpad=8)
    style_axes(ax)
    fig.savefig(os.path.join(OUT, "tti_matrix.png"), dpi=300)
    plt.close(fig)


def reference_figure(per_run, delay_sub):
    """Three-way comparison: static FYA vs sensor-actuated vs protected-only."""
    modes = [m for m in ("baseline", "dynamic", "protected") if m in set(per_run["mode"])]
    fig = plt.figure(figsize=(12, 7.2), dpi=100)
    header(fig, "Is the controller better than just removing the permissive left?",
           "Safety per run over 50 matched seeds (left) and what it costs in delay (right; other-vehicle delay from 10 seeds)")
    rng = np.random.default_rng(2)
    ax1 = fig.add_axes([0.07, 0.14, 0.40, 0.68])
    for i, m in enumerate(modes):
        y = per_run.loc[per_run["mode"] == m, "pet15"].values
        glow(ax1, i + rng.uniform(-0.15, 0.15, len(y)), y, COL[m], 24)
        ax1.hlines(y.mean(), i - 0.28, i + 0.28, color=COL[m], linewidth=3, zorder=8)
        ax1.text(i, y.max() + 0.6, f"{y.mean():.1f}", ha="center", color=COL[m], fontsize=11, fontweight="bold")
    ax1.set_xticks(range(len(modes)))
    ax1.set_xticklabels([LABEL[m] for m in modes], fontsize=10)
    ax1.set_xlim(-0.6, len(modes) - 0.4)
    ax1.set_ylabel(f"Conflicts with PET ≤ {CRIT}s per run", fontsize=10.5, labelpad=8)
    ax1.set_title("Safety (lower is better)", loc="left", fontsize=11, color=THEME["text"], pad=10)
    style_axes(ax1)

    ax2 = fig.add_axes([0.57, 0.14, 0.38, 0.68])
    rows = [("Left-turners", "mean_left_timeloss_s", per_run)]
    if delay_sub is not None and len(delay_sub):
        seeds_sub = delay_sub["seed"].unique()
        other = pd.concat([delay_sub, per_run[(per_run["mode"] == "protected") & per_run["seed"].isin(seeds_sub)]],
                          ignore_index=True)
        rows.append((f"Other vehicles ({len(seeds_sub)} seeds)", "mean_oncoming_timeloss_s", other))
    width = 0.26
    for r, (name, col, df) in enumerate(rows):
        for i, m in enumerate(modes):
            v = df.loc[df["mode"] == m, col].dropna()
            if len(v):
                ax2.bar(r + (i - (len(modes) - 1) / 2) * width, v.mean(), width * 0.9, color=COL[m], alpha=0.9,
                        label=LABEL[m] if r == 0 else None)
                ax2.text(r + (i - (len(modes) - 1) / 2) * width, v.mean() + 0.8, f"{v.mean():.0f}s", ha="center",
                         fontsize=9, color=THEME["text"])
    ax2.set_xticks(range(len(rows)))
    ax2.set_xticklabels([r[0] for r in rows], fontsize=10.5)
    ax2.set_ylabel("Mean delay per vehicle (s, time loss)", fontsize=10.5, labelpad=8)
    ax2.set_title("Cost (lower is better)", loc="left", fontsize=11, color=THEME["text"], pad=10)
    style_axes(ax2)
    ax2.legend(frameon=False, loc="upper right", fontsize=9)
    fig.savefig(os.path.join(OUT, "reference_comparison.png"), dpi=300)
    plt.close(fig)


def reference_lines(per_run, delay_sub):
    out = ["", "=" * 60, "REFERENCE: PROTECTED-ONLY LEFT TURNS (same cycle length, no permissive left)", "=" * 60]
    if "protected" not in set(per_run["mode"]):
        return out + ["(no protected runs found)"]
    n_prot = int((per_run["mode"] == "protected").sum())
    out.append(f"Protected runs: {n_prot}")
    for name, col, hib in (("PET <= 1.5 s per run", "pet15", False), ("TTC <= 3 s per run", "ttc3", False),
                           ("mean TTC", "mean_ttc", True)):
        row = {}
        for a, c in (("baseline", "protected"), ("dynamic", "protected")):
            row[(a, c)] = test_block(per_run, col, hib, two_sided=True, a=a, c=c)
        b = row[("baseline", "protected")]
        d = row[("dynamic", "protected")]
        if name == "mean TTC":
            sub = per_run.groupby("mode")["mean_ttc"].mean()
            out.append(f"{name:22s} static {sub.get('baseline', float('nan')):6.2f} | sensor-actuated {sub.get('dynamic', float('nan')):6.2f} | "
                       f"protected: no free-flow conflicts in {int(per_run.loc[per_run['mode'] == 'protected', 'mean_ttc'].isna().sum())} of {n_prot} runs, so no mean to compare")
            continue
        out.append(f"{name:22s} static {b['base']:6.2f} | sensor-actuated {d['base']:6.2f} | protected {b['dyn']:6.2f}")
        out.append(f"{'':22s} sensor vs protected: two-sided Wilcoxon p {fp(d['p_w'])}   (paired t p {fp(d['p_t'])})")
        out.append(f"{'':22s} static vs protected: two-sided Wilcoxon p {fp(b['p_w'])}")
    for name, col in (("left-turner delay (s)", "mean_left_timeloss_s"),):
        vals = {m: per_run.loc[per_run["mode"] == m, col].mean() for m in ("baseline", "dynamic", "protected")}
        dp = test_block(per_run, col, False, two_sided=True, a="dynamic", c="protected")
        out.append(f"{name:22s} static {vals['baseline']:6.1f} | sensor-actuated {vals['dynamic']:6.1f} | protected {vals['protected']:6.1f}"
                   f"   (sensor vs protected p {fp(dp['p_t'])})")
    if delay_sub is not None and len(delay_sub):
        sub = delay_sub.groupby("mode")["mean_oncoming_timeloss_s"].mean()
        seeds = sorted(delay_sub["seed"].unique())
        prot = per_run[(per_run["mode"] == "protected") & per_run["seed"].isin(seeds)]["mean_oncoming_timeloss_s"].mean()
        out.append(f"{'other vehicles delay (s)':22s} static {sub.get('baseline', float('nan')):6.1f} | sensor-actuated "
                   f"{sub.get('dynamic', float('nan')):6.1f} | protected {prot:6.1f}   (seeds {seeds[0]}-{seeds[-1]}, {len(seeds)} runs each)")
    return out


def main():
    apply_theme()
    conf = pd.read_csv(os.path.join(OUT, "conflicts.csv"))
    runs = load_runs()
    per_run = per_run_table(conf, runs)

    h1 = test_block(per_run, "mean_ttc", higher_is_better=True)
    crit_col = {"TTC": "ttc15", "PET": "pet15"}[CRIT_METRIC]
    crit_label = f"{CRIT_METRIC} ≤ {CRIT}s"
    h2 = test_block(per_run, crit_col, higher_is_better=False)
    extras = {name: test_block(per_run, col, higher_is_better=False)
              for name, col in (("TTC <= 1.5 s", "ttc15"), ("PET <= 1.5 s", "pet15"), ("TTC <= 3 s", "ttc3"))}
    delay = test_block(per_run, "mean_left_timeloss_s", higher_is_better=False, two_sided=True)
    red = per_run.groupby("mode")["red_seconds"].mean()
    ov = per_run.groupby("mode")["overrides"].mean()
    coll = per_run.groupby("mode")["collisions"].sum()
    n_hidden = per_run.groupby("mode")["exposed_to_hidden_car"].mean()

    lines = [
        f"Paired runs (same seed in both modes): {h2['n']}",
        "Conflicts counted: free-flow only (crossing in progress, oncoming speed >= 8 m/s)",
        "",
        f"H1  mean TTC per run: static {h1['base']:.3f} s, sensor-actuated {h1['dyn']:.3f} s",
        f"    paired t = {h1['t']:.3f}, p (one-sided, sensor > static) {fp(h1['p_t'])}, d_z = {h1['dz']:.2f}",
        "",
        f"H2  critical conflicts per run, defined as {crit_label}: static {h2['base']:.2f}, sensor-actuated {h2['dyn']:.2f}",
        f"    Wilcoxon p (one-sided, sensor < static) {fp(h2['p_w'])}; paired t p {fp(h2['p_t'])}; d_z = {h2['dz']:.2f}",
        "",
        "All candidate critical-conflict definitions (per run, static -> sensor-actuated, one-sided Wilcoxon p):",
    ]
    for k, v in extras.items():
        lines.append(f"    {k:14s} {v['base']:6.2f} -> {v['dyn']:6.2f}   p {fp(v['p_w'])}")
    lines += [
        "",
        "Cost of the controller:",
        f"    left-turner mean delay (time loss): static {delay['base']:.1f} s -> sensor {delay['dyn']:.1f} s  (paired t, two-sided p {fp(delay['p_t'])}; difference {delay['dyn'] - delay['base']:+.1f} s)",
        f"    left arrow forced red: {red.get('dynamic', float('nan')):.0f} s per run; override triggered by {ov.get('dynamic', float('nan')):.0f} oncoming cars per run",
        "",
        f"Left-turners released while a hidden car was on the approach: {n_hidden.get('baseline', float('nan')):.0f} of 80 per run",
        f"Collisions detected (all runs): static {int(coll.get('baseline', 0))}, sensor-actuated {int(coll.get('dynamic', 0))}",
    ]
    delay_sub = None
    sub_files = glob.glob("delay_subset/summary_*_*.csv")
    if sub_files:
        rows = []
        for p in sub_files:
            m = re.match(r"summary_(baseline|dynamic)_(\d+)\.csv", os.path.basename(p))
            if m:
                r = pd.read_csv(p).iloc[0].to_dict()
                r.update(mode=m.group(1), seed=int(m.group(2)))
                rows.append(r)
        delay_sub = pd.DataFrame(rows)
    lines += reference_lines(per_run, delay_sub)
    text = "\n".join(lines) + "\n"
    print(text)
    open(os.path.join(OUT, "stats_summary.txt"), "w").write(text)
    per_run.to_csv(os.path.join(OUT, "per_run_metrics.csv"), index=False)

    results_figure(conf, per_run, h1, h2, crit_label, crit_col)
    reference_figure(per_run, delay_sub)
    tti_figure()
    print(">>> saved safety_results.png, reference_comparison.png, tti_matrix.png, stats_summary.txt, per_run_metrics.csv in", OUT)


if __name__ == "__main__":
    main()