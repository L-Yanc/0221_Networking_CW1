import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

LOG_DIR = Path(".") 

# ---------- 1) filename -> metadata ----------
def parse_meta(fname: str) -> dict:
    stem = Path(fname).stem  # e.g. log_attack_false_data
    meta = {"file": fname, "role": None, "attack": None, "variant": None}
    if stem.startswith("log_peaceful_"):
        meta["role"] = "baseline"
        meta["attack"] = "none"
        meta["variant"] = stem.replace("log_peaceful_", "")
    elif stem.startswith("log_attack_"):
        meta["role"] = "attacker"
        meta["attack"] = stem.replace("log_attack_", "")
    elif stem.startswith("log_defend_"):
        meta["role"] = "defender"
        meta["attack"] = stem.replace("log_defend_", "")
    else:
        meta["role"] = "unknown"
        meta["attack"] = "unknown"
    return meta

# ---------- 2) regex patterns ----------
re_prefix = re.compile(r"I\s*\((\d+)\)\s+([A-Z0-9_]+):\s+(.*)")

# TASK line pattern (PHYSICS/FLOCK/RADIO/TELE)
re_task = re.compile(
    r"^(PHYSICS|FLOCK|RADIO|TELE)\s+.*?"
    r"exp_us=(?P<exp>-?\d+)\s+start_us=(?P<start>-?\d+)\s+end_us=(?P<end>-?\d+)\s+"
    r"exec_us=(?P<exec>-?\d+)\s+lat_us=(?P<lat>-?\d+)\s+"
    r"(?:jitter_us=(?P<jitter>-?\d+)\s+)?"
    r"p_us=(?P<p>-?\d+)"
)

re_flock_extra = re.compile(r".*neigh=(?P<neigh>\d+)\s+age_max_ms=(?P<age>\d+).*")
re_radio_extra = re.compile(r".*neigh=(?P<neigh>\d+)\s+age_max_ms=(?P<age>\d+).*")
re_tele_extra  = re.compile(r".*neigh=(?P<neigh>\d+).*mqtt_ok=(?P<mqtt_ok>-?\d+)\s+mqtt_exec_us=(?P<mqtt_exec>-?\d+).*")

# STAB
re_stab = re.compile(
    r"^STAB\s+neigh=(?P<neigh>\d+)\s+dist_cent_mm=(?P<dist>-?\d+)\s+"
    r"min_sep_mm=(?P<minsep>-?\d+)\s+heading_cos_x1000=(?P<heading>-?\d+)"
)

# ENERGY
re_energy = re.compile(
    r"^ENERGY\s+lora_tx=(?P<tx>\d+)\s+lora_rx_ok=(?P<rx_ok>\d+)\s+"
    r"lora_rx_mac_fail=(?P<rx_fail>\d+)\s+mqtt_ok=(?P<mqtt_ok>\d+)\s+mqtt_fail=(?P<mqtt_fail>\d+)"
)

# RX packet log (optional)
re_rx = re.compile(r"^RX:\s+from\s+(?P<from>[0-9a-f:]+)\s+seq=(?P<seq>\d+).*")

# ---------- 3) parse files ----------
task_rows, stab_rows, energy_rows, rx_rows = [], [], [], []

for path in sorted(LOG_DIR.glob("log_*.txt")):
    meta = parse_meta(path.name)

    with path.open("r", errors="ignore") as f:
        for line in f:
            m = re_prefix.search(line)
            if not m:
                continue

            t_ms = int(m.group(1))
            tag = m.group(2)
            msg = m.group(3).strip()

            # TASK
            mt = re_task.match(msg)
            if mt:
                d = mt.groupdict()
                row = {
                    **meta,
                    "t_ms": t_ms,
                    "task": mt.group(1),
                    "exp_us": int(d["exp"]),
                    "start_us": int(d["start"]),
                    "end_us": int(d["end"]),
                    "exec_us": int(d["exec"]),
                    "lat_us": int(d["lat"]),
                    "p_us": int(d["p"]),
                    "jitter_us": int(d["jitter"]) if d["jitter"] is not None else None,
                }

                # extras
                if row["task"] == "FLOCK":
                    me = re_flock_extra.match(msg)
                    if me:
                        row["neigh"] = int(me.group("neigh"))
                        row["age_max_ms"] = int(me.group("age"))
                if row["task"] == "RADIO":
                    me = re_radio_extra.match(msg)
                    if me:
                        row["neigh"] = int(me.group("neigh"))
                        row["age_max_ms"] = int(me.group("age"))
                if row["task"] == "TELE":
                    me = re_tele_extra.match(msg)
                    if me:
                        row["neigh"] = int(me.group("neigh"))
                        row["mqtt_ok"] = int(me.group("mqtt_ok"))
                        row["mqtt_exec_us"] = int(me.group("mqtt_exec"))

                task_rows.append(row)
                continue

            # STAB
            ms = re_stab.match(msg)
            if ms:
                stab_rows.append({
                    **meta,
                    "t_ms": t_ms,
                    "neigh": int(ms.group("neigh")),
                    "dist_cent_mm": int(ms.group("dist")),
                    "min_sep_mm": int(ms.group("minsep")),
                    "heading_cos": int(ms.group("heading")) / 1000.0,
                })
                continue

            # ENERGY
            me = re_energy.match(msg)
            if me:
                energy_rows.append({
                    **meta,
                    "t_ms": t_ms,
                    "lora_tx": int(me.group("tx")),
                    "lora_rx_ok": int(me.group("rx_ok")),
                    "lora_rx_mac_fail": int(me.group("rx_fail")),
                    "mqtt_ok": int(me.group("mqtt_ok")),
                    "mqtt_fail": int(me.group("mqtt_fail")),
                })
                continue

            # RX (optional)
            mr = re_rx.match(msg)
            if mr:
                rx_rows.append({
                    **meta,
                    "t_ms": t_ms,
                    "from": mr.group("from"),
                    "seq": int(mr.group("seq")),
                })
                continue

df_tasks = pd.DataFrame(task_rows).sort_values(["attack","role","task","t_ms"])
df_stab = pd.DataFrame(stab_rows).sort_values(["attack","role","t_ms"])
df_energy = pd.DataFrame(energy_rows).sort_values(["attack","role","t_ms"])
df_rx = pd.DataFrame(rx_rows).sort_values(["attack","role","t_ms"])

print("tasks:", df_tasks.shape, "stab:", df_stab.shape, "energy:", df_energy.shape, "rx:", df_rx.shape)


# Plotting section
# Assumes df_tasks, df_stab, df_energy already exist from your parsing stage.

OUT_DIR = Path("plots_ieee")
OUT_DIR.mkdir(exist_ok=True)

# ---------- 0) helpers ----------
def role_to_regime(role: str) -> str:
    if role == "baseline":
        return "clean"
    if role == "attacker":
        return "attack"
    if role == "defender":
        return "defence"
    return "other"

def savefig(name: str):
    plt.tight_layout()
    plt.savefig(OUT_DIR / name, dpi=300)
    plt.close()

def add_regime(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["regime"] = df["role"].map(role_to_regime)
    return df[df["regime"].isin(["clean", "attack", "defence"])]

def add_relative_time(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df["t0_ms"] = df.groupby("file")["t_ms"].transform("min")
    df["t_rel_s"] = (df["t_ms"] - df["t0_ms"]) / 1000.0
    return df

def resample_per_run(df: pd.DataFrame, value_col: str, how: str = "median", dt_s: int = 1) -> pd.DataFrame:
    """
    Convert to per-run per-second series.
    df must contain: regime, file, t_rel_s, value_col
    how: 'median'|'mean'|'sum'
    """
    df = df.copy()
    df["sec"] = (np.floor(df["t_rel_s"] / dt_s) * dt_s).astype(int)

    if how == "sum":
        out = df.groupby(["regime", "file", "sec"], as_index=False)[value_col].sum()
    elif how == "mean":
        out = df.groupby(["regime", "file", "sec"], as_index=False)[value_col].mean()
    else:
        out = df.groupby(["regime", "file", "sec"], as_index=False)[value_col].median()
    return out

def median_iqr_across_runs(per_run_df: pd.DataFrame, value_col: str) -> pd.DataFrame:
    q = per_run_df.groupby(["regime", "sec"])[value_col].quantile([0.25, 0.5, 0.75]).unstack()
    q = q.rename(columns={0.25: "q25", 0.5: "median", 0.75: "q75"}).reset_index()
    return q

def plot_triptych(per_run_df: pd.DataFrame, value_col: str, title: str, ylabel: str, fname: str,
                  show_iqr: bool = True, thin_alpha: float = 0.30, thin_lw: float = 0.8,
                  med_lw: float = 2.0):
    """
    One figure with three panels: clean | attack | defence.
    Thin lines = individual runs (files), thick line = median across runs, optional IQR band.
    """
    regimes = ["clean", "attack", "defence"]
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.4), sharey=True)

    for ax, reg in zip(axes, regimes):
        sub = per_run_df[per_run_df["regime"] == reg].copy()
        ax.set_title(reg)
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.25)

        if sub.empty:
            continue

        # thin per-run lines
        for file, g in sub.groupby("file"):
            g = g.sort_values("sec")
            ax.plot(g["sec"], g[value_col], linewidth=thin_lw, alpha=thin_alpha)

        # median + IQR
        q = median_iqr_across_runs(sub, value_col).sort_values("sec")
        ax.plot(q["sec"], q["median"], linewidth=med_lw)
        if show_iqr:
            ax.fill_between(q["sec"], q["q25"], q["q75"], alpha=0.15)

    axes[0].set_ylabel(ylabel)
    fig.suptitle(title)
    plt.tight_layout()
    plt.savefig(OUT_DIR / fname, dpi=300)
    plt.close()

def plot_task_lines_per_regime(df_tasks_rel: pd.DataFrame, metric_col: str, regime: str,
                               title_prefix: str, ylabel: str, fname: str,
                               tasks=("PHYSICS", "FLOCK", "RADIO", "TELE"),
                               dt_s: int = 1, per_run_agg: str = "median", across_runs_agg: str = "median"):
    """
    One regime per figure, four lines per plot (one per task).
    Lines are aggregated across runs to avoid spaghetti:
      - per-run resample with per_run_agg
      - across-runs aggregate per second with across_runs_agg
    """
    fig, ax = plt.subplots(figsize=(3.6, 2.4))

    sub_reg = df_tasks_rel[(df_tasks_rel["regime"] == regime) & df_tasks_rel[metric_col].notna()].copy()
    if sub_reg.empty:
        ax.set_title(f"{title_prefix} — {regime} (no data)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        savefig(fname)
        return

    for task in tasks:
        sub = sub_reg[sub_reg["task"] == task].copy()
        if sub.empty:
            continue

        # per-run per-second resample
        pr = resample_per_run(sub, metric_col, how=per_run_agg, dt_s=dt_s)  # cols: regime,file,sec,val
        # aggregate across runs per sec
        if across_runs_agg == "mean":
            agg = pr.groupby("sec", as_index=False)[metric_col].mean()
        else:
            agg = pr.groupby("sec", as_index=False)[metric_col].median()

        agg = agg.sort_values("sec")
        ax.plot(agg["sec"], agg[metric_col], label=task)

    ax.set_title(f"{title_prefix} — {regime}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    plt.tight_layout()
    plt.savefig(OUT_DIR / fname, dpi=300)
    plt.close()

def plot_task_jitter_boxes(df_tasks_rel: pd.DataFrame, regime: str, title_prefix: str, ylabel: str, fname: str,
                           tasks=("PHYSICS", "FLOCK", "RADIO", "TELE"),
                           clip_quantiles=(0.01, 0.99)):
    """
    One regime per figure, boxplot with 4 boxes (one per task).
    Clips extreme outliers to avoid 1e9-scale destruction.
    """
    sub = df_tasks_rel[(df_tasks_rel["regime"] == regime) & df_tasks_rel["jitter_us"].notna()].copy()

    data = []
    labels = []
    for task in tasks:
        vals = sub[sub["task"] == task]["jitter_us"].astype(float)
        if vals.empty:
            continue
        lo, hi = vals.quantile(list(clip_quantiles))
        vals = vals.clip(lo, hi).values
        data.append(vals)
        labels.append(task)

    fig, ax = plt.subplots(figsize=(3.6, 2.4))
    ax.set_title(f"{title_prefix} — {regime}")
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.25)

    if len(data) == 0:
        ax.text(0.5, 0.5, "No jitter data", ha="center", va="center", transform=ax.transAxes)
        savefig(fname)
        return

    ax.boxplot(data, labels=labels, showfliers=False)
    plt.tight_layout()
    plt.savefig(OUT_DIR / fname, dpi=300)
    plt.close()

# ---------- 1) select regimes + relative time ----------
df_tasks2 = add_regime(df_tasks)
df_stab2 = add_regime(df_stab)
df_energy2 = add_regime(df_energy)

df_tasks2 = add_relative_time(df_tasks2)
df_stab2 = add_relative_time(df_stab2)
df_energy2 = add_relative_time(df_energy2)

REGIMES = ["clean", "attack", "defence"]
TASKS = ["PHYSICS", "FLOCK", "RADIO", "TELE"]

# ---------- 2) TIMING: latency per regime (4 tasks = 4 lines) ----------
# Produces 3 plots: one per regime
for r in REGIMES:
    plot_task_lines_per_regime(
        df_tasks2,
        metric_col="lat_us",
        regime=r,
        title_prefix="Task latency over time",
        ylabel="Latency (us)",
        fname=f"latency_tasks_{r}.png",
        tasks=TASKS,
        dt_s=1,
        per_run_agg="median",
        across_runs_agg="median",
    )

# ---------- 3) TIMING: jitter per regime (4 tasks = 4 boxes) ----------
# Produces 3 plots: one per regime
for r in REGIMES:
    plot_task_jitter_boxes(
        df_tasks2,
        regime=r,
        title_prefix="Task jitter distribution",
        ylabel="Jitter (us)",
        fname=f"jitter_tasks_{r}.png",
        tasks=TASKS,
    )

# ---------- 4) COMMS: receive rate triptych (rx_ok per second) ----------
# Use ENERGY cumulative counters -> deltas -> per-second sum -> triptych
e = df_energy2.sort_values(["regime", "file", "t_ms"]).copy()
e["d_rx_ok"] = e.groupby(["regime", "file"])["lora_rx_ok"].diff().clip(lower=0)
e = e.dropna(subset=["d_rx_ok"])

rx_sec = resample_per_run(e, "d_rx_ok", how="sum", dt_s=1)
plot_triptych(
    rx_sec,
    value_col="d_rx_ok",
    title="LoRa receive rate",
    ylabel="rx_ok per second",
    fname="comms_rx_rate_triptych.png",
    show_iqr=True
)

# ---------- 5) COMMS: CMAC reject events triptych (mac_fail per second) ----------
# This is the plot that prevents "one attack gets averaged away"
e2 = df_energy2.sort_values(["regime", "file", "t_ms"]).copy()
e2["d_mac_fail"] = e2.groupby(["regime", "file"])["lora_rx_mac_fail"].diff().clip(lower=0)
e2 = add_relative_time(e2).dropna(subset=["d_mac_fail"])

fail_sec = resample_per_run(e2, "d_mac_fail", how="sum", dt_s=1)
plot_triptych(
    fail_sec,
    value_col="d_mac_fail",
    title="CMAC rejection events",
    ylabel="mac_fail per second",
    fname="comms_cmac_fail_triptych.png",
    show_iqr=False  # you want the per-run thin lines visible here
)

# ---------- 6) STABILITY: minimum separation triptych (clipped) ----------
# Use min_sep_mm; clip 1%/99% to prevent 1e9 axis blow-ups.
s = df_stab2.dropna(subset=["min_sep_mm"]).copy()
lo, hi = s["min_sep_mm"].quantile([0.01, 0.99])
s["min_sep_mm_clip"] = s["min_sep_mm"].clip(lo, hi)

minsep_sec = resample_per_run(s, "min_sep_mm_clip", how="mean", dt_s=1)
plot_triptych(
    minsep_sec,
    value_col="min_sep_mm_clip",
    title="Stability proxy: minimum separation",
    ylabel="Min separation (mm)",
    fname="stab_minsep_triptych.png",
    show_iqr=True
)

# ---------- 7) OPTIONAL: dist to centroid triptych (clipped) ----------
# Keep only if it looks meaningful after clipping.
if "dist_cent_mm" in df_stab2.columns and df_stab2["dist_cent_mm"].notna().any():
    s2 = df_stab2.dropna(subset=["dist_cent_mm"]).copy()
    lo2, hi2 = s2["dist_cent_mm"].quantile([0.01, 0.99])
    s2["dist_cent_mm_clip"] = s2["dist_cent_mm"].clip(lo2, hi2)

    dist_sec = resample_per_run(s2, "dist_cent_mm_clip", how="mean", dt_s=1)
    plot_triptych(
        dist_sec,
        value_col="dist_cent_mm_clip",
        title="Stability proxy: distance to centroid",
        ylabel="Distance to centroid (mm)",
        fname="stab_dist_to_centroid_triptych.png",
        show_iqr=True
    )

print(f"Saved plots to: {OUT_DIR.resolve()}")
