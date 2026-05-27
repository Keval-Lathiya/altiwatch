#!/usr/bin/env python3
"""
plot_jump.py — visualise an AltiWatch jump CSV.

Importable API
--------------
    from plot_jump import load_jump, plot_jump, print_summary

CLI usage
---------
    python plot_jump.py                         # most recent CSV in jumps/
    python plot_jump.py jumps/jump_t53.csv      # specific file
    python plot_jump.py --no-show               # save PNG, skip display window
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import MaxNLocator


# ---------------------------------------------------------------------------
# Colour palette — GitHub-dark inspired
# ---------------------------------------------------------------------------
_BG      = "#0e1117"
_PANEL   = "#161b22"
_FG      = "#c9d1d9"
_MUTED   = "#8b949e"
_GRID    = "#30363d"
_BORDER  = "#21262d"

C_ALT    = "#58a6ff"   # blue   — altitude line
C_CLIMB  = "#3fb950"   # green  — positive vspeed
C_DESC   = "#f85149"   # red    — negative vspeed
C_ZERO   = "#484f58"   # dim    — deadband zeros
C_ACC    = "#d2a8ff"   # purple — acceleration
C_ANNOT  = "#ffa657"   # orange — point annotations

JUMPS_DIR = Path("jumps")


# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------

def load_jump(path: Path | str) -> pd.DataFrame:
    """
    Load a jump CSV and attach two derived columns:
      t_s   : seconds elapsed from the first sample
      acc_g : acceleration magnitude  sqrt(ax² + ay² + az²)
    """
    df = pd.read_csv(path)
    df["t_s"]   = (df["time_ms"] - df["time_ms"].iat[0]) / 1000.0
    df["acc_g"] = np.sqrt(df["ax"] ** 2 + df["ay"] ** 2 + df["az"] ** 2)
    return df


def _stats(df: pd.DataFrame) -> dict:
    """Compute summary stats from a loaded DataFrame."""
    vs_nz = df.loc[df["vspeed_fps"] != 0.0, "vspeed_fps"]
    return {
        "duration":     df["t_s"].iat[-1],
        "n":            len(df),
        "agl_min":      df["agl_ft"].min(),
        "agl_max":      df["agl_ft"].max(),
        "agl_range":    df["agl_ft"].max() - df["agl_ft"].min(),
        "peak_descent": vs_nz.min() if not vs_nz.empty else 0.0,
        "peak_climb":   vs_nz.max() if not vs_nz.empty else 0.0,
        "max_acc":      df["acc_g"].max(),
        "idx_min_agl":  int(df["agl_ft"].idxmin()),
    }


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def _style_ax(ax: plt.Axes) -> None:
    ax.set_facecolor(_PANEL)
    for spine in ax.spines.values():
        spine.set_edgecolor(_BORDER)
    ax.tick_params(colors=_MUTED, labelsize=8.5)
    ax.grid(True, color=_GRID, linewidth=0.5, linestyle="--", alpha=0.65)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5))


def plot_jump(df: pd.DataFrame, source_path: Path | str) -> plt.Figure:
    """
    Build a 3-panel, shared-x figure for a loaded jump DataFrame.
    Returns the Figure; the caller saves / shows it.

    Panels (top to bottom):
      1. AGL altitude in ft  (largest)
      2. Vertical speed in ft/s, fill-coded by direction
      3. Acceleration magnitude in g, with 1 g reference
    """
    source_path = Path(source_path)
    st = _stats(df)
    t  = df["t_s"]
    vs = df["vspeed_fps"]

    # -- layout --------------------------------------------------------------
    fig = plt.figure(figsize=(14, 7), facecolor=_BG)
    gs  = gridspec.GridSpec(
        2, 1,
        height_ratios=[3, 2],
        hspace=0.06,
        left=0.07, right=0.97, top=0.93, bottom=0.08,
    )
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)

    for ax in (ax1, ax2):
        _style_ax(ax)
    plt.setp(ax1.get_xticklabels(), visible=False)

    # -- panel 1 — altitude --------------------------------------------------
    ax1.fill_between(t, df["agl_ft"], alpha=0.15, color=C_ALT)
    ax1.plot(t, df["agl_ft"], color=C_ALT, linewidth=1.6, zorder=3)
    ax1.axhline(0, color=_MUTED, linewidth=0.8, linestyle=":", alpha=0.7)
    ax1.set_ylabel("Altitude  (ft AGL)", color=_FG, fontsize=10)

    # Y-axis padding so the annotation arrow head is never clipped
    pad = max(st["agl_range"] * 0.14, 2.0)
    ax1.set_ylim(st["agl_min"] - pad, st["agl_max"] + pad)

    # Annotate the lowest point (only meaningful if there is real motion)
    if st["agl_range"] > 2.0:
        row = df.loc[st["idx_min_agl"]]
        ax1.annotate(
            f"  low: {row['agl_ft']:.1f} ft @ {row['t_s']:.0f} s",
            xy=(row["t_s"], row["agl_ft"]),
            xytext=(0, 30),
            textcoords="offset points",
            color=C_ANNOT, fontsize=8.5, ha="left",
            arrowprops=dict(arrowstyle="->", color=C_ANNOT, lw=0.9),
            zorder=5,
        )

    # -- panel 2 — vertical speed --------------------------------------------
    # fill_between naturally handles deadband: zeros produce no fill area
    ax2.fill_between(t, vs, 0, where=(vs > 0), color=C_CLIMB, alpha=0.55, label="climb")
    ax2.fill_between(t, vs, 0, where=(vs < 0), color=C_DESC,  alpha=0.55, label="descent")

    # Deadband-zero samples as a faint dotted marker on the zero baseline
    ax2.plot(
        t, vs.where(vs == 0.0),
        color=C_ZERO, linewidth=1.0, linestyle=":", alpha=0.6,
        label="deadband (0.5 fps floor)",
    )
    ax2.axhline(0, color=_MUTED, linewidth=0.7, linestyle=":", alpha=0.7)
    ax2.set_ylabel("Vert speed  (fps)", color=_FG, fontsize=10)
    ax2.set_xlabel("Time  (s)", color=_FG, fontsize=10)
    ax2.tick_params(axis="x", colors=_FG)
    ax2.legend(
        loc="upper right", fontsize=7.5,
        facecolor=_BG, edgecolor=_BORDER, labelcolor=_MUTED, framealpha=0.9,
    )

    # -- title ---------------------------------------------------------------
    fig.suptitle(
        f"{source_path.name}   |   {st['duration']:.1f} s   "
        f"({st['n']} samples @ 10 Hz)",
        color=_FG, fontsize=11, fontweight="bold",
    )

    return fig


# ---------------------------------------------------------------------------
# Console summary
# ---------------------------------------------------------------------------

def print_summary(df: pd.DataFrame, source_path: Path | str) -> None:
    """Print a compact text summary of a loaded jump to stdout."""
    st = _stats(df)
    sep = "-" * 54
    print(f"\n{sep}")
    print(f"  {Path(source_path).name}")
    print(sep)
    print(f"  Duration   {st['duration']:.1f} s   ({st['n']} samples)")
    print(f"  AGL        min {st['agl_min']:+.1f} ft   max {st['agl_max']:+.1f} ft"
          f"   range {st['agl_range']:.1f} ft")
    print(f"  Vspeed     peak descent {st['peak_descent']:+.2f} fps"
          f"   peak climb {st['peak_climb']:+.2f} fps")
    print(f"  Accel      max magnitude {st['max_acc']:.3f} g")
    print(f"{sep}\n")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _most_recent_csv(directory: Path) -> Path | None:
    csvs = sorted(directory.glob("*.csv"), key=lambda p: p.stat().st_mtime)
    return csvs[-1] if csvs else None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualise an AltiWatch jump CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "file", nargs="?",
        help="Path to jump CSV (default: most recent file in jumps/)",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Save PNG without opening an interactive window",
    )
    args = parser.parse_args()

    if args.file:
        path = Path(args.file)
    else:
        path = _most_recent_csv(JUMPS_DIR)
        if path is None:
            sys.exit(f"No CSV files found in {JUMPS_DIR}/")
        print(f"Using most recent: {path}")

    if not path.exists():
        sys.exit(f"File not found: {path}")

    df = load_jump(path)
    print_summary(df, path)

    fig = plot_jump(df, path)

    out_png = path.with_suffix(".png")
    fig.savefig(out_png, dpi=150, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"Saved: {out_png}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
