#!/usr/bin/env python3
"""Plot successful planning latency and preserve every case in a success matrix."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with (args.directory / "runs.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (args.directory / "scenarios.csv").open() as stream:
        scenarios = list(csv.DictReader(stream))
    methods = ["baseline", "full_step", "reduced_5d", "adaptive_lm", "predictor",
               "sparse_adaptive", "atlas", "bounded_qp", "rigid_cache"]
    fig, (ax, matrix) = plt.subplots(1, 2, figsize=(15, 8),
                                   gridspec_kw={"width_ratios": [1, 1.6]})
    for y, method in enumerate(methods):
        positive = [r for r in rows if r["method"] == method and r["split"] != "negative"]
        values = [float(r["planning_seconds"]) for r in positive if r["success"] == "1"]
        if values:
            median, p95 = np.quantile(values, [.5, .95])
            ax.barh(y, median, color="#267469" if method == "rigid_cache" else "#52799d")
            ax.plot([median, p95], [y, y], color="#182c3c", linewidth=2)
            ax.plot(max(values), y, "|", color="#182c3c", markersize=12)
        ax.text(0.01, y-.28, f"{len(values)}/{len(positive)} accepted", fontsize=8)
    ax.set_yticks(range(len(methods)), methods)
    ax.invert_yaxis()
    ax.axvline(3, color="#b14539", linestyle="--", linewidth=1, label="3 s target")
    ax.set_xlabel("Complete planning latency (s), successes only")
    ax.set_title("Median bar / p95 line / maximum tick")
    ax.legend(loc="lower right")
    ax.grid(axis="x", alpha=.2)
    cases = [s["case"] for s in scenarios]
    rates = np.empty((len(cases), len(methods)))
    for y, case in enumerate(cases):
        for x, method in enumerate(methods):
            selected = [r for r in rows if r["case"] == case and r["method"] == method]
            rates[y, x] = sum(r["success"] == "1" for r in selected) / len(selected)
    from matplotlib.colors import ListedColormap
    matrix.imshow(rates, cmap=ListedColormap(["#ead7cc", "#93bca9"]), vmin=0, vmax=1,
                  aspect="auto")
    for y in range(len(cases)):
        for x in range(len(methods)):
            matrix.text(x, y, f"{rates[y,x]:.0%}", ha="center", va="center", fontsize=7)
    matrix.set_yticks(range(len(cases)), cases, fontsize=8)
    matrix.set_xticks(range(len(methods)), methods, rotation=45, ha="right", fontsize=8)
    for y in range(1, len(scenarios)):
        if scenarios[y]["split"] != scenarios[y-1]["split"]:
            matrix.axhline(y-.5, color="white", linewidth=3)
    matrix.set_title("All requests: fraction accepted (negative fixtures should be 0%)")
    fig.suptitle("Level-3 exchange | one CPU | collision validation included", fontsize=14)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    fig.savefig(args.output.with_suffix(".pdf"))
    plt.close(fig)


if __name__ == "__main__":
    main()
