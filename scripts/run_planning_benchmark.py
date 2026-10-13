#!/usr/bin/env python3
"""Bounded, single-CPU benchmark launcher; CSV is the complete raw record."""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def command_output(command):
    try:
        run = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=15)
        return {"returncode": run.returncode, "stdout": run.stdout, "stderr": run.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def source_hashes():
    listing = command_output(["git", "ls-files", "-co", "--exclude-standard"])
    hashes = {}
    for name in sorted(set(listing.get("stdout", "").splitlines())):
        path = ROOT / name
        if path.suffix not in {".cpp", ".hpp", ".h", ".py", ".xml", ".urdf", ".cmake", ".txt"}:
            continue
        if any(part.startswith(".") or part.startswith("build") or part in {"reference", "results", "artifacts", "cache"}
               for part in Path(name).parts):
            continue
        if path.is_file() and path.stat().st_size <= 10_000_000:
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def distribution(values):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return {"n": 0, "p50": None, "p95": None, "max": None}

    def quantile(fraction):
        location = fraction * (len(values) - 1)
        lower = math.floor(location)
        upper = math.ceil(location)
        return values[lower] + (values[upper] - values[lower]) * (location - lower)

    return {"n": len(values), "p50": quantile(.5), "p95": quantile(.95), "max": values[-1]}


def summarize(output):
    raw = output / "runs.csv"
    if not raw.exists():
        return {"runs": 0, "discarded_incomplete_rows": 0, "groups": [], "paired_speedup": [], "failures": []}
    with raw.open(newline="") as stream:
        candidates = list(csv.DictReader(stream))
    rows = []
    for row in candidates:
        # A hard process timeout can interrupt the last CSV write. Preserve all
        # complete rows and expose the discarded count in the report.
        if None in row or any(value is None for value in row.values()):
            continue
        try:
            for key in ("planning_seconds", "dense_recheck_seconds"):
                if not math.isfinite(float(row[key])) or float(row[key]) < 0:
                    raise ValueError("invalid timing")
            int(row["repeat"])
            if row["success"] not in {"0", "1"}:
                raise ValueError("invalid success")
        except (KeyError, TypeError, ValueError):
            continue
        rows.append(row)
    splits = [name for name in ("positive", "development", "confirmation", "negative", "all")
              if name in {"positive", "all"} or any(row["split"] == name for row in rows)]

    def in_split(row, split):
        return split == "all" or (split == "positive" and row["split"] != "negative") or row["split"] == split

    groups = []
    for split in splits:
        for method in sorted({row["method"] for row in rows}):
            selected = [r for r in rows if r["method"] == method and in_split(r, split)]
            if not selected:
                continue
            successful = [r for r in selected if r["success"] == "1"]
            groups.append({
                "method": method, "split": split, "runs": len(selected), "successes": len(successful),
                "success_rate": len(successful) / len(selected),
                "expected_rejection_rate": (1 - len(successful) / len(selected)) if split == "negative" else None,
                "planning_seconds_all_runs": distribution([float(r["planning_seconds"]) for r in selected]),
                "planning_seconds_success_only": distribution([float(r["planning_seconds"]) for r in successful]),
                "dense_recheck_seconds": distribution([float(r["dense_recheck_seconds"]) for r in selected
                                                           if r["dense_recheck_performed"] == "1"]),
            })
    baseline = {(r["case"], r["repeat"]): r for r in rows if r["method"] == "baseline"}
    pairs = []
    for method in sorted({r["method"] for r in rows} - {"baseline"}):
        for split in splits:
            ratios, all_ratios = [], []
            regressions, gains = [], []
            matched = 0
            for row in rows:
                if row["method"] != method or not in_split(row, split):
                    continue
                base = baseline.get((row["case"], row["repeat"]))
                if base is None:
                    continue
                matched += 1
                seconds = float(row["planning_seconds"])
                if seconds > 0:
                    ratio = float(base["planning_seconds"]) / seconds
                    all_ratios.append(ratio)
                    if row["success"] == base["success"] == "1":
                        ratios.append(ratio)
                if base["success"] == "1" and row["success"] != "1":
                    regressions.append({"case": row["case"], "repeat": int(row["repeat"])})
                if base["success"] != "1" and row["success"] == "1":
                    gains.append({"case": row["case"], "repeat": int(row["repeat"])})
            if matched:
                pairs.append({"method": method, "split": split, "matched": matched,
                              "baseline_over_method_all_runs": distribution(all_ratios),
                              "baseline_over_method_both_successful": distribution(ratios),
                              "success_regressions": regressions, "success_gains": gains})
    return {"runs": len(rows), "discarded_incomplete_rows": len(candidates) - len(rows),
            "groups": groups, "paired_speedup": pairs,
            "failures": [{key: row[key] for key in ("method", "case", "split", "repeat", "reason")}
                         for row in rows if row["success"] != "1"]}


def write_summary(output, summary):
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    lines = ["# Planning benchmark", "", "Planning time includes the planner's validation. Cold model load, atlas fitting, and denser recheck using the same validator are separate. The recheck doubles sampling density; it is not an independent validation algorithm.",
             "The positive aggregate excludes negative cases. Negative-case rejection is expected; the all aggregate mixes both and must not be used to claim positive-case acceleration.",
             "All-run timings include failed/infeasible runs. Paired success-only speedup compares the same case and repeat; values above 1 favor the accelerated method.", "",
             "| Split | Method | Success | All p50 / p95 / max (s) | Success p50 / p95 / max (s) |",
             "|---|---|---:|---:|---:|"]

    def numbers(stats):
        return " / ".join("—" if stats[key] is None else f"{stats[key]:.6f}" for key in ("p50", "p95", "max"))

    for group in summary["groups"]:
        lines.append(f"| {group['split']} | {group['method']} | {group['successes']}/{group['runs']} | "
                     f"{numbers(group['planning_seconds_all_runs'])} | {numbers(group['planning_seconds_success_only'])} |")
    lines.extend(["", "| Split | Method | Paired successful n | Speedup p50 / p95 / max | Success regressions |",
                  "|---|---|---:|---:|---:|"])
    for pair in summary["paired_speedup"]:
        stats = pair["baseline_over_method_both_successful"]
        lines.append(f"| {pair['split']} | {pair['method']} | {stats['n']} | {numbers(stats)} | {len(pair['success_regressions'])} |")
    if summary["failures"]:
        lines.extend(["", "## Failed runs", ""])
        for failure in summary["failures"]:
            reason = failure["reason"].replace("\n", " ")
            lines.append(f"- {failure['split']} / {failure['case']} / {failure['method']} / repeat {failure['repeat']}: {reason}")
    (output / "summary.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=ROOT / "build" / "rm26_plan")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "level3_planning" / "manual")
    parser.add_argument("--robot", default="rm26", help="robot id or model directory")
    parser.add_argument("--method", default="all")
    parser.add_argument("--case", default="all")
    parser.add_argument("--split", choices=["development", "confirmation", "negative", "all"], default="all")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--motion-speed-scale", type=float, default=1.)
    parser.add_argument("--process-timeout", type=float, default=None)
    parser.add_argument("--dense-check", action="store_true")
    parser.add_argument("--summarize-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.summarize_only:
        write_summary(output, summarize(output))
        return 0
    if args.repeat < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("repeat and timeout must be positive")
    if (not math.isfinite(args.motion_speed_scale) or args.motion_speed_scale <= 0 or
            not math.isfinite(2*args.motion_speed_scale*args.motion_speed_scale)):
        parser.error("motion speed scale must be positive and finite")
    if (output / "runs.csv").exists():
        parser.error("output already contains runs.csv; select a new directory or use --summarize-only")
    executable = args.executable.resolve()
    try:
        listing = subprocess.run([str(executable), "--robot", args.robot, "--list"], check=True, text=True, capture_output=True, timeout=20)
    except (OSError, subprocess.SubprocessError) as error:
        parser.error(f"cannot enumerate benchmark executable: {error}")
    inventory = list(csv.DictReader(listing.stdout.splitlines()))
    cases = [r for r in inventory if r["kind"] == "case" and
             (args.case == "all" or r["name"] == args.case) and
             (args.split == "all" or r["split"] == args.split)]
    if not cases:
        parser.error("no cases match the selected case/split")
    if args.method != "all" and args.method not in {r["name"] for r in inventory if r["kind"] == "method"}:
        parser.error("unknown method; use a name reported by rm26_plan --list")
    method_count = sum(r["kind"] == "method" for r in inventory) if args.method == "all" else 1
    dev_count = sum(r["kind"] == "case" and r["split"] == "development" for r in inventory)
    runs = len(cases) * method_count * args.repeat
    fitting_runs = dev_count if args.method in {"all", "atlas"} else 0
    process_timeout = args.process_timeout if args.process_timeout is not None else (
        (runs * (2 if args.dense_check else 1) + fitting_runs) * args.timeout + 120)
    if not math.isfinite(process_timeout) or process_timeout <= 0:
        parser.error("process timeout must be finite and positive")
    environment = os.environ.copy()
    environment.update({"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                        "NUMEXPR_NUM_THREADS": "1", "CUDA_VISIBLE_DEVICES": ""})
    # Restrict to one currently allowed CPU. MuJoCo FK/collision uses no renderer.
    allowed = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    chosen_cpu = allowed[0] if allowed else None
    if chosen_cpu is not None:
        os.sched_setaffinity(0, {chosen_cpu})
    command = [str(executable), "--robot", args.robot, "--method", args.method, "--case", args.case, "--split", args.split,
               "--repeat", str(args.repeat), "--timeout", str(args.timeout), "--output", str(output),
               "--motion-speed-scale", str(args.motion_speed_scale)]
    if args.dense_check:
        command.append("--dense-check")
    metadata = {"utc_started": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "command": command, "expected_runs": runs, "process_timeout_seconds": process_timeout,
                "platform": platform.platform(), "python": sys.version, "uname": command_output(["uname", "-a"]),
                "lscpu": command_output(["lscpu"]), "git_head": command_output(["git", "rev-parse", "HEAD"]),
                "git_status": command_output(["git", "status", "--short"]), "source_sha256": source_hashes(),
                "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(), "cpu": chosen_cpu,
                "environment": {key: environment[key] for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                               "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "CUDA_VISIBLE_DEVICES")}}
    (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
    # A finite run count and hard process deadline bound this one log. No
    # background processes, shared-memory segments, or temporary files persist.
    with (output / "benchmark.log").open("w") as log:
        try:
            process = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=process_timeout)
            returncode = process.returncode
            metadata["termination"] = "completed" if returncode == 0 else "process_error"
        except subprocess.TimeoutExpired:
            returncode = 124
            metadata["termination"] = "process_timeout"
        except OSError as error:
            returncode = 127
            metadata["termination"] = "launch_error"
            metadata["error"] = str(error)
    metadata["returncode"] = returncode
    metadata["utc_finished"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary = summarize(output)
    metadata["recorded_runs"] = summary["runs"]
    metadata["complete_run_count"] = summary["runs"] == runs
    if (output / "scenarios.csv").exists():
        metadata["scenarios_sha256"] = hashlib.sha256((output / "scenarios.csv").read_bytes()).hexdigest()
    (output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
    write_summary(output, summary)
    print(f"Recorded {summary['runs']}/{runs} runs; output: {output}")
    return returncode if returncode else (0 if summary["runs"] == runs else 3)


if __name__ == "__main__":
    raise SystemExit(main())
