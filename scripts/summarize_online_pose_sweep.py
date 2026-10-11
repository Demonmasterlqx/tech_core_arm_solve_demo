#!/usr/bin/env python3
"""Audit the online 40-pose input/timing contract and compare recorded outcomes.

Run audit_planning_trajectories.py separately for independent URDF kinematics.
This script checks population, physical-start, deadline and accounting evidence.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def distribution(values):
    values = sorted(values)
    require(all(math.isfinite(v) and v >= 0 for v in values), "Invalid statistic")
    if not values:
        return dict(n=0)

    def quantile(p):
        at = p*(len(values)-1)
        lo, hi = math.floor(at), math.ceil(at)
        return values[lo]+(at-lo)*(values[hi]-values[lo])

    return dict(n=len(values), min=values[0], median=quantile(.5), p95=quantile(.95),
                max=values[-1], total=sum(values))


def category(reason):
    if "collision" in reason:
        return "collision_or_clearance"
    if "deadline" in reason:
        return "deadline"
    if reason.startswith("ik_"):
        return "ik"
    if reason.startswith("tracking_"):
        return "tracking"
    return reason


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--previous", type=Path, required=True)
    parser.add_argument("--direct", type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    config = json.loads((directory/"experiment.json").read_text())
    requested = rows(directory/"requested_poses.csv")
    scenarios = rows(directory/"scenarios.csv")
    runs = rows(directory/"runs.csv")
    case_summary = rows(directory/"case_summary.csv")
    attempts = rows(directory/"online_attempts.csv")
    trials = rows(directory/"trials/runs.csv")
    trial_scenarios = {r["case"]: r for r in rows(directory/"trials/scenarios.csv")}
    ids = [r["case"] for r in requested]
    require(len(ids) == len(set(ids)) == 40, "Need the complete 40-case population")
    for table in (scenarios, runs, case_summary):
        require([r["case"] for r in table] == ids, "Case order/population changed")
    require(sha(directory/"requested_poses.csv") == sha(args.previous/"requested_poses.csv"),
            "Online run changed the original pose inputs")
    require(config["planning_mode"] == "online_fixed_start" and not config["atlas"]
            and not config["cross_request_warm_start"], "Unexpected online contract")
    for pose, scenario in zip(requested, scenarios):
        for key in pose.keys()-{"case"}:
            require(float(pose[key]) == float(scenario[key]), "Changed requested pose")
        require([float(scenario[f"start_q{i}"]) for i in range(7)] == config["physical_start_q_rad"],
                "Actual start was replaced by a search waypoint")
        require(scenario["q_sweep"] == "1" and float(scenario["target_degrees"]) == -90
                and float(scenario["arm_base_z"]) == -.1, "Changed requested mechanism/base")
    for scenario in trial_scenarios.values():
        require([float(scenario[f"start_q{i}"]) for i in range(7)] == config["physical_start_q_rad"],
                "A trial replaced the actual input start")
    for run, summary, scenario in zip(runs, case_summary, scenarios):
        t = float(run["planning_seconds"])
        require(t == float(summary["online_planning_seconds"]), "Timing tables disagree")
        require(run["success"] == summary["success"] and run["atlas_source"] == "", "Result mismatch/atlas use")
        require(float(run["validation_joint_step"]) == .004 and int(run["validation_min_substeps"]) == 4,
                "Online collision sampling was weakened")
        require(float(run["motion_speed_scale"]) == 4, "Motion speed changed")
        require(float(run["ik_seconds"])+float(run["validation_seconds"]) <= t+1e-9,
                "Stage time exceeds the complete request")
        own = [a for a in attempts if a["case"] == run["case"]]
        planned = [a for a in own if a["plan_called"] == "1"]
        require(len(planned) == int(summary["planning_attempts"]), "Missing planning attempt")
        require(sum(a["kind"] == "waypoint" for a in own) == int(summary["ready_attempts"]), "Missing ready attempt")
        subset = [r for r in trials if r["case"].startswith(run["case"]+"_try")]
        require(len(subset) == len(planned), "Trial trajectory not preserved")
        spent = sum(float(r["planning_seconds"]) for r in subset)+sum(float(a["ready_ik_seconds"]) for a in own)
        require(spent <= t+1e-9, "Retries/waypoint IK lie outside reported online time")
        if run["success"] == "1":
            require(t < 3 and run["plan_success"] == run["rule_success"] == run["dense_recheck_valid"] == "1",
                    "Success does not meet the complete online acceptance contract")
            require(run["validation_samples"] == run["dense_recheck_samples"], "Cache audit sampling differs")
            knots = rows(directory/run["trajectory"])
            require([float(knots[0][f"q{i}"]) for i in range(7)] == config["physical_start_q_rad"],
                    "Executable path teleports to a different start")
            if int(summary["selected_attempt"]) > 0:
                selected = next(a for a in own if int(a["attempt"]) == int(summary["selected_attempt"]))
                waypoint = [float(selected[f"waypoint{i}"]) for i in range(7)]
                require(any(k["phase"] == "approach" and max(abs(float(k[f"q{i}"])-waypoint[i]) for i in range(7)) < 1e-12
                            for k in knots[1:]), "Online waypoint has no continuous approach connection")
    independent = json.loads((directory/"audit.json").read_text())
    require(independent["valid"] and independent["run_count"] == 40, "Independent kinematic audit failed")
    prior = {r["case"]: r for r in rows(args.previous/"runs.csv")}
    prior_summary = {r["case"]: r for r in rows(args.previous/"case_summary.csv")}
    direct = {r["case"]: r for r in rows(args.direct/"runs.csv")} if args.direct else {}
    comparison = []
    for run in runs:
        name = run["case"]
        comparison.append(dict(case=name, online_success=run["success"], online_seconds=run["planning_seconds"],
            previous_success=prior[name]["success"], previous_selected_plan_seconds=prior[name]["planning_seconds"],
            previous_search_wall_seconds=prior_summary[name]["search_wall_seconds"],
            direct_success=direct.get(name, {}).get("success", ""), reason=run["reason"]))
    with (directory/"comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(comparison[0]))
        writer.writeheader(); writer.writerows(comparison)
    success = [r for r in runs if r["success"] == "1"]
    report = dict(valid=True, cases=40, successes=len(success), failures=40-len(success),
        requested_poses_sha256=sha(directory/"requested_poses.csv"),
        all_online_seconds=distribution([float(r["planning_seconds"]) for r in runs]),
        successful_online_seconds=distribution([float(r["planning_seconds"]) for r in success]),
        successful_execution_seconds=distribution([float(r["execution_seconds"]) for r in case_summary if r["success"] == "1"]),
        uncached_audit_seconds=distribution([float(r["dense_recheck_seconds"]) for r in success]),
        request_reasons=dict(Counter(category(r["reason"]) for r in runs)),
        trial_reasons=dict(Counter(category(r["reason"]) for r in trials)),
        planning_attempts=len(trials), ready_attempts=sum(a["kind"] == "waypoint" for a in attempts),
        success_cases=[r["case"] for r in success],
        successes_using_online_waypoint=[r["case"] for r in case_summary if r["success"] == "1" and int(r["selected_attempt"]) > 0],
        previous_successes=sum(r["success"] == "1" for r in prior.values()),
        direct_successes=sum(r["success"] == "1" for r in direct.values()) if direct else None,
        dense_collision_samples=sum(int(r["validation_samples"]) for r in success),
        independent_fk_samples=sum(r.get("samples", 0) for r in independent["trajectories"]),
        max_position_error_m=max((r.get("max_position_error_m", 0) for r in independent["trajectories"]), default=0),
        max_axis_error_rad=max((r.get("max_axis_error_rad", 0) for r in independent["trajectories"]), default=0),
        note="Uncached equivalence audit is outside online time; the complete same-density cached collision acceptance is inside. Previous results used a different per-case physical start.")
    (directory/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
