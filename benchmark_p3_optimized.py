#!/usr/bin/env python3
"""Paired offline benchmark for the saved P3 baseline and active P3 policy."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
import statistics
import sys
from pathlib import Path
from types import ModuleType

import robot_strategy as optimized
from offline_verify import OfflineResponses


BASELINE_PATH = (
    Path(__file__).resolve().parent
    / "backup"
    / "p3_7point_baseline_before_optimization"
    / "robot_strategy.py"
)


def load_baseline() -> ModuleType:
    spec = importlib.util.spec_from_file_location("p3_saved_baseline", BASELINE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load baseline from {BASELINE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_sources(seed: int) -> dict:
    rng = random.Random(seed)
    count = rng.randint(10, 16)
    sources = {}
    for channel in rng.sample(range(1, 21), count):
        radial = 1800.0 * math.sqrt(rng.random())
        angle = rng.random() * 2.0 * math.pi
        point = (radial * math.cos(angle), radial * math.sin(angle))
        sources[channel] = (point, rng.uniform(1000.0, 1500.0), None)
    return sources


def run_suite(strategy: ModuleType, cases: int) -> dict:
    times = []
    measures = []
    clears = []
    failures = 0
    for index in range(cases):
        sources = make_sources(12000 + index)
        oracle = OfflineResponses(sources)
        done, unresolved, _ = strategy.run_problem(oracle, 3)
        if set(done) != set(sources) or unresolved:
            failures += 1
        times.append(oracle.virtual_time)
        measures.append(oracle.measures)
        clears.append(oracle.clears)

    percentile_index = max(0, math.ceil(0.9 * cases) - 1)
    return {
        "failures": failures,
        "mean_s": statistics.mean(times),
        "median_s": statistics.median(times),
        "p90_s": sorted(times)[percentile_index],
        "max_s": max(times),
        "mean_measures": statistics.mean(measures),
        "mean_clears": statistics.mean(clears),
        "raw_times": times,
    }


def rounded_summary(result: dict) -> dict:
    return {
        key: value if isinstance(value, int) else round(value, 3)
        for key, value in result.items()
        if key != "raw_times"
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.cases <= 0:
        parser.error("--cases must be positive")

    baseline = run_suite(load_baseline(), args.cases)
    active = run_suite(optimized, args.cases)
    reductions = [
        (old - new) / old * 100.0
        for old, new in zip(baseline["raw_times"], active["raw_times"])
    ]
    report = {
        "problem": 3,
        "cases": args.cases,
        "baseline": rounded_summary(baseline),
        "optimized": rounded_summary(active),
        "mean_paired_reduction_pct": round(statistics.mean(reductions), 3),
        "minimum_paired_reduction_pct": round(min(reductions), 3),
        "improved_cases": sum(new < old for old, new in zip(
            baseline["raw_times"], active["raw_times"]
        )),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
