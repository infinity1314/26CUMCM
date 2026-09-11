#!/usr/bin/env python3
"""Paired offline benchmark for the baseline and optimized policies."""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import robot_strategy as strategy
from offline_verify import OfflineResponses


BASELINE_P4_STATIONS = strategy.p4_search_stations
BASELINE_RECEIVER = strategy.guaranteed_receiver_station

import robot_strategy_optimized as optimized


def make_sources(seed: int, mixed: bool) -> dict:
    rng = random.Random(seed)
    count = rng.randint(10, 16)
    sources = {}
    for channel in rng.sample(range(1, 21), count):
        radial = 1800.0 * math.sqrt(rng.random())
        angle = rng.random() * 2.0 * math.pi
        point = (radial * math.cos(angle), radial * math.sin(angle))
        direction = (rng.random() * 2.0 * math.pi
                     if mixed and rng.random() < 0.5 else None)
        sources[channel] = (point, rng.uniform(1000.0, 1500.0), direction)
    return sources


def run_suite(problem: int, use_optimized: bool, cases: int) -> dict:
    strategy.p4_search_stations = (
        optimized.optimized_p4_search_stations
        if use_optimized else BASELINE_P4_STATIONS
    )
    strategy.guaranteed_receiver_station = (
        optimized.time_aware_receiver_station
        if use_optimized else BASELINE_RECEIVER
    )
    times = []
    measures = []
    clears = []
    failures = 0
    for index in range(cases):
        sources = make_sources(12000 + index, mixed=(problem == 4))
        oracle = OfflineResponses(sources)
        done, unresolved, _ = strategy.run_problem(oracle, problem)
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    args = parser.parse_args()
    if args.cases <= 0:
        parser.error("--cases must be positive")

    for problem in (3, 4):
        baseline = run_suite(problem, False, args.cases)
        improved = run_suite(problem, True, args.cases)
        reductions = [
            (old - new) / old * 100.0
            for old, new in zip(baseline["raw_times"], improved["raw_times"])
        ]
        print({
            "problem": problem,
            "cases": args.cases,
            "baseline": {key: round(value, 3)
                         for key, value in baseline.items() if key != "raw_times"},
            "optimized": {key: round(value, 3)
                          for key, value in improved.items() if key != "raw_times"},
            "mean_paired_reduction_pct": round(statistics.mean(reductions), 3),
            "minimum_paired_reduction_pct": round(min(reductions), 3),
        })


if __name__ == "__main__":
    main()
