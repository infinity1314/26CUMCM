#!/usr/bin/env python3
"""Paired offline benchmark of the certified 25- and 26-station policies."""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
from pathlib import Path

import robot_strategy as strategy
from offline_verify import OfflineResponses


def stations_25_without_center():
    """Archived translated-lattice layout used before the concentric layout."""
    spacing = 989.0
    row_height = spacing * math.sqrt(3.0) / 2.0
    offset_x, offset_y = 816.0, 100.0
    route_indices = [
        (-1, 0), (-1, 1), (-2, 1), (-2, 0), (-1, -1),
        (0, -2), (0, -1), (1, -1), (0, 0), (0, 1),
        (-1, 2), (-2, 2), (-3, 2), (-3, 1), (-3, 0),
        (-2, -1), (-1, -2), (0, -3), (1, -3), (1, -2),
        (2, -2), (2, -1), (1, 0), (1, 1), (0, 2),
    ]
    return [
        (spacing * (i + j / 2.0) + offset_x, row_height * j + offset_y)
        for i, j in route_indices
    ]


STATIONS_25 = stations_25_without_center


def stations_26():
    return [(0.0, 0.0), *STATIONS_25()]


def make_case(seed: int):
    rng = random.Random(seed)
    count = rng.randint(10, 16)
    channels = sorted(rng.sample(range(1, 21), count))
    directional_count = round(count * 5 / 16)
    directional = set(rng.sample(channels, directional_count))
    sources = {}
    for channel in channels:
        radial = 1780.0 * math.sqrt(rng.random())
        angle = rng.random() * 2.0 * math.pi
        point = (radial * math.cos(angle), radial * math.sin(angle))
        direction = rng.random() * 2.0 * math.pi if channel in directional else None
        sources[channel] = (point, rng.uniform(1000.0, 1500.0), direction)
    return sources, directional_count


def run_layout(sources, station_provider):
    original_stations = strategy.p4_search_stations
    original_search = strategy.search_phase

    def tracked_search(client, problem):
        result = original_search(client, problem)
        client.search_time = client.virtual_time
        client.search_measures = client.measures
        client.search_clears = client.clears
        return result

    strategy.p4_search_stations = station_provider
    strategy.search_phase = tracked_search
    try:
        oracle = OfflineResponses(sources)
        cleared, unresolved, absent = strategy.run_problem(oracle, 4)
    finally:
        strategy.p4_search_stations = original_stations
        strategy.search_phase = original_search

    expected = sorted(sources)
    expected_absent = [channel for channel in range(1, 21) if channel not in sources]
    assert cleared == expected
    assert not unresolved
    assert absent == expected_absent
    return {
        "total_time_s": oracle.virtual_time,
        "search_time_s": oracle.search_time,
        "localization_time_s": oracle.virtual_time - oracle.search_time,
        "measure_commands": oracle.measures,
        "clear_commands": oracle.clears,
        "search_measure_commands": oracle.search_measures,
        "search_clear_commands": oracle.search_clears,
    }


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def aggregate(cases):
    differences = [case["policy_26"]["total_time_s"]
                   - case["policy_25"]["total_time_s"] for case in cases]
    mean_difference = statistics.mean(differences)
    standard_error = statistics.stdev(differences) / math.sqrt(len(differences))
    by_count = {}
    for source_count in range(10, 17):
        subset = [case for case in cases if case["source_count"] == source_count]
        if not subset:
            continue
        subset_differences = [case["policy_26"]["total_time_s"]
                              - case["policy_25"]["total_time_s"]
                              for case in subset]
        by_count[str(source_count)] = {
            "cases": len(subset),
            "mean_25_s": statistics.mean(
                case["policy_25"]["total_time_s"] for case in subset
            ),
            "mean_26_s": statistics.mean(
                case["policy_26"]["total_time_s"] for case in subset
            ),
            "mean_25_per_source_s": statistics.mean(
                case["policy_25"]["total_time_s"] / source_count
                for case in subset
            ),
            "mean_26_per_source_s": statistics.mean(
                case["policy_26"]["total_time_s"] / source_count
                for case in subset
            ),
            "wins_25": sum(value > 1e-9 for value in subset_differences),
            "wins_26": sum(value < -1e-9 for value in subset_differences),
            "mean_saving_25_s": statistics.mean(subset_differences),
        }

    return {
        "cases": len(cases),
        "all_cleared_25": True,
        "all_cleared_26": True,
        "mean_total_25_s": statistics.mean(
            case["policy_25"]["total_time_s"] for case in cases
        ),
        "mean_total_26_s": statistics.mean(
            case["policy_26"]["total_time_s"] for case in cases
        ),
        "median_total_25_s": statistics.median(
            case["policy_25"]["total_time_s"] for case in cases
        ),
        "median_total_26_s": statistics.median(
            case["policy_26"]["total_time_s"] for case in cases
        ),
        "mean_search_25_s": statistics.mean(
            case["policy_25"]["search_time_s"] for case in cases
        ),
        "mean_search_26_s": statistics.mean(
            case["policy_26"]["search_time_s"] for case in cases
        ),
        "mean_localization_25_s": statistics.mean(
            case["policy_25"]["localization_time_s"] for case in cases
        ),
        "mean_localization_26_s": statistics.mean(
            case["policy_26"]["localization_time_s"] for case in cases
        ),
        "wins_25": sum(value > 1e-9 for value in differences),
        "wins_26": sum(value < -1e-9 for value in differences),
        "ties": sum(abs(value) <= 1e-9 for value in differences),
        "mean_saving_25_s": mean_difference,
        "median_saving_25_s": statistics.median(differences),
        "p10_saving_25_s": percentile(differences, 0.10),
        "p90_saving_25_s": percentile(differences, 0.90),
        "minimum_saving_25_s": min(differences),
        "maximum_saving_25_s": max(differences),
        "mean_saving_25_approx_95pct_ci_s": [
            mean_difference - 1.984 * standard_error,
            mean_difference + 1.984 * standard_error,
        ],
        "by_source_count": by_count,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2026091101)
    parser.add_argument("--output", type=Path,
                        default=Path("output/p4_25_vs_26_100cases.json"))
    args = parser.parse_args()
    if args.cases < 2:
        parser.error("--cases must be at least 2")

    cases = []
    for index in range(args.cases):
        seed = args.seed + index
        sources, directional_count = make_case(seed)
        result_25 = run_layout(sources, STATIONS_25)
        result_26 = run_layout(sources, stations_26)
        cases.append({
            "case": index + 1,
            "seed": seed,
            "source_count": len(sources),
            "directional_count": directional_count,
            "policy_25": result_25,
            "policy_26": result_26,
            "saving_25_s": result_26["total_time_s"] - result_25["total_time_s"],
        })

    document = {
        "method": (
            "paired deterministic offline oracle; identical sources for both layouts; "
            "positive saving means the 25-station policy is faster"
        ),
        "base_seed": args.seed,
        "summary": aggregate(cases),
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(document["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
