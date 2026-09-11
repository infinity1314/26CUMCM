#!/usr/bin/env python3
"""Compare the current concentric P4 layout with both archived layouts."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import robot_strategy as strategy
from benchmark_25_vs_26 import (
    STATIONS_25,
    make_case,
    run_layout,
    stations_26,
)


LAYOUTS = {
    "25_without_center": STATIONS_25,
    "26_with_center": stations_26,
    "25_with_center_rings": strategy.p4_search_stations,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2026091101)
    parser.add_argument("--output", type=Path,
                        default=Path("output/p4_three_layouts_100cases.json"))
    args = parser.parse_args()

    cases = []
    for index in range(args.cases):
        seed = args.seed + index
        sources, directional_count = make_case(seed)
        cases.append({
            "case": index + 1,
            "seed": seed,
            "source_count": len(sources),
            "directional_count": directional_count,
            **{name: run_layout(sources, provider)
               for name, provider in LAYOUTS.items()},
        })

    summary = {}
    for name in LAYOUTS:
        summary[name] = {
            "mean_total_s": statistics.mean(
                case[name]["total_time_s"] for case in cases
            ),
            "median_total_s": statistics.median(
                case[name]["total_time_s"] for case in cases
            ),
            "mean_search_s": statistics.mean(
                case[name]["search_time_s"] for case in cases
            ),
            "mean_localization_s": statistics.mean(
                case[name]["localization_time_s"] for case in cases
            ),
        }

    names = list(LAYOUTS)
    for first_index, first in enumerate(names):
        for second in names[first_index + 1:]:
            savings = [
                case[second]["total_time_s"] - case[first]["total_time_s"]
                for case in cases
            ]
            summary[f"{first}_vs_{second}"] = {
                "wins_first": sum(value > 1e-9 for value in savings),
                "wins_second": sum(value < -1e-9 for value in savings),
                "mean_saving_first_s": statistics.mean(savings),
                "minimum_saving_first_s": min(savings),
                "maximum_saving_first_s": max(savings),
            }

    document = {
        "method": "paired deterministic offline oracle",
        "base_seed": args.seed,
        "summary": summary,
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
