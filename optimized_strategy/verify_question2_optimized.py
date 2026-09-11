#!/usr/bin/env python3
"""Regression checks for the independent Problem 2 selector."""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import localization_geometry as geometry
from question2_optimized import (
    quality_first_second_station,
    sampled_worst_diameter,
)


def run_case(station, bearing):
    observations = [geometry.Bearing(station, bearing)]
    poly = geometry.feasible_polygon(observations)
    old = geometry.guaranteed_receiver_station(poly, observations, station)
    new = quality_first_second_station(poly, observations, station)
    assert old is not None and new is not None
    assert geometry.guaranteed_reception(poly, observations, new, margin=2.0)
    old_score = sampled_worst_diameter(poly, observations, old, dense=True)
    new_score = sampled_worst_diameter(poly, observations, new, dense=True)
    return old, new, old_score, new_score


def main() -> None:
    old, new, old_score, new_score = run_case((0.0, 0.0), 0.0)
    assert new_score <= old_score + 1e-7
    assert min(geometry.distance(new, expected) for expected in (
        (833.9406067149831, 548.2217292955887),
        (833.9406067149831, -548.2217292955887),
    )) <= 25.0
    print({"case": "canonical", "old_score_m": old_score,
           "new_score_m": new_score, "new_station": new})

    old, new, old_score, new_score = run_case((1500.0, 0.0), math.pi / 2.0)
    assert new_score < 0.9 * old_score
    print({"case": "arena_boundary", "old_score_m": old_score,
           "new_score_m": new_score, "improvement_pct":
           100.0 * (old_score - new_score) / old_score,
           "new_station": new})

    rng = random.Random(20260911)
    checked = 0
    while checked < 8:
        station = (rng.uniform(-1600.0, 1600.0),
                   rng.uniform(-1600.0, 1600.0))
        source_radius = rng.uniform(5.01, 1500.0)
        true_angle = rng.uniform(-math.pi, math.pi)
        source = (station[0] + source_radius * math.cos(true_angle),
                  station[1] + source_radius * math.sin(true_angle))
        if math.hypot(*source) >= 1799.0:
            continue
        measured = true_angle + rng.uniform(-0.999, 0.999) * geometry.DEG
        observations = [geometry.Bearing(station, measured)]
        poly = geometry.feasible_polygon(observations)
        candidate = quality_first_second_station(poly, observations, station)
        assert candidate is not None
        assert geometry.guaranteed_reception(poly, observations, candidate,
                                             margin=2.0)
        effective_radius = max(1000.0, source_radius)
        assert geometry.distance(candidate, source) <= effective_radius + 1e-7
        checked += 1
    print({"case": "random_reception", "checked": checked})


if __name__ == "__main__":
    main()
