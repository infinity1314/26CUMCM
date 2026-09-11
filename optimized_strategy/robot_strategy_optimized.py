#!/usr/bin/env python3
"""Time-optimized policy overlay for CUMCM 2026 problem B.

The original implementation remains unchanged.  This module replaces only
the expensive P3 next-bearing selector and the certified P4 discovery layout,
then delegates protocol handling and fail-safe clearing to robot_strategy.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import localization_geometry as geometry
import robot_strategy as strategy


Point = Tuple[float, float]
INFORMATION_FRACTION = 0.10


def optimized_p4_search_stations() -> List[Point]:
    """Certified 28-station layout for directional-source discovery.

    For each point in the radius-1800 arena, the convex hull of stations less
    than 1000 m away contains that point.  Thus every emission half-plane has
    at least one receiving station.  The accompanying verifier converts a
    dense finite check into a continuous certificate.
    """
    stations: List[Point] = [(0.0, 0.0)]
    for count, radius in ((5, 900.0), (10, 1400.0), (12, 1900.0)):
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            stations.append((radius * math.cos(angle),
                             radius * math.sin(angle)))
    return strategy._two_opt_open(stations, (0.0, 0.0))


def _candidate_stations(poly: Sequence[Point], observations: Sequence[geometry.Bearing],
                        current: Point) -> List[Point]:
    """Generate guaranteed-reception candidates with short-move options."""
    first = observations[0]
    ux, uy = math.cos(first.angle), math.sin(first.angle)
    wx, wy = -uy, ux
    candidates: List[Point] = [current]

    # First exploit the robot's current location and small local moves.  This
    # is the part missing from the information-only selector.
    for radial in (100.0, 200.0, 300.0, 400.0, 500.0):
        for index in range(16):
            angle = 2.0 * math.pi * index / 16.0
            candidates.append((current[0] + radial * math.cos(angle),
                               current[1] + radial * math.sin(angle)))

    # Cautious-greedy candidates relative to the first bearing.
    for along in range(0, 1501, 100):
        for lateral in range(100, 1001, 100):
            for sign in (-1.0, 1.0):
                candidates.append((first.point[0] + along * ux + sign * lateral * wx,
                                   first.point[1] + along * uy + sign * lateral * wy))

    # Candidates around the current uncertainty set support later iterations.
    mec = geometry.minimum_enclosing_circle(poly)
    for radial in range(100, 901, 100):
        for index in range(24):
            angle = 2.0 * math.pi * index / 24.0
            candidates.append((mec.center[0] + radial * math.cos(angle),
                               mec.center[1] + radial * math.sin(angle)))
    return candidates


def time_aware_receiver_station(
    poly: Sequence[Point],
    observations: Sequence[geometry.Bearing],
    current: Point,
) -> Optional[Point]:
    """Choose a short move without discarding bearing observability.

    Tokekar's cautious-greedy idea supplies the observability constraint.  The
    contest objective, however, is elapsed time.  We therefore retain points
    with at least a fixed fraction of the best worst-case information and take
    the closest one.  Every retained point also passes the exact
    source-dependent guaranteed-reception test.  If no point survives, the
    original minimax selector remains the fallback.
    """
    if not poly or not observations:
        return None

    samples = geometry._localization_samples(poly)
    previous_points = [item.point for item in observations]
    ranked = []
    for candidate in _candidate_stations(poly, observations, current):
        if min(geometry.distance(candidate, point)
               for point in previous_points) < 1.0:
            continue
        if not geometry.guaranteed_reception(
            poly, observations, candidate, margin=2.0
        ):
            continue
        stations = previous_points + [candidate]
        worst_information = min(
            geometry._information_min_eigenvalue(source, stations)
            for source in samples
        )
        ranked.append((worst_information,
                       geometry.distance(current, candidate), candidate))

    if not ranked:
        return geometry.guaranteed_receiver_station(poly, observations, current)

    best_information = max(item[0] for item in ranked)
    cutoff = INFORMATION_FRACTION * best_information
    eligible = [item for item in ranked if item[0] >= cutoff]
    return min(eligible, key=lambda item: item[1])[2]


def install_optimized_policy() -> None:
    """Patch the two policy hooks while preserving all fail-safe fallbacks."""
    strategy.p4_search_stations = optimized_p4_search_stations
    strategy.guaranteed_receiver_station = time_aware_receiver_station


install_optimized_policy()


if __name__ == "__main__":
    raise SystemExit(strategy.main())
