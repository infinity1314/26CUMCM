#!/usr/bin/env python3
"""Quality-first second-station selection for CUMCM 2026 Problem B-2.

Problem 2 asks for a good localization result, while Problems 3 and 4 ask
for short total mission time.  This module therefore remains independent of
the runtime policy overlay: it minimizes a sampled bounded-error diameter and
uses travel only to break near ties.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import localization_geometry as geometry


Point = Tuple[float, float]
RECEPTION_MARGIN_M = 2.0


def _deduplicate(points: Iterable[Point], resolution: float = 1e-6) -> List[Point]:
    result: List[Point] = []
    seen = set()
    for point in points:
        key = (round(point[0] / resolution), round(point[1] / resolution))
        if key not in seen:
            seen.add(key)
            result.append(point)
    return result


def _boundary_samples(poly: Sequence[Point], count: int) -> List[Point]:
    if not poly:
        return []
    edges = list(zip(poly, list(poly[1:]) + [poly[0]]))
    lengths = [geometry.distance(a, b) for a, b in edges]
    perimeter = sum(lengths)
    if perimeter <= 1e-9:
        return [poly[0]]

    result = list(poly)
    for index in range(count):
        target = perimeter * index / count
        traversed = 0.0
        for edge_index, ((a, b), length) in enumerate(zip(edges, lengths)):
            if target <= traversed + length or edge_index == len(edges) - 1:
                fraction = 0.0 if length <= 1e-12 else (target - traversed) / length
                result.append((a[0] + fraction * (b[0] - a[0]),
                               a[1] + fraction * (b[1] - a[1])))
                break
            traversed += length
    return _deduplicate(result)


def _source_samples(poly: Sequence[Point], dense: bool) -> List[Point]:
    """Sample the complete conditional source polygon, including its interior."""
    if not poly:
        return []
    boundary = _boundary_samples(poly, 48 if dense else 16)
    centroid = (sum(point[0] for point in poly) / len(poly),
                sum(point[1] for point in poly) / len(poly))
    levels = (0.25, 0.5, 0.75) if dense else (0.5,)
    result = [centroid]
    result.extend(boundary)
    for level in levels:
        result.extend(
            (centroid[0] + level * (point[0] - centroid[0]),
             centroid[1] + level * (point[1] - centroid[1]))
            for point in boundary
        )
    return _deduplicate(result)


def sampled_worst_diameter(
    poly: Sequence[Point],
    observations: Sequence[geometry.Bearing],
    candidate: Point,
    *,
    dense: bool,
) -> float:
    """Evaluate the conditional bounded-error diameter objective numerically.

    Samples inconsistent with an earlier ``direction`` response are omitted.
    A source within 5 m of the candidate yields ``near`` and immediate clear,
    so it contributes zero localization diameter.
    """
    error_steps = 8 if dense else 4
    worst = 0.0
    range_limited_poly = geometry.clip_disk(list(poly), candidate, 1500.0)
    for source in _source_samples(poly, dense):
        if any(geometry.distance(source, item.point) <= 5.0
               for item in observations):
            continue
        if geometry.distance(source, candidate) <= 5.0:
            continue
        true_angle = math.atan2(source[1] - candidate[1],
                                source[0] - candidate[0])
        for index in range(error_steps + 1):
            error = (-1.0 + 2.0 * index / error_steps) * geometry.DEG
            updated = geometry.clip_bearing(
                list(range_limited_poly),
                geometry.Bearing(candidate, true_angle + error),
            )
            if updated:
                worst = max(worst, geometry.polygon_diameter(updated)[0])
    return worst


def _initial_candidates(
    poly: Sequence[Point],
    observations: Sequence[geometry.Bearing],
    current: Point,
) -> List[Point]:
    first = observations[0]
    ux, uy = math.cos(first.angle), math.sin(first.angle)
    wx, wy = -uy, ux
    candidates: List[Point] = []

    # Preserve the old answer as an explicit candidate, so the new direct
    # objective can reject it only when it measures a genuine improvement.
    baseline = geometry.guaranteed_receiver_station(poly, observations, current)
    if baseline is not None:
        candidates.append(baseline)

    # Fine candidates in coordinates aligned with the first bearing.
    for along in range(0, 1501, 50):
        for lateral in range(150, 1001, 50):
            for sign in (-1.0, 1.0):
                candidates.append((first.point[0] + along * ux + sign * lateral * wx,
                                   first.point[1] + along * uy + sign * lateral * wy))

    # The universal three-disk subset supplies reliable full-sector points.
    for radial in (850.0, 900.0, 925.0, 950.0, 975.0, 990.0, 998.0):
        for degrees in range(5, 86, 5):
            for sign in (-1.0, 1.0):
                angle = sign * math.radians(degrees)
                candidates.append((first.point[0] + radial * (math.cos(angle) * ux
                                                               + math.sin(angle) * wx),
                                   first.point[1] + radial * (math.cos(angle) * uy
                                                               + math.sin(angle) * wy)))

    # Arena clipping can enlarge the exact reception region asymmetrically.
    # Candidates around the conditional polygon capture those boundary cases.
    mec = geometry.minimum_enclosing_circle(poly)
    for radial in range(100, 901, 100):
        for index in range(24):
            angle = 2.0 * math.pi * index / 24.0
            candidates.append((mec.center[0] + radial * math.cos(angle),
                               mec.center[1] + radial * math.sin(angle)))
    return _deduplicate(candidates)


def _safe_candidates(
    candidates: Iterable[Point],
    poly: Sequence[Point],
    observations: Sequence[geometry.Bearing],
) -> List[Point]:
    previous = [item.point for item in observations]
    return [
        candidate
        for candidate in candidates
        if min(geometry.distance(candidate, point) for point in previous) >= 1.0
        and geometry.guaranteed_reception(
            poly, observations, candidate, margin=RECEPTION_MARGIN_M
        )
    ]


def quality_first_second_station(
    poly: Sequence[Point],
    observations: Sequence[geometry.Bearing],
    current: Point,
) -> Optional[Point]:
    """Return a guaranteed-reception approximate minimizer of worst diameter."""
    if not poly or not observations:
        return None

    candidates = _safe_candidates(
        _initial_candidates(poly, observations, current), poly, observations
    )
    if not candidates:
        return None

    coarse = sorted(
        (sampled_worst_diameter(poly, observations, point, dense=False),
         geometry.distance(current, point), point)
        for point in candidates
    )

    # Refine around several geometrically different leaders.  Every new point
    # is checked against the same exact reception predicate before scoring.
    leaders = [item[2] for item in coarse[:12]]
    refined = list(leaders)
    for step in (40.0, 15.0, 5.0):
        neighbours = []
        for point in leaders:
            for index in range(16):
                angle = 2.0 * math.pi * index / 16.0
                neighbours.append((point[0] + step * math.cos(angle),
                                   point[1] + step * math.sin(angle)))
        neighbours = _safe_candidates(_deduplicate(neighbours), poly, observations)
        scored = sorted(
            (sampled_worst_diameter(poly, observations, point, dense=False),
             geometry.distance(current, point), point)
            for point in neighbours
        )
        leaders = [item[2] for item in scored[:12]]
        refined.extend(leaders)

    refined_scores = sorted(
        (sampled_worst_diameter(poly, observations, point, dense=False),
         geometry.distance(current, point), point)
        for point in _deduplicate([item[2] for item in coarse[:16]] + refined)
    )
    # Keep the original coarse leaders as well as locally refined leaders;
    # local samples can rank them differently from the denser final score.
    finalists = _deduplicate(
        [item[2] for item in coarse[:8]]
        + [item[2] for item in refined_scores[:16]]
    )
    best = min(
        finalists,
        key=lambda point: (
            sampled_worst_diameter(poly, observations, point, dense=True),
            geometry.distance(current, point),
        ),
    )
    # Finish with a small direct-objective search.  This prevents the coarse
    # sampling order from deciding among nearby candidates.
    for step in (10.0, 2.0):
        neighbours = [best]
        for index in range(16):
            angle = 2.0 * math.pi * index / 16.0
            neighbours.append((best[0] + step * math.cos(angle),
                               best[1] + step * math.sin(angle)))
        neighbours = _safe_candidates(neighbours, poly, observations)
        best = min(
            neighbours,
            key=lambda point: (
                sampled_worst_diameter(poly, observations, point, dense=True),
                geometry.distance(current, point),
            ),
        )
    return best


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Choose Problem 2's second station after one direction reading."
    )
    parser.add_argument("--x", type=float, required=True,
                        help="x coordinate of the first measurement")
    parser.add_argument("--y", type=float, required=True,
                        help="y coordinate of the first measurement")
    parser.add_argument("--bearing-deg", type=float, required=True,
                        help="measured bearing in degrees")
    args = parser.parse_args()

    first = (args.x, args.y)
    observations = [geometry.Bearing(first, math.radians(args.bearing_deg % 360.0))]
    poly = geometry.feasible_polygon(observations)
    candidate = quality_first_second_station(poly, observations, first)
    if candidate is None:
        raise RuntimeError("no guaranteed-reception second station found")
    result = {
        "second_station": {"x": candidate[0], "y": candidate[1]},
        "move_distance_m": geometry.distance(first, candidate),
        "guaranteed_reception": geometry.guaranteed_reception(
            poly, observations, candidate, margin=RECEPTION_MARGIN_M
        ),
        "sampled_worst_diameter_m": sampled_worst_diameter(
            poly, observations, candidate, dense=True
        ),
        "status": "numerical approximate minimizer; reception is guaranteed",
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
