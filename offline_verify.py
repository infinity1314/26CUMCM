#!/usr/bin/env python3
"""Pure local verification; this file never opens an HTTP connection."""

from __future__ import annotations

import math
import random

from localization_geometry import (
    Bearing,
    DEG,
    covering_clear_points,
    distance,
    feasible_polygon,
    guaranteed_reception,
    guaranteed_receiver_station,
    minimum_enclosing_circle,
    point_in_convex,
)
from robot_strategy import (
    _directional_receiver_pair,
    p3_search_stations,
    p4_search_stations,
    run_problem,
)


class OfflineResponses:
    """Small protocol-shaped oracle used only to exercise policy branches."""

    def __init__(self, sources):
        self.sources = sources
        self.dead = set()
        self.current = (0.0, 0.0)
        self.current_channel = 1
        self.measures = 0
        self.clears = 0
        self.virtual_time = 0.0

    def measure(self, point, channel):
        self.virtual_time += distance(self.current, point) / 5.0
        if channel != self.current_channel:
            self.virtual_time += 1.0
        self.virtual_time += 5.0
        self.current = point
        self.current_channel = channel
        self.measures += 1
        if channel not in self.sources or channel in self.dead:
            return {"measure_result": "no_signal"}
        source, receive_radius, direction = self.sources[channel]
        source_range = distance(point, source)
        covered = direction is None or (
            (point[0] - source[0]) * math.cos(direction)
            + (point[1] - source[1]) * math.sin(direction) >= -1e-9
        )
        if source_range > receive_radius or not covered:
            return {"measure_result": "no_signal"}
        if source_range <= 5.0:
            return {"measure_result": "near"}
        true_angle = math.degrees(math.atan2(source[1] - point[1],
                                             source[0] - point[0]))
        # Fixed at a fixed station, as required by the problem statement.
        error = 0.97 * math.sin(0.013 * point[0] + 0.017 * point[1] + channel)
        return {"measure_result": "direction", "svd_deg": (true_angle + error) % 360.0}

    def clear(self, point, channel):
        self.virtual_time += distance(self.current, point) / 5.0
        self.current = point
        self.clears += 1
        if (channel in self.sources and channel not in self.dead
                and distance(point, self.sources[channel][0]) <= 20.0 + 1e-8):
            self.virtual_time += 5.0
            self.dead.add(channel)
            return {"clear_result": "success"}
        self.virtual_time += 3.0
        return {"clear_result": "no_target_in_range"}


def sampled_cover_radius(radius, stations):
    worst = 0.0
    for radial_index in range(181):
        radial = radius * radial_index / 180.0
        for angle_index in range(720):
            angle = 2.0 * math.pi * angle_index / 720.0
            point = (radial * math.cos(angle), radial * math.sin(angle))
            worst = max(worst, min(distance(point, station) for station in stations))
    return worst


def _convex_hull(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def verify_directional_discovery_layout():
    stations = p4_search_stations()
    radial_step = 10.0
    angular_step = math.radians(0.5)
    cell_radius = math.sqrt(
        1800.0 ** 2 + 1805.0 ** 2
        - 2.0 * 1800.0 * 1805.0 * math.cos(angular_step / 2.0)
    ) + 0.04
    minimum_hull_margin = math.inf
    checked = 0
    for radial_index in range(181):
        radial = radial_index * radial_step
        for angle_index in range(720):
            if radial == 0.0 and angle_index:
                break
            angle = angle_index * angular_step
            sample = (radial * math.cos(angle), radial * math.sin(angle))
            nearby = [station for station in stations
                      if distance(sample, station) <= 1000.0 - cell_radius]
            hull = _convex_hull(nearby)
            assert len(hull) >= 3
            margin = min(
                ((b[0] - a[0]) * (sample[1] - a[1])
                 - (b[1] - a[1]) * (sample[0] - a[0])) / distance(a, b)
                for a, b in zip(hull, hull[1:] + hull[:1])
            )
            assert margin >= cell_radius
            minimum_hull_margin = min(minimum_hull_margin, margin)
            checked += 1
    return {"p4_stations": len(stations),
            "continuous_cell_radius_m": cell_radius,
            "minimum_hull_margin_m": minimum_hull_margin,
            "certificate_samples": checked}


def verify_geometry():
    rng = random.Random(7)
    checked = 0
    for _ in range(300):
        source = (rng.uniform(-1200, 1200), rng.uniform(-1200, 1200))
        station = (rng.uniform(-500, 500), rng.uniform(-500, 500))
        if math.hypot(*source) > 1750 or distance(source, station) > 1499:
            continue
        true_angle = math.atan2(source[1] - station[1], source[0] - station[0])
        observation = Bearing(station, true_angle + rng.uniform(-0.999, 0.999) * DEG)
        poly = feasible_polygon([observation])
        assert point_in_convex(poly, source)
        clear_points = covering_clear_points(poly)
        assert min(distance(source, point) for point in clear_points) <= 19.000001
        checked += 1

    triangle = [(0.0, 0.0), (2.0, 0.0), (1.0, math.sqrt(3.0))]
    triangle_radius = minimum_enclosing_circle(triangle).radius
    assert abs(triangle_radius - 2.0 / math.sqrt(3.0)) < 1e-7
    return {"feasible_polygon_cases": checked,
            "equilateral_triangle_mec": triangle_radius}


def verify_second_station():
    rng = random.Random(20260911)
    checked = 0
    minimum_margin = math.inf
    while checked < 32:
        station = (rng.uniform(-900.0, 900.0), rng.uniform(-900.0, 900.0))
        source_range = rng.uniform(5.01, 1500.0)
        true_angle = rng.uniform(-math.pi, math.pi)
        source = (station[0] + source_range * math.cos(true_angle),
                  station[1] + source_range * math.sin(true_angle))
        if math.hypot(*source) > 1799.0:
            continue
        measured = true_angle + rng.uniform(-0.999, 0.999) * DEG
        observations = [Bearing(station, measured)]
        poly = feasible_polygon(observations)
        candidate = guaranteed_receiver_station(poly, observations, station)
        assert candidate is not None
        assert guaranteed_reception(poly, observations, candidate, margin=2.0)

        minimum_radius = max(1000.0, source_range)
        margin = minimum_radius - distance(candidate, source)
        assert margin >= -1e-7
        minimum_margin = min(minimum_margin, margin)
        checked += 1

    # This point uses the reception radius implied by the first detection. It
    # is guaranteed, but lies outside the old intersection of 1000 m disks.
    canonical_observations = [Bearing((0.0, 0.0), 0.0)]
    canonical_poly = feasible_polygon(canonical_observations)
    expanded_witness = (600.0, 700.0)
    assert guaranteed_reception(canonical_poly, canonical_observations,
                                 expanded_witness, margin=2.0)
    assert max(distance(expanded_witness, vertex)
               for vertex in canonical_poly) > 1000.0
    return {"second_station_cases": checked,
            "expanded_candidate_witness": expanded_witness,
            "minimum_reception_margin_m": minimum_margin}


def verify_directional_pair():
    worst_range_margin = math.inf
    worst_halfplane_margin = math.inf
    cases = 0
    for range_index in range(101):
        source_range = 20.0001 + (1500.0 - 20.0001) * range_index / 100.0
        for error_index in range(-20, 21):
            error = error_index / 20.0 * DEG
            observation = Bearing((source_range, 0.0), math.pi + error)
            pair = _directional_receiver_pair(observation)
            effective_radius = max(1000.0, source_range)
            worst_range_margin = min(
                worst_range_margin,
                min(effective_radius - distance((0.0, 0.0), point) for point in pair),
            )
            for direction_index in range(-90, 91):
                direction = math.radians(direction_index)
                dots = [point[0] * math.cos(direction) + point[1] * math.sin(direction)
                        for point in pair]
                worst_halfplane_margin = min(worst_halfplane_margin, max(dots))
                assert max(dots) >= -1e-8
                cases += 1
            assert all(distance((0.0, 0.0), point) <= effective_radius + 1e-8
                       for point in pair)
    return {"directional_pair_cases": cases,
            "minimum_range_margin_m": worst_range_margin,
            "minimum_halfplane_dot": worst_halfplane_margin}


def make_random_sources(seed, mixed):
    rng = random.Random(seed)
    sources = {}
    for channel in range(1, 11):
        radial = 1780.0 * math.sqrt(rng.random())
        angle = rng.random() * 2.0 * math.pi
        point = (radial * math.cos(angle), radial * math.sin(angle))
        direction = rng.random() * 2.0 * math.pi if mixed and channel % 2 == 0 else None
        sources[channel] = (point, rng.uniform(1000.0, 1500.0), direction)
    return sources


def sixteen_mixed_sources(seed):
    """Sixteen sources with the 11 omnidirectional / 5 directional mix."""
    rng = random.Random(seed)
    sources = {}
    directional_channels = set(rng.sample(range(1, 17), 5))
    for channel in range(1, 17):
        radial = 1780.0 * math.sqrt(rng.random())
        angle = rng.random() * 2.0 * math.pi
        point = (radial * math.cos(angle), radial * math.sin(angle))
        direction = (rng.random() * 2.0 * math.pi
                     if channel in directional_channels else None)
        sources[channel] = (point, rng.uniform(1000.0, 1500.0), direction)
    return sources


def boundary_outward_sources():
    sources = {}
    for channel in range(1, 11):
        angle = 2.0 * math.pi * (channel - 1) / 10.0
        point = (1800.0 * math.cos(angle), 1800.0 * math.sin(angle))
        sources[channel] = (point, 1000.0, angle)
    return sources


def verify_complete_policy(problem, sources):
    oracle = OfflineResponses(sources)
    cleared, unresolved, absent = run_problem(oracle, problem)
    expected = sorted(sources)
    expected_absent = [channel for channel in range(1, 21)
                       if channel not in sources]
    assert cleared == expected
    assert not unresolved and absent == expected_absent
    return {"problem": problem, "cleared": len(cleared),
            "measure_commands": oracle.measures, "clear_commands": oracle.clears,
            "virtual_time_s": oracle.virtual_time,
            "average_time_s": oracle.virtual_time / len(cleared)}


def main():
    print({"p3_sampled_cover_radius_m": sampled_cover_radius(1800.0, p3_search_stations())})
    print(verify_directional_discovery_layout())
    print(verify_geometry())
    print(verify_second_station())
    print(verify_directional_pair())
    print(verify_complete_policy(3, make_random_sources(93, False)))
    print(verify_complete_policy(4, make_random_sources(94, True)))
    print(verify_complete_policy(4, sixteen_mixed_sources(95)))
    print(verify_complete_policy(4, boundary_outward_sources()))


if __name__ == "__main__":
    main()
