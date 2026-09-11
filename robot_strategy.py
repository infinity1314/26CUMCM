#!/usr/bin/env python3
"""Minimax adaptive command policy for CUMCM 2026 problem B.

No simulator is started by this file.  Once an operator starts a test, the
client sends exactly one action at a time, waits for feedback, updates the
source feasible sets, and only then creates the next command.  The resulting
JSONL log is the complete instruction sequence for that run.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from localization_geometry import (
    Bearing,
    Point,
    covering_clear_points,
    distance,
    feasible_polygon,
    guaranteed_receiver_station,
    minimum_enclosing_circle,
    nearest_neighbour_order,
    polygon_diameter,
)


CHANNELS = tuple(range(1, 21))
CLEAR_RADIUS = 20.0
SAFE_MEC_RADIUS = 19.0
P3_PROBE_RADIUS = 40.0
P4_SEARCH_REFINEMENT_RADIUS = 60.0


def p3_search_stations() -> List[Point]:
    """Seven-station covering with an almost shortest regular open route.

    Six 1000 m disks cannot cover B(0,1800); seven can.  For one central and
    six regular ring stations, 1125 m is just above the smallest ring radius
    that covers the outer boundary (1122.96 m).
    """
    ring = 1125.0
    return [(0.0, 0.0)] + [
        (ring * math.cos(k * math.pi / 3.0),
         ring * math.sin(k * math.pi / 3.0))
        for k in range(6)
    ]


def _two_opt_open(points: Sequence[Point], start: Point) -> List[Point]:
    route = nearest_neighbour_order(points, start)
    improved = True
    while improved:
        improved = False
        best_delta = -1e-7
        best_segment: Optional[Tuple[int, int]] = None
        anchored = bool(route and distance(route[0], start) <= 1e-8)
        for i in range(1 if anchored else 0, len(route)):
            previous = start if i == 0 else route[i - 1]
            for j in range(i + 1, len(route)):
                old = distance(previous, route[i])
                new = distance(previous, route[j])
                if j + 1 < len(route):
                    old += distance(route[j], route[j + 1])
                    new += distance(route[i], route[j + 1])
                delta = new - old
                if delta < best_delta:
                    best_delta = delta
                    best_segment = (i, j)
        if best_segment is not None:
            i, j = best_segment
            route[i:j + 1] = reversed(route[i:j + 1])
            improved = True
    return route


def p4_search_stations() -> List[Point]:
    """A certified 28-station triangular layout for directional discovery.

    At every possible source point, the convex hull of stations no farther
    than 1000 m contains that point.  Hence every directed emission half-plane
    contains at least one receiving station.  Twenty-seven translated lattice
    points provide the coverage; the origin is added for a high-yield first
    scan.  A continuous-cell certificate verifies the complete target disk.
    """
    spacing = 950.0
    row_height = spacing * math.sqrt(3.0) / 2.0
    offset_x = 0.45 * spacing
    offset_y = 0.35 * row_height
    lattice = [
        (spacing * (column + row / 2.0) + offset_x,
         row_height * row + offset_y)
        for row in range(-6, 7)
        for column in range(-6, 7)
    ]
    stations = heapq.nsmallest(
        27,
        lattice,
        key=lambda point: point[0] * point[0] + point[1] * point[1],
    )
    stations.append((0.0, 0.0))
    return _two_opt_open(stations, (0.0, 0.0))


@dataclass
class SourceState:
    channel: int
    observations: List[Bearing] = field(default_factory=list)

    def polygon(self) -> List[Point]:
        return feasible_polygon(self.observations)

    def radius(self) -> float:
        poly = self.polygon()
        return math.inf if not poly else minimum_enclosing_circle(poly).radius


@dataclass
class RobotClient:
    base_url: str
    robot_id: str
    log_path: Path
    sequence: int = 0
    current: Point = (0.0, 0.0)
    current_channel: int = 1
    deadline: Optional[float] = None
    virtual_time_s: float = 0.0

    def __post_init__(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w", encoding="utf-8")

    def close_log(self) -> None:
        self._log.close()

    def _request_id(self, kind: str) -> str:
        self.sequence += 1
        return f"{kind}-{self.sequence:06d}"

    def _post(self, path: str, payload: dict, retries: int = 2) -> dict:
        body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        last_error: Optional[Exception] = None
        for attempt in range(retries + 1):
            try:
                request = Request(self.base_url + path, body,
                                  headers={"Content-Type": "application/json"}, method="POST")
                with urlopen(request, timeout=5) as response:
                    result = json.loads(response.read().decode("utf-8"))
                self._record(path, payload, result)
                return result
            except HTTPError as exc:
                raw = exc.read().decode("utf-8", errors="replace")
                try:
                    result = json.loads(raw)
                except json.JSONDecodeError:
                    result = {"accepted": False, "http_status": exc.code, "raw": raw}
                self._record(path, payload, result)
                return result
            except (URLError, TimeoutError, OSError) as exc:
                last_error = exc
                if attempt < retries:
                    time.sleep(0.2)
        raise RuntimeError(f"request failed after retries: {path}: {last_error}")

    def _record(self, path: str, payload: dict, response: dict) -> None:
        if response.get("accepted") is True and "virtual_time_s" in response:
            self.virtual_time_s = float(response["virtual_time_s"])
        self._log.write(json.dumps({"path": path, "request": payload,
                                    "response": response}, ensure_ascii=True) + "\n")
        self._log.flush()

    def _base(self, kind: str) -> dict:
        return {"arena_id": "default", "robot_id": self.robot_id,
                "request_id": self._request_id(kind)}

    def enter(self) -> dict:
        result = self._post("/enter", self._base("enter"))
        if result.get("accepted") is not True:
            raise RuntimeError(f"/enter rejected: {result}")
        remaining = int(result.get("remaining_real_duration_s", 0))
        self.deadline = time.monotonic() + max(0, remaining - 8)
        return result

    def _check(self, result: dict, path: str) -> None:
        if result.get("accepted") is not True:
            raise RuntimeError(f"{path} rejected: {result}")

    def measure(self, point: Point, channel: int) -> dict:
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise TimeoutError("local real-time deadline reached")
        payload = self._base("measure")
        payload.update({"position": {"x": point[0], "y": point[1]}, "channel": channel})
        result = self._post("/measure", payload)
        self._check(result, "/measure")
        self.current = point
        self.current_channel = channel
        return result

    def clear(self, point: Point, channel: int) -> dict:
        if self.deadline is not None and time.monotonic() >= self.deadline:
            raise TimeoutError("local real-time deadline reached")
        payload = self._base("clear")
        payload.update({"position": {"x": point[0], "y": point[1]}, "channel": channel})
        result = self._post("/clear", payload)
        self._check(result, "/clear")
        self.current = point
        return result

    def exit(self) -> dict:
        return self._post("/exit", self._base("exit"), retries=0)


def _scan_order(active: Iterable[int], current_channel: int, reverse: bool) -> List[int]:
    channels = sorted(active, reverse=reverse)
    if current_channel in channels:
        channels.remove(current_channel)
        channels.insert(0, current_channel)
    return channels


def _add_direction(state: SourceState, station: Point, response: dict) -> None:
    angle = math.radians(float(response["svd_deg"]))
    state.observations.append(Bearing(station, angle))


def search_phase(client: RobotClient, problem: int) -> Tuple[Dict[int, SourceState], Set[int]]:
    stations = p3_search_stations() if problem == 3 else p4_search_stations()
    refinement_radius = (SAFE_MEC_RADIUS if problem == 3
                         else P4_SEARCH_REFINEMENT_RADIUS)
    states: Dict[int, SourceState] = {}
    cleared: Set[int] = set()

    for station_index, station in enumerate(stations):
        found_count = len(set(states) | cleared)
        # There are at most 16 sources.  In P4, once all 16 distinct channels
        # have been observed, further global stations cannot discover another
        # source and individual guaranteed localization is faster on average.
        if problem == 4 and found_count >= 16:
            break
        active: List[int] = []
        for channel in CHANNELS:
            if channel in cleared:
                continue
            if channel not in states:
                if found_count < 16:
                    active.append(channel)
            elif states[channel].radius() > refinement_radius:
                # Measurements at already-required search stations cost no
                # extra travel and may eliminate a later localization detour.
                # For P4, logs show that refining below 60 m spends more on
                # five-second measurements than it saves during final clear.
                active.append(channel)

        for channel in _scan_order(active, client.current_channel,
                                   reverse=bool(station_index % 2)):
            response = client.measure(station, channel)
            result = response["measure_result"]
            if result == "near":
                if client.clear(station, channel)["clear_result"] == "success":
                    cleared.add(channel)
                    states.pop(channel, None)
                continue
            if result == "direction":
                state = states.setdefault(channel, SourceState(channel))
                _add_direction(state, station, response)
    return states, cleared


def clear_polygon(client: RobotClient, state: SourceState) -> bool:
    """Guarantee a clear by covering the complete error-consistent polygon."""
    poly = state.polygon()
    if not poly:
        return False
    circle = minimum_enclosing_circle(poly)
    if circle.radius <= CLEAR_RADIUS:
        return client.clear(circle.center, state.channel)["clear_result"] == "success"

    candidates = covering_clear_points(poly, cover_radius=19.0)
    for point in nearest_neighbour_order(candidates, client.current):
        if client.clear(point, state.channel)["clear_result"] == "success":
            return True
    return False


def localize_and_clear_p3(client: RobotClient, state: SourceState) -> bool:
    """Add guaranteed-reception bearings until one clear disk covers P."""
    probed_centres: List[Point] = []
    for _ in range(4):
        poly = state.polygon()
        if not poly:
            return False
        circle = minimum_enclosing_circle(poly)
        if circle.radius <= SAFE_MEC_RADIUS:
            return client.clear(circle.center, state.channel)["clear_result"] == "success"

        # Logs show that multi-bearing polygons just above the guaranteed
        # threshold usually have their true source within 20 m of the MEC
        # centre.  Probe once before paying for another long bearing detour;
        # failure costs only three seconds and the guaranteed fallback remains.
        if (circle.radius <= P3_PROBE_RADIUS and
                all(distance(circle.center, point) > 1.0 for point in probed_centres)):
            if client.clear(circle.center, state.channel)["clear_result"] == "success":
                return True
            probed_centres.append(circle.center)

        station = guaranteed_receiver_station(poly, state.observations, client.current)
        if station is None:
            break
        response = client.measure(station, state.channel)
        result = response["measure_result"]
        if result == "near":
            return client.clear(station, state.channel)["clear_result"] == "success"
        if result != "direction":
            # The station was selected within 998 m of the conservative set;
            # reaching this branch indicates numerical/model inconsistency.
            break
        _add_direction(state, station, response)
    return clear_polygon(client, state)


def _directional_receiver_pair(observation: Bearing) -> Tuple[Point, Point]:
    """Two stations of which at least one receives the same directional source.

    This is used only after a failed clear at the observation point, hence the
    source range is greater than 20 m.  The 19 m forward and +-170 m transverse
    displacement accounts for the full one-degree bearing error.
    """
    ux, uy = math.cos(observation.angle), math.sin(observation.angle)
    wx, wy = -uy, ux
    forward = (observation.point[0] + 19.0 * ux,
               observation.point[1] + 19.0 * uy)
    return ((forward[0] + 170.0 * wx, forward[1] + 170.0 * wy),
            (forward[0] - 170.0 * wx, forward[1] - 170.0 * wy))


def localize_and_clear_p4(client: RobotClient, state: SourceState) -> bool:
    """Guarantee new directional bearings with symmetric receiver pairs."""
    tried_anchor_clears: Set[Point] = set()
    for _ in range(5):
        poly = state.polygon()
        if not poly:
            return False
        circle = minimum_enclosing_circle(poly)
        if circle.radius <= SAFE_MEC_RADIUS:
            return client.clear(circle.center, state.channel)["clear_result"] == "success"

        # If the remaining region is already cheap to cover, another radio
        # detour cannot improve the minimax action count enough to justify it.
        cover = covering_clear_points(poly, cover_radius=19.0)
        if len(cover) <= 12:
            return clear_polygon(client, state)

        anchor = min(state.observations,
                     key=lambda item: distance(client.current, item.point))
        if anchor.point not in tried_anchor_clears:
            if client.clear(anchor.point, state.channel)["clear_result"] == "success":
                return True
            tried_anchor_clears.add(anchor.point)

        pair = sorted(_directional_receiver_pair(anchor),
                      key=lambda point: distance(client.current, point))
        received = False
        for station in pair:
            response = client.measure(station, state.channel)
            result = response["measure_result"]
            if result == "near":
                return client.clear(station, state.channel)["clear_result"] == "success"
            if result == "direction":
                _add_direction(state, station, response)
                received = True
                break
        if not received:
            # The geometry says this cannot occur for a valid prior direction
            # response; preserve the ultimate polygon-cover guarantee anyway.
            break
    return clear_polygon(client, state)


def _processing_order(states: Dict[int, SourceState], start: Point) -> List[int]:
    centres = {
        channel: minimum_enclosing_circle(state.polygon()).center
        for channel, state in states.items()
    }
    route = _two_opt_open(list(centres.values()), start)
    remaining = dict(centres)
    order: List[int] = []
    for point in route:
        channel = min(remaining, key=lambda item: distance(point, remaining[item]))
        order.append(channel)
        del remaining[channel]
    return order


def run_problem(client: RobotClient, problem: int) -> Tuple[List[int], List[int], List[int]]:
    states, cleared_set = search_phase(client, problem)
    unresolved: List[int] = []
    remaining = dict(states)
    while remaining:
        # Recompute after every source because an extra bearing or failed probe
        # may leave the robot away from the initially predicted clear centre.
        channel = _processing_order(remaining, client.current)[0]
        state = remaining.pop(channel)
        try:
            success = (localize_and_clear_p3(client, state)
                       if problem == 3 else localize_and_clear_p4(client, state))
        except (RuntimeError, TimeoutError):
            success = False
        if success:
            cleared_set.add(channel)
        else:
            unresolved.append(channel)
    absent = [channel for channel in CHANNELS
              if channel not in states and channel not in cleared_set]
    return sorted(cleared_set), unresolved, absent


def write_policy(problem: int, robot_id: str, output: Path) -> None:
    """Write the response-dependent command policy without contacting a port."""
    stations = p3_search_stations() if problem == 3 else p4_search_stations()
    policy = {
        "format": "cumcm2026b-adaptive-policy-v3",
        "problem": problem,
        "robot_id": robot_id,
        "search_stations": [{"x": x, "y": y} for x, y in stations],
        "channel_rule": ("scan 1..20 in snake order; skip cleared channels; stop refining a known "
                         "channel at MEC<=19m for P3 or <=60m for P4; stop unknown channels after 16 sources"),
        "direction_rule": "intersect arena, 1500m disks and all +/-1deg bearing wedges",
        "clear_rule": ("MEC centre if radius<=19m; P3 probes the MEC centre once at radius<=40m; "
                       "P4 first uses guaranteed symmetric receiver pairs; otherwise use a 19m "
                       "triangular cover of the feasible polygon"),
        "execution": "run robot_strategy.py; each next HTTP request depends on the previous response",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(policy, ensure_ascii=False, indent=2), encoding="utf-8")


def timestamped_log_path(path: Path, timestamp: str | None = None) -> Path:
    """Append a run timestamp while preserving the requested directory and suffix."""
    timestamp = timestamp or time.strftime("%Y%m%d_%H%M%S")
    suffix = path.suffix or ".jsonl"
    stem = path.stem if path.suffix else path.name
    return path.with_name(f"{stem}_{timestamp}{suffix}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--problem", type=int, choices=(3, 4), required=True)
    parser.add_argument("--robot-id", default=os.environ.get("ROBOT_ID", ""))
    parser.add_argument("--base-url", default="http://127.0.0.1:2026")
    parser.add_argument(
        "--log",
        type=Path,
        help="log filename prefix; a YYYYMMDD_HHMMSS timestamp is appended automatically",
    )
    parser.add_argument("--write-policy", type=Path,
                        help="write the adaptive policy only; do not connect to the simulator")
    args = parser.parse_args()
    if not args.robot_id:
        parser.error("--robot-id or ROBOT_ID is required")
    if args.write_policy:
        write_policy(args.problem, args.robot_id, args.write_policy)
        return 0

    log = timestamped_log_path(args.log or Path("output") / f"p{args.problem}_run.jsonl")
    client = RobotClient(args.base_url.rstrip("/"), args.robot_id, log)
    entered = False
    try:
        client.enter()
        entered = True
        cleared, unresolved, absent = run_problem(client, args.problem)
        average_time = (client.virtual_time_s / len(cleared)) if cleared else None
        summary = {
            "cleared_channels": cleared,
            "cleared_count": len(cleared),
            "unresolved_channels": unresolved,
            "proved_absent_channels": absent,
            "virtual_time_s": round(client.virtual_time_s, 6),
            "average_time_per_cleared_s": (
                round(average_time, 6) if average_time is not None else None
            ),
            "command_log": str(log),
        }
        print(json.dumps(summary, ensure_ascii=False))
        if average_time is None:
            print("平均每个已清除干扰源用时：无法计算（清除数量为 0）")
        else:
            print(f"平均每个已清除干扰源用时：{average_time:.3f} 秒")
        return 0 if not unresolved else 2
    except Exception as exc:
        print(f"fatal: {exc}", file=sys.stderr)
        return 1
    finally:
        if entered:
            try:
                client.exit()
            except Exception:
                pass
        client.close_log()


if __name__ == "__main__":
    raise SystemExit(main())
