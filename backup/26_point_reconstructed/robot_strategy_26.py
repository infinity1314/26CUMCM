#!/usr/bin/env python3
"""Reconstructed 26-station entry point preserved from the 2026-09-11 logs.

The logged route is exactly the current certified 25-station route with the
origin prepended.  Importing the main strategy keeps the localization, logging,
and command handling identical to the snapshot used to build this backup.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import robot_strategy as strategy


_write_policy = strategy.write_policy


def translated_lattice_25():
    """Return the archived 25-point translated-lattice route."""
    import math

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


def p4_search_stations_26():
    """Return the reconstructed route: origin followed by the 25-point route."""
    return [(0.0, 0.0), *translated_lattice_25()]


def write_policy_26(problem: int, robot_id: str, output: Path) -> None:
    _write_policy(problem, robot_id, output)
    if problem != 4:
        return
    policy = json.loads(output.read_text(encoding="utf-8"))
    policy["format"] = "cumcm2026b-adaptive-policy-v5-reconstructed"
    policy["provenance"] = (
        "Reconstructed from 20260911 26-station logs: origin plus the "
        "certified translated 25-station lattice route"
    )
    output.write_text(
        json.dumps(policy, ensure_ascii=False, indent=2), encoding="utf-8"
    )


strategy.p4_search_stations = p4_search_stations_26
strategy.write_policy = write_policy_26


if __name__ == "__main__":
    raise SystemExit(strategy.main())
