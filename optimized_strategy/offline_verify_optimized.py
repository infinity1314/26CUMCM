#!/usr/bin/env python3
"""Run the original completeness checks with the optimized policy installed."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import robot_strategy_optimized  # noqa: F401  (installs the policy overlay)
from offline_verify import main


if __name__ == "__main__":
    main()
