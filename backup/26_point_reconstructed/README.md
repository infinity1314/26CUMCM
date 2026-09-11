# 26-point strategy backup

This directory preserves the 26-station problem 4 strategy used by the
`20260911_120305` through `20260911_120823` run logs.

## Provenance

This version was never committed to Git. Git contains committed 31-station and
28-station versions only. The 26-station route was reconstructed without
ambiguity from the run logs: it visits `(0,0)` first and then exactly the 25
translated triangular-lattice stations retained by the current strategy.

The executable is the self-contained archival artifact. The Python entry point
is a small compatibility wrapper around the project strategy snapshot and is
included to document and reproduce the single behavioral difference.

## Files

- `cumcm_robot_26.exe`: self-contained reconstructed executable.
- `robot_strategy_26.py`: reconstruction entry point.
- `p4_policy_26.json`: inspectable 26-station adaptive policy.
- `SHA256SUMS.txt`: integrity hashes.

No official simulator was started while creating this backup.
