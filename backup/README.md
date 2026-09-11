# Strategy backup index

- `26_point_reconstructed/`: 26-point translated lattice with the center.
- `25_point_without_center/`: 25-point translated lattice with the center removed.
- `25_point_dual_ring_unified/`: active 25-point concentric strategy before
  splitting question 3 and question 4 into dedicated executables.
- `p3_7point_baseline_before_optimization/`: dedicated seven-station P3
  executable before time-aware localization was integrated.

The dedicated programs in `../dist/cumcm_robot_p3.exe` and
`../dist/cumcm_robot_p4.exe` replace the unified command-line entry point.
Question 4 uses the newer 25-point concentric layout: center + 12-point inner
ring + 12-point outer ring.
