# Open issue: the speed run stops in every cell

## Problem

`doSpeedRun` calls `navigate(true, RUN_SPEED, 400, false, true)`. That is the same per cell loop exploration uses. In every cell the robot:

1. floods the map and picks a direction
2. turns if needed
3. calls `advanceOneCell`, which calls `driveStraight` for one `CELL_MM`. This means a fresh odometry reset, a 25 ms kick start, a 60 mm acceleration ramp, a 60 mm deceleration ramp, a 40 ms reverse brake and a 70 ms settle
4. calls `refreshSensors(200)` before deciding whether the move succeeded

With 186 mm cells and 60 mm ramps at each end, the robot spends most of each cell speeding up or slowing down, and it stops completely at every cell boundary. Raising `RUN_SPEED` changes very little.

## Proposed fix

The speed run uses the known only map, so the whole path is fixed before the robot moves. The run can therefore be planned up front and driven as a sequence of segments.

1. **Precompute the path.** After `floodFill(true, true)`, walk down the distance gradient from the start cell to the goal and record the absolute direction of each step.
2. **Compress it into segments.** Merge consecutive equal directions into one straight segment of N cells, with a turn between segments.
3. **Drive each straight segment as one move.** Call `driveStraight(N * CELL_MM, RUN_SPEED, true, false, ...)` once, so there is one acceleration ramp, one cruise and one deceleration per segment. Wall following, the front wall distance measurement and the slowdown near a front wall all work as they already do.
4. **Turn between segments** with `turnTo`, as now.
5. **Update the position** by N cells at the end of each segment, and square up the heading when a front wall is visible.

## Things to handle

- **Odometry over long runs.** Over several cells, encoder distance drifts. Side wall endings (a wall flag going from `#` to `.`) happen at known positions within a cell, about `SENSOR_AHEAD_MM` before the cell boundary, and can be used to correct `travelled()` with `setTravelled`. The current code never needs this, because it reseeds every cell.
- **Failure mid segment.** If a segment stops early, the robot does not know how many cells it covered. The simplest safe fallback is to work out the nearest cell centre from `travelled()`, set the position to it, and hand over to the normal `navigate` loop for the rest of the run.
- **Lateral estimate across openings.** `latUpdate` already drops `latValid` after 400 ms without a reading. At run speed that is about 160 mm, nearly a whole cell. It may need to depend on speed or distance instead.
- **Consecutive cell stats.** `consecutiveCells` and `bestConsecutive` are counted per `advanceOneCell`. A segment of N cells should add N.

## Later steps, not needed first

- Curved (smooth) turns instead of stopping to turn in place
- Different speeds for long straight segments and short ones
- Diagonal paths
