# Calibration

The rules allow no reflashing once the maze is revealed, so everything below is done from the robot's own menu. Values are saved to NVS when you leave Settings with the stop button, and they survive power cycles and battery swaps.

## Editing a setting

Open Settings. Up and Down, or Select, move between parameters. The value only starts following the potentiometer once you have turned the knob noticeably, so moving to a parameter does not overwrite it. Press Stop to save everything and return to the menu.

## Order of work

Do these in order. Each step depends on the ones before it.

### 1. Gyro

Put the robot on a flat surface, keep it still and run **Calibrate gyro**. If it shows `MOVED!`, it was disturbed. Run it again.

This also clears the learned heading bias. Recalibrate whenever `bias` on the diagnostics screen is large and the robot drives oddly.

### 2. Signs

Open **Test: signs**.

- Rotate the robot clockwise by hand. `head` should go negative. If it goes positive, flip **Gyro sign**.
- Push the robot toward the left wall of a corridor. `lat` should go positive.
- Run **Test: turn 90**. The robot should turn right. If it turns left, flip **Turn sign**.
- Run **Test: corridor** starting off centre. The robot should move back toward the middle. If it drives into the wall it is nearest, flip **Steer sign**.

Never fix a sign by editing the code. The conventions are listed in the README.

### 3. Drive distance

Run **Test: drive 500** on open floor and measure how far the robot actually went. The wheel diameter is a compile time constant, so on the day you only check this. If it is far off, a wheel or tyre has changed.

### 4. Maze geometry

Set **Maze size** (16 for competition, smaller for practice). Set **Mirror L/R** if the maze extends to the robot's left from the start cell rather than to its right.

Measure the real cell pitch, from wall centre to wall centre, and set **Cell pitch mm**. The default of 186 is 174 mm clear plus a 12 mm wall.

### 5. Centring

Park the robot by hand at the exact centre of a cell, with walls on both sides and a wall in front. Open **Test: cell probe**.

- `F` goes into **Front centre mm**
- `L` goes into **Centre L mm**
- `R` goes into **Centre R mm**

The left and right values are expected to differ. Each side sensor is compared against its own centred reading, so do not average them.

Afterwards, open **Diagnostics** with the robot still centred. `sum` should be close to `want`, and `lat` close to zero.

### 6. Wall thresholds

In **Diagnostics**, move the robot through cells with and without side walls and watch the `#` and `.` flags next to each reading. **Side thresh mm** should sit comfortably between a near wall reading and an open side reading. The front threshold is fixed at 160 mm.

### 7. Speeds

**Explore speed** defaults to 250 mm/s and **Run speed** to 400 mm/s. Run **Test: one cell** and **Test: corridor** at the speed you plan to use. Lower it if the robot overshoots cells or hits walls. A crash costs 30 seconds, and a slower clean exploration is worth more than a faster one that needs a retrieval.

**Min move mm/s** is the speed below which the motors only hum. Raise it if the robot stalls at the end of moves.

## Control gains

**Head gain**, **Aim deg per mm**, **Aim learn rate** and **Max aim deg** are the cascade wall follower gains. The defaults were chosen in simulation and confirmed on the robot. Do not lower **Head gain** below 6, because the inner loop goes unstable below that.
