# Hardware reference

## Pin map

All of these were verified on the hardware.

| Function | GPIO |
| --- | --- |
| Motor IN1 (right) | 17 |
| Motor IN2 (right) | 19 |
| Motor IN3 (left) | 18 |
| Motor IN4 (left) | 23 |
| Right encoder A / B | 25 / 26 |
| Left encoder A / B | 27 / 14 |
| I2C SDA / SCL | 21 / 22 |
| XSHUT right ToF | 32 |
| XSHUT front ToF | 4 |
| XSHUT left ToF | 33 |
| Button select | 2 |
| Button up | 16 |
| Button down | 13 |
| Button stop | 5 |
| Potentiometer | 35 |

## I2C addresses

| Device | Address |
| --- | --- |
| VL53L0X right | 0x30 |
| VL53L0X front | 0x31 |
| VL53L0X left | 0x32 |
| LSM6DS3 | 0x6A |
| SSD1306 | 0x3C |

All three VL53L0X sensors power up at 0x29. At boot every sensor is held in reset through its XSHUT pin. They are then released one at a time, and each is given its own address before the next one wakes up.

The LSM6DS3 is configured at 104 Hz, 2000 dps. Its WHO_AM_I is accepted as either 0x69 or 0x6A, because different LSM6 variants report one or the other.

## ESP32 pin constraints

GPIO 34, 35, 36 and 39 are input only and cannot be used as outputs. The potentiometer sits on 35 for that reason.

GPIO 0, 2, 5, 12 and 15 are strapping pins and can interfere with boot. Select is on 2 and Stop is on 5. Both are inputs with pull ups and are only read after boot. Do not hold them down while powering on.

## Power

Three 18650 cells in series give 11.1V nominal, about 12.6V fully charged. Power goes through an LM2596 buck regulator. The VL53L0X breakout boards need 5V on VIN. At 3.3V they initialise unreliably or not at all.

## Measured constants

These were measured, not estimated.

| Constant | Value | How it was found |
| --- | --- | --- |
| `COUNTS_PER_REV` | 418 | encoder counts per wheel revolution |
| `WHEEL_DIAM_MM` | 31.97 | adjusted until a 500 mm drive command measured 500 mm |
| `TRACK_MM` | 80 | wheel centre to wheel centre |
| `DEADBAND_R` / `DEADBAND_L` | 25 | PWM at which each wheel starts turning |
| `MAX_SPEED` | 1295 mm/s | speed at full PWM |
| `GYRO_SCALE` | 0.074 | degrees per second per raw count, calibrated over repeated 360 degree turns |
| `CELL_MM` | 186 | 174 mm clear plus 12 mm wall |
| `SENSOR_AHEAD_MM` | 30 | how far the sensors sit ahead of the wheel axis |

## Fixed control values

| Constant | Value |
| --- | --- |
| Control loop | 100 Hz |
| Velocity PID | Kp 0.08, Ki 0.30, Kd 0.001, with a feedforward from the deadband and `MAX_SPEED` |
| Valid ToF range | 15 to 1500 mm |
| Wall flag hysteresis | 15 mm |
| Corridor width tolerance | 30 mm |
| Gyro weight in heading fusion | 0.95, encoders 0.05 |
| Sensor stale timeout | 250 ms |
