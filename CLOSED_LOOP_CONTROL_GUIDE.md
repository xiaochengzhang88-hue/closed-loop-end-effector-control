# Closed-Loop Pressure Control Guide

## 1. System overview

This project currently supports:

- Manual control of `motor1`
- Manual control of `motor2`
- One HX711 pressure sensor
- Closed-loop pressure control for `motor1` only
- `motor2` manual-only mode with no pressure feedback

Current design intent:

- `motor1` is the pressure-control motor
- `motor2` is available for manual movement only
- The single HX711 sensor is bound only to `motor1`
- The architecture leaves room to expand later to dual-sensor / dual-motor control

## 2. Wiring

### 2.1 Motor driver pins

These pins are defined in [src/main.cpp](/D:/CAR/CAR/src/main.cpp#L5):

- `motor1 STEP` -> `GPIO4`
- `motor1 DIR` -> `GPIO5`
- `motor1 EN` -> `GPIO6`
- `motor2 STEP` -> `GPIO7`
- `motor2 DIR` -> `GPIO15`
- `motor2 EN` -> `GPIO16`

### 2.2 HX711 pins

Also defined in [src/main.cpp](/D:/CAR/CAR/src/main.cpp#L11):

- `HX711 DT/DOUT` -> `GPIO17`
- `HX711 SCK` -> `GPIO18`

### 2.3 Common wiring notes

- Make sure the controller GND, motor driver GND, and HX711 GND share a common ground.
- HX711 `VCC` and logic level must match your board requirements.
- The pressure sensor connected to HX711 is currently the only pressure feedback source.
- That sensor is used only for `motor1` closed-loop control.
- `motor2` does not read pressure and does not participate in closed-loop logic.

## 3. Boot behavior

When the board starts:

- Serial starts at `115200`
- Both motors are initialized
- Both motor drivers start in disabled state
- HX711 starts on `GPIO17/GPIO18`
- The system attempts to load saved HX711 calibration from NVS
- If no valid calibration exists, the code performs an initial tare
- Help and current status are printed to the serial console

Main initialization entry:

- [src/main.cpp](/D:/CAR/CAR/src/main.cpp#L16)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L22)

## 4. Serial command summary

All commands are handled in:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L211)

### 4.1 Help and status

#### `mh`

Show the help menu.

Purpose:

- Print all supported motor, pressure, and closed-loop commands

#### `ms`

Show both motors status.

Purpose:

- Print enable state
- Print direction setting
- Print current logical position
- Print soft limits
- Print pulse width
- Print busy status
- Print queued steps

#### `ms m1`

Show `motor1` status only.

#### `ms m2`

Show `motor2` status only.

#### `ph`

Show pressure status and closed-loop status.

Purpose:

- Print HX711 calibration state
- Print current pressure
- Print closed-loop state
- Print routing information

#### `pp`

Show pressure calibration status.

#### `cls`

Show closed-loop status.

Purpose:

- Print whether closed-loop is enabled
- Print which motor is controlled
- Print whether the pressure sensor is assigned
- Print whether reverse correction is enabled
- Print target pressure range
- Print filter sample count
- Print the last filtered pressure

## 5. Manual motor control commands

Motor control core code:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L241)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L558)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L592)

### 5.1 `me <m1|m2> <0|1>`

Enable or disable a motor driver.

Examples:

- `me m1 1`
- `me m1 0`
- `me m2 1`
- `me m2 0`

Meaning:

- `1` = enable driver
- `0` = disable driver

Notes:

- When disabled, any queued movement is cleared immediately.
- You must enable a motor before motion commands will work.

### 5.2 `mdir <m1|m2> <0|1>`

Set the logical forward direction level for the driver.

Examples:

- `mdir m1 1`
- `mdir m1 0`

Purpose:

- Adjust direction polarity if your motor wiring or driver direction behavior is opposite to expectation

Important note:

- This does not move the motor.
- It changes how the code maps logical forward to the DIR pin level.

### 5.3 `mj <m1|m2> <steps> [pulse_us]`

Queue a jog move using signed steps.

Examples:

- `mj m1 100`
- `mj m1 -100`
- `mj m1 200 300`

Meaning:

- Positive steps = forward
- Negative steps = reverse
- Optional `pulse_us` changes step pulse width for that move only

### 5.4 `mrun <m1|m2> <f|r> <steps> [pulse_us]`

Queue a move with explicit forward or reverse direction.

Examples:

- `mrun m1 f 160`
- `mrun m1 r 160`
- `mrun m2 f 320 250`

Meaning:

- `f` = forward
- `r` = reverse

This is the recommended manual motion command when you want clear direction control.

### 5.5 `mlimit <m1|m2> <min> <max>`

Set software position limits.

Examples:

- `mlimit m1 -2000 2000`
- `mlimit m2 0 5000`

Purpose:

- Prevent the motor from moving outside a logical safe position range

Notes:

- Limits are checked before each step
- If a move would exceed the limit, motion stops

## 6. Pressure sensor commands

Pressure code areas:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L596)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L613)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L650)

### 6.1 `pt`

Tare the pressure sensor.

Purpose:

- Treat the current unloaded condition as zero
- Save zero offset to NVS

Use when:

- The pressure sensor is unloaded
- You want to reset the zero reference

### 6.2 `pc <known_grams>`

Calibrate the HX711 using a known weight.

Examples:

- `pc 100`
- `pc 200`

Purpose:

- Compute calibration factor in grams
- Save calibration factor and offset to NVS

Typical sequence:

1. Remove all load
2. Run `pt`
3. Place a known weight
4. Run `pc 100` or whatever your known mass is

### 6.3 `pr [samples]`

Read pressure in grams.

Examples:

- `pr`
- `pr 5`
- `pr 20`

Meaning:

- Optional `samples` controls how many HX711 samples are averaged for this read

## 7. Closed-loop commands

Closed-loop command handling:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L424)

### 7.1 `clmotor m1`

Assign the current pressure sensor to `motor1`.

Current behavior:

- Only `motor1` is allowed
- `motor2` cannot enter pressure closed-loop mode right now

If you try `clmotor m2`, the code rejects it.

### 7.2 `clrange <low> <high>`

Set the pressure target band in grams.

Examples:

- `clrange 50 60`
- `clrange 100 120`

Meaning:

- `low` = lower pressure limit
- `high` = upper pressure limit

Behavior:

- Below `low`, the system tries to move forward
- Above `high`, the system may reverse if reverse correction is enabled
- Inside the band, the system does not move

### 7.3 `cl <0|1>`

Disable or enable closed-loop control.

Examples:

- `cl 1`
- `cl 0`

Enable requirements:

- Pressure sensor must be calibrated
- Pressure sensor must be assigned to the axis
- The selected motor must be enabled
- The pressure range must be valid

### 7.4 `clreverse <0|1>`

Reverse the overall closed-loop motor direction.

Examples:

- `clreverse 0`
- `clreverse 1`

Meaning:

- `0` = use the normal closed-loop direction
- `1` = reverse all automatic closed-loop motor motion

Default:

- `0`

This command exists because some mechanisms are mounted in opposite direction, so the whole closed-loop action may need to be inverted.

## 8. Recommended operating sequence

### 8.1 First-time manual test

1. Power the system
2. Open serial monitor at `115200`
3. Run `ms`
4. Enable `motor1`: `me m1 1`
5. Test forward: `mrun m1 f 50`
6. Test reverse: `mrun m1 r 50`
7. Confirm motion direction is correct

### 8.2 First-time pressure calibration

1. Make sure the sensor is unloaded
2. Run `pt`
3. Put a known calibration weight on the sensor
4. Run `pc 100` or your known mass
5. Verify with `pr`

### 8.3 First-time closed-loop test for motor1

1. Enable `motor1`: `me m1 1`
2. Confirm pressure sensor is calibrated
3. Bind sensor to `motor1`: `clmotor m1`
4. Set pressure window: `clrange 50 60`
5. Keep reverse disabled first: `clreverse 0`
6. Enable closed-loop: `cl 1`
7. Observe pressure behavior

If closed-loop pushes in the wrong physical direction:

8. Run `clreverse 1`

## 9. Current control logic

Main runtime loop:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L95)

The loop currently does three things repeatedly:

1. Service non-blocking motor motion
2. Service closed-loop pressure logic
3. Read and handle serial commands

## 10. Motor control logic

### 10.1 Non-blocking move queue

The code no longer drives a full move inside one blocking `for` loop.

Instead:

- A command queues a move
- The loop sends steps gradually over time
- The rest of the program keeps running between steps

Relevant code:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L558)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L592)

Why this matters:

- Serial commands stay responsive
- Closed-loop pressure checks can continue running
- This is safer for rigid-contact pressure systems than a long blocking move

### 10.2 Busy behavior

If a motor is already moving:

- A new move command for that same motor is rejected
- You must wait until the current queued move finishes

Status fields printed by `ms` tell you:

- Whether the motor is busy
- How many queued steps remain

## 11. Pressure reading logic

### 11.1 Raw pressure read

Pressure reads come from HX711:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L690)

If calibrated:

- The reading is converted to grams using saved factor and offset

If not calibrated:

- The code returns the raw tare-removed value

### 11.2 Filtered pressure read

Closed-loop uses filtered pressure:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L709)

Current filter method:

- Take `3` reads
- Each read averages `8` HX711 samples by default
- Average the valid reads

This helps reduce noise and makes pressure control more stable.

## 12. Closed-loop control logic

Closed-loop execution:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L734)

Current logic is range-based control:

- If `pressure < low`, queue a small corrective move
- If `low <= pressure <= high`, do nothing
- If `pressure > high`, queue a small corrective move in the opposite direction

Direction mapping:

- `clreverse = 0` uses the default closed-loop direction
- `clreverse = 1` flips both low-pressure and high-pressure correction directions

This is not PID control.

It is a simpler band controller intended for initial tuning and safer early-stage testing.

## 13. Pressure routing logic

Routing display:

- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp#L205)

Current routing is fixed as:

- `pressure sensor 1 -> motor1 closed-loop`
- `motor2 -> manual only`

This prevents accidental sharing of one pressure sensor across both motors.

## 14. Safety and practical notes

### 14.1 Closed-loop reverse default

Closed-loop reverse is disabled by default.

Reason:

- If the mechanical direction is misunderstood, the whole closed-loop action could go the wrong way

That is why you can explicitly flip it with:

- `clreverse 1`

### 14.2 For a screw-driven rigid-contact system

Because the pressure sensor may touch a rigid object:

- Small adjustment steps are safer
- Non-blocking motor motion is safer than long blocking moves
- Pressure filter reduces noise-driven oscillation

### 14.3 1/16 microstepping

Your driver is `1/16` microstep.

If the motor is a standard `200 steps/rev` motor:

- `1 revolution = 200 * 16 = 3200 step pulses`

Examples:

- `160 pulses` = `1/20 revolution`
- `320 pulses` = `1/10 revolution`

Actual linear travel depends on your lead screw pitch.

## 15. Current limitations

Current limitations of this version:

- Only one HX711 pressure sensor is supported
- Only `motor1` can be pressure-controlled
- `motor2` has no pressure feedback
- Closed-loop uses simple range logic, not PID
- I could not run local PlatformIO build verification in this environment because `pio` is not available here

## 16. Key source files

- [src/main.cpp](/D:/CAR/CAR/src/main.cpp)
- [include/motor_calibration_console.h](/D:/CAR/CAR/include/motor_calibration_console.h)
- [src/motor_calibration_console.cpp](/D:/CAR/CAR/src/motor_calibration_console.cpp)
