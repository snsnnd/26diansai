# Competition Task Menu

The reusable menu is implemented by:

- `framework/ec_mode_manager`: task registration and lifecycle.
- `framework/ec_menu`: task selection and start/stop behavior.
- `framework/ec_keys`: EXTI input, debounce and event queue.

The framework does not depend on OLED. Each application supplies a render
callback; the line-car renderer is in `user/src/app/line_car.c`.

## Keys

- KEY1: select previous task while stopped; latched emergency stop while running.
- KEY2: select next task while stopped.
- KEY3: start the selected task, or stop the active task.

These are external active-low inputs, not the board's single PB21 `KEY` button:

- KEY1: PB6 to GND.
- KEY2: PB7 to GND.
- KEY3: PB23 to GND.

Internal pull-ups hold idle inputs high. OLED row 7 displays `RAW K:111` at
idle; pressing one key must change its corresponding digit to `0`. EXTI is the
primary event source and a 1 ms GPIO polling path provides a shared-interrupt
fallback. Key events also print as `[CAR][KEY]` on the debug UART.

After an emergency stop, release KEY1 and press KEY3 to acknowledge the latch;
press KEY3 again to start a task.

## Current LED/RGB bring-up mode

`LINE_CAR_LED_RGB_TEST_AUTOSTART=1` currently starts a motor-disabled hardware
test immediately after boot:

- KEY1 toggles LED1 and selects RGB red.
- KEY2 toggles LED2 and selects RGB green.
- KEY3 toggles LED3 and selects RGB blue.
- Reset the board to restart this temporary test.

Battery/T8 faults are shown while stopped but no longer prevent browsing the
menu. Starting a route with a required sensor fault still enters the latched
fault state and cannot drive the motors.

## Current Line-Car Tasks

1. `LINE FOLLOW`: T8 line input, PD steering, optional wheel-speed PID and
   bounded lost-line recovery.
2. `SPEED TEST`: straight voltage-compensated drive, optionally with the
   dual-wheel speed loop enabled.
3. `MOTOR L RAW` / `MOTOR R RAW`: drives only the selected motor at its configured
   startup PWM for five seconds. These modes do not use encoder feedback,
   so they isolate PWM, motor-driver and wiring faults. KEY3 stops early and KEY1
   remains the emergency stop.
4. `MOTOR DEADZONE`: tests the left and right motors independently with raw
   forward PWM. It ramps from 4000 to 10000 in steps of 250, records the first
   level producing four valid encoder edges, and displays `NO EDGE` if an
   encoder never responds. OLED row 6 shows the live left/right A/B input
   levels so disconnected, static, or incorrectly mapped encoder signals can
   be isolated before repeating the ramp.
5. `TUNING`: reusable parameter menu for target RPM, feedforward, speed/line/
   heading gains, direction, heading sign, `SPEED LOOP` selection and optional
   VOFA output. The default is open-loop PWM with battery-voltage compensation.
   Select `EXIT` and press KEY3 to return.
6. `GYRO TEST`: motor-disabled diagnostic page for the selected `CAR_GYRO_SOURCE`.
   M0 shows UART raw yaw/rate, checksum and overflow diagnostics; MPU6050 shows
   address/WHO_AM_I, acceleration and I2C diagnostics. It emits a diagnostic
   frame every 500 ms and pauses VOFA output while active.
7. `2024 H1` through `2024 H4`: non-blocking H-task state machines registered by
   `app/h2024_tasks`. They combine gyro heading hold on unpainted segments, T8
   following on arcs, and debounced line transitions at A/B/C/D. H4 repeats the
   crossed route four times and forward-aligns at A between laps. The default
   CCS line-car image starts stopped with `2024 H1` selected.

Inside `TUNING`, KEY1/KEY2 select an item, KEY3 enters edit mode, KEY1/KEY2
decrease/increase the value, and KEY3 leaves edit mode. The old page hierarchy
was retained but is now implemented by the data-driven
`framework/ec_parameter_menu` instead of hardcoding line-car variables.

The line-car battery reference is 14.8 V. PWM is multiplied by
`14.8 V / measured voltage`, limited to 0.85-1.30. Ten invalid samples or ten
samples below 13.0 V stop and latch the car. Recovery requires fifteen samples
at or above 14.0 V followed by KEY3 acknowledgement. `LINE KP`, `LINE KD`,
`LEFT GAIN`, and `RIGHT GAIN` tune open-loop tracking and motor mismatch. Lost
line searching is limited to 1.5 seconds before a latched stop.

When an H-task entry is active, OLED shows its route phase, heading/target/error,
and line enter/exit counts. `H DONE` is a stopped-motor completion state; press
KEY3 to return to the task menu. `H TIMEOUT` and `GYRO FAULT` require KEY3 fault
acknowledgement. See [COMPETITION_TASKS.md](COMPETITION_TASKS.md) for placement,
tuning, telemetry, and route details.

For complete MPU6050 wiring, units, status codes, code examples, and fault
isolation, see [MPU6050_USAGE_ZH.md](MPU6050_USAGE_ZH.md).

## Adding A Task

Implement start, run and stop callbacks, then add an `ec_mode_t` entry in
`register_car_modes()`. The run callback must be non-blocking and use `now_ms`
for timing. The stop callback must leave actuators safe.
