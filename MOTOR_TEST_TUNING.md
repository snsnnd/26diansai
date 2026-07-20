# Car Speed Tuning

Runtime tuning values are stored in `car_tuning_t`; their defaults and limits
are defined in one place:

- Header: `user/inc/app/car_tuning.h`
- Source: `user/src/app/car_tuning.c`
- Configuration: `user/inc/config.h`

Current defaults are target 10 rpm, base PWM 4900, feedforward gain 13.25,
speed `KP=8`, `KI=0`, `KD=0`, and line `KP=300`, `KD=0`. Base is 4900 while
minimum-run is 4500, which gives 400 PWM of deceleration-only steering room
without triggering the anti-stall boost. The T8 position error
is passed linearly to the line PID so each sensor-position step produces a
proportional correction instead of an increasingly steep edge correction. The
signed line correction is limited to 10000 PWM/s, so a left-to-right error
change must pass smoothly through zero instead of instantly swapping wheel
commands.
During the current chassis test, speed feedforward, wheel-speed PID and PWM voltage compensation
are disabled in `config.h`; steering therefore keeps the outer wheel at base
PWM initially and subtracts the line PID magnitude from the inner wheel. If that
would put the inner wheel below `MOTOR_MIN_RUN_PWM_L/R`, a slew-limited common
boost keeps the inner wheel running while progressively increasing the outer
wheel. `CAR_STEER_BOOST_MAX_PWM` and `CAR_STEER_BOOST_SLEW_PWM_PER_S` bound this
compensation at 800 PWM and 6000 PWM/s so steering does not create an
instantaneous acceleration step. `LINE FOLLOW` first drives both wheels at PWM
5500 for 20 ms without steering,
then switches to the 4500 low-speed base and enables normal line correction.
After losing the line, FIND pivots continuously at 4500 PWM until black is
detected in the center sensor region; it has no automatic timeout.
When feedforward is enabled, the command is:

```text
feedforward_pwm = base_pwm + target_rpm * feedforward_gain
```

Wheel speed uses AB quadrature: left TIMG8 hardware QEI on `PA26/PA27`, right
GPIO dual-edge decoding on `PA14/PB24`. The nominal 13 PPR, 30:1 configuration
is `1560 count/rev`; verify it against the assembled motor and gearbox.

For a motor-only check, select `MOTOR L RAW` or `MOTOR R RAW`. The selected
motor receives the 5500 startup PWM for five seconds while the other motor
remains at zero. The RAW value is kept at the startup threshold so it can
start from rest. This test deliberately ignores encoder data, so a
disconnected or failed encoder cannot terminate it. Lift the chassis before
starting; KEY3 stops normally and KEY1 performs an emergency stop.

The reusable test state machine is implemented in `app/motor_test.h` and
`app/motor_test.c`. It owns RAW timing and dead-zone sequencing but has no menu,
OLED or vehicle-global dependencies. `line_car` supplies motor-output and
encoder callbacks, then only registers the modes and renders their state.

## Startup boost test

Select `MOTOR L START` or `MOTOR R START` to test low-speed startup compensation
on one wheel at a time. The current test sends raw forward PWM 6500 for 150 ms,
then switches directly to PWM 4000 and holds it until KEY3 stops normally,
KEY1 performs an emergency stop, or a safety fault occurs. OLED shows the active
raw PWM command and both measured RPM values. The sequence bypasses line control,
speed PID, feedforward and battery PWM compensation.

The values are configured by `MOTOR_STARTUP_BOOST_PWM`,
`MOTOR_STARTUP_BOOST_DURATION_MS`, `MOTOR_STARTUP_LOW_PWM`, and
`MOTOR_STARTUP_LOW_DURATION_MS` in `user/inc/config.h`; a low-stage duration of
zero means continuous operation. The generic 30-40%
startup example is not used because this chassis previously measured startup
near PWM 5250/5500 on its configured 0-8000 range. Test with the wheel safely
loaded and enough room for a single-wheel pivot; KEY1 stops immediately.

Select `SPEED TEST` from the task menu while the chassis is lifted. VOFA output
contains target RPM, wheel RPM, motor commands, PID corrections, feedforward,
active task, battery voltage and line position.

Tune in this order: verify direction, measure encoder CPR, fit feedforward at
several speeds, increase KP, then add only enough KI to remove steady error.
Keep line KD at zero unless measured overshoot requires it. This PID differentiates
the error, so increasing KD also creates a steering spike when line error changes
abruptly; use derivative filtering and small increments if KD is later enabled.

## Automatic dead-zone test

Select `MOTOR DEADZONE` while the chassis is safely positioned with room for a
single-wheel pivot, then press KEY3. The mode tests the left wheel first, stops
for 500 ms, and then tests the right wheel. Raw forward PWM starts at 4000,
increases by 250 every 400 ms, and stops at the unified PWM maximum of 10000.
This path bypasses feedforward, battery compensation and the normal
minimum-running-PWM policy, so the displayed value is the physical PWM command.

The first PWM level that produces four valid encoder edges is displayed as the
wheel's startup threshold. The final result is also emitted on the debug UART:

```text
电机死区 left=5250 right=5500 edges=4,4
```

If a wheel reaches 10000 without four valid edges, the result is `NO EDGE`.
That means the firmware cannot distinguish a disconnected or misconfigured
encoder from a motor that did not move; repair encoder feedback before using
the automatic result. OLED row 6 and stopped-state sensor telemetry show live
encoder levels as `AB L:ab R:ab` and `ab=Lab/Rab`. Turn each wheel slowly by
hand: at least one digit for that wheel must change, and `enc` plus
`GPIO irq/evt` must increase. Press KEY1 for emergency stop or KEY3 to exit.

## Direct PWM and dead-zone test

The serial parser can command both motors directly while the task menu is in
the stopped state:

```text
cmd -3000
cmd -3000 -3200
stop
```

The first form applies the same command to both wheels. The second form uses
independent left and right commands. Values are raw duty commands in the range
`-10000..10000`; negative values match the current chassis forward direction.
These commands bypass line following, PD, feedforward, battery compensation and
the normal minimum-running-PWM policy. The command value is the physical PWM.

The parser refreshes an active command every 100 ms. Firmware stops the motors
if no refresh arrives for 300 ms. `stop`, `q`, closing the GUI, or pressing the
hardware emergency key stops the direct test.

Measure the loaded dead zone on the floor, starting at a low value and
increasing in small steps:

```text
cmd -2500
cmd -2750
cmd -3000
```

Record the first command at which each wheel starts reliably, then decrease in
the same steps and record where each wheel stops. Put the loaded continuous-run
thresholds in `MOTOR_MIN_RUN_PWM_L/R` in `user/inc/config.h`; then fit
`CAR_DEFAULT_BASE_PWM` and `CAR_DEFAULT_FEEDFORWARD_GAIN` independently. Do not
add the measured dead zone a second time in the output path.
