# Motor Test And Speed PID Tuning

This project has an independent motor test app module:

- Header: `user/inc/app/motor_test.h`
- Source: `user/src/app/motor_test.c`

The car should be lifted off the ground while running these tests.

`motor_test.c` is the single source of truth for test/tuning parameters: target RPM, base PWM, test mode, test PWM, and speed PID gains. The OLED menu only edits these values through pointers.

## Test Modes

Select `Test Mode` from the OLED menu.

- `PID Speed`: normal speed closed loop. Tune PID here.
- `Open PWM`: fixed PWM output. Use this to verify direction, wiring, and rough PWM-to-RPM response.
- `Deadzone +`: ramps positive PWM by 100 every 200 ms until either wheel exceeds 5 rpm.
- `Deadzone -`: ramps negative PWM by 100 every 200 ms until either wheel exceeds 5 rpm.

Deadzone modes output the actual electrical positive/negative motor command and do not apply the `Motor Dir` setting.

Use `Test PWM` to set the open-loop PWM value for `Open PWM` mode.

## VOFA 12 Channels

JustFloat frame, little-endian float, tail `00 00 80 7F`.

1. `target_rpm`
2. `avg_abs_rpm`
3. `left_abs_rpm`
4. `right_abs_rpm`
5. `left_cmd`
6. `right_cmd`
7. `left_pid_out`
8. `right_pid_out`
9. `feedforward_or_test_pwm`
10. `test_mode`
11. `battery_mv`
12. `deadzone_pwm`

## Tuning Order

1. Deadzone test:
   Run `Deadzone +` and `Deadzone -`. Send the VOFA log. The first `deadzone_pwm` where RPM exceeds about 5 rpm is the startup deadzone.

2. Open-loop response:
   Run `Open PWM` at several values above deadzone, for example deadzone + 500, +1000, +1500. Send the logs so PWM-to-RPM gain can be estimated.

3. Feedforward tuning:
   Use `FF Deadzone` and `FF Gain` to fit `feedforward_pwm = deadzone_pwm + target_rpm * k`. PID output should stay small after this.

4. KP-only tuning:
   Set `KI = 0`, `KD = 0`. Increase `KP` until response is fast but not oscillating. Send logs after each step.

5. Add KI:
   Increase `KI` slowly only after KP is acceptable. KI removes steady-state error but can cause slow oscillation.

6. KD usually stays 0:
   Add KD only if there is clear overshoot or oscillation that KP/KI cannot fix.

## Current Defaults

- `target_rpm = 80`
- `ff_deadzone_pwm = 5400`
- `ff_gain_k = 13.25 PWM/rpm`
- `KP = 8.0`
- `KI = 0.0`
- `KD = 0.0`
- `test_pwm = 3000`
- `VOFA = ON`
- Default motor direction is reverse for the current wiring.

## Current Measured Notes

- `Deadzone +` first motion appears around `5700 PWM`.
- In that scan, the right wheel moved earlier than the left wheel.
- `Deadzone -` first motion also appears around `5700 PWM`; both wheels are much more balanced in this direction.
- With `KP=8`, `KI=0`, `KD=0`, `base_pwm=5200`, PID Speed settled around `55 rpm` with one startup overshoot around `80 rpm`; base PWM was reduced to `5000` for the next test.
- With `KP=8`, `KI=0`, `KD=0`, `base_pwm=5000`, PID Speed settled around `44-45 rpm` with startup overshoot around `65 rpm`; base PWM was raised to `5100` for the next test.
- Fixed base PWM has been replaced by feedforward. Initial formula `5700 + target_rpm * 9.5` worked well at `80 rpm` but was high at `50 rpm`.
- Current two-point fit from 50/80 rpm data: `feedforward_pwm = 5400 + target_rpm * 13.25`.
