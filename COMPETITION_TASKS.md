# Competition Task Modules

## Menu Integration

The OLED menu is backed by `ec_mode_manager`. Any mode registered with
`ec_mode_manager_add()` automatically participates in KEY1/KEY2 selection and
KEY3 start/stop behavior. The manager currently accepts up to 16 modes.

The line-car profile registers its reusable diagnostic modes first, then calls
`h2024_tasks_register()`. The resulting menu contains executable `2024 H1`
through `2024 H4` state machines. They use M0 gyro heading hold on unmarked
segments, T8 PD following on the black semicircles, and debounced T8 line
enter/exit counts for point transitions. Wheel odometry is reset for diagnostics
but is not used as a task completion condition.

The official field geometry is `AB=CD=100 cm`, `AD=BC=80 cm`, with only the
left and right radius-40-cm semicircles painted. The two diagonal headings are
38.66 degrees from the horizontal. The implemented routes are:

- H1: heading hold `A->B`, stop on the debounced line entry at B.
- H2: heading hold `A->B`, follow the right arc `B->C`, heading hold `C->D`,
  then follow the left arc `D->A`.
- H3: heading hold `A->C`, follow the right arc `C->B`, make a forward left
  transition of 38.66 degrees, heading hold `B->D`, then follow the left arc
  `D->A`.
- H4: repeat H3 for four laps. At each non-final A return, the known eastward
  arc-exit tangent creates a new local `A->C` target 38.66 degrees to the right;
  one wheel remains stopped while the other moves forward to that target. This
  prevents MPU6050 integration drift from accumulating across all four laps.

Place the T8 over the black endpoint at A before starting. Face B for H1/H2;
face C for H3/H4. The current gyro heading at KEY3 start becomes the first
unmarked-segment target, so no persistent zero operation is required. Every
reached point produces a 250 ms buzzer and PB22 LED indication. Completion
leaves the motors stopped and displays `H DONE` until KEY3 is pressed.

T8 state changes require three consecutive successful 10 ms samples. An arc
exit is accepted only after at least 800 ms and 130 degrees of heading change;
an earlier line loss is consumed as a tracking dropout rather than a route
point. Entry into the second arc is ignored until the second straight has lasted
at least 800 ms and 60 percent of the measured first-straight time. While an arc
is temporarily lost, line control retains the last steering direction to
reacquire it. Standalone and H-task following deliberately share the same
runtime-tunable `LINE KP/KD`; there is no task-specific gain or hidden scaling.
H1, H2, and H3 enforce their official 15 s, 30 s,
and 40 s limits. H4 has a separate 180 s fail-safe timeout, not an official
scoring limit. A missing gyro yaw frame for 250 ms latches `GYRO FAULT`; a
missing route event reaches `H TIMEOUT` and also stops the motors.

H-task motor control is open-loop PWM. Encoder RPM remains available for
diagnostics but is not used by H1-H4. Normal heading and arc tracking use
symmetric differential mixing around the straight base PWM and clamp both wheels
to their configured continuous-running thresholds. The deliberate A-point
forward-alignment action may still stop one wheel, and standalone `LINE FOLLOW`
lost-line recovery may still pivot in place.

Add another competition as a separate `user/src/app/<event>_tasks.c` module.
That module should own its task state machines and expose one registration
function that adds its `ec_mode_t` entries. The OLED renderer does not need to
be changed merely to add another mode.

## Encoder Odometry

### Current implementation

The current hardware uses timer QEI AB quadrature: left TIMG8 on `PA26/PA27`
and right TIMG12 on `PA14/PB24`. The timer peripherals count transitions while
the CPU is busy, removing the previous GPIO pending-bit edge-coalescing limit.
Callers must still account for all of the following:

- `ENCODER_CPR=1560` is derived from nominal 13 PPR, 30:1 and AB x4 decoding.
  Gearbox ratio and encoder definitions vary; measure the actual output-shaft
  count over multiple revolutions.
- `WHEEL_DIAMETER_MM=65` is only a bring-up value. Tyre compression, slip and
  manufacturing tolerance change effective travel per revolution.
- `ENCODER1_DIRECTION_SIGN` and `ENCODER2_DIRECTION_SIGN` must be checked on the
  assembled chassis. Forward wheel movement must increase both signed values.
- Reset odometry only while stopped. A coasting wheel after reset legitimately
  contributes to the next task's distance.
- Wheel slip is invisible to any wheel encoder. Encoder distance is not the
  same as ground-truth vehicle displacement.
- The 16-bit hardware counter is sampled with modular deltas. The scheduler
  must sample often enough that one interval never exceeds 32767 transitions.
- Wheel encoders remain relative sensors and are not safety-critical absolute
  position references.

Each `dt_encoder_t` now maintains:

- `edge_count`: total absolute QEI transitions since reset.
- `signed_edge_count`: signed QEI transitions after `direction_sign`.
- `invalid_transition_count`: GPIO fallback diagnostic; hardware QEI leaves it 0.
- `rpm`: the existing filtered speed measurement.

The public odometry operations are:

```c
dt_encoder_reset_odometry(&encoder);
uint32_t travel_edges = dt_encoder_get_edges(&encoder);
int32_t displacement_edges = dt_encoder_get_signed_edges(&encoder);
uint32_t invalid = dt_encoder_get_invalid_transitions(&encoder);
float travel_mm = dt_encoder_get_travel_mm(&encoder);
float displacement_mm = dt_encoder_get_distance_mm(&encoder);
```

`WHEEL_DIAMETER_MM` defaults to 65 mm only as a bring-up value. Measure the
effective rolling circumference under vehicle load and update it before using
distance thresholds. A useful calibration is:

```text
corrected_circumference = configured_circumference
                        * commanded_distance / measured_distance
```

The current wiring map and timer channels are documented in
`HARDWARE_REVISION_2026.md`. Verify each A/B phase and signed direction before
enabling the wheel-speed loop.

While an H task is active, OLED shows route heading, target, error, stable line
state, and line enter/exit counts. VOFA channels 15/16 remain left/right signed
distance and channels 17/18 remain left/right quadrature error counts.
Binary run frames also include the H state, yaw/target/error, line event counts,
stable on-line state, and the active `LINE KP/KD` values so route transitions
and online tuning can be diagnosed from one log.

## Heading Sensors

The line-car uses the common `dt_heading` interface. Its backend is selected by
the single `CAR_GYRO_SOURCE` macro:

- The default M0 backend uses UART0 TX PA10/RX PA11 at 115200 and supplies yaw
  plus Z angular rate directly.
- The optional MPU6050 backend uses software I2C SCL PA1/SDA PA0. It probes
  addresses 0x68 and 0x69, measures local Z bias, and integrates yaw at 100 Hz.
- Hardware-test: the M0 Z-axis module uses UART0 TX PA10/RX PA11 for independent
  module diagnostics.
- The retained `dt_imu_mahony` module remains available for roll/pitch users,
  but is not the line-car heading source.

When selected, MPU6050 heading initialization samples 1000 readings with a 2 ms delay and
requires at least 900 valid readings with low Z-rate variance after a 500 ms
settling delay and 32 discarded warm-up readings. Keep the chassis completely
stationary until the heading initialization frame appears (normally about 7 seconds with the
current software I2C timing). A failed initialization or a
missing successful sample for 250 ms prevents H-task motion and latches
`GYRO FAULT`.
Use the motor-disabled `GYRO TEST` menu task to inspect the selected source.
M0 displays raw yaw/rate and UART diagnostics; MPU6050 displays six-axis and
I2C diagnostics. Full Chinese instructions are in
[MPU6050_USAGE_ZH.md](MPU6050_USAGE_ZH.md).
The straight controller uses heading PD with yaw rate as the derivative
measurement and exposes `HEADING KP`, `HEADING KD`, `HEADING MAX`, and
`HEADING SIGN` in `TUNING`. Heading and line steering reduce the inner wheel
first. If it would fall below its minimum-running PWM, a slew-limited common
boost keeps it moving while progressively increasing the outer wheel.

Route heading is normalized so a physical right turn is positive. Set
`HEADING SIGN` in the stopped-motor `TUNING` mode so the displayed `Y` value
increases when the chassis is turned right by hand. Verify this before allowing
the wheels to touch the ground. Default gains (`KP=90`, `KD=3`, max steer
`2500`) are bring-up values, not field-calibrated values.

MPU6050 has no magnetometer, so accelerometer/Mahony processing cannot eliminate
yaw drift. Recalibrate after temperature changes and do not expect an absolute
compass heading. H2/H3 use local headings at arc exits, and H4 rebuilds its next
diagonal from the A tangent each lap to limit long-term drift.

VOFA channels 19-26 are route heading, route yaw rate, heading target, heading
error, gyro sample age in milliseconds, stable line state, line-enter count,
and line-exit count.
