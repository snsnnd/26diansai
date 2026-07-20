# Driver Audit and Competition Framework

## Confirmed root causes

1. The PIT IRQ handlers did not read `IIDX`. After the first load event, the
   interrupt could remain pending and make the software millisecond counter run
   at ISR speed. RPM then used the wrong `dt_ms`.
2. The custom CCP counter started from `LOAD=65535`. Restarting after a clear
   loaded 65535 again, so a stationary encoder could report `-1` and every
   sample carried a fixed one-count offset.
3. The revised board routes the left encoder to TIMG8 PA26/PA27 and the right
   encoder to PA14/PB24. TIMG12 does not support QEI, so the right wheel now uses
   GPIO dual-edge AB x4 decoding while the left wheel remains on TIMG8 hardware QEI.
4. The GPIO decoder supports both single-phase and AB quadrature operation. The
   current right encoder selects its AB quadrature path.
5. Gimbal code treated a single-turn motor encoder as an absolute multi-turn
   gimbal encoder. Folding motor error to +/-180 degrees makes an 8:1 yaw axis
   distinguish only +/-22.5 output degrees.

## Encoder count definitions

Do not tune PID until one mechanical revolution has been measured and the count
definition is written down.

| Decoder | Counts per output revolution for 13 PPR, 30:1 |
| --- | ---: |
| A rising only + B direction | `13 * 30 = 390` |
| A both edges | `13 * 2 * 30 = 780` |
| AB quadrature x4 (optional) | `13 * 4 * 30 = 1560` |

The gearbox's real ratio may differ from the advertised 30:1. Mark the output
shaft, rotate it exactly 10 turns at low speed, and use:

```text
counts_per_rev = measured_edges / 10
rpm = delta_counts * 60000 / counts_per_rev / sample_ms
```

Use a logic analyzer to compare physical edges and software counts over the same
50 ms gate. First test with the motor unpowered and the encoder driven by hand,
then test under PWM load.

## Encoder hardware check

The current encoder counters run in TIMG8 QEI / right GPIO quadrature mode. The
left wheel is sampled through `encoder_get_delta()`; right-wheel edges are
accumulated by GPIO interrupts. Verify all four A/B signals with a logic analyzer
before enabling speed control. The left 16-bit modular delta requires the 50 ms
sampling interval to contain fewer than 32768 transitions.

## Current timer allocation

- TIMG0: framework 1 ms time base.
- TIMA0: four motor PWM channels.
- TIMG8: left encoder hardware QEI.
- GPIO PA14/PB24: right encoder software quadrature; TIMG12 QEI must not be used.
- Add external Schmitt buffering or RC filtering when encoder wires are long.

## Hardware profiles and conflicts

Select the firmware in `user/inc/config.h` with `EC_APP_PROFILE`:

- `EC_APP_PROFILE_HARDWARE_TEST`: UART1 gyro, UART2 EMM, UART3 vision.
- `EC_APP_PROFILE_LINE_CAR`: motor, wheel encoder, trace sensor, OLED.
- `EC_APP_PROFILE_EMPTY`: framework-only starting point.

Known conflicts that must not be enabled in one profile:

- PB15/PB16: line-car Bluetooth debug UART2 versus hardware-test EMM UART2.
- PB2/PB3: line-car T8 software I2C versus hardware-test MaixCam UART3.
- hardware-test uses UART0 PA10/PA11 for the M0 gyro and UART1 PA8/PA9 for
  debug; line-car PA8 is a TIMA0 motor PWM output.

## Framework layout

```text
user/src/main.c                 boot only
user/src/framework/ec_app.c     profile composition and emergency stop
user/src/framework/ec_time.c    monotonic 1 ms PIT time base
user/src/framework/ec_scheduler.c fixed-size cooperative scheduler
user/src/framework/ec_mode_manager.c competition task lifecycle
user/src/framework/ec_menu.c     task selection and start/stop behavior
user/src/framework/ec_keys.c     debounced key event queue
user/src/driver                 board/device drivers
user/src/gimbal                 gimbal domain and protocol
user/src/app                    competition state machines
```

The line-car profile demonstrates separate scheduled jobs for key input, gyro,
line sampling, wheel/battery sampling, control, menu rendering and telemetry.
Its mode menu registers `LINE FOLLOW`, `SPEED TEST`, `TUNING`, a motor-disabled `GYRO TEST`
slot and executable `2024 H1` through `2024 H4` route state machines.

## Removed redundant implementations

- The UART ring buffer moved from `gimbal` to `user/lib` and is shared by EMM,
  MaixCam and the serial gyro.
- `gimbal.c` was split into core motion, EMM transport and calibration modules.
- The reusable structure of the old parameter menu was extracted into
  `ec_parameter_menu`; its direct dependency on line-car globals was replaced
  by data-driven item descriptors and `car_tuning_t`.
- Unreferenced legacy entry points `user_app`, `gyro_z_test` and the
  uninitialized gimbal-T8 adapter were removed. Gimbal position and speed
  control are both retained as selectable strategies.

## Gimbal control strategies

- `position_control.c` sends bounded relative position increments and uses the
  central gimbal soft-limit path.
- `speed_control.c` sends continuous jog speed, converts output-axis deg/s to
  motor RPM using each gear ratio, stops on vision loss and maintains a bounded
  output-angle estimate for soft limits.
- `GIMBAL_DEFAULT_CONTROL_MODE` selects the startup strategy. Runtime code can
  call `hardware_test_set_gimbal_control_mode()` to switch after stopping.

Tasks must be short and non-blocking. OLED refresh, buzzer patterns, HC-05
configuration and telemetry are now separate services outside the control loop.
`system_delay_ms()` reconfigures SysTick and is acceptable only during controlled
startup and maintenance procedures.

## Gimbal position limitation

`gimbal_move_to()` now keeps the command in output-axis degrees, applies soft
limits, and multiplies by the axis gear ratio exactly once. Position control
uses this central entry instead of sending EMM pulses directly.

The 0x31 encoder is still single-turn. The displayed value selects the encoder
turn nearest the software target; this is not a substitute for homing. On power
up, after a stall, or after manual movement, home the mechanism or use a real
multi-turn position response before enabling automatic tracking.

## Remaining limitations

- Software I2C now records NACK/invalid-parameter status and new code uses
  checked transactions. Legacy void APIs remain for compatibility.
- MPU6050 provides raw accelerometer, gyroscope, and temperature data. Line-car
  uses a calibrated Z-rate integrator for heading; Mahony remains the retained
  roll/pitch attitude solution and the unused DMP implementation was removed.
- HC-05 configuration is a non-blocking OK/ERROR state machine behind
  `EC_ENABLE_HC05`; it still needs an exclusive UART transport in its profile.
- OLED uses a framebuffer and dirty-page batch writes. A page transaction is
  still synchronous and should remain a low-priority task.
- Buzzer sequences are non-blocking. The compatibility `dt_buzzer_beep()` API
  remains blocking for old callers.
- UART IRQ handlers still need a full multi-source drain loop if TX/error IRQs
  are enabled; current profiles use RX callbacks.
- The project still needs a hardware watchdog and actuator command timeout for
  protection if the cooperative scheduler itself stalls.

## Bring-up order

1. Verify the PIT pin toggles or logged timestamp advances exactly 1000 per real
   second.
2. Verify raw encoder delta against a logic analyzer, with motors off and on.
3. Measure effective CPR; do not assume 780.
4. Verify motor direction and encoder sign separately for each wheel.
5. Close the speed loop with the chassis lifted and add command-timeout stop.
6. Home the gimbal, verify microstep=16 and each gear ratio, then test +/-5
   degree commands before enabling vision control.
