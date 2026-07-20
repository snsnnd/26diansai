# MSPM0G3507 Competition Firmware

Bare-metal MSPM0G3507 firmware with a 1 ms monotonic time base, cooperative
scheduler, line-car competition profile, and a separate vision-gimbal hardware
test profile. The default CCS `Debug` image is the line-car application, not an
empty DriverLib example.

For the architecture, complete project-owned API reference, code reading order,
usage examples, extension rules, and safety constraints, see
[API_AND_CODE_GUIDE_ZH.md](API_AND_CODE_GUIDE_ZH.md).

当前 line-car 硬件版本的完整引脚表、接线和上电检查见
[HARDWARE_REVISION_2026.md](HARDWARE_REVISION_2026.md)。

## Hardware And Build

The application pin map targets the LCSC Tianmengxing MSPM0G3507 board. It uses
only PA/PB pins; the retained LaunchPad/SysConfig metadata is not the application
wiring definition. See `user/inc/config.h`, `user/inc/pin_mapping.h`, and the
complete guide before connecting hardware.

Line-car heading uses `driver/dt_heading`. Select the M0 UART gyro or MPU6050
backend with the single `CAR_GYRO_SOURCE` macro in `user/inc/config.h`; M0 is the
default.

The verified project products are MSPM0 SDK `2.10.0.04`, SysConfig `1.26.2`, and
TI Arm Clang `5.1.1.LTS`. Import `.project/.cproject` into CCS, build `Debug`,
then load the selected `.out` through `targetConfigs/MSPM0G3507.ccxml`. The batch
build script contains local `D:\Ti_M0\CCS` paths and must be adjusted for another
CCS installation.

For an external SWD probe, connect PA20/SWCLK, PA19/SWDIO, and GND using 3.3 V
logic. The battery sense input must go through the documented voltage divider
and must never be connected directly to PA25.

## Project Notes

- Complete Chinese architecture and project API guide: [API_AND_CODE_GUIDE_ZH.md](API_AND_CODE_GUIDE_ZH.md)
- SeekFree dependency API quick reference: [zf_api_reference.md](zf_api_reference.md)
- Competition task menu and keys: [UI_MENU.md](UI_MENU.md)
- Car speed PID tuning: [MOTOR_TEST_TUNING.md](MOTOR_TEST_TUNING.md)
- Driver audit, encoder diagnosis and competition framework: [DRIVER_AUDIT.md](DRIVER_AUDIT.md)
- H1-H4 route control, gyro tuning and odometry limits: [COMPETITION_TASKS.md](COMPETITION_TASKS.md)
- MPU6050 Chinese wiring, code, test and troubleshooting guide: [MPU6050_USAGE_ZH.md](MPU6050_USAGE_ZH.md)

## Application Profile

The CCS `Debug` configuration explicitly builds the 2024 line-car profile.
Run `build_profiles.bat` to perform clean line-car and hardware-test builds;
the separate images and maps are written to `BuildProfiles`. The common entry
point, monotonic time base and cooperative scheduler are under
`user/src/framework`.

`Debug/MSPM0G3507_ZF.out` and
`BuildProfiles/MSPM0G3507_ZF_line_car.out` both run the 2024 car menu. The image
starts safely stopped with `2024 H1` selected; use KEY1/KEY2 to choose H1-H4 and
press KEY3 to start. Hardware diagnostics remain available separately as
`BuildProfiles/MSPM0G3507_ZF_hardware_test.out`.
