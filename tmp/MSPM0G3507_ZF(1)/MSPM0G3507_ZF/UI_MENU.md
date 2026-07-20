# OLED Menu Controls

The OLED UI uses three levels:

1. Home page: shows car status, RPM, target RPM, base PWM, motor direction, VOFA state, and battery voltage.
2. Menu page: shows previous, current, and next menu item.
3. Function page: shows only the selected function's value/status.

## Global Keys

- KEY1: previous item or decrease value.
- KEY2: next item or increase value.
- KEY3: enter, confirm, or return to menu.
- On any non-home page, press KEY3 then KEY1 within 800 ms to return home.
- If a safety confirmation countdown is active, KEY3 keeps its confirmation behavior.

## Home Page

- KEY3: enter the menu page.
- When stopped: press KEY1, then press KEY2 within 1.5 seconds to start immediately.
- When stopped: press KEY2 alone to toggle VOFA output.
- When running: press KEY1 or KEY2 to stop immediately.

## Menu Page

- KEY1: previous menu item.
- KEY2: next menu item.
- KEY3: enter the selected function page.
- The OLED menu page only shows the current item and its index to keep the text large/readable.

## Function Pages

- Target RPM: KEY1 decreases by 10 rpm. KEY2 increases by 10 rpm. KEY3 returns to menu.
- FF Deadzone: KEY1 decreases by 100. KEY2 increases by 100. Used as the deadzone term in feedforward.
- FF Gain: KEY1 decreases by 0.5. KEY2 increases by 0.5. Used as `k` in `PWM = deadzone + rpm * k`.
- Test Mode: KEY1/KEY2 switches between PID Speed, Open PWM, Deadzone +, and Deadzone -.
- Test PWM: KEY1 decreases by 250. KEY2 increases by 250. Used in Open PWM mode.
- Motor Dir: KEY1 sets forward. KEY2 sets reverse. KEY3 returns to menu.
- PID KP: KEY1 decreases by 0.5. KEY2 increases by 0.5.
- PID KI: KEY1 decreases by 0.1. KEY2 increases by 0.1.
- PID KD: KEY1 decreases by 0.01. KEY2 increases by 0.01.
- VOFA Output: KEY1 turns VOFA off. KEY2 turns VOFA on. KEY3 returns to menu.
- Counter: KEY1 decreases by 1. KEY2 increases by 1. KEY3 returns to menu.
- Reset Counter: KEY3 resets after a second KEY3 press within 5 seconds. KEY1 returns to menu.

## Notes

- Start/stop is only available on the home page. Reset counter uses double confirmation to avoid accidental operation.
- Default target speed is 50 rpm.
- Default feedforward is `PWM = 5400 + target_rpm * 13.25`.
- VOFA is enabled by default.
- Motor direction reverses both motors together. Default direction is reverse for the current wiring.
- OLED no longer shows full key hints on every page to keep the display readable.
- VOFA mode outputs binary JustFloat data, so normal UART text output should not be mixed while it is enabled.
