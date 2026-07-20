#include "app/ui_menu.h"

typedef enum {
    UI_MENU_TARGET_RPM = 0,
    UI_MENU_BASE_PWM,
    UI_MENU_FF_K,
    UI_MENU_TEST_MODE,
    UI_MENU_TEST_PWM,
    UI_MENU_MOTOR_DIR,
    UI_MENU_PID_KP,
    UI_MENU_PID_KI,
    UI_MENU_PID_KD,
    UI_MENU_VOFA,
    UI_MENU_USER_COUNT,
    UI_MENU_RESET_COUNT,
    UI_MENU_COUNT
} ui_menu_item_t;

typedef enum {
    UI_SCREEN_HOME = 0,
    UI_SCREEN_MENU,
    UI_SCREEN_PAGE
} ui_screen_t;

typedef enum {
    UI_CONFIRM_NONE = 0,
    UI_CONFIRM_RESET_COUNT
} ui_confirm_t;

static ui_menu_context_t g_ctx;
static ui_screen_t g_screen = UI_SCREEN_HOME;
static ui_menu_item_t g_item = UI_MENU_TARGET_RPM;
static ui_confirm_t g_confirm = UI_CONFIRM_NONE;
static uint32_t g_confirm_deadline = 0;
static uint32_t g_home_start_deadline = 0;
static uint32_t g_home_return_deadline = 0;
static int16_t g_user_count = 0;

static int16_t ui_base_pwm(void)
{
    return (g_ctx.base_pwm != NULL) ? *g_ctx.base_pwm : 0;
}

static float ui_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t ui_is_running(void)
{
    return (g_ctx.base_speed != NULL && *g_ctx.base_speed > 0) ? 1u : 0u;
}

static const char *car_state_name(car_state_t state)
{
    static const char *names[] = {"INIT", "READY", "FOLLOW", "FIND", "STOP"};

    if ((uint32_t)state >= (sizeof(names) / sizeof(names[0])))
    {
        return "?";
    }
    return names[state];
}

static const char *item_name(ui_menu_item_t item)
{
    static const char *names[] = {
        "Target RPM",
        "FF Deadzone",
        "FF Gain",
        "Test Mode",
        "Test PWM",
        "Motor Dir",
        "PID KP",
        "PID KI",
        "PID KD",
        "VOFA Output",
        "Counter",
        "Reset Counter"
    };

    if ((uint32_t)item >= (sizeof(names) / sizeof(names[0])))
    {
        return "?";
    }
    return names[item];
}

static const char *test_mode_name(uint8_t mode)
{
    static const char *names[] = {"PID Speed", "Open PWM", "Deadzone +", "Deadzone -"};

    if (mode >= (sizeof(names) / sizeof(names[0])))
    {
        return "?";
    }
    return names[mode];
}

static void ui_beep(uint16_t ms)
{
    if (g_ctx.buzzer != NULL)
    {
        if (ms > 8u)
        {
            ms = 8u;
        }
        dt_buzzer_beep(g_ctx.buzzer, ms);
    }
}

static void clear_confirm(void)
{
    g_confirm = UI_CONFIRM_NONE;
    g_confirm_deadline = 0;
}

static void clear_home_start_arm(void)
{
    g_home_start_deadline = 0;
}

static void return_home(void)
{
    g_screen = UI_SCREEN_HOME;
    clear_confirm();
    clear_home_start_arm();
    g_home_return_deadline = 0;
}

static uint8_t home_start_is_armed(uint32_t now_ms)
{
    return (g_home_start_deadline != 0u && (int32_t)(g_home_start_deadline - now_ms) > 0) ? 1u : 0u;
}

static uint32_t confirm_left_s(uint32_t now_ms)
{
    if (g_confirm != UI_CONFIRM_NONE && (int32_t)(g_confirm_deadline - now_ms) > 0)
    {
        return ((g_confirm_deadline - now_ms) + 999u) / 1000u;
    }
    return 0;
}

static void request_confirm(ui_confirm_t confirm, uint32_t now_ms)
{
    if (g_confirm == confirm && (int32_t)(g_confirm_deadline - now_ms) > 0)
    {
        if (confirm == UI_CONFIRM_RESET_COUNT)
        {
            g_user_count = 0;
        }

        clear_confirm();
        ui_beep(60);
        return;
    }

    g_confirm = confirm;
    g_confirm_deadline = now_ms + 5000u;
    ui_beep(30);
}

static void start_now(void)
{
    if (g_ctx.start_cb != NULL)
    {
        g_ctx.start_cb(ui_base_pwm());
        ui_beep(8);
    }
}

static void stop_now(void)
{
    if (g_ctx.stop_cb != NULL)
    {
        g_ctx.stop_cb();
        ui_beep(8);
    }
}

static void adjust_value(int16_t delta)
{
    switch (g_item)
    {
    case UI_MENU_TARGET_RPM:
        if (g_ctx.target_rpm != NULL)
        {
            *g_ctx.target_rpm += (float)(delta * 10);
            if (*g_ctx.target_rpm < 0.0f) *g_ctx.target_rpm = 0.0f;
            if (*g_ctx.target_rpm > 500.0f) *g_ctx.target_rpm = 500.0f;
        }
        break;
    case UI_MENU_BASE_PWM:
        if (g_ctx.base_pwm != NULL)
        {
            *g_ctx.base_pwm += (int16_t)(delta * 100);
            if (*g_ctx.base_pwm < 0) *g_ctx.base_pwm = 0;
            if (*g_ctx.base_pwm > 10000) *g_ctx.base_pwm = 10000;
            if (g_ctx.base_speed != NULL && *g_ctx.base_speed > 0)
            {
                *g_ctx.base_speed = *g_ctx.base_pwm;
            }
        }
        break;
    case UI_MENU_FF_K:
        if (g_ctx.ff_k != NULL)
        {
            *g_ctx.ff_k += (float)delta * 0.5f;
            if (*g_ctx.ff_k < 0.0f) *g_ctx.ff_k = 0.0f;
        }
        break;
    case UI_MENU_TEST_PWM:
        if (g_ctx.test_pwm != NULL)
        {
            *g_ctx.test_pwm += (int16_t)(delta * 250);
            if (*g_ctx.test_pwm < 0) *g_ctx.test_pwm = 0;
            if (*g_ctx.test_pwm > 10000) *g_ctx.test_pwm = 10000;
        }
        break;
    case UI_MENU_PID_KP:
        if (g_ctx.pid_kp != NULL)
        {
            *g_ctx.pid_kp += (float)delta * 0.5f;
            if (*g_ctx.pid_kp < 0.0f) *g_ctx.pid_kp = 0.0f;
        }
        break;
    case UI_MENU_PID_KI:
        if (g_ctx.pid_ki != NULL)
        {
            *g_ctx.pid_ki += (float)delta * 0.1f;
            if (*g_ctx.pid_ki < 0.0f) *g_ctx.pid_ki = 0.0f;
        }
        break;
    case UI_MENU_PID_KD:
        if (g_ctx.pid_kd != NULL)
        {
            *g_ctx.pid_kd += (float)delta * 0.01f;
            if (*g_ctx.pid_kd < 0.0f) *g_ctx.pid_kd = 0.0f;
        }
        break;
    case UI_MENU_USER_COUNT:
        g_user_count += delta;
        if (g_user_count < 0) g_user_count = 0;
        break;
    default:
        break;
    }
}

static void show_line(uint8_t row, const char *text)
{
    char line[22];

    if (g_ctx.oled == NULL)
    {
        return;
    }

    snprintf(line, sizeof(line), "%-21s", text);
    dt_oled_show_string(g_ctx.oled, 0, row, line);
}

static void draw_home(uint32_t now_ms)
{
    char buf[22];
    car_state_t state = (g_ctx.car_state != NULL) ? *g_ctx.car_state : CAR_INIT;
    uint32_t left_s = confirm_left_s(now_ms);

    show_line(0, "      LINE CAR       ");
    show_line(1, ui_is_running() ? "        RUN          " : "        STOP         ");

    snprintf(buf, sizeof(buf), "State: %s", car_state_name(state));
    show_line(2, buf);

    snprintf(buf, sizeof(buf), "RPM %3d | %3d",
             (g_ctx.rpm_l != NULL) ? (int)ui_absf(*g_ctx.rpm_l) : 0,
             (g_ctx.rpm_r != NULL) ? (int)ui_absf(*g_ctx.rpm_r) : 0);
    show_line(3, buf);

    snprintf(buf, sizeof(buf), "Target:%3d FF:%4d",
             (g_ctx.target_rpm != NULL) ? (int)*g_ctx.target_rpm : 0,
             ui_base_pwm());
    show_line(4, buf);

    snprintf(buf, sizeof(buf), "Dir:%s VOFA:%s",
             (g_ctx.motor_dir != NULL && *g_ctx.motor_dir < 0) ? "REV" : "FWD",
             (g_ctx.vofa_mode != NULL && *g_ctx.vofa_mode) ? "ON" : "OFF");
    show_line(5, buf);

    snprintf(buf, sizeof(buf), "Bat:%5dmV",
             (g_ctx.bat_mv != NULL) ? *g_ctx.bat_mv : 0);
    show_line(6, buf);

    if (!ui_is_running() && home_start_is_armed(now_ms))
    {
        snprintf(buf, sizeof(buf), "Start armed");
        show_line(7, buf);
    }
    else if (left_s > 0)
    {
        snprintf(buf, sizeof(buf), "Confirm: %lus", (unsigned long)left_s);
        show_line(7, buf);
    }
    else
    {
        show_line(7, " ");
    }
}

static void draw_menu(void)
{
    char buf[22];

    show_line(0, "        MENU         ");
    show_line(1, " ");
    snprintf(buf, sizeof(buf), "        %u/%u", (unsigned)(g_item + 1), (unsigned)UI_MENU_COUNT);
    show_line(2, buf);
    show_line(3, "---------------------");
    snprintf(buf, sizeof(buf), "  %s", item_name(g_item));
    show_line(4, buf);
    show_line(5, "---------------------");
    show_line(6, " ");
    show_line(7, " ");
}

static void draw_page(uint32_t now_ms)
{
    char buf[22];
    uint32_t left_s = confirm_left_s(now_ms);

    show_line(0, item_name(g_item));
    show_line(1, "---------------------");

    switch (g_item)
    {
    case UI_MENU_TARGET_RPM:
        snprintf(buf, sizeof(buf), "Target RPM");
        show_line(2, buf);
        snprintf(buf, sizeof(buf), "      %4d", (g_ctx.target_rpm != NULL) ? (int)*g_ctx.target_rpm : 0);
        show_line(3, buf);
        snprintf(buf, sizeof(buf), "Now L:%3d R:%3d",
                 (g_ctx.rpm_l != NULL) ? (int)ui_absf(*g_ctx.rpm_l) : 0,
                 (g_ctx.rpm_r != NULL) ? (int)ui_absf(*g_ctx.rpm_r) : 0);
        show_line(4, buf);
        show_line(5, "Step: 10 rpm");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_BASE_PWM:
        show_line(2, "FF Deadzone PWM");
        snprintf(buf, sizeof(buf), "      %5d", ui_base_pwm());
        show_line(3, buf);
        snprintf(buf, sizeof(buf), "Measured deadzone");
        show_line(4, buf);
        show_line(5, "Step: 100 pwm");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_FF_K:
        show_line(2, "Feedforward Gain");
        snprintf(buf, sizeof(buf), "K x10: %5d", (g_ctx.ff_k != NULL) ? (int)(*g_ctx.ff_k * 10.0f) : 0);
        show_line(3, buf);
        show_line(4, "PWM = DZ + rpm*K");
        show_line(5, "Step: 0.5");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_TEST_MODE:
        show_line(2, "Motor Test Mode");
        snprintf(buf, sizeof(buf), "%s", (g_ctx.test_mode != NULL) ? test_mode_name(*g_ctx.test_mode) : "?");
        show_line(3, buf);
        snprintf(buf, sizeof(buf), "DZ PWM:%5d", (g_ctx.deadzone_pwm != NULL) ? *g_ctx.deadzone_pwm : 0);
        show_line(4, buf);
        show_line(5, "0 PID 1 Open 2/3 DZ");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_TEST_PWM:
        show_line(2, "Open Loop PWM");
        snprintf(buf, sizeof(buf), "      %5d", (g_ctx.test_pwm != NULL) ? *g_ctx.test_pwm : 0);
        show_line(3, buf);
        show_line(4, "Used in Open PWM");
        show_line(5, "Step: 250 pwm");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_MOTOR_DIR:
        show_line(2, "Motor Direction");
        snprintf(buf, sizeof(buf), "Status: %s",
                 (g_ctx.motor_dir != NULL && *g_ctx.motor_dir < 0) ? "REVERSE" : "FORWARD");
        show_line(3, buf);
        show_line(4, "Both motors change");
        show_line(5, " ");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_PID_KP:
        show_line(2, "Speed PID KP");
        snprintf(buf, sizeof(buf), "KP x10: %5d", (g_ctx.pid_kp != NULL) ? (int)(*g_ctx.pid_kp * 10.0f) : 0);
        show_line(3, buf);
        show_line(4, "Step: 0.5");
        show_line(5, "Tune KP first");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_PID_KI:
        show_line(2, "Speed PID KI");
        snprintf(buf, sizeof(buf), "KI x10: %5d", (g_ctx.pid_ki != NULL) ? (int)(*g_ctx.pid_ki * 10.0f) : 0);
        show_line(3, buf);
        show_line(4, "Step: 0.1");
        show_line(5, "Use after KP");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_PID_KD:
        show_line(2, "Speed PID KD");
        snprintf(buf, sizeof(buf), "KD x100:%5d", (g_ctx.pid_kd != NULL) ? (int)(*g_ctx.pid_kd * 100.0f) : 0);
        show_line(3, buf);
        show_line(4, "Step: 0.01");
        show_line(5, "Usually keep 0");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_VOFA:
        show_line(2, "VOFA JustFloat");
        snprintf(buf, sizeof(buf), "Status: %s", (g_ctx.vofa_mode != NULL && *g_ctx.vofa_mode) ? "ON" : "OFF");
        show_line(3, buf);
        show_line(4, "Mode: binary stream");
        show_line(5, " ");
        show_line(6, " ");
        show_line(7, "UART will be binary");
        break;
    case UI_MENU_USER_COUNT:
        show_line(2, "Counter");
        snprintf(buf, sizeof(buf), "      %5d", g_user_count);
        show_line(3, buf);
        show_line(4, "Step: 1");
        show_line(5, " ");
        show_line(6, " ");
        show_line(7, " ");
        break;
    case UI_MENU_RESET_COUNT:
        show_line(2, "Reset Counter");
        snprintf(buf, sizeof(buf), "Current: %5d", g_user_count);
        show_line(3, buf);
        if (left_s > 0) snprintf(buf, sizeof(buf), "Confirm: %lus", (unsigned long)left_s);
        else snprintf(buf, sizeof(buf), "Await confirm");
        show_line(4, buf);
        show_line(5, " ");
        show_line(6, "Double press safety");
        show_line(7, " ");
        break;
    default:
        show_line(2, "Unknown page");
        break;
    }
}

void ui_menu_init(const ui_menu_context_t *ctx)
{
    if (ctx != NULL)
    {
        g_ctx = *ctx;
    }

    g_screen = UI_SCREEN_HOME;
    g_item = UI_MENU_TARGET_RPM;
    clear_confirm();
    clear_home_start_arm();
    g_home_return_deadline = 0;
    g_user_count = 0;
}

void ui_menu_handle_key(uint8_t key, uint32_t now_ms)
{
    if (g_confirm != UI_CONFIRM_NONE && (int32_t)(g_confirm_deadline - now_ms) <= 0)
    {
        clear_confirm();
    }

    if (key != 3u)
    {
        clear_confirm();
    }

    if (g_screen != UI_SCREEN_HOME && key == 1u && g_confirm == UI_CONFIRM_NONE &&
        g_home_return_deadline != 0u && (int32_t)(g_home_return_deadline - now_ms) > 0)
    {
        return_home();
        return;
    }

    if (g_screen != UI_SCREEN_HOME && key != 1u && key != 3u)
    {
        g_home_return_deadline = 0;
    }

    if (g_screen == UI_SCREEN_HOME)
    {
        if (key == 3u)
        {
            clear_home_start_arm();
            g_screen = UI_SCREEN_MENU;
            g_home_return_deadline = 0;
        }
        else if (key == 1u)
        {
            if (ui_is_running())
            {
                stop_now();
                clear_home_start_arm();
            }
            else
            {
                g_home_start_deadline = now_ms + 1500u;
                ui_beep(8);
            }
        }
        else if (key == 2u)
        {
            if (ui_is_running())
            {
                stop_now();
                clear_home_start_arm();
            }
            else if (home_start_is_armed(now_ms))
            {
                start_now();
                clear_home_start_arm();
            }
            else if (g_ctx.vofa_mode != NULL)
            {
                *g_ctx.vofa_mode = !*g_ctx.vofa_mode;
                ui_beep(8);
            }
        }
        return;
    }

    clear_home_start_arm();

    if (g_screen == UI_SCREEN_MENU)
    {
        if (key == 1u)
        {
            g_item = (g_item == 0) ? (UI_MENU_COUNT - 1) : (ui_menu_item_t)(g_item - 1);
        }
        else if (key == 2u)
        {
            g_item = (ui_menu_item_t)((g_item + 1) % UI_MENU_COUNT);
        }
        else
        {
            g_home_return_deadline = now_ms + 800u;
            g_screen = UI_SCREEN_PAGE;
        }
        return;
    }

    switch (g_item)
    {
    case UI_MENU_TARGET_RPM:
    case UI_MENU_BASE_PWM:
    case UI_MENU_FF_K:
    case UI_MENU_TEST_PWM:
    case UI_MENU_PID_KP:
    case UI_MENU_PID_KI:
    case UI_MENU_PID_KD:
    case UI_MENU_USER_COUNT:
        if (key == 1u)
        {
            adjust_value(-1);
        }
        else if (key == 2u)
        {
            adjust_value(1);
        }
        else
        {
            g_home_return_deadline = now_ms + 800u;
            g_screen = UI_SCREEN_MENU;
        }
        break;
    case UI_MENU_TEST_MODE:
        if (g_ctx.test_mode != NULL)
        {
            if (key == 1u)
            {
                *g_ctx.test_mode = (*g_ctx.test_mode == 0u) ? 3u : (uint8_t)(*g_ctx.test_mode - 1u);
            }
            else if (key == 2u)
            {
                *g_ctx.test_mode = (uint8_t)((*g_ctx.test_mode + 1u) % 4u);
            }
            else if (key == 3u)
            {
                g_home_return_deadline = now_ms + 800u;
                g_screen = UI_SCREEN_MENU;
            }
        }
        break;
    case UI_MENU_MOTOR_DIR:
        if (key == 1u && g_ctx.motor_dir != NULL)
        {
            *g_ctx.motor_dir = 1;
            ui_beep(8);
        }
        else if (key == 2u && g_ctx.motor_dir != NULL)
        {
            *g_ctx.motor_dir = -1;
            ui_beep(8);
        }
        else if (key == 3u)
        {
            g_home_return_deadline = now_ms + 800u;
            g_screen = UI_SCREEN_MENU;
        }
        break;
    case UI_MENU_VOFA:
        if (key == 1u && g_ctx.vofa_mode != NULL)
        {
            *g_ctx.vofa_mode = 0;
            ui_beep(30);
        }
        else if (key == 2u && g_ctx.vofa_mode != NULL)
        {
            *g_ctx.vofa_mode = 1;
            ui_beep(30);
        }
        else if (key == 3u)
        {
            g_home_return_deadline = now_ms + 800u;
            g_screen = UI_SCREEN_MENU;
        }
        break;
    case UI_MENU_RESET_COUNT:
        if (key == 1u)
        {
            g_screen = UI_SCREEN_MENU;
        }
        else if (key == 3u)
        {
            request_confirm(UI_CONFIRM_RESET_COUNT, now_ms);
        }
        break;
    default:
        g_screen = UI_SCREEN_HOME;
        break;
    }
}

void ui_menu_update(uint32_t now_ms)
{
    if (g_confirm != UI_CONFIRM_NONE && (int32_t)(g_confirm_deadline - now_ms) <= 0)
    {
        clear_confirm();
    }

    if (g_home_start_deadline != 0u && (int32_t)(g_home_start_deadline - now_ms) <= 0)
    {
        clear_home_start_arm();
    }

    if (g_home_return_deadline != 0u && (int32_t)(g_home_return_deadline - now_ms) <= 0)
    {
        g_home_return_deadline = 0;
    }
}

void ui_menu_draw(uint32_t now_ms)
{
    if (g_screen == UI_SCREEN_HOME)
    {
        draw_home(now_ms);
    }
    else if (g_screen == UI_SCREEN_MENU)
    {
        draw_menu();
    }
    else
    {
        draw_page(now_ms);
    }
}
