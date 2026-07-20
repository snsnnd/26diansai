#include "app/line_car.h"
#include "app/motor_test.h"
#include "app/ui_menu.h"
#include "app/vofa.h"
#include "driver/dt_motor.h"
#include "driver/dt_buzzer.h"
#include "driver/dt_encoder.h"
#include "driver/dt_oled.h"
#include "lib/pid_controller.h"
#include "driver/dt_hc05.h"
#include "device/t8_gray_sensor.h"
#include "config.h"

/* ---- 双电机 + 编码器 ---- */
static dt_motor_config_t g_motor_l, g_motor_r;
static dt_encoder_t      g_enc_l, g_enc_r;
static dt_buzzer_config_t g_buzzer;
static dt_oled_config_t   g_oled;

static soft_iic_info_struct g_t8_iic;
static T8I2cDevice          g_t8;

/* ---- 状态机 ---- */
#define KEY_STARTUP_LOCK_MS  1000u
static volatile car_state_t g_state = CAR_INIT;
static volatile uint32_t g_tick_ms = 0;
static volatile int16_t  g_base_speed = 0;
static volatile uint8_t   g_vofa_mode  = 1;
static volatile uint32_t g_key1_last = 0, g_key2_last = 0, g_key3_last = 0;
static volatile uint8_t g_key_queue[16];
static volatile uint8_t g_key_queue_head = 0;
static volatile uint8_t g_key_queue_tail = 0;
static uint8_t g_key1_armed = 0;

/* ---- 传感器 & 记忆 ---- */
static uint8_t  g_line_bits  = 0xFF;
static int16_t  g_line_pos   = 0;
static int16_t  g_prev_pos   = 0;
static uint8_t  g_white_cnt  = 0;
static float    g_pos_conf   = 1.0f;   /* 置信度: 1.0=正常, 0.5=缝隙猜测 */

/* ---- PD ---- */
#define LINE_FOLLOW_ENABLE  0      /* 当前重点调速度PID，默认关闭循迹修正 */
#define PD_KP   4.0f      /* 前置传感器: KP略低防过冲 */
#define PD_KD   0.0f
static int16_t g_pd_last_err = 0;

/* ---- 电池电压 ---- */
static uint16_t g_bat_mv = 0;
static uint16_t g_bat_raw = 0;

/* ---- 速度 PID ---- */
static PidController g_spd_pid_l;
static PidController g_spd_pid_r;
static float g_rpm_l = 0, g_rpm_r = 0;
static float g_spd_out_l = 0, g_spd_out_r = 0;
static int16_t g_cmd_l = 0, g_cmd_r = 0;
static int8_t g_motor_dir = -1;

/* ---- 任务时间片 ---- */
#define SENSOR_PERIOD_MS   50u
#define CONTROL_PERIOD_MS  10u
static uint32_t last_sensor  = 0;
static uint32_t last_control = 0;
static uint32_t last_oled    = 0;

static float car_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void push_key_event(uint8_t key)
{
    uint8_t next = (uint8_t)((g_key_queue_head + 1u) & 0x0Fu);

    if (next != g_key_queue_tail)
    {
        g_key_queue[g_key_queue_head] = key;
        g_key_queue_head = next;
    }
}

static uint8_t pop_key_event(uint8_t *key)
{
    uint8_t has_event = 0;

    __disable_irq();
    if (g_key_queue_tail != g_key_queue_head)
    {
        *key = g_key_queue[g_key_queue_tail];
        g_key_queue_tail = (uint8_t)((g_key_queue_tail + 1u) & 0x0Fu);
        has_event = 1;
    }
    __enable_irq();

    return has_event;
}

static void car_stop_all(void)
{
    g_base_speed = 0;
    g_spd_out_l = 0.0f;
    g_spd_out_r = 0.0f;
    pid_reset(&g_spd_pid_l);
    pid_reset(&g_spd_pid_r);
    dt_motor_stop(&g_motor_l);
    dt_motor_stop(&g_motor_r);
    g_cmd_l = 0;
    g_cmd_r = 0;
    g_state = CAR_STOP;
}

static void car_start_run(int16_t base_speed)
{
    if (base_speed < 0)
    {
        base_speed = 0;
    }
    g_base_speed = base_speed;
    if (g_state == CAR_STOP)
    {
        g_state = CAR_READY;
    }
}

static void apply_motor_cmd(int16_t l_speed, int16_t r_speed)
{
    dt_motor_set_speed(&g_motor_l, l_speed);
    dt_motor_set_speed(&g_motor_r, -r_speed);   /* 右轮接线反了，软件翻转 */
    g_cmd_l = l_speed;
    g_cmd_r = r_speed;
}

/* ---- 线位置 : 8 位 → -7~+7 (0=居中) ---- */
static int16_t line_pos_from_bits(uint8_t bits)
{
    int16_t sum = 0, cnt = 0;
    for (int i = 0; i < 8; i++)
    {
        if (!((bits >> i) & 1)) { sum += (int16_t)(i * 2 - 7); cnt++; }
    }
    return (cnt > 0) ? (sum / cnt) : 99;
}

/* ---- 状态转移 ---- */
static void state_machine(void)
{
    switch (g_state)
    {
    case CAR_INIT:
        g_state = CAR_READY;
        printf("[CAR] READY, KEY1/KEY2 menu, KEY3 ok\r\n");
        break;
    case CAR_READY:
        if (g_base_speed > 0) g_state = CAR_LINE_FOLLOW;
        break;
    case CAR_LINE_FOLLOW:
        if (LINE_FOLLOW_ENABLE && g_line_pos == 99 && g_white_cnt > 3)
            g_state = CAR_TURN_FIND;
        break;
    case CAR_TURN_FIND:
        if (g_line_pos != 99)
            g_state = CAR_LINE_FOLLOW;
        break;
    case CAR_STOP:
        break;
    }
}

/* ---- 差速控制 (PD + 电压补偿) ---- */
static void diff_control(void)
{
    int16_t l_speed = 0, r_speed = 0;
    int16_t steer = 0;
    int16_t comp_speed = motor_test_feedforward_pwm();

    /* 速度前馈输出为实际PWM命令，PID只做小范围修正。 */
    motor_test_set_voltage_pwm(comp_speed);

    if (g_state == CAR_LINE_FOLLOW)
    {
        if (LINE_FOLLOW_ENABLE && g_line_pos != 99)
        {
            int16_t err = g_line_pos;
            steer = (int16_t)(PD_KP * (float)err + PD_KD * (float)(err - g_pd_last_err));
            g_pd_last_err = err;
        }

        l_speed = (int16_t)(comp_speed + (int16_t)g_spd_out_l + steer);
        r_speed = (int16_t)(comp_speed + (int16_t)g_spd_out_r - steer);
    }
    else if (g_state == CAR_TURN_FIND)
    {
        /* 用上一帧位置决定找线方向和力度 */
        int16_t dir = g_prev_pos;
        if (dir < -4)       { l_speed = -comp_speed / 3; r_speed =  comp_speed / 3; }  /* 线在左, 左自旋 */
        else if (dir > 4)   { l_speed =  comp_speed / 3; r_speed = -comp_speed / 3; }  /* 线在右, 右自旋 */
        else                { l_speed = -comp_speed / 2; r_speed =  comp_speed / 2; }  /* 居中丢线, 原地旋 */
    }

    l_speed = (int16_t)(l_speed * g_motor_dir);
    r_speed = (int16_t)(r_speed * g_motor_dir);

    apply_motor_cmd(l_speed, r_speed);
}

/* ---- I2C transport ---- */
static bool t8_write(uint8_t addr, const uint8_t *data, size_t len, void *ctx)
{
    soft_iic_info_struct *iic = (soft_iic_info_struct *)ctx;
    iic->addr = addr;
    soft_iic_write_8bit_array(iic, data, len);
    return true;
}
static bool t8_read(uint8_t addr, uint8_t *data, size_t len, uint32_t to_ms, void *ctx)
{
    soft_iic_info_struct *iic = (soft_iic_info_struct *)ctx;
    (void)to_ms;
    iic->addr = addr;
    soft_iic_read_8bit_array(iic, data, len);
    return true;
}

/* ---- ISR ---- */
static void tick_isr(uint32_t ev, void *p) { (void)ev;(void)p; g_tick_ms++; }

static void key1_isr(uint32_t ev, void *p)
{
    (void)ev;(void)p;
    if (g_tick_ms < KEY_STARTUP_LOCK_MS) return;
    if ((uint32_t)(g_tick_ms - g_key1_last) < 50) return;
    g_key1_last = g_tick_ms;
    push_key_event(1u);
}
static void key2_isr(uint32_t ev, void *p)
{
    (void)ev;(void)p;
    if ((uint32_t)(g_tick_ms - g_key2_last) < 50) return;
    g_key2_last = g_tick_ms;
    push_key_event(2u);
}
static void key3_isr(uint32_t ev, void *p)
{
    (void)ev;(void)p;
    if ((uint32_t)(g_tick_ms - g_key3_last) < 50) return;
    g_key3_last = g_tick_ms;
    push_key_event(3u);
}

/* ================================================================
 * 初始化
 * ================================================================ */
void line_car_init(void)
{
    motor_test_init();

    g_motor_l.in1_pin = MOTOR_L_IN1; g_motor_l.in2_pin = MOTOR_L_IN2;
    g_motor_l.pwm_freq = MOTOR_PWM_FREQ;
    dt_motor_init(&g_motor_l);

    g_motor_r.in1_pin = MOTOR_R_IN1; g_motor_r.in2_pin = MOTOR_R_IN2;
    g_motor_r.pwm_freq = MOTOR_PWM_FREQ;
    dt_motor_init(&g_motor_r);

    pid_init(&g_spd_pid_l);
    pid_init(&g_spd_pid_r);
    pid_set_gain(&g_spd_pid_l, motor_test_kp(), motor_test_ki(), motor_test_kd());
    pid_set_gain(&g_spd_pid_r, motor_test_kp(), motor_test_ki(), motor_test_kd());
    pid_set_limits(&g_spd_pid_l, -2000, 2000, -3000, 3000);
    pid_set_limits(&g_spd_pid_r, -2000, 2000, -3000, 3000);

    g_enc_l.a_pin = ENCODER1_A_PIN;
    g_enc_l.counts_per_rev = ENCODER_CPR;
    g_enc_l.rpm_lpf_alpha = 1.0f;
    dt_encoder_init(&g_enc_l);

    g_enc_r.a_pin = ENCODER2_A_PIN;
    g_enc_r.counts_per_rev = ENCODER_CPR;
    g_enc_r.rpm_lpf_alpha = 1.0f;
    dt_encoder_init(&g_enc_r);

    g_buzzer.pin = BUZZER_PIN;
    dt_buzzer_init(&g_buzzer);

    soft_iic_init(&g_t8_iic, T8_DEFAULT_I2C_ADDRESS, 100, TRACE_SCL, TRACE_SDA);
    { T8I2cTransport trans = { t8_write, t8_read, &g_t8_iic };
      t8_i2c_init(&g_t8, &trans, T8_DEFAULT_I2C_ADDRESS); }

    soft_iic_init(&g_oled.iic, DT_OLED_DEFAULT_ADDR, 100, OLED_SCL, OLED_SDA);
    dt_oled_init(&g_oled);
    dt_oled_clear(&g_oled);
    dt_oled_show_string(&g_oled, 0, 2, "Line Car Ready");
    {
        ui_menu_context_t ui_ctx = {
            &g_oled,
            &g_buzzer,
            &g_state,
            &g_base_speed,
            &g_vofa_mode,
            &g_motor_dir,
            motor_test_base_pwm_ptr(),
            motor_test_ff_k_ptr(),
            motor_test_mode_ptr(),
            motor_test_pwm_ptr(),
            motor_test_deadzone_pwm_ptr(),
            motor_test_target_rpm_ptr(),
            motor_test_kp_ptr(),
            motor_test_ki_ptr(),
            motor_test_kd_ptr(),
            &g_rpm_l,
            &g_rpm_r,
            &g_cmd_l,
            &g_cmd_r,
            &g_bat_mv,
            &g_bat_raw,
            &g_line_bits,
            &g_line_pos,
            LINE_FOLLOW_ENABLE,
            car_start_run,
            car_stop_all
        };
        ui_menu_init(&ui_ctx);
    }

    exti_init(KEY1_PIN, EXTI_TRIGGER_FALLING, key1_isr, NULL);
    exti_init(KEY2_PIN, EXTI_TRIGGER_FALLING, key2_isr, NULL);
    exti_init(KEY3_PIN, EXTI_TRIGGER_FALLING, key3_isr, NULL);

    adc_init(BAT_ADC, ADC_12BIT);

    pit_ms_init(PIT_TIM_G0, 1, tick_isr, NULL);
    g_key1_last = g_tick_ms;
    g_key2_last = g_tick_ms;
    g_key3_last = g_tick_ms;

    g_state = CAR_INIT;
    g_base_speed = 0;
    g_spd_out_l = 0.0f;
    g_spd_out_r = 0.0f;
    dt_motor_stop(&g_motor_l);
    dt_motor_stop(&g_motor_r);
    dt_buzzer_beep(&g_buzzer, 80);
    system_delay_ms(80);
    dt_buzzer_beep(&g_buzzer, 80);
}

/* ================================================================
 * 主循环
 * ================================================================ */
void line_car_run(void)
{
    uint32_t now = g_tick_ms;
    uint8_t key_handled = 0;
    uint8_t key;

    if (!g_key1_armed && now >= KEY_STARTUP_LOCK_MS && gpio_get_level(KEY1_PIN))
    {
        g_key1_armed = 1;
    }

    while (pop_key_event(&key))
    {
        if (key == 1u && !g_key1_armed)
        {
            continue;
        }
        ui_menu_handle_key(key, now);
        key_handled = 1;
    }

    ui_menu_update(now);

    if (key_handled)
    {
        ui_menu_draw(now);
        last_oled = now;
    }

    /* ===== 1. 传感器 (无条件, 先读) ===== */
    if ((uint32_t)(now - last_sensor) >= SENSOR_PERIOD_MS) {
        uint32_t dt_ms = now - last_sensor;
        last_sensor = now;

        if (LINE_FOLLOW_ENABLE)
        {
            t8_i2c_get_digital(&g_t8, &g_line_bits);
            g_line_pos = line_pos_from_bits(g_line_bits);

            if (g_line_bits == 0x00) { g_line_pos = 0; g_white_cnt = 0; g_pos_conf = 1.0f; }
            else if (g_line_pos == 99) {
                g_white_cnt++;
                if (g_white_cnt <= 3 && g_prev_pos != 99) { g_line_pos = g_prev_pos; g_pos_conf = 0.5f; }
                else { g_pos_conf = 1.0f; }
            } else { g_white_cnt = 0; g_prev_pos = g_line_pos; g_pos_conf = 1.0f; }
        }
        else
        {
            g_line_bits = 0x00;
            g_line_pos = 0;
            g_white_cnt = 0;
            g_pos_conf = 1.0f;
        }

        g_bat_raw = adc_mean_filter_convert(BAT_ADC, 8);
        g_bat_mv = (uint16_t)((uint64_t)g_bat_raw * BAT_ADC_REF_MV * BAT_DIVIDER / 409500u);

        g_rpm_l = dt_encoder_compute_rpm(&g_enc_l, dt_ms);
        g_rpm_r = dt_encoder_compute_rpm(&g_enc_r, dt_ms);
    }

    /* ===== 2. 状态机 (最高优先级, 每帧决策) ===== */
    state_machine();

    /* ===== 3. 控制 (遵从状态机输出) ===== */
    if ((uint32_t)(now - last_control) >= CONTROL_PERIOD_MS) {
        uint32_t control_dt_ms = now - last_control;
        float control_dt_s;
        last_control = now;
        control_dt_s = (float)control_dt_ms / 1000.0f;

        if (g_base_speed > 0 && (g_state == CAR_LINE_FOLLOW || g_state == CAR_TURN_FIND))
        {
            pid_set_gain(&g_spd_pid_l, motor_test_kp(), motor_test_ki(), motor_test_kd());
            pid_set_gain(&g_spd_pid_r, motor_test_kp(), motor_test_ki(), motor_test_kd());

            if (motor_test_is_pid_mode())
            {
                g_spd_out_l = pid_update(&g_spd_pid_l, motor_test_target_rpm() - car_absf(g_rpm_l), control_dt_s);
                g_spd_out_r = pid_update(&g_spd_pid_r, motor_test_target_rpm() - car_absf(g_rpm_r), control_dt_s);
            }
        }
        else
        {
            g_spd_out_l = 0.0f;
            g_spd_out_r = 0.0f;
            pid_reset(&g_spd_pid_l);
            pid_reset(&g_spd_pid_r);
        }

        if (g_base_speed > 0 && !motor_test_is_pid_mode())
        {
            g_spd_out_l = 0.0f;
            g_spd_out_r = 0.0f;
            motor_test_update_deadzone(now, g_motor_dir, g_rpm_l, g_rpm_r, apply_motor_cmd);
        }
        else
        {
            diff_control();
        }
    }

    /* oled */
    if ((uint32_t)(now - last_oled) >= 200) {
        last_oled = now;
        ui_menu_draw(now);
    }

    /* debug / VOFA */
    {
        static uint32_t last_vofa = 0, last_dbg = 0;

        if (g_vofa_mode && (uint32_t)(now - last_vofa) >= 20) {
            last_vofa = now;
            float vofa_data[12];
            vofa_data[0] = motor_test_target_rpm();
            vofa_data[1] = (car_absf(g_rpm_l) + car_absf(g_rpm_r)) * 0.5f;
            vofa_data[2] = car_absf(g_rpm_l);
            vofa_data[3] = car_absf(g_rpm_r);
            vofa_data[4] = (float)g_cmd_l;
            vofa_data[5] = (float)g_cmd_r;
            vofa_data[6] = g_spd_out_l;
            vofa_data[7] = g_spd_out_r;
            vofa_data[8] = (float)motor_test_voltage_pwm();
            vofa_data[9] = (float)motor_test_mode();
            vofa_data[10] = (float)g_bat_mv;
            vofa_data[11] = (float)*motor_test_deadzone_pwm_ptr();
            vofa_send(vofa_data, 12);
        }

        if (!g_vofa_mode && (uint32_t)(now - last_dbg) >= 500) {
            last_dbg = now;
            printf("[CAR] st=%d pos=%d spd=%d rpmL=%d rpmR=%d bat=%d\r\n",
                   g_state, g_line_pos, g_base_speed,
                   (int)g_rpm_l, (int)g_rpm_r, g_bat_mv);
        }
    }

    system_delay_ms(1);
}
