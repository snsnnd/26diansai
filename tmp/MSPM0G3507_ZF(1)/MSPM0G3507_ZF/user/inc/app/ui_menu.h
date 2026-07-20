#ifndef _UI_MENU_H_
#define _UI_MENU_H_

#include "zf_common_headfile.h"
#include "app/line_car.h"
#include "driver/dt_buzzer.h"
#include "driver/dt_oled.h"

typedef void (*ui_menu_start_cb_t)(int16_t base_speed);
typedef void (*ui_menu_stop_cb_t)(void);

typedef struct {
    dt_oled_config_t *oled;
    dt_buzzer_config_t *buzzer;

    volatile car_state_t *car_state;
    volatile int16_t *base_speed;
    volatile uint8_t *vofa_mode;
    int8_t *motor_dir;
    int16_t *base_pwm;
    float *ff_k;
    uint8_t *test_mode;
    int16_t *test_pwm;
    int16_t *deadzone_pwm;

    float *target_rpm;
    float *pid_kp;
    float *pid_ki;
    float *pid_kd;
    float *rpm_l;
    float *rpm_r;
    int16_t *cmd_l;
    int16_t *cmd_r;
    uint16_t *bat_mv;
    uint16_t *bat_raw;
    uint8_t *line_bits;
    int16_t *line_pos;

    uint8_t line_follow_enable;
    ui_menu_start_cb_t start_cb;
    ui_menu_stop_cb_t stop_cb;
} ui_menu_context_t;

void ui_menu_init(const ui_menu_context_t *ctx);
void ui_menu_handle_key(uint8_t key, uint32_t now_ms);
void ui_menu_update(uint32_t now_ms);
void ui_menu_draw(uint32_t now_ms);

#endif
