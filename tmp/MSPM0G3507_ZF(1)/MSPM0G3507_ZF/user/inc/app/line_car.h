#ifndef _LINE_CAR_H_
#define _LINE_CAR_H_

#include "zf_common_headfile.h"

/* 状态机 */
typedef enum {
    CAR_INIT = 0,
    CAR_READY,
    CAR_LINE_FOLLOW,
    CAR_TURN_FIND,
    CAR_STOP
} car_state_t;

/* 循迹 PID */
typedef struct {
    float kp;
    float kd;
    int16_t last_error;
} car_pid_t;

/* 电池电压补偿 */
#define BAT_NOMINAL_MV   12600   /* 电池标称 (3S LiPo 满电 12.6V) */
#define BAT_DIVIDER       447     /* 分压比 x100 (4.47) */
#define BAT_ADC_REF_MV    3300

void line_car_init(void);
void line_car_run(void);

#endif
