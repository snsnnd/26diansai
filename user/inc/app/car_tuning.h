#ifndef CAR_TUNING_H
#define CAR_TUNING_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 智能车参数调优结构体
 *
 * 集中管理小车的所有可调参数，包括目标速度、PID控制参数、转向增益等。
 * 这些参数可以在运行时通过调参菜单或上位机实时修改，便于赛道调试。
 * 所有参数均可通过串口协议或OLED菜单进行在线调整。
 */
typedef struct
{
    /* ========== 速度控制参数 ========== */
    float target_rpm;           /* 目标转速(RPM)：期望电机达到的转速值 */
    int16_t base_pwm;           /* 基础PWM值：开环控制时的前馈PWM基准 */
    float feedforward_gain;     /* 前馈增益：将目标RPM映射到PWM的增益系数 */

    /* ========== 速度环PID参数 ========== */
    float speed_kp;             /* 速度环比例增益：对转速误差的即时响应强度 */
    float speed_ki;             /* 速度环积分增益：消除稳态误差，提高低速性能 */
    float speed_kd;             /* 速度环微分增益：抑制转速波动，提高动态响应 */

    /* ========== 巡线控制参数 ========== */
    float line_kp;              /* 巡线比例增益：根据偏离中心线的距离调整转向 */
    float line_kd;              /* 巡线微分增益：根据偏离变化率预测性调整转向 */
    int8_t line_steer_sign;     /* 巡线转向符号(+1或-1)：加-号，设成-1就原地反转修正方向 */

    /* ========== 航向控制参数(用于H2024比赛任务) ========== */
    float heading_kp;           /* 航向比例增益：对角度误差的响应强度 */
    float heading_kd;           /* 航向微分增益：对角速度(角速率)的阻尼控制 */
    float heading_max_steer;    /* 航向最大转向PWM：限制航向控制的输出上限 */

    /* ========== 电机特性补偿参数 ========== */
    float left_gain;            /* 左轮增益系数：补偿左右电机差异(通常0.8~1.2) */
    float right_gain;           /* 右轮增益系数：补偿左右电机差异(通常0.8~1.2) */
    int8_t heading_steer_sign;  /* 航向转向符号(+1或-1)：根据陀螺仪安装方向校准 */
    bool speed_loop_enabled;    /* 速度闭环使能：true为闭环PID控制，false为开环控制 */
} car_tuning_t;

/**
 * @brief 将调优参数恢复为默认值
 * @param tuning 调优结构体指针
 * @note 默认值来源于config.h中定义的CAR_DEFAULT_*宏
 */
void car_tuning_defaults(car_tuning_t *tuning);

/**
 * @brief 基于当前目标转速计算前馈PWM值
 * @param tuning 调优结构体指针
 * @return 前馈PWM值，限幅在[0, MOTOR_PWM_DUTY_MAX]范围内
 * @note 计算公式：pwm = base_pwm + target_rpm * feedforward_gain
 */
int16_t car_tuning_feedforward_pwm(const car_tuning_t *tuning);

/**
 * @brief 根据指定的目标转速计算前馈PWM值
 * @param tuning 调优结构体指针
 * @param target_rpm 期望的目标转速(RPM)
 * @return 前馈PWM值，限幅在[0, MOTOR_PWM_DUTY_MAX]范围内
 * @note 此函数允许使用临时目标转速而非tuning中存储的值，用于差速控制
 */
int16_t car_tuning_feedforward_pwm_for_rpm(const car_tuning_t *tuning,
    float target_rpm);

#endif
