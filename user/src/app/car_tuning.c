#include "app/car_tuning.h"
#include "config.h"

#include <stddef.h>

/**
 * @brief 将调优参数恢复为默认值
 *
 * 所有默认值从config.h中的CAR_DEFAULT_*宏读取，这些宏在项目的配置文件中
 * 定义。此函数在启动时调用，也可以在运行时通过菜单选择"恢复默认"来重置。
 *
 * @param tuning 调优结构体指针(不能为NULL)
 */
void car_tuning_defaults(car_tuning_t *tuning)
{
    if (tuning == NULL) return;
    /* 目标转速：电机期望达到的转速，单位RPM */
    tuning->target_rpm = CAR_DEFAULT_TARGET_RPM;
    /* 基础PWM：速度开环控制时的前馈基准值 */
    tuning->base_pwm = CAR_DEFAULT_BASE_PWM;
    /* 前馈增益：target_rpm到PWM的转换系数 */
    tuning->feedforward_gain = CAR_DEFAULT_FEEDFORWARD_GAIN;
    /* 速度环PID参数 */
    tuning->speed_kp = CAR_DEFAULT_SPEED_KP;
    tuning->speed_ki = CAR_DEFAULT_SPEED_KI;
    tuning->speed_kd = CAR_DEFAULT_SPEED_KD;
    /* 巡线控制PID参数 */
    tuning->line_kp = CAR_DEFAULT_LINE_KP;
    tuning->line_kd = CAR_DEFAULT_LINE_KD;
    tuning->line_steer_sign = (int8_t)CAR_DEFAULT_LINE_SIGN;    /* 巡线转向符号 */
    /* 航向控制PID参数(用于H2024比赛任务) */
    tuning->heading_kp = CAR_DEFAULT_HEADING_KP;
    tuning->heading_kd = CAR_DEFAULT_HEADING_KD;
    /* 航向最大转向PWM限制 */
    tuning->heading_max_steer = CAR_DEFAULT_HEADING_MAX_PWM;
    /* 左右轮增益系数，用于修正电机个体差异 */
    tuning->left_gain = CAR_DEFAULT_LEFT_GAIN;
    tuning->right_gain = CAR_DEFAULT_RIGHT_GAIN;
    /* 航向转向符号，根据陀螺仪安装方向配置(+1或-1) */
    tuning->heading_steer_sign = CAR_DEFAULT_HEADING_SIGN;
    /* 速度闭环使能标志 */
    tuning->speed_loop_enabled = CAR_ENABLE_WHEEL_SPEED_PID != 0 &&
        CAR_DEFAULT_SPEED_LOOP_ENABLED != 0;
}

/**
 * @brief 使用tuning中存储的target_rpm计算前馈PWM
 *
 * 前馈控制是一种开环控制方式：根据目标转速直接计算出需要的PWM值，
 * 不依赖反馈。这为闭环PID控制提供了一个良好的初始工作点。
 *
 * @param tuning 调优结构体指针
 * @return 前馈PWM值，范围[0, MOTOR_PWM_DUTY_MAX]
 */
int16_t car_tuning_feedforward_pwm(const car_tuning_t *tuning)
{
    return car_tuning_feedforward_pwm_for_rpm(tuning,
        tuning != NULL ? tuning->target_rpm : 0.0f);
}

/**
 * @brief 根据指定的目标转速计算前馈PWM值
 *
 * 前馈模型：PWM = base_pwm + target_rpm * feedforward_gain
 * base_pwm是克服电机静摩擦和系统死区所需的最小PWM，
 * feedforward_gain反映电机和驱动器的电压-转速转换特性。
 *
 * 此函数也用于差速转向场景：左右轮分别传入不同的target_rpm，
 * 得到不同的前馈PWM，从而实现差速转弯。
 *
 * @param tuning 调优结构体指针
 * @param target_rpm 目标转速(RPM)
 * @return 计算得到的前馈PWM值，异常情况返回0
 * @note 内部包含NaN检测(target_rpm != target_rpm，NaN不自等)
 */
int16_t car_tuning_feedforward_pwm_for_rpm(const car_tuning_t *tuning,
    float target_rpm)
{
    float pwm;

    if (tuning == NULL) return 0;
    /* 检测无效输入：NaN或非正转速 */
    if (target_rpm != target_rpm || target_rpm <= 0.0f) return 0;

    /* 前馈计算公式：PWM = 基础PWM + 目标转速 * 增益系数 */
    pwm = (float)tuning->base_pwm
        + target_rpm * tuning->feedforward_gain;

    /* 输出安全检查：NaN或非正值检测 */
    if (pwm != pwm || pwm <= 0.0f) return 0;
    /* 限幅到PWM最大允许值 */
    if (pwm >= (float)MOTOR_PWM_DUTY_MAX) return MOTOR_PWM_DUTY_MAX;
    return (int16_t)pwm;
}
