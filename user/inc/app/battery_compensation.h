#ifndef BATTERY_COMPENSATION_H
#define BATTERY_COMPENSATION_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 电池补偿状态枚举
 *
 * 定义电池电压补偿模块的当前工作状态，用于判断系统是否可以安全运行。
 */
typedef enum
{
    BATTERY_COMP_STARTUP = 0,    /* 启动阶段：尚未完成初始化或仍在等待有效样本 */
    BATTERY_COMP_OK,              /* 正常状态：电池电压在正常范围内，可正常运行 */
    BATTERY_COMP_INVALID,         /* 无效状态：ADC采样异常或数据不可靠 */
    BATTERY_COMP_UNDERVOLTAGE     /* 欠压状态：电池电压低于截止阈值，需要保护性停机 */
} battery_compensation_status_t;

/**
 * @brief 电池补偿配置结构体
 *
 * 包含了电池电压补偿模块所需的所有配置参数，包括参考电压、有效范围、
 * 截止/恢复阈值、补偿因子限制以及滤波和容错参数。
 */
typedef struct
{
    uint16_t reference_mv;      /* 参考电压(毫伏)：用于计算补偿因子的基准电压值 */
    uint16_t valid_min_mv;      /* 有效电压下限(毫伏)：ADC采样值低于此值则视为无效 */
    uint16_t valid_max_mv;      /* 有效电压上限(毫伏)：ADC采样值高于此值则视为无效 */
    uint16_t cutoff_mv;         /* 欠压截止电压(毫伏)：低于此值触发欠压保护 */
    uint16_t recovery_mv;       /* 恢复电压(毫伏)：电压回升至此值以上后可尝试恢复运行 */
    float minimum_factor;        /* 补偿因子最小值：限制补偿的下界，防止过度补偿 */
    float maximum_factor;        /* 补偿因子最大值：限制补偿的上界，防止补偿不足 */
    float filter_alpha;          /* 低通滤波系数(0~1)：值越小滤波越平滑，响应越慢 */
    uint8_t fault_samples;      /* 触发故障所需的连续异常样本数：用于去抖容错 */
    uint8_t recovery_samples;   /* 恢复所需的连续正常样本数：确保电压稳定再恢复 */
} battery_compensation_config_t;

/**
 * @brief 电池补偿运行时状态结构体
 *
 * 维护电池补偿模块在运行过程中的动态状态，包括滤波后的电压值、
 * 当前补偿因子、状态机信息和计数器等。
 */
typedef struct
{
    battery_compensation_config_t config;  /* 配置参数副本 */
    float filtered_mv;                      /* 一阶低通滤波后的电压值(毫伏) */
    float factor;                           /* 当前计算的补偿因子(参考电压/实际电压) */
    battery_compensation_status_t status;   /* 当前工作状态 */
    uint8_t fault_count;                    /* 连续异常样本计数：用于故障判定 */
    uint8_t recovery_count;                 /* 连续正常样本计数：用于恢复判定 */
    bool has_sample;                        /* 是否已接收到第一个有效样本(初始化完成标志) */
    bool recovery_ready;                    /* 恢复条件是否已满足(等待上层确认) */
} battery_compensation_t;

/**
 * @brief 初始化电池补偿模块
 * @param compensation 电池补偿实例指针
 * @param config 配置参数指针，结构体会被拷贝到实例中
 * @note 对配置参数做合法性检查和修正，确保最小因子>0、故障样本数>0等
 */
void battery_compensation_init(battery_compensation_t *compensation,
    const battery_compensation_config_t *config);

/**
 * @brief 更新电池采样值，执行滤波和状态机转换
 * @param compensation 电池补偿实例指针
 * @param sample_mv 本次ADC采样值(毫伏)
 * @param sample_valid ADC采样是否有效标志
 * @note 实现了一阶低通滤波、故障计数、欠压检测和恢复检测等核心逻辑
 */
void battery_compensation_update(battery_compensation_t *compensation,
    uint16_t sample_mv, bool sample_valid);

/**
 * @brief 确认故障恢复：当恢复条件满足时，上层调用此函数确认退出故障状态
 * @param compensation 电池补偿实例指针
 * @return true 确认成功，状态已转为OK；false 未满足恢复条件
 */
bool battery_compensation_acknowledge(battery_compensation_t *compensation);

/**
 * @brief 检查电池补偿是否允许系统运行
 * @param compensation 电池补偿实例指针
 * @return true 状态为OK，可正常运行；false 状态异常，需停机保护
 */
bool battery_compensation_can_run(const battery_compensation_t *compensation);

/**
 * @brief 获取当前补偿因子值
 * @param compensation 电池补偿实例指针
 * @return 补偿因子(参考电压/实际电压)，异常时返回1.0f(无补偿)
 */
float battery_compensation_factor(const battery_compensation_t *compensation);

/**
 * @brief 获取滤波后的电池电压值
 * @param compensation 电池补偿实例指针
 * @return 滤波后的电压值(毫伏)，异常时返回0.0f
 */
float battery_compensation_voltage_mv(const battery_compensation_t *compensation);

/**
 * @brief 应用电池补偿到PWM输出值
 * @param compensation 电池补偿实例指针
 * @param reference_pwm 期望的PWM输出值(补偿前的参考值)
 * @param maximum_pwm PWM输出的最大绝对值限制
 * @return 补偿并限幅后的最终PWM值
 * @note 计算公式：output = reference_pwm * factor，然后限幅在[-maximum_pwm, maximum_pwm]
 *       当补偿状态不可运行时直接返回0，确保安全
 */
float battery_compensation_apply(const battery_compensation_t *compensation,
    float reference_pwm, float maximum_pwm);

#endif
