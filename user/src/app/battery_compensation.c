#include "app/battery_compensation.h"

#include <stddef.h>

/**
 * @brief 浮点数限幅函数：将值限制在指定的最小值和最大值之间
 * @param value 待限幅的浮点数
 * @param minimum 下限
 * @param maximum 上限
 * @return 限幅后的值
 */
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/**
 * @brief 更新电池补偿因子
 *
 * 根据滤波后的实际电压计算补偿因子。补偿因子 = 参考电压 / 实际电压，
 * 当电池电压低于参考值时，因子>1，从而增大PWM输出以补偿电压下降。
 * 因子被限制在配置的[min_factor, max_factor]范围内。
 *
 * @param compensation 电池补偿实例指针
 */
static void update_factor(battery_compensation_t *compensation)
{
    /* 如果滤波后的电压为0或负值(未初始化或异常)，使用最大补偿因子 */
    if (compensation->filtered_mv <= 1.0f)
    {
        compensation->factor = compensation->config.maximum_factor;
        return;
    }

    /* 补偿因子 = 参考电压 / 实际电压，并限幅在合法范围内 */
    compensation->factor = clamp_float(
        (float)compensation->config.reference_mv / compensation->filtered_mv,
        compensation->config.minimum_factor,
        compensation->config.maximum_factor);
}

/**
 * @brief 初始化电池补偿模块
 *
 * 将传入的配置拷贝到实例中，并对配置参数进行合法性检查和修正。
 * 初始化完成后，状态为BATTERY_COMP_STARTUP，等待第一个有效样本。
 *
 * @param compensation 电池补偿实例指针(不能为NULL)
 * @param config 配置参数指针(不能为NULL)
 */
void battery_compensation_init(battery_compensation_t *compensation,
    const battery_compensation_config_t *config)
{
    if (compensation == NULL || config == NULL) return;

    /* 复制配置并进行参数合法性修正 */
    compensation->config = *config;
    /* 滤波系数必须在[0,1]范围内 */
    compensation->config.filter_alpha = clamp_float(
        compensation->config.filter_alpha, 0.0f, 1.0f);
    /* 故障判定至少需要1个样本，防止除零或逻辑错误 */
    if (compensation->config.fault_samples == 0u)
        compensation->config.fault_samples = 1u;
    /* 恢复判定至少需要1个样本 */
    if (compensation->config.recovery_samples == 0u)
        compensation->config.recovery_samples = 1u;
    /* 最小补偿因子必须为正数 */
    if (compensation->config.minimum_factor <= 0.0f)
        compensation->config.minimum_factor = 0.1f;
    /* 最大因子不能小于最小因子 */
    if (compensation->config.maximum_factor < compensation->config.minimum_factor)
        compensation->config.maximum_factor = compensation->config.minimum_factor;

    /* 初始化运行时变量 */
    compensation->filtered_mv = 0.0f;     /* 滤波电压初始为0 */
    compensation->factor = 1.0f;          /* 补偿因子初始为1(无补偿) */
    compensation->status = BATTERY_COMP_STARTUP;  /* 启动状态 */
    compensation->fault_count = 0u;        /* 故障计数器清零 */
    compensation->recovery_count = 0u;     /* 恢复计数器清零 */
    compensation->has_sample = false;      /* 尚未收到有效样本 */
    compensation->recovery_ready = false;  /* 恢复条件未满足 */
}

/**
 * @brief 更新电池采样值，执行一阶低通滤波和状态机转换
 *
 * 核心功能环路：
 * 1. 检查采样值是否在有效范围内，无效则累加故障计数
 * 2. 对有效采样值进行一阶低通滤波(首次直接赋值)
 * 3. 根据滤波后的电压更新补偿因子
 * 4. 状态机处理：
 *    - OK状态：检测欠压条件，连续异常进入UNDERVOLTAGE
 *    - 非OK状态：检测恢复条件，连续正常标记recovery_ready
 *    - STARTUP状态：首次恢复成功后跳转到OK状态
 *
 * @param compensation 电池补偿实例指针
 * @param sample_mv 本次ADC采样值(毫伏)
 * @param sample_valid ADC采样是否有效标志
 */
void battery_compensation_update(battery_compensation_t *compensation,
    uint16_t sample_mv, bool sample_valid)
{
    bool in_range;

    if (compensation == NULL) return;

    /* 判断采样值是否在配置的有效电压范围内 */
    in_range = sample_valid &&
        sample_mv >= compensation->config.valid_min_mv &&
        sample_mv <= compensation->config.valid_max_mv;

    if (!in_range)
    {
        /* ========== 无效采样处理 ========== */
        /* 重置恢复计数，因为当前采样无效 */
        compensation->recovery_count = 0u;
        compensation->recovery_ready = false;
        /* 累加故障计数 */
        if (compensation->fault_count < compensation->config.fault_samples)
            compensation->fault_count++;
        /* 连续故障达到阈值，标记为INVALID状态 */
        if (compensation->fault_count >= compensation->config.fault_samples)
            compensation->status = BATTERY_COMP_INVALID;
        return;
    }

    /* ========== 有效采样处理 ========== */

    /* 第一个有效样本：直接赋值，不进行滤波 */
    if (!compensation->has_sample)
    {
        compensation->filtered_mv = (float)sample_mv;
        compensation->has_sample = true;
    }
    /* 从INVALID状态恢复时：直接用新值覆盖，防止旧值影响 */
    else if (compensation->status == BATTERY_COMP_INVALID)
    {
        /* 不要让故障前的陈旧电压值使恢复看起来有效 */
        compensation->filtered_mv = (float)sample_mv;
    }
    /* 正常运行时：一阶低通滤波，公式：y += alpha * (x - y) */
    else
    {
        compensation->filtered_mv += compensation->config.filter_alpha *
            ((float)sample_mv - compensation->filtered_mv);
    }
    /* 基于滤波后的电压更新补偿因子 */
    update_factor(compensation);

    /* ========== OK状态下的欠压检测 ========== */
    if (compensation->status == BATTERY_COMP_OK)
    {
        if (sample_mv < compensation->config.cutoff_mv)
        {
            /* 电压低于截止值，累加故障计数 */
            if (compensation->fault_count < compensation->config.fault_samples)
                compensation->fault_count++;
            /* 连续欠压达到阈值，进入欠压保护状态 */
            if (compensation->fault_count >= compensation->config.fault_samples)
            {
                compensation->status = BATTERY_COMP_UNDERVOLTAGE;
                compensation->recovery_count = 0u;
            }
        }
        else
        {
            /* 电压正常，清零故障计数 */
            compensation->fault_count = 0u;
        }
        return;
    }

    /* ========== 非OK状态下的恢复检测 ========== */
    /* 进入此分支：状态为STARTUP、INVALID或UNDERVOLTAGE */

    compensation->fault_count = 0u;  /* 有效采样，清零故障计数 */

    if (sample_mv >= compensation->config.recovery_mv)
    {
        /* 电压达到恢复阈值，累加恢复计数 */
        if (compensation->recovery_count < compensation->config.recovery_samples)
            compensation->recovery_count++;
        /* 连续正常样本达到恢复阈值，标记恢复就绪 */
        if (compensation->recovery_count >= compensation->config.recovery_samples)
            compensation->recovery_ready = true;
    }
    else
    {
        /* 电压未达到恢复阈值，重置恢复计数 */
        compensation->recovery_count = 0u;
        compensation->recovery_ready = false;
        /* 启动阶段如果电压就低于截止值，直接进入欠压状态 */
        if (compensation->status == BATTERY_COMP_STARTUP &&
            sample_mv < compensation->config.cutoff_mv)
            compensation->status = BATTERY_COMP_UNDERVOLTAGE;
    }

    /* 启动状态且恢复就绪：自动转入OK状态 */
    if (compensation->status == BATTERY_COMP_STARTUP && compensation->recovery_ready)
    {
        compensation->status = BATTERY_COMP_OK;
        compensation->recovery_ready = false;
        compensation->recovery_count = 0u;
    }
}

/**
 * @brief 确认故障恢复
 *
 * 当电池从INVALID或UNDERVOLTAGE状态恢复(recovery_ready为true)时，
 * 上层代码调用此函数来确认退出故障状态。这种设计防止电压刚刚波动到
 * 恢复阈值就立即恢复运行，需要上层根据实际情况做最终决策。
 *
 * @param compensation 电池补偿实例指针
 * @return true 确认成功，状态已转为OK；false 不满足恢复条件
 */
bool battery_compensation_acknowledge(battery_compensation_t *compensation)
{
    if (compensation == NULL || !compensation->recovery_ready) return false;
    if (compensation->status != BATTERY_COMP_INVALID &&
        compensation->status != BATTERY_COMP_UNDERVOLTAGE)
        return false;

    /* 执行状态转换：故障 -> OK */
    compensation->status = BATTERY_COMP_OK;
    compensation->fault_count = 0u;
    compensation->recovery_count = 0u;
    compensation->recovery_ready = false;
    return true;
}

/**
 * @brief 检查电池补偿是否允许系统运行(安全门禁函数)
 * @param compensation 电池补偿实例指针
 * @return true 可正常运行；false 状态异常需停机
 */
bool battery_compensation_can_run(const battery_compensation_t *compensation)
{
    return compensation != NULL && compensation->status == BATTERY_COMP_OK;
}

/**
 * @brief 获取当前补偿因子(线程安全，返回1.0作为默认值)
 * @param compensation 电池补偿实例指针
 * @return 补偿因子，异常时返回1.0f(无补偿，保持原始PWM)
 */
float battery_compensation_factor(const battery_compensation_t *compensation)
{
    return (compensation == NULL) ? 1.0f : compensation->factor;
}

/**
 * @brief 获取滤波后的电池电压
 * @param compensation 电池补偿实例指针
 * @return 滤波后的电压(毫伏)，异常时返回0.0f
 */
float battery_compensation_voltage_mv(const battery_compensation_t *compensation)
{
    return (compensation == NULL) ? 0.0f : compensation->filtered_mv;
}

/**
 * @brief 应用电池补偿到期望的PWM输出值
 *
 * 当电池电压低于参考值时，补偿因子>1，PWM输出相应增大以补偿功率损失。
 * 最终输出被限幅在[-maximum_pwm, maximum_pwm]范围内防止电机过驱。
 *
 * @param compensation 电池补偿实例指针
 * @param reference_pwm 期望的PWM参考值(补偿前)
 * @param maximum_pwm PWM输出最大绝对值
 * @return 补偿限幅后的最终PWM值；状态不可运行时返回0
 */
float battery_compensation_apply(const battery_compensation_t *compensation,
    float reference_pwm, float maximum_pwm)
{
    float output;

    /* 安全检查：状态不可用或参数异常时返回0(NaN自比较为false用于检测NaN) */
    if (!battery_compensation_can_run(compensation) ||
        reference_pwm != reference_pwm || maximum_pwm <= 0.0f)
        return 0.0f;

    /* 应用补偿：output = reference * factor */
    output = reference_pwm * compensation->factor;
    /* 限幅输出，保护电机和驱动器 */
    return clamp_float(output, -maximum_pwm, maximum_pwm);
}
