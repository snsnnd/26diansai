/**
 * @file dt_encoder.c
 * @brief 编码器驱动模块实现
 *        支持 GPIO 中断和硬件定时器正交解码两种模式，
 *        提供里程计数据（脉冲计数、有符号计数、RPM、行驶距离等）。
 *        内置非法状态跳变检测用于诊断脉冲丢失情况。
 */

#include "driver/dt_encoder.h"

#include <stdint.h>

/**
 * @brief 获取编码器 A/B 引脚的实时电平状态
 *        非正交模式仅采样 A 引脚，正交模式同时采样 A 和 B
 * @param enc 编码器结构体指针
 * @return 状态字：bit1=A相电平，bit0=B相电平
 *         （单相模式时 B 位恒为0）
 */
uint8_t dt_encoder_get_ab_state(const dt_encoder_t *enc)
{
    if (enc == NULL)
    {
        return 0u;
    }
    if (!enc->quadrature_enabled)
    {
        /* 单相模式：仅读取 A 引脚电平，左移1位到bit1位置 */
        return (uint8_t)(gpio_get_level(enc->a_pin) << 1u);
    }
    /* 正交模式：A相电平在bit1，B相电平在bit0 */
    return (uint8_t)((gpio_get_level(enc->a_pin) << 1u) |
        gpio_get_level(enc->b_pin));
}

/**
 * @brief 从硬件正交解码定时器更新编码器计数
 *        读取定时器的脉冲差值，转换为有符号计数并累加到内部状态中。
 *        操作时需关中断以保证原子性，避免中断嵌套导致计数不一致。
 * @param enc 编码器结构体指针
 * @note 仅在 hardware_quadrature=true 时生效
 */
static void dt_encoder_update_hardware(dt_encoder_t *enc)
{
    int16_t delta;           /* 定时器硬件报告的脉冲增量（有符号16位） */
    int32_t signed_delta;    /* 应用方向符号后的有符号增量 */
    uint32_t magnitude;      /* 增量的绝对值 */
    uint32_t primask;        /* 保存中断屏蔽状态，用于恢复 */

    if (enc == NULL || !enc->initialized || !enc->hardware_quadrature) return;
    delta = encoder_get_delta(enc->timer_index); /* 从硬件读取脉冲增量 */
    if (delta == 0) return;

    /* 应用方向符号：direction_sign为1或-1 */
    signed_delta = (int32_t)delta * (int32_t)enc->direction_sign;
    /* 计算绝对值（用于无符号计数累加） */
    magnitude = (signed_delta < 0) ? (uint32_t)(-signed_delta) :
        (uint32_t)signed_delta;

    /* 关中断保护，防止与中断服务程序同时访问同一变量 */
    primask = __get_PRIMASK();
    __disable_irq();

    /* edge_count累加，检测溢出并钳位到UINT32_MAX */
    enc->edge_count = (UINT32_MAX - enc->edge_count < magnitude) ?
        UINT32_MAX : enc->edge_count + magnitude;
    /* sampled_transition_count同步累加，保持与edge_count一致 */
    enc->sampled_transition_count =
        (UINT32_MAX - enc->sampled_transition_count < magnitude) ?
        UINT32_MAX : enc->sampled_transition_count + magnitude;

    /* signed_edge_count按方向累加，检测溢出并钳位到INT32_MAX/MIN */
    if (signed_delta > 0)
    {
        enc->signed_edge_count =
            (enc->signed_edge_count > INT32_MAX - signed_delta) ?
            INT32_MAX : enc->signed_edge_count + signed_delta;
    }
    else
    {
        enc->signed_edge_count =
            (enc->signed_edge_count < INT32_MIN - signed_delta) ?
            INT32_MIN : enc->signed_edge_count + signed_delta;
    }

    /* 恢复之前的中断状态（如果进入时中断是开启的，则重新开启） */
    if (primask == 0u) __enable_irq();
}

/**
 * @brief 编码器 GPIO 中断服务程序
 *        响应 A 相和/或 B 相的电平跳变中断，更新脉冲计数和方向计数。
 *        正交模式下通过查表法将 A/B 状态转换解码为+1/-1/0步进。
 *        单相模式下仅计数 A 相边沿，忽略 B 相。
 * @param event 中断事件标志（本驱动中未使用）
 * @param ptr 指向 dt_encoder_t 结构体的 void 指针
 * @note 中断中不应执行耗时操作；quadrature_step 静态表格使用前一次和当前
 *       AB状态组合作为索引，直接查表得到方向步进值，避免条件分支。
 */
static void dt_encoder_isr(uint32_t event, void *ptr)
{
    /* 正交解码状态转换表：
     * 索引 = (上次AB状态 << 2) | 当前AB状态
     * AB状态编码：bit1 = A, bit0 = B
     * 表值含义：+1 = 正转一步, -1 = 反转一步, 0 = 非法跳变或状态未变
     * 此表的理论基础：正交编码器的合法状态转换只有4种（00->10, 10->11, 11->01, 01->00
     * 为正转；反向则为00->01, 01->11, 11->10, 10->00） */
    static const int8_t quadrature_step[16] = {
         0,  1, -1,  0,   /* 00->00=0, 00->01=+1, 00->10=-1, 00->11=非法 */
        -1,  0,  0,  1,   /* 01->00=-1, 01->01=0, 01->10=非法, 01->11=+1 */
         1,  0,  0, -1,   /* 10->00=+1, 10->01=非法, 10->10=0, 10->11=-1 */
         0, -1,  1,  0    /* 11->00=非法, 11->01=-1, 11->10=+1, 11->11=0 */
    };
    dt_encoder_t *enc = (dt_encoder_t *)ptr;
    uint8_t current_state;  /* 当前AB引脚电平状态 */
    int8_t step;            /* 查表得到的步进值 */

    (void)event;

    current_state = dt_encoder_get_ab_state(enc);

    if (!enc->quadrature_enabled)
    {
        /* === 单相模式 ===
         * 每次中断（A相边沿）计数+1，按direction_sign方向增减 */
        enc->edge_count++;
        if (enc->direction_sign > 0 && enc->signed_edge_count < INT32_MAX)
        {
            enc->signed_edge_count++;
        }
        else if (enc->direction_sign < 0 && enc->signed_edge_count > INT32_MIN)
        {
            enc->signed_edge_count--;
        }
        enc->last_ab_state = current_state;
        return;
    }

    /* === 正交模式 ===
     * 将上次状态和当前状态组合为4位索引查表 */
    step = quadrature_step[(enc->last_ab_state << 2u) | current_state];

    if (step != 0)
    {
        /* 有效跳变：应用方向符号后更新计数 */
        int8_t signed_step = (int8_t)(step * enc->direction_sign);
        enc->edge_count++;
        if (signed_step > 0 && enc->signed_edge_count < INT32_MAX)
        {
            enc->signed_edge_count++;
        }
        else if (signed_step < 0 && enc->signed_edge_count > INT32_MIN)
        {
            enc->signed_edge_count--;
        }
    }
    else if (current_state != enc->last_ab_state)
    {
        /* 状态发生了改变但查表结果为0——说明是非法跳变
         * 可能原因：干扰脉冲、边沿丢失或电机抖动 */
        enc->invalid_transition_count++;
    }

    enc->last_ab_state = current_state; /* 保存当前状态供下一次判断 */
}

/**
 * @brief 初始化编码器驱动
 *        根据配置选择以下两种方式之一：
 *        1) 硬件正交解码：使用定时器编码器接口，配置A/B通道
 *        2) GPIO中断方式：配置GPIO引脚和双边沿中断，可选正交或单相模式
 *        执行参数检查、状态变量复位、引脚初始化及中断注册。
 * @param enc 编码器结构体指针（需已填充引脚及参数配置）
 * @return true 配置受支持且初始化完成；false 配置无效
 * @note 当前逐飞底层仅支持 TIMG8 硬件 QEI；其他定时器应使用 GPIO 解码
 */
bool dt_encoder_init(dt_encoder_t *enc)
{
    if (enc == NULL)
    {
        return false;
    }

    enc->initialized = false;
    if (enc->counts_per_rev == 0u ||
        (enc->quadrature_enabled && enc->a_pin == enc->b_pin))
    {
        return false;  /* 无效参数，无法初始化 */
    }

    if (enc->hardware_quadrature &&
        (enc->timer_index != TIM_G8 ||
         (((uint32_t)enc->channel_a >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) !=
             (uint32_t)enc->timer_index ||
         (((uint32_t)enc->channel_b >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) !=
             (uint32_t)enc->timer_index))
    {
        return false;
    }

    /* RPM低通滤波系数合法性检查，无效则使用默认值0.35 */
    if (enc->rpm_lpf_alpha <= 0.0f || enc->rpm_lpf_alpha >= 1.0f)
    {
        enc->rpm_lpf_alpha = 0.35f;
    }

    /* --- 复位所有状态变量 --- */
    enc->edge_count = 0u;
    enc->signed_edge_count = 0;
    enc->invalid_transition_count = 0u;
    enc->sampled_transition_count = 0u;
    enc->last_edge_count = 0u;
    enc->last_signed_edge_count = 0;
    enc->rpm = 0.0f;
    enc->rpm_signed = 0.0f;

    /* direction_sign 只能为 1 或 -1，防止调用方传入0 */
    enc->direction_sign = (enc->direction_sign < 0) ? -1 : 1;

    /* 车轮周长若为负值则修正为0 */
    if (enc->wheel_circumference_mm < 0.0f)
    {
        enc->wheel_circumference_mm = 0.0f;
    }

    if (enc->hardware_quadrature)
    {
        /* === 硬件正交解码模式 === */
        enc->quadrature_enabled = true;
        encoder_quad_init(enc->timer_index, enc->channel_a, enc->channel_b);
        encoder_clear_count(enc->timer_index); /* 清除定时器计数初始值 */

        enc->last_ab_state = 0u;
        enc->sampled_ab_state = 0u;
        enc->initialized = true;
        return true;
    }

    /* === GPIO中断模式 === */
    gpio_init(enc->a_pin, GPI, GPIO_HIGH, GPI_PULL_UP); /* A相输入，上拉 */
    if (enc->quadrature_enabled)
    {
        gpio_init(enc->b_pin, GPI, GPIO_HIGH, GPI_PULL_UP); /* B相输入，上拉 */
    }

    /* 获取初始AB状态 */
    enc->last_ab_state = dt_encoder_get_ab_state(enc);
    enc->sampled_ab_state = enc->last_ab_state;

    /* 注册A相双边沿中断 */
    exti_init(enc->a_pin, EXTI_TRIGGER_BOTH, dt_encoder_isr, enc);
    if (enc->quadrature_enabled)
    {
        /* 正交模式还需要B相中断 */
        exti_init(enc->b_pin, EXTI_TRIGGER_BOTH, dt_encoder_isr, enc);
    }

    enc->initialized = true;
    return true;
}

bool dt_encoder_is_ready(const dt_encoder_t *enc)
{
    return enc != NULL && enc->initialized;
}

/**
 * @brief 独立于中断的编码器输入采样函数
 *        通过轮询方式检测AB引脚状态变化，记录到 sampled_transition_count。
 *        该计数独立于中断驱动的 edge_count，可用于诊断中断是否丢失。
 * @param enc 编码器结构体指针
 * @note 硬件正交解码模式下，此函数内部调用 dt_encoder_update_hardware()
 *       从定时器读取增量并更新计数。
 */
void dt_encoder_sample_inputs(dt_encoder_t *enc)
{
    uint32_t primask;
    uint8_t current_state;

    if (!dt_encoder_is_ready(enc))
    {
        return;
    }

    if (enc->hardware_quadrature)
    {
        /* 硬件模式：从定时器寄存器同步硬件计数到软件变量 */
        dt_encoder_update_hardware(enc);
        return;
    }

    /* GPIO模式：轮询AB引脚电平并与上次采样值比较 */
    current_state = dt_encoder_get_ab_state(enc);

    primask = __get_PRIMASK();
    __disable_irq(); /* 关中断保护临界区 */

    if (current_state != enc->sampled_ab_state)
    {
        /* 状态发生变化，递增采样计数并更新采样状态 */
        enc->sampled_transition_count++;
        enc->sampled_ab_state = current_state;
    }

    if (primask == 0u)
    {
        __enable_irq(); /* 恢复中断 */
    }
}

/**
 * @brief 获取采样方式检测到的状态变化总次数（诊断用）
 *        与 edge_count 比较可评估中断丢失的程度
 * @param enc 编码器结构体指针
 * @return 采样检测到的跳变次数
 */
uint32_t dt_encoder_get_sampled_transitions(dt_encoder_t *enc)
{
    uint32_t count;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();          /* 关中断读取volatile变量 */
    count = enc->sampled_transition_count;
    if (primask == 0u)
    {
        __enable_irq();       /* 恢复中断 */
    }
    return count;
}

/**
 * @brief 复位编码器里程计数据
 *        清零脉冲计数、符号计数、非法跳变计数、RPM等所有状态，
 *        并重新获取当前AB状态作为基准。
 * @param enc 编码器结构体指针
 * @warning 应确保车轮停止时调用！复位后到达的脉冲属于新的测量周期
 */
void dt_encoder_reset_odometry(dt_encoder_t *enc)
{
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq(); /* 关中断保护全部复位操作 */

    if (enc->hardware_quadrature)
    {
        encoder_clear_count(enc->timer_index); /* 清除硬件定时器计数 */
    }

    /* 清零所有里程计状态变量 */
    enc->edge_count = 0u;
    enc->signed_edge_count = 0;
    enc->invalid_transition_count = 0u;
    enc->last_ab_state = dt_encoder_get_ab_state(enc);
    enc->sampled_transition_count = 0u;
    enc->sampled_ab_state = enc->last_ab_state;
    enc->last_edge_count = 0u;
    enc->last_signed_edge_count = 0;
    enc->rpm = 0.0f;
    enc->rpm_signed = 0.0f;

    if (primask == 0u)
    {
        __enable_irq(); /* 恢复中断 */
    }
}

/**
 * @brief 获取自复位以来有效的正交边沿总数（无方向信息）
 *        获取前先同步硬件计数（若使用硬件正交模式）
 * @param enc 编码器结构体指针
 * @return 有效边沿总数
 */
uint32_t dt_encoder_get_edges(dt_encoder_t *enc)
{
    uint32_t edges;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0u;
    }

    dt_encoder_update_hardware(enc); /* 硬件模式：先同步定时器计数 */
    primask = __get_PRIMASK();
    __disable_irq();                  /* 关中断读取volatile变量 */
    edges = enc->edge_count;
    if (primask == 0u)
    {
        __enable_irq();
    }

    return edges;
}

/**
 * @brief 获取非法状态跳变的累计次数（诊断用）
 *        非法跳变通常由电气噪声、脉冲丢失或电机抖动引起。
 *        注意：零值并不保证没有脉冲丢失（两次丢失可能相互抵消）。
 * @param enc 编码器结构体指针
 * @return 非法跳变计数
 */
uint32_t dt_encoder_get_invalid_transitions(dt_encoder_t *enc)
{
    uint32_t count;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0u;
    }

    dt_encoder_update_hardware(enc);
    primask = __get_PRIMASK();
    __disable_irq();
    count = enc->invalid_transition_count;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return count;
}

/**
 * @brief 获取应用 direction_sign 后的带符号边沿计数
 *        正数=正方向移动，负数=反方向移动
 * @param enc 编码器结构体指针
 * @return 带符号的边沿计数
 */
int32_t dt_encoder_get_signed_edges(dt_encoder_t *enc)
{
    int32_t edges;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0;
    }

    dt_encoder_update_hardware(enc);
    primask = __get_PRIMASK();
    __disable_irq();
    edges = enc->signed_edge_count;
    if (primask == 0u)
    {
        __enable_irq();
    }

    return edges;
}

/**
 * @brief 获取自上次调用后新增的脉冲增量（差分值）
 *        适用于周期性采样计算瞬时速度
 * @param enc 编码器结构体指针
 * @return 脉冲增量数（无符号）
 */
uint32_t dt_encoder_get_delta(dt_encoder_t *enc)
{
    uint32_t current;
    uint32_t delta;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0u;
    }

    dt_encoder_update_hardware(enc); /* 硬件模式先同步计数 */
    primask = __get_PRIMASK();
    __disable_irq();                  /* 关中断保护 */
    current = enc->edge_count;
    delta = current - enc->last_edge_count;   /* 当前值减上次保存值 = 增量 */
    enc->last_edge_count = current;           /* 更新基准值 */
    if (primask == 0u)
    {
        __enable_irq();               /* 恢复中断 */
    }
    return delta;
}

/**
 * @brief 计算当前电机转速（RPM，转/分钟）
 *        基于脉冲增量和时间间隔计算原始RPM，再通过一阶低通滤波平滑输出。
 *        滤波公式：RPM = RPM_prev * (1-alpha) + RPM_raw * alpha
 * @param enc 编码器结构体指针
 * @param dt_ms 距离上次调用的时间间隔（毫秒）
 * @return 滤波后的转速值（RPM）
 */
float dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms)
{
    uint32_t edge_delta;
    float rpm_raw;

    if (!dt_encoder_is_ready(enc))
    {
        return 0.0f;
    }

    edge_delta = dt_encoder_get_delta(enc); /* 获取最新脉冲增量 */

    /* RPM转换公式：增量 * (60000ms/min) / (每转脉冲数) / (采样间隔ms) */
    if (edge_delta != 0 && dt_ms > 0 && enc->counts_per_rev > 0)
    {
        rpm_raw = (float)edge_delta * 60000.0f / (float)enc->counts_per_rev / (float)dt_ms;
    }
    else
    {
        rpm_raw = 0.0f; /* 无脉冲或无有效参数时原始RPM为0 */
    }

    /* 一阶低通滤波：平滑RPM输出，滤除瞬时波动 */
    enc->rpm = enc->rpm * (1.0f - enc->rpm_lpf_alpha) + rpm_raw * enc->rpm_lpf_alpha;
    return enc->rpm;
}

/**
 * @brief 获取带符号脉冲增量（使用 signed_edge_count）
 */
int32_t dt_encoder_get_signed_delta(dt_encoder_t *enc)
{
    int32_t current;
    int32_t delta;
    uint32_t primask;

    if (!dt_encoder_is_ready(enc))
    {
        return 0;
    }

    dt_encoder_update_hardware(enc);
    primask = __get_PRIMASK();
    __disable_irq();
    current = enc->signed_edge_count;
    delta = current - enc->last_signed_edge_count;
    enc->last_signed_edge_count = current;
    if (primask == 0u)
    {
        __enable_irq();
    }
    return delta;
}

/**
 * @brief 计算带符号转速（RPM），正值为前进方向
 */
float dt_encoder_compute_signed_rpm(dt_encoder_t *enc, uint32_t dt_ms)
{
    int32_t signed_delta;
    float rpm_raw;

    if (!dt_encoder_is_ready(enc))
    {
        return 0.0f;
    }

    signed_delta = dt_encoder_get_signed_delta(enc);

    if (signed_delta != 0 && dt_ms > 0 && enc->counts_per_rev > 0)
    {
        rpm_raw = (float)signed_delta * 60000.0f
            / (float)enc->counts_per_rev / (float)dt_ms;
    }
    else
    {
        rpm_raw = 0.0f;
    }

    enc->rpm_signed = enc->rpm_signed * (1.0f - enc->rpm_lpf_alpha)
        + rpm_raw * enc->rpm_lpf_alpha;
    return enc->rpm_signed;
}

/**
 * @brief 计算总行驶路径长度（绝对值，无方向）
 *        公式：总脉冲数 * 轮周长 / 每转脉冲数
 * @param enc 编码器结构体指针
 * @return 总行驶距离（毫米）
 */
float dt_encoder_get_travel_mm(dt_encoder_t *enc)
{
    if (enc == NULL || enc->counts_per_rev == 0u ||
        enc->wheel_circumference_mm <= 0.0f)
    {
        return 0.0f;  /* 参数无效时返回0 */
    }

    /* 脉冲总数 * 每脉冲对应的行走距离 */
    return (float)dt_encoder_get_edges(enc) * enc->wheel_circumference_mm /
        (float)enc->counts_per_rev;
}

/**
 * @brief 计算有符号位移量（考虑方向，单位为毫米）
 *        正值 = 正方向前进，负值 = 反方向后退
 *        公式：有符号脉冲数 * 轮周长 / 每转脉冲数
 * @param enc 编码器结构体指针
 * @return 有符号位移（毫米）
 */
float dt_encoder_get_distance_mm(dt_encoder_t *enc)
{
    if (enc == NULL || enc->counts_per_rev == 0u ||
        enc->wheel_circumference_mm <= 0.0f)
    {
        return 0.0f;
    }

    return (float)dt_encoder_get_signed_edges(enc) * enc->wheel_circumference_mm /
        (float)enc->counts_per_rev;
}
