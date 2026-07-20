/**
 * @file dt_buzzer.c
 * @brief 蜂鸣器驱动模块实现
 *        提供蜂鸣器的初始化、开关控制、同步/异步鸣叫以及序列播放功能。
 *        支持通过周期性调用service函数实现非阻塞的节拍序列播放。
 */

#include "driver/dt_buzzer.h"

/**
 * @brief 设置蜂鸣器GPIO输出电平
 * @param cfg 蜂鸣器配置结构体指针
 * @param on true=输出高电平（开启蜂鸣器），false=输出低电平（关闭蜂鸣器）
 * @note 无参数校验，调用方需确保cfg非空且已初始化
 */
static void dt_buzzer_set_output(dt_buzzer_config_t *cfg, bool on)
{
    if (on)
    {
        gpio_high(cfg->pin);   /* 引脚输出高电平，驱动蜂鸣器发声 */
    }
    else
    {
        gpio_low(cfg->pin);    /* 引脚输出低电平，蜂鸣器静音 */
    }
}

/**
 * @brief 取消蜂鸣器正在执行的序列服务
 *        清空序列相关状态并将服务标记为非激活
 * @param cfg 蜂鸣器配置结构体指针
 */
static void dt_buzzer_cancel_service(dt_buzzer_config_t *cfg)
{
    cfg->sequence = NULL;           /* 清空序列指针，不再引用任何序列 */
    cfg->sequence_length = 0u;      /* 序列长度归零 */
    cfg->sequence_index = 0u;       /* 序列索引从头开始 */
    cfg->service_active = false;    /* 标记服务停止 */
}

/**
 * @brief 初始化蜂鸣器引脚及驱动状态
 *        将GPIO配置为推挽输出，初始为低电平（蜂鸣器关闭），
 *        同时清空所有序列服务状态
 * @param cfg 蜂鸣器配置结构体指针
 */
void dt_buzzer_init(dt_buzzer_config_t *cfg)
{
    gpio_init(cfg->pin, GPO, GPIO_LOW, GPO_PUSH_PULL); /* 推挽输出，初始低电平 */
    dt_buzzer_cancel_service(cfg);  /* 清空序列状态 */
    cfg->deadline_ms = 0u;          /* 超时时间戳复位 */
}

/**
 * @brief 强制开启蜂鸣器（先取消任何正在播放中的序列）
 *        调用后蜂鸣器持续发声，直到手动调用off或beep等函数
 * @param cfg 蜂鸣器配置结构体指针
 */
void dt_buzzer_on(dt_buzzer_config_t *cfg)
{
    dt_buzzer_cancel_service(cfg);  /* 停止正在进行的序列播放 */
    dt_buzzer_set_output(cfg, true); /* GPIO输出高电平，蜂鸣器发声 */
}

/**
 * @brief 强制关闭蜂鸣器（先取消任何正在播放中的序列）
 * @param cfg 蜂鸣器配置结构体指针
 */
void dt_buzzer_off(dt_buzzer_config_t *cfg)
{
    dt_buzzer_cancel_service(cfg);   /* 停止正在进行的序列播放 */
    dt_buzzer_set_output(cfg, false); /* GPIO输出低电平，蜂鸣器静音 */
}

/**
 * @brief 同步方式使蜂鸣器鸣叫指定的时长
 *        先开启蜂鸣器，阻塞等待duration_ms毫秒后关闭。
 *        注意：此函数是阻塞的，在等待期间CPU无法执行其他任务，
 *        适用于短暂的提示音或不需要并行的场景。
 * @param cfg 蜂鸣器配置结构体指针
 * @param duration_ms 鸣叫持续时间（毫秒）
 */
void dt_buzzer_beep(dt_buzzer_config_t *cfg, uint32_t duration_ms)
{
    dt_buzzer_on(cfg);                     /* 开启蜂鸣器 */
    system_delay_ms(duration_ms);          /* 阻塞等待指定时长 */
    dt_buzzer_off(cfg);                    /* 关闭蜂鸣器 */
}

/**
 * @brief 异步方式使蜂鸣器鸣叫指定时长
 *        立即开启蜂鸣器并设置超时时间戳，随后由dt_buzzer_service()
 *        在主循环中周期性调用以判断何时超时关闭。
 *        相比同步beep函数，此函数不阻塞，适合在控制循环中配合使用。
 * @param cfg 蜂鸣器配置结构体指针
 * @param duration_ms 鸣叫持续时间（毫秒）
 * @param now_ms 当前系统时间戳（毫秒），需与service函数使用相同的时间基准
 */
void dt_buzzer_beep_async(dt_buzzer_config_t *cfg, uint32_t duration_ms,
    uint32_t now_ms)
{
    dt_buzzer_cancel_service(cfg);        /* 取消之前的序列 */
    dt_buzzer_set_output(cfg, true);       /* 开启蜂鸣器 */
    cfg->deadline_ms = now_ms + duration_ms; /* 计算超时时间点 */
    cfg->service_active = true;            /* 激活服务轮询 */
    dt_buzzer_service(cfg, now_ms);        /* 立即执行一次服务检查 */
}

/**
 * @brief 播放一段蜂鸣器节拍序列（如"滴-滴-滴-"等自定义节奏）
 *        立即执行序列的第一个节拍，并记录后续节拍信息，
 *        节拍切换由dt_buzzer_service()在主循环中驱动。
 * @param cfg 蜂鸣器配置结构体指针
 * @param sequence 指向dt_buzzer_step_t数组的指针，定义各节拍的开/关状态和时长
 * @param length 序列中的节拍总数
 * @param now_ms 当前系统时间戳（毫秒），用于计算首个节拍的超时时间
 * @note 传入的sequence数组需保持有效，直到序列播放完成或被取消
 */
void dt_buzzer_play_sequence(dt_buzzer_config_t *cfg,
    const dt_buzzer_step_t *sequence, uint8_t length, uint32_t now_ms)
{
    if (sequence == NULL || length == 0u)
    {
        dt_buzzer_off(cfg);    /* 无效参数则关闭蜂鸣器 */
        return;
    }

    cfg->sequence = sequence;              /* 保存序列指针 */
    cfg->sequence_length = length;         /* 保存序列总长度 */
    cfg->sequence_index = 0u;              /* 从第0个节拍开始 */
    cfg->service_active = true;            /* 激活服务 */
    dt_buzzer_set_output(cfg, sequence[0].on); /* 执行第一个节拍的输出 */
    cfg->deadline_ms = now_ms + sequence[0].duration_ms; /* 第一个节拍的超时时间 */
    dt_buzzer_service(cfg, now_ms);        /* 立即执行服务检查 */
}

/**
 * @brief 蜂鸣器状态服务函数 - 非阻塞，需在主循环中周期性调用
 *        检查当前节拍是否超时，超时则自动切换到下一节拍
 *        或（若序列结束）关闭蜂鸣器。
 *        使用 while 循环一次性处理所有已超时的节拍，避免多次调用时累积延迟。
 * @param cfg 蜂鸣器配置结构体指针
 * @param now_ms 当前系统时间戳（毫秒），用于与deadline_ms比较判断超时
 * @note 时间比较使用(int32_t)差值方式，能够正确处理uint32_t溢出的情况
 */
void dt_buzzer_service(dt_buzzer_config_t *cfg, uint32_t now_ms)
{
    /* 循环处理所有已超时的节拍：
     * (int32_t)(now_ms - deadline_ms) >= 0 表示当前时间已到达或超过deadline，
     * 使用int32_t转换利用有符号数溢出特性实现鲁棒的时间比较 */
    while (cfg->service_active &&
        (int32_t)(now_ms - cfg->deadline_ms) >= 0)
    {
        /* 检查序列中是否还有下一个节拍 */
        if (cfg->sequence != NULL &&
            (uint8_t)(cfg->sequence_index + 1u) < cfg->sequence_length)
        {
            const dt_buzzer_step_t *step;

            cfg->sequence_index++;           /* 前进到下一个节拍 */
            step = &cfg->sequence[cfg->sequence_index];
            dt_buzzer_set_output(cfg, step->on);  /* 设置该节拍的输出状态 */
            cfg->deadline_ms += step->duration_ms; /* 累加计算新的超时时间 */
        }
        else
        {
            /* 序列结束或无序列时，取消服务并关闭蜂鸣器 */
            dt_buzzer_cancel_service(cfg);   /* 清空序列状态 */
            dt_buzzer_set_output(cfg, false); /* GPIO低电平，蜂鸣器静音 */
        }
    }
}

/**
 * @brief 兼容任务调度器的服务封装函数
 *        用于将dt_buzzer_service适配为定时器/任务回调格式，
 *        方便在调度框架中以统一接口注册执行。
 * @param now_ms 当前系统时间戳（毫秒），由调度器传入
 * @param context 指向dt_buzzer_config_t的void指针，由调度器传入
 */
void dt_buzzer_service_task(uint32_t now_ms, void *context)
{
    if (context != NULL)
    {
        /* 将context转换回dt_buzzer_config_t指针并执行服务逻辑 */
        dt_buzzer_service((dt_buzzer_config_t *)context, now_ms);
    }
}
