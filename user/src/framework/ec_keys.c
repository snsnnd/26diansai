/**
 * @file    ec_keys.c
 * @brief   按键输入管理与消抖模块 - 实现文件
 * @details 本文件实现了一个完整的按键输入管理系统，采用"中断生产 + 队列缓冲
 *          + 轮询消费"的生产者-消费者模式。中断服务函数负责检测按键事件并
 *          执行消抖处理，然后将事件放入环形缓冲区；主循环中的业务代码通过
 *          ec_keys_pop() 非阻塞读取按键事件。
 *
 *          【核心设计模式：生产者-消费者】
 *          - 生产者：三个按键的 EXTI 中断服务函数(key1_isr/key2_isr/key3_isr)
 *          - 缓冲区：无锁环形队列 (g_queue, g_head, g_tail)
 *          - 消费者：ec_keys_pop() 函数，在主循环中被业务逻辑调用
 *
 *          【消抖算法】
 *          每个按键独立记录上次触发的时间戳 (g_last_ms[3])，当一个按键事件
 *          发生时，检查距离上次事件的时间间隔，如果小于消抖时间(debounce_ms)，
 *          则视为机械弹跳噪声，丢弃该事件。
 *
 *          【中断安全性设计】
 *          1. 所有共享变量声明为 volatile，防止编译器优化
 *          - g_queue/g_head/g_tail：ISR 写入，主循环读取
 *          - g_last_ms：ISR 写入，ISR 读取
 *          - g_emergency_pending：ISR 写入，主循环读取并清除
 *          2. 关键代码段通过 __disable_irq()/__enable_irq() 保护
 *          3. 保护代码会保存并恢复 PRIMASK 状态，支持嵌套调用
 */
#include "framework/ec_keys.h"

#include "framework/ec_time.h"

/** @brief 按键事件环形缓冲区大小。只能存储8个事件，超出则丢弃最旧的事件 */
#define EC_KEY_QUEUE_SIZE 8u

/** @brief 按键模块配置（保存初始化时传入的配置参数） */
static ec_keys_config_t g_config;

/** @brief 按键事件环形缓冲区。volatile 防止编译器优化读取 */
static volatile uint8_t g_queue[EC_KEY_QUEUE_SIZE];

/**
 * @brief 环形缓冲区头指针（写指针）
 * @details 指向下一个空闲存储位置。ISR 中写入事件后头指针前进。
 *          head == tail 表示队列为空。
 */
static volatile uint8_t g_head;

/**
 * @brief 环形缓冲区尾指针（读指针）
 * @details 指向下一个待读取的事件。主循环中调用 ec_keys_pop 后尾指针前进。
 */
static volatile uint8_t g_tail;

/**
 * @brief 每个按键上次触发的时间戳
 * @details 用于软件消抖。索引 0/1/2 分别对应 KEY1/KEY2/KEY3。
 *          当按键事件发生时，检查当前时间与上次时间的差值是否 > debounce_ms。
 */
static volatile uint32_t g_last_ms[3];

/**
 * @brief 紧急停止标志
 * @details 当 KEY1 被按下时设为 true，由 ec_keys_emergency_pending() 读取并清零。
 *          这是"边沿触发"模式：每次 KEY1 按下只会被处理一次。
 */
static volatile bool g_emergency_pending;

/**
 * @brief 系统启动时间戳
 * @details 在 ec_keys_init 中记录，用于启动锁定判断。
 *          在 startup_lock_ms 时间内的所有按键事件都会被忽略。
 */
static volatile uint32_t g_startup_ms;

/**
 * @brief 将按键事件压入环形缓冲区
 * @details 内部函数，ISR 中使用。将按键编号写入队列的头指针位置，
 *          然后头指针前进。如果队列已满，则丢弃该事件（静默丢弃策略）。
 *
 * @param key 按键编号（1/2/3）
 * @note 调用者需确保在禁用中断的环境中调用此函数
 */
static void push_key(uint8_t key)
{
    uint8_t next = (uint8_t)((g_head + 1u) % EC_KEY_QUEUE_SIZE);
    if (next == g_tail) return;     /* 队列已满，丢弃事件 */
    g_queue[g_head] = key;          /* 写入按键编号 */
    g_head = next;                  /* 头指针前进 */
}

/**
 * @brief 按键事件生产者——处理消抖逻辑后将事件入队
 * @details 此函数实现了完整的按键事件过滤逻辑：
 *          1. 启动锁定检查：系统启动初期屏蔽所有按键
 *          2. 软件消抖检查：距离上次触发太近则丢弃
 *          3. 通过检查后：更新时间戳、入队
 *
 *          中断安全设计：函数入口禁用中断，出口恢复。
 *          使用 primask 保存/恢复机制支持中断嵌套场景。
 *
 * @param key 按键编号（1/2/3）
 */
static void key_event_producer(uint8_t key)
{
    uint32_t now;
    uint8_t index;
    uint32_t primask;

    /* 保存当前中断状态并禁用中断 */
    primask = __get_PRIMASK();
    __disable_irq();
    now = ec_time_ms();
    index = (uint8_t)(key - 1u);    /* 按键编号1→索引0，便利数组操作 */

    /**
     * 启动锁定检查。
     * 如果当前时间仍在启动锁定窗口内，则丢弃所有按键事件。
     * 防止系统上电/复位过程中，由于 GPIO 电平不稳定而产生的误触发。
     */
    if ((uint32_t)(now - g_startup_ms) < g_config.startup_lock_ms)
    {
        if (primask == 0u) __enable_irq();
        return;
    }

    /**
     * 软件消抖检查。
     * 如果距离该按键上次触发的时间小于消抖时间，认为是机械弹跳噪声。
     * 机械按键在按下和释放时通常会产生 5-20ms 的电平抖动(弹跳)，
     * 软件消抖通过时间窗口过滤掉这些虚假触发信号。
     */
    if ((uint32_t)(now - g_last_ms[index]) < g_config.debounce_ms)
    {
        if (primask == 0u) __enable_irq();
        return;
    }

    /* 通过所有检查：更新该按键的最后触发时间，将事件入队 */
    g_last_ms[index] = now;
    push_key(key);

    /* 恢复之前的中断状态 */
    if (primask == 0u) __enable_irq();
}

/**
 * @brief KEY1 中断服务函数（紧急停止键）
 * @details KEY1 被设计为特殊按键，除了产生普通按键事件外，
 *          还会触发紧急停止处理：
 *          1. 如果注册了 emergency_hook，立即调用它（在中断中！）
 *          2. 设置紧急停止标志 g_emergency_pending
 *          3. 作为普通按键1事件入队
 *
 * @param event 硬件事件类型（未使用）
 * @param context 用户上下文（未使用）
 * @note 运行在中断上下文中。emergency_hook 必须非常轻量且非阻塞！
 */
static void key1_isr(uint32 event, void *context)
{
    (void)event;
    (void)context;
    /* 先执行紧急停止钩子（若注册） */
    if (g_config.emergency_hook != NULL)
    {
        g_config.emergency_hook(g_config.emergency_context);
    }
    g_emergency_pending = true;         /* 置位紧急停止标志 */
    key_event_producer(1u);             /* 将KEY1事件入队 */
}

/**
 * @brief KEY2 中断服务函数
 * @details KEY2 是普通按键，只产生按键事件入队。
 * @param event 硬件事件类型（未使用）
 * @param context 用户上下文（未使用）
 */
static void key2_isr(uint32 event, void *context)
{
    (void)event;
    (void)context;
    key_event_producer(2u);
}

/**
 * @brief KEY3 中断服务函数
 * @details KEY3 是普通按键，只产生按键事件入队。
 * @param event 硬件事件类型（未使用）
 * @param context 用户上下文（未使用）
 */
static void key3_isr(uint32 event, void *context)
{
    (void)event;
    (void)context;
    key_event_producer(3u);
}

/**
 * @brief 初始化按键模块
 * @details 完成以下初始化步骤：
 *          1. 保存配置参数到全局变量
 *          2. 初始化环形队列（头=尾=0）
 *          3. 清除紧急停止标志
 *          4. 记录启动时间
 *          5. 初始化三个按键的 GPIO 为上拉输入
 *          6. 初始化三个按键的 EXTI 外部中断（下降沿触发）
 *
 * @param config 配置结构体指针。传入 NULL 则跳过初始化
 */
void ec_keys_init(const ec_keys_config_t *config)
{
    if (config == NULL) return;
    g_config = *config;                         /* 保存配置 */
    g_head = 0u;                                /* 清空队头 */
    g_tail = 0u;                                /* 清空队尾 */
    g_emergency_pending = false;                /* 清除急停标志 */
    g_startup_ms = ec_time_ms();                 /* 记录启动时间 */
    g_last_ms[0] = g_last_ms[1] = g_last_ms[2] = g_startup_ms; /* 初始化消抖时间戳 */

    /* 配置 GPIO：上拉输入模式，默认低电平 */
    gpio_init(g_config.key1_pin, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(g_config.key2_pin, GPI, GPIO_LOW, GPI_PULL_UP);
    gpio_init(g_config.key3_pin, GPI, GPIO_LOW, GPI_PULL_UP);

    /* 配置 EXTI 外部中断：下降沿触发（按键按下时电平从高→低） */
    exti_init(g_config.key1_pin, EXTI_TRIGGER_FALLING, key1_isr, NULL);
    exti_init(g_config.key2_pin, EXTI_TRIGGER_FALLING, key2_isr, NULL);
    exti_init(g_config.key3_pin, EXTI_TRIGGER_FALLING, key3_isr, NULL);
}

/**
 * @brief 从按键事件队列中弹出一个按键事件
 * @details 非阻塞式出队操作：
 *          1. 检查队列是否为空（g_tail == g_head）
 *          2. 从队尾位置读取按键编号
 *          3. 队尾指针前进（循环）
 *
 *          使用中断保护确保 ISR 和主循环之间不会产生竞态条件。
 *
 * @param key 输出参数，接收按键编号（1/2/3）
 * @return true=成功取出按键，false=队列为空或参数无效
 */
bool ec_keys_pop(uint8_t *key)
{
    uint32_t primask;

    if (key == NULL) return false;
    primask = __get_PRIMASK();
    __disable_irq();        /* 禁止中断，防止 ISR 同时修改队列 */
    if (g_tail == g_head)   /* 队列为空 */
    {
        if (primask == 0u) __enable_irq();
        return false;
    }
    *key = g_queue[g_tail];                         /* 读取按键编号 */
    g_tail = (uint8_t)((g_tail + 1u) % EC_KEY_QUEUE_SIZE); /* 尾指针前进 */
    if (primask == 0u) __enable_irq();               /* 恢复中断 */
    return true;
}

/**
 * @brief 查询并清除紧急停止状态
 * @details 检查是否有紧急停止事件触发（KEY1 按下）。
 *          该函数会原子地读取并清除 g_emergency_pending 标志，
 *          确保每个紧急停止事件只被处理一次。
 *
 *          典型用法是在主循环中周期性调用：
 *          if (ec_keys_emergency_pending()) {
 *              ec_app_emergency_stop();  // 执行紧急停止
 *          }
 *
 * @return true=有紧急停止事件待处理
 */
bool ec_keys_emergency_pending(void)
{
    bool pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();            /* 原子操作保护 */
    pending = g_emergency_pending;
    g_emergency_pending = false; /* 读取后立即清除 */
    if (primask == 0u)
    {
        __enable_irq();
    }
    return pending;
}
