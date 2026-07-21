/**
 * @file    ec_app.c
 * @brief   应用程序入口与主循环管理层 - 实现文件
 * @details 本文件实现了应用的初始化、任务注册和主循环驱动逻辑。
 *          根据编译时配置 (EC_APP_PROFILE)，系统可以编译为不同的应用形态：
 *
 *          【三种应用形态】
 *          1. 硬件测试模式 (EC_APP_PROFILE_HARDWARE_TEST)
 *             用于产线测试或硬件调试，通过一个简单的循环任务测试各模块。
 *             包含云台(gimbal)控制。
 *
 *          2. 巡线车模式 (EC_APP_PROFILE_LINE_CAR)
 *             完整的智能巡线小车应用，包含11个周期任务：
 *             输入(5ms)、陀螺仪(10ms)、巡线传感器(10ms)、传感器融合(50ms)、
 *             控制(10ms)、菜单(200ms)、蜂鸣器(5ms)、OLED显示(20ms)、
 *             遥测(20ms)、调试(100ms)、调参(5ms)
 *
 *          【任务周期设计思路】
 *          各任务的执行周期根据其功能需求和对实时性的要求而设定：
 *          - 5ms (200Hz)：输入采集、蜂鸣器、调参——对实时性要求最高
 *          - 10ms (100Hz)：陀螺仪、巡线传感器、控制——控制环路核心
 *          - 20ms (50Hz)：OLED显示、遥测——输出刷新，不需要太快
 *          - 50ms (20Hz)：传感器融合——较慢的传感器数据融合
 *          - 100ms (10Hz)：调试——调试信息输出
 *          - 200ms (5Hz)：菜单——人机交互，不需要高频率
 *
 *          【错相启动策略】
 *          部分任务的 start_ms 设置为 now_ms + offset，使各任务的执行时间点
 *          相互错开，避免所有任务在同一毫秒集中执行，降低瞬时CPU负载。
 *          例如 control(now_ms+10) 和 line(now_ms+10) 同时注册，但它们在
 *          调度器中按注册顺序依次执行，且由于周期相同(10ms)，下次会再次同时触发。
 */
#include "framework/ec_app.h"

#include "config.h"
#include "framework/ec_scheduler.h"
#include "framework/ec_time.h"
#include "zf_common_headfile.h"

/* 根据应用配置包含相应的头文件 */
#if EC_APP_PROFILE == EC_APP_PROFILE_HARDWARE_TEST
#include "app/hardware_test.h"
#include "gimbal/gimbal.h"
#elif EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR
#include "app/line_car.h"
#include "app/h2024_tasks.h"
#include "app/e2025_task.h"
#endif

/**
 * 全局调度器实例（静态，仅本文件可见）。
 * 采用模块内静态变量的设计，对外隐藏调度器实例的具体实现细节，
 * 外部只能通过 ec_app_get_scheduler() 以 const 指针方式只读访问。
 */
static ec_scheduler_t g_scheduler;

/**
 * @brief 向调度器注册任务的便捷封装函数
 * @details 封装 ec_scheduler_add，当注册失败时触发断言。
 *          这样做的好处是：
 *          1. 简化调用代码——调用者不需要检查返回值
 *          2. 快速失败(Fast Fail)——如果任务注册失败，系统在开发阶段就崩溃
 *             而不是默默地运行在不完整的状态下
 *          3. 上下文参数默认传递 NULL——大多数智能车任务不需要上下文
 *
 * @param name 任务名称
 * @param run 任务函数指针
 * @param period_ms 执行周期（毫秒）
 * @param start_ms 首次执行时间（毫秒）
 */
static void ec_app_add_task(const char *name, ec_task_fn run,
    uint32_t period_ms, uint32_t start_ms)
{
    if (!ec_scheduler_add(&g_scheduler, name, run, NULL, period_ms, start_ms))
    {
        zf_assert(0);   /* 任务注册失败→系统不可用→触发断言停止 */
    }
}

/**
 * @brief 非巡线车模式下的通用任务函数
 * @details 当系统不工作在巡线车模式时，使用此任务作为唯一的周期性任务。
 *          它根据具体配置调用相应的运行函数。
 *
 * @note 本函数体在巡线车模式下不会被编译（由 #if 条件控制），
 *       因为巡线车模式有自己的任务集。
 */
#if EC_APP_PROFILE != EC_APP_PROFILE_LINE_CAR
static void ec_app_task(uint32_t now_ms, void *context)
{
    (void)now_ms;
    (void)context;

#if EC_APP_PROFILE == EC_APP_PROFILE_HARDWARE_TEST
    hardware_test_run();
#endif
}
#endif

#if EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR

bool g_vision_ready = false;
bool g_foc_gimbal_ready = false;

void ec_vision_lazy_init(void)
{
    if (!g_vision_ready) { /* maixcam2_init(); -- 待硬件就绪 */ g_vision_ready = true; }
}

void ec_foc_gimbal_lazy_init(void)
{
    if (!g_foc_gimbal_ready) { /* foc_gimbal_init(); -- 待硬件就绪 */ g_foc_gimbal_ready = true; }
}
#endif

/**
 * @brief 初始化整个应用程序
 * @details 初始化流程：
 *          1. 初始化调度器——必须先于任务注册
 *          2. 初始化时间系统——为任务提供时间基准
 *          3. 根据不同模式初始化应用层
 *          4. 注册所有周期性任务
 *
 *          任务注册时采用的"错相启动"策略：
 *          - 立即启动的任务：input、menu、buzzer、tune（start_ms = now_ms）
 *          - 延时启动的任务：gyro、line、control（start_ms = now_ms + 10ms）
 *            传感器融合延迟更大（start_ms = now_ms + 50ms）
 *          - 目的是分散系统启动初期的瞬时负载
 */
void ec_app_init(void)
{
    uint32_t now_ms;

    ec_scheduler_init(&g_scheduler);                /* 1. 初始化调度器 */
    ec_time_init();                                 /* 2. 初始化系统时间 */

    /* 3. 根据应用配置初始化应用层 */
#if EC_APP_PROFILE == EC_APP_PROFILE_HARDWARE_TEST  /* 硬件测试模式 */
    printf("[APP] profile=hardware_test\r\n");
    hardware_test_init();
#elif EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR     /* 巡线车模式 */
    line_car_init();

    {
        ec_mode_manager_t *mgr = line_car_get_mode_manager();
        h2024_tasks_register(mgr);
        e2025_tasks_register(mgr);
    }
#else                                               /* 空应用模式（默认） */
    printf("[APP] profile=empty\r\n");
#endif

    /* 4. 注册周期性任务到调度器 */
#if EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR
    now_ms = ec_time_ms();
    /* 输入采集任务 5ms周期 - 最高优先级，读取遥控/按键输入 */
    ec_app_add_task("input", line_car_input_task, 5u, now_ms);
    /* 陀螺仪读取任务 10ms周期 - 稍后启动，给input任务先执行的机会 */
    ec_app_add_task("gyro", line_car_gyro_task, 10u, now_ms + 10u);
    /* 巡线传感器读取任务 10ms周期 */
    ec_app_add_task("line", line_car_line_sensor_task, 10u, now_ms + 10u);
    /* 传感器数据融合任务 10ms周期 - 与控制任务同频，保证RPM实时性 */
    ec_app_add_task("sensor", line_car_sensor_task, 10u, now_ms + 10u);
    /* PID控制计算任务 10ms周期 - 控制环路核心 */
    ec_app_add_task("control", line_car_control_task, 10u, now_ms + 10u);
    /* 菜单系统任务 200ms周期 - 人机交互 */
    ec_app_add_task("menu", line_car_menu_task, 200u, now_ms);
    /* 蜂鸣器驱动任务 5ms周期 - 产生PWM音频信号需要高频率 */
    ec_app_add_task("buzzer", line_car_buzzer_task, 5u, now_ms);
    /* OLED显示刷新任务 20ms周期 (50Hz刷新率) */
    ec_app_add_task("oled", line_car_oled_task, 20u, now_ms);
    /* 遥测数据发送任务 20ms周期 - 无线模块数据上报 */
    ec_app_add_task("telemetry", line_car_telemetry_task, 20u, now_ms + 20u);
    /* 调试信息输出任务 100ms周期 - 串口打印 */
    ec_app_add_task("debug", line_car_debug_task, 100u, now_ms + 100u);
    /* PID调参任务 5ms周期 - 运行时参数调整 */
    ec_app_add_task("tune", line_car_tune_task, 5u, now_ms);
#else
    now_ms = ec_time_ms();
    /* 非巡线车模式下只有一个通用任务，运行频率为 1ms (1000Hz) */
    ec_app_add_task("app", ec_app_task, 1u, now_ms);
#endif
}

/**
 * @brief 单步运行应用程序——在主循环中被反复调用
 * @details 每次调用完成以下步骤：
 *          1. ec_scheduler_run() - 运行调度器，执行所有到期的任务
 *          2. __WFI() - 等待中断指令，CPU进入休眠直到下一个中断唤醒
 *
 *          【关于 __WFI() 的设计思考】
 *          在无 RTOS 的裸机系统中，主循环通常是一个 while(1) {} 空转。
 *          使用 __WFI() 可以：
 *          - 降低功耗（智能车电池供电时尤为重要）
 *          - 减少电磁干扰（CPU停止取指执行）
 *          - 精确同步到系统定时器中断
 *
 *          定时器每 1ms 产生一次中断，唤醒 CPU，执行调度器，然后再次休眠。
 *          这样系统以 1ms 为"心跳"周期性工作。
 */
void ec_app_run(void)
{
    ec_scheduler_run(&g_scheduler, ec_time_ms());
    __WFI();    /* 等待中断，降低功耗 */
}

/**
 * @brief 紧急停止接口
 * @details 在检测到危险情况（如传感器数据异常、通信超时等）时调用此函数。
 *          不同模式下有不同的紧急停止行为：
 *          - 硬件测试：调用硬件测试模块的急停函数，可能返回需要进行额外处理
 *          - 巡线车：调用巡线车模块的急停函数（如立即停止电机、置位安全状态）
 * @return 0=紧急停止已处理，非0=需要调用者进一步处理
 */
int ec_app_emergency_stop(void)
{
#if EC_APP_PROFILE == EC_APP_PROFILE_HARDWARE_TEST
    return (int)hardware_test_emergency_stop();
#elif EC_APP_PROFILE == EC_APP_PROFILE_LINE_CAR
    line_car_emergency_stop();
    return 0;
#else
    return 0;
#endif
}

/**
 * @brief 获取全局调度器的常量指针
 * @details 返回 const 指针，外部代码只能读取调度器的状态（如任务统计信息）
 *          而不能修改调度器的内部状态。这是一种"只读访问"设计模式，
 *          常用于提供调试接口和状态监控。
 *
 * @return 指向 g_scheduler 的 const 指针
 */
const ec_scheduler_t *ec_app_get_scheduler(void)
{
    return &g_scheduler;
}
