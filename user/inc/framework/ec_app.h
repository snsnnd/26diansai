/**
 * @file    ec_app.h
 * @brief   应用程序入口与主循环管理层
 * @details 本模块是整个智能车系统的"顶层胶水代码"，负责：
 *          1. 系统初始化：初始化调度器、时间系统、以及根据编译配置(EC_APP_PROFILE)
 *             选择加载不同的应用程序（硬件测试 / 巡线车 / 空应用）
 *          2. 注册任务：根据不同应用场景向调度器注册相应的周期性任务
 *          3. 主循环驱动：提供 ec_app_run() 供主循环调用，驱动调度器执行
 *          4. 紧急停止：提供统一的紧急停止接口
 *
 *          【应用配置模式】
 *          通过编译时宏 EC_APP_PROFILE 选择应用场景：
 *          - EC_APP_PROFILE_HARDWARE_TEST：硬件测试模式，用于调试各传感器/执行器
 *          - EC_APP_PROFILE_LINE_CAR：巡线小车模式，完整的智能车应用
 *          - 其他值：空应用模式，仅运行一个空循环任务
 *
 *          【设计模式】
 *          本模块实现了"外观模式(Facade Pattern)"——对外提供简洁的接口
 *          (init/run/emergency_stop)，内部封装了调度器、时间系统、应用层
 *          的复杂初始化逻辑。
 */
#ifndef EC_APP_H
#define EC_APP_H

#include "framework/ec_scheduler.h"

/**
 * @brief 初始化整个应用程序
 * @details 按顺序完成以下初始化：
 *          1. 初始化调度器 (ec_scheduler_init)
 *          2. 初始化时间系统 (ec_time_init)
 *          3. 根据 EC_APP_PROFILE 选择初始化硬件测试或巡线车应用
 *          4. 向调度器注册所有周期性任务
 * @note 该函数应在 main() 的一开始被调用，且只调用一次
 */
void ec_app_init(void);

/**
 * @brief 单步运行应用程序
 * @details 每次调用时执行调度器一次，然后进入 WFI(Wait For Interrupt)休眠
 *          等待下一次中断唤醒。这种方式在保证实时性的同时最大程度降低了功耗。
 *
 *          【WFI 节能策略】
 *          - 所有任务在 ec_scheduler_run 中执行完毕
 *          - __WFI() 让 CPU 进入休眠状态，直到下一个中断到来（如定时器中断）
 *          - 中断唤醒后回到主循环，再次调用 ec_app_run
 *          - 这样实现了"运行-休眠-唤醒-运行"的低功耗循环模式
 *
 * @note 该函数应在 main() 的超级循环中被反复调用
 */
void ec_app_run(void);

/**
 * @brief 紧急停止接口
 * @details 根据当前应用配置执行不同的紧急停止操作：
 *          - 硬件测试模式：调用 hardware_test_emergency_stop()
 *          - 巡线车模式：调用 line_car_emergency_stop()
 * @return 0=已处理紧急停止，非0=需要进一步处理
 */
int ec_app_emergency_stop(void);

/**
 * @brief 获取调度器实例指针
 * @return 指向全局调度器 g_scheduler 的常量指针
 * @note 返回值声明为 const，只允许外部读取调度器状态（如任务统计信息），
 *       不允许外部修改调度器内部数据，体现了最小权限原则
 */
const ec_scheduler_t *ec_app_get_scheduler(void);

#endif
