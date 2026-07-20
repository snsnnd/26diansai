#ifndef _HARDWARE_TEST_H_
#define _HARDWARE_TEST_H_

/**
 * @file hardware_test.h
 * @brief 硬件测试模块
 *
 * 提供对小车各子系统的独立测试功能，包括：
 * - 陀螺仪(UART接口)数据读取测试
 * - 云台(Yaw/Pitch)电机控制测试
 * - 视觉模块(MaixCam2)数据接收测试
 * 主要用于硬件调试和系统验证阶段。
 */

#include "zf_common_headfile.h"
#include "gimbal/vision_gimbal_control.h"
#include "gimbal/gimbal.h"

/**
 * 硬件测试专用的启动参考位置策略。
 *
 * 如果开启此宏，硬件测试启动时云台将直接使用已知的参考位置
 * (yaw=0, pitch=0)而不需要先找零位，加快启动速度。
 * 默认关闭，因为只有在校准好的情况下才能使用。
 */
#ifndef HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE
#define HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE 0  /* 默认不使用开机参考位置 */
#endif
#ifndef HW_TEST_GIMBAL_STARTUP_YAW_DEG
#define HW_TEST_GIMBAL_STARTUP_YAW_DEG 0.0f        /* 开机参考偏航角(度) */
#endif
#ifndef HW_TEST_GIMBAL_STARTUP_PITCH_DEG
#define HW_TEST_GIMBAL_STARTUP_PITCH_DEG 0.0f      /* 开机参考俯仰角(度) */
#endif

/** 编译时检查：开机参考位置使能标志必须为0或1 */
#if (HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE != 0) && \
    (HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE != 1)
#error "HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE must be 0 or 1"
#endif

/**
 * @brief 初始化硬件测试模块
 * 初始化UART、陀螺仪、云台电机和视觉模块的硬件接口
 */
void hardware_test_init(void);

/**
 * @brief 硬件测试主循环(需周期性调用)
 * 周期性更新陀螺仪、云台控制(cor)、视觉模块数据，并打印测试信息
 */
void hardware_test_run(void);

/**
 * @brief 设置云台控制模式
 * @param mode 控制模式(GIMBAL_CONTROL_POSITION位置控制 / GIMBAL_CONTROL_SPEED速度控制)
 */
void hardware_test_set_gimbal_control_mode(GimbalControlMode mode);

/**
 * @brief 获取当前云台控制模式
 * @return 当前控制模式
 */
GimbalControlMode hardware_test_get_gimbal_control_mode(void);

/**
 * @brief 执行紧急停止：停止云台电机并锁定安全状态
 * @return 操作结果状态
 */
GimbalStatus hardware_test_emergency_stop(void);

/**
 * @brief 重新准备云台(解除急停)
 * 需要满足：云台已初始化、已找零位、位置有效、按键未按下
 * @return 操作结果状态
 */
GimbalStatus hardware_test_rearm(void);

/**
 * @brief 检查云台是否已就绪(armed)
 * @return true 已就绪可运行；false 未就绪
 */
bool hardware_test_is_gimbal_armed(void);

/**
 * @brief 检查是否处于急停锁定状态
 * @return true 急停已锁定；false 未锁定
 */
bool hardware_test_is_emergency_latched(void);

#endif
