/**
 * @file dt_hc05.h
 * @brief HC-05 蓝牙模块驱动头文件
 *        支持AT指令自动配置模块名称、配对密码和通信波特率，
 *        主要用于竞速小车与上位机之间的无线数据通信。
 */

#ifndef _DT_HC05_H_
#define _DT_HC05_H_

#include "zf_common_headfile.h"

#define HC05_NAME    "MSPM0_Car"   /**< HC-05 蓝牙名称，手机/电脑搜索时显示 */
#define HC05_PIN     "1234"         /**< HC-05 配对密码 */
#define HC05_BAUD    115200         /**< 蓝牙模块完成配置后的工作波特率 */

/**
 * @brief HC-05 模块状态枚举
 */
typedef enum
{
    DT_HC05_STATUS_DISABLED = 0,        /**< 模块被禁用（EC_ENABLE_HC05未定义） */
    DT_HC05_STATUS_IDLE,                /**< 空闲状态，未开始配置 */
    DT_HC05_STATUS_BUSY,                /**< 正在执行AT指令配置流程 */
    DT_HC05_STATUS_READY,               /**< 配置完成，模块已就绪 */
    DT_HC05_STATUS_ERROR_RESPONSE,      /**< AT指令配置过程中收到ERROR响应 */
    DT_HC05_STATUS_ERROR_TIMEOUT        /**< AT指令配置过程中超时 */
} dt_hc05_status_t;

/**
 * @brief 启动HC-05配置流程（非阻塞，立即返回）
 *        上电EN引脚开始AT指令配置，配置完成后自动切换到工作模式。
 * @param en_pin HC-05 EN使能引脚（用于控制AT/工作模式切换）
 * @param now_ms 当前系统时间戳（毫秒），需与后续update调用使用同一时间基准
 * @return true=成功启动配置流程，false=模块忙或已就绪
 */
bool dt_hc05_begin(gpio_pin_enum en_pin, uint32_t now_ms);

/**
 * @brief HC-05状态机更新函数（需在主循环中周期性调用）
 *        驱动AT指令配置流程的各个阶段切换
 * @param now_ms 当前系统时间戳（毫秒），需与begin调用使用同一时间基准
 */
void dt_hc05_update(uint32_t now_ms);

/**
 * @brief 获取HC-05模块当前状态
 * @return 当前状态枚举值
 */
dt_hc05_status_t dt_hc05_get_status(void);

/**
 * @brief （兼容接口）简化初始化，从时间戳0开始配置
 *        调用者需周期性调用 dt_hc05_update() 驱动配置完成
 * @param en_pin HC-05 EN使能引脚
 */
void dt_hc05_init(gpio_pin_enum en_pin);

#endif
