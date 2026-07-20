/**
 * @file dt_hc05.c
 * @brief HC-05 蓝牙模块自动配置驱动实现
 *        通过 AT 指令自动设置模块名称、配对密码和波特率。
 *        采用状态机方式实现非阻塞配置流程：
 *        拉高EN脚 -> 切换UART到AT波特率(38400) -> 发送AT指令序列 ->
 *        收到OK -> 拉低EN脚退出AT模式 -> 恢复UART到工作波特率。
 *        当 EC_ENABLE_HC05 未定义时，所有函数为空实现。
 */

#include "driver/dt_hc05.h"

#include "config.h"

#if EC_ENABLE_HC05

#include "lib/serial_rx_buffer.h"

/* ===== HC-05 配置时序参数 ===== */
#define HC05_AT_BAUD             38400u   /* AT指令模式使用的波特率（出厂默认） */
#define HC05_EN_SETTLE_MS          200u   /* EN引脚电平变化后的稳定等待时间 */
#define HC05_AT_STARTUP_MS         300u   /* AT模式启动后等待模块就绪的时间 */
#define HC05_RESPONSE_TIMEOUT_MS  1000u   /* 等待AT指令响应的超时时间 */
#define HC05_EXIT_SETTLE_MS        200u   /* 退出AT模式后的稳定等待时间 */
#define HC05_RX_BUFFER_SIZE         64u   /* AT指令响应接收缓冲区大小 */

/* ===== 内部状态机枚举 ===== */
typedef enum
{
    HC05_STATE_IDLE = 0,           /* 空闲 */
    HC05_STATE_WAIT_EN,            /* 等待EN引脚稳定 */
    HC05_STATE_WAIT_AT_STARTUP,    /* 等待AT模式启动完成 */
    HC05_STATE_WAIT_RESPONSE,      /* 等待AT指令响应 */
    HC05_STATE_WAIT_EXIT           /* 等待退出AT模式 */
} hc05_state_t;

/* ===== AT指令响应解析结果 ===== */
typedef enum
{
    HC05_RESPONSE_NONE = 0,        /* 尚未解析到完整响应 */
    HC05_RESPONSE_OK,              /* 收到 "OK" */
    HC05_RESPONSE_ERROR            /* 收到 "ERROR" */
} hc05_response_t;

/* ===== 内部全局状态 ===== */
static gpio_pin_enum g_hc05_en_pin;                    /* EN使能引脚 */
static hc05_state_t g_hc05_state = HC05_STATE_IDLE;    /* 当前状态 */
static dt_hc05_status_t g_hc05_status = DT_HC05_STATUS_IDLE; /* 对外报告的状态 */
static uint32_t g_hc05_deadline_ms;                    /* 当前状态的超时时间戳 */
static uint8_t g_hc05_command_index;                   /* 当前执行的AT指令索引 */
/* 响应字符串匹配状态机变量 */
static uint8_t g_hc05_ok_match;       /* "OK"字符串已匹配到的字符数 */
static uint8_t g_hc05_error_match;    /* "ERROR"字符串已匹配到的字符数 */
static uint8_t g_hc05_rx_storage[HC05_RX_BUFFER_SIZE]; /* 响应数据存储区 */
static SerialRxBuffer g_hc05_rx_buffer;                /* 响应数据环形缓冲区 */
/* 保存原来的UART回调，配置完成后恢复 */
static void_callback_uint32_ptr g_hc05_previous_callback;
static void *g_hc05_previous_callback_context;

/**
 * @brief 检查当前时间是否已到达或超过设定的截止时间
 *        使用有符号整数比较来处理 uint32_t 时间溢出的情况
 * @param now_ms 当前系统时间（毫秒）
 * @return true=已超时或正好到截止时间，false=未到截止时间
 */
static bool hc05_deadline_reached(uint32_t now_ms)
{
    return (int32_t)(now_ms - g_hc05_deadline_ms) >= 0;
}

/**
 * @brief HC-05 UART 接收中断回调函数（AT配置期间使用）
 *        将接收到的AT响应字节存入环形缓冲区供后续解析
 * @param state UART中断状态
 * @param context 用户上下文指针（未使用）
 */
static void hc05_uart_rx_callback(uint32_t state, void *context)
{
    uint8_t byte;

    (void)context;
    if ((state & UART_INTERRUPT_STATE_RX) == 0u)
    {
        return; /* 非接收中断，忽略 */
    }

    /* 读取所有接收到的字节并存入环形缓冲区 */
    while (uart_query_byte(DEBUG_UART_INDEX, &byte) == ZF_TRUE)
    {
        (void)serial_rx_buffer_push(&g_hc05_rx_buffer, byte);
    }
}

/**
 * @brief 复位AT指令响应匹配状态
 *        清空接收缓冲区和OK/ERROR字符串匹配进度
 */
static void hc05_reset_response(void)
{
    serial_rx_buffer_clear(&g_hc05_rx_buffer);
    g_hc05_ok_match = 0u;          /* "OK"匹配进度归零 */
    g_hc05_error_match = 0u;        /* "ERROR"匹配进度归零 */
}

/**
 * @brief 从缓冲区中读取AT指令响应数据，检测"OK"或"ERROR"字符串
 *        使用逐字符流式匹配算法，无需等待完整行结束即可判断结果
 * @return 响应解析结果
 *   - HC05_RESPONSE_NONE: 尚未识别到完整响应
 *   - HC05_RESPONSE_OK: 检测到"OK"
 *   - HC05_RESPONSE_ERROR: 检测到"ERROR"
 */
static hc05_response_t hc05_read_response(void)
{
    static const char ok_text[] = "OK";
    static const char error_text[] = "ERROR";
    hc05_response_t result = HC05_RESPONSE_NONE;
    uint8_t byte;

    while (serial_rx_buffer_pop(&g_hc05_rx_buffer, &byte))
    {
        /* 检测 "ERROR" 子串 */
        if (byte == (uint8_t)error_text[g_hc05_error_match])
        {
            ++g_hc05_error_match;            /* 匹配字符前进 */
            if (g_hc05_error_match == sizeof(error_text) - 1u)
            {
                return HC05_RESPONSE_ERROR;  /* 完全匹配"ERROR" */
            }
        }
        else
        {
            /* 当前字符不匹配，重置匹配进度（但检查是否以'E'开头重新匹配） */
            g_hc05_error_match = (byte == (uint8_t)error_text[0]) ? 1u : 0u;
        }

        /* 检测 "OK" 子串（与ERROR检测并行进行） */
        if (byte == (uint8_t)ok_text[g_hc05_ok_match])
        {
            ++g_hc05_ok_match;
            if (g_hc05_ok_match == sizeof(ok_text) - 1u)
            {
                result = HC05_RESPONSE_OK;
                g_hc05_ok_match = 0u;  /* 复位，继续检测后续可能的"OK" */
            }
        }
        else
        {
            g_hc05_ok_match = (byte == (uint8_t)ok_text[0]) ? 1u : 0u;
        }
    }

    return result;
}

/**
 * @brief 恢复UART配置到正常工作模式
 *        将UART重新初始化为HC05_BAUD指定的工作波特率，
 *        并恢复之前保存的UART回调函数
 */
static void hc05_restore_normal_uart(void)
{
    /* 使用工作波特率重新初始化UART */
    uart_init(DEBUG_UART_INDEX, HC05_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
    /* 恢复应用程序原来注册的UART回调 */
    uart_set_callback(DEBUG_UART_INDEX, g_hc05_previous_callback,
        g_hc05_previous_callback_context);
    uart_set_interrupt_config(DEBUG_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

/**
 * @brief 配置失败处理函数
 *        拉低EN引脚退出AT模式，恢复UART到工作模式，更新状态
 * @param status 失败状态（ERROR_RESPONSE 或 ERROR_TIMEOUT）
 */
static void hc05_fail(dt_hc05_status_t status)
{
    gpio_low(g_hc05_en_pin);          /* 拉低EN，退出AT模式 */
    hc05_restore_normal_uart();        /* 恢复UART */
    g_hc05_state = HC05_STATE_IDLE;   /* 状态机回到空闲 */
    g_hc05_status = status;            /* 记录错误状态 */
}

/**
 * @brief 发送当前索引对应的AT指令
 *        指令序列：AT -> AT+NAME=... -> AT+PSWD=... -> AT+UART=...
 *        发送后状态切换到 WAIT_RESPONSE 等待响应
 * @param now_ms 当前系统时间戳，用于计算响应超时时间
 */
static void hc05_send_current_command(uint32_t now_ms)
{
    char command[32];

    hc05_reset_response();                              /* 清空上次响应 */
    g_hc05_deadline_ms = now_ms + HC05_RESPONSE_TIMEOUT_MS; /* 设置超时 */
    g_hc05_state = HC05_STATE_WAIT_RESPONSE;

    /* 按索引发送对应的AT指令 */
    switch (g_hc05_command_index)
    {
        case 0u:
            uart_write_string(DEBUG_UART_INDEX, "AT\r\n");           /* 测试连接 */
            break;
        case 1u:
            uart_write_string(DEBUG_UART_INDEX, "AT+NAME=" HC05_NAME "\r\n"); /* 设置名称 */
            break;
        case 2u:
            uart_write_string(DEBUG_UART_INDEX, "AT+PSWD=" HC05_PIN "\r\n"); /* 设置密码 */
            break;
        default:
            /* 设置工作波特率：格式 AT+UART=<baud>,<stopbits>,<parity> */
            snprintf(command, sizeof(command), "AT+UART=%u,0,0\r\n",
                (unsigned int)HC05_BAUD);
            uart_write_string(DEBUG_UART_INDEX, command);
            break;
    }
}

/**
 * @brief 启动HC-05 AT指令配置流程
 *        保存当前UART回调，初始化EN引脚（拉高进入AT模式），
 *        设置状态机初始状态为等待EN稳定。
 * @param en_pin HC-05 EN使能引脚
 * @param now_ms 当前系统时间戳（毫秒）
 * @return true=成功启动，false=模块忙（正在配置中）
 */
bool dt_hc05_begin(gpio_pin_enum en_pin, uint32_t now_ms)
{
    if (g_hc05_status == DT_HC05_STATUS_BUSY)
    {
        return false; /* 已有配置流程进行中 */
    }

    g_hc05_en_pin = en_pin;
    /* 保存应用程序注册的UART回调，配置完成后恢复 */
    g_hc05_previous_callback = uart_callback_list[DEBUG_UART_INDEX];
    g_hc05_previous_callback_context = uart_callback_ptr_list[DEBUG_UART_INDEX];
    /* 初始化AT响应接收环形缓冲区 */
    serial_rx_buffer_init(&g_hc05_rx_buffer, g_hc05_rx_storage,
        sizeof(g_hc05_rx_storage));

    /* 初始化EN引脚为推挽输出，拉高使HC-05进入AT指令模式 */
    gpio_init(en_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_high(en_pin);
    g_hc05_command_index = 0u;                             /* 从第一条指令开始 */
    g_hc05_deadline_ms = now_ms + HC05_EN_SETTLE_MS;      /* 设置EN稳定等待 */
    g_hc05_state = HC05_STATE_WAIT_EN;
    g_hc05_status = DT_HC05_STATUS_BUSY;                   /* 标记配置进行中 */
    return true;
}

/**
 * @brief HC-05配置状态机更新函数（需在主循环中周期性调用）
 *        驱动整个AT指令配置流程：
 *        WAIT_EN -> WAIT_AT_STARTUP -> WAIT_RESPONSE(循环) -> WAIT_EXIT -> READY
 * @param now_ms 当前系统时间戳（毫秒）
 * @note 非阻塞设计，每次update只做有限工作，
 *       所有延时通过时间戳比较实现，不阻塞CPU
 */
void dt_hc05_update(uint32_t now_ms)
{
    hc05_response_t response;

    /* 非BUSY状态不需要处理 */
    if (g_hc05_status != DT_HC05_STATUS_BUSY)
    {
        return;
    }

    switch (g_hc05_state)
    {
        case HC05_STATE_WAIT_EN:
            /* 等待EN引脚电平稳定后切换到AT模式UART */
            if (hc05_deadline_reached(now_ms))
            {
                /* 切换到AT指令波特率并注册中断回调 */
                uart_init(DEBUG_UART_INDEX, HC05_AT_BAUD,
                    DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
                uart_set_callback(DEBUG_UART_INDEX, hc05_uart_rx_callback, NULL);
                uart_set_interrupt_config(DEBUG_UART_INDEX,
                    UART_INTERRUPT_CONFIG_RX_ENABLE);
                hc05_reset_response();
                g_hc05_deadline_ms = now_ms + HC05_AT_STARTUP_MS;
                g_hc05_state = HC05_STATE_WAIT_AT_STARTUP;
            }
            break;

        case HC05_STATE_WAIT_AT_STARTUP:
            /* 等待AT模式启动完成后发送第一条指令 */
            if (hc05_deadline_reached(now_ms))
            {
                hc05_send_current_command(now_ms);
            }
            break;

        case HC05_STATE_WAIT_RESPONSE:
            /* 等待AT指令响应 */
            response = hc05_read_response();
            if (response == HC05_RESPONSE_ERROR)
            {
                hc05_fail(DT_HC05_STATUS_ERROR_RESPONSE); /* AT指令返回ERROR */
            }
            else if (response == HC05_RESPONSE_OK)
            {
                if (g_hc05_command_index < 3u)
                {
                    /* 还有指令未发送，继续下一条 */
                    ++g_hc05_command_index;
                    hc05_send_current_command(now_ms);
                }
                else
                {
                    /* 所有AT指令已完成，拉低EN退出AT模式 */
                    gpio_low(g_hc05_en_pin);
                    g_hc05_deadline_ms = now_ms + HC05_EXIT_SETTLE_MS;
                    g_hc05_state = HC05_STATE_WAIT_EXIT;
                }
            }
            else if (hc05_deadline_reached(now_ms))
            {
                hc05_fail(DT_HC05_STATUS_ERROR_TIMEOUT); /* 响应超时 */
            }
            break;

        case HC05_STATE_WAIT_EXIT:
            /* 等待退出AT模式后恢复UART */
            if (hc05_deadline_reached(now_ms))
            {
                hc05_restore_normal_uart();
                g_hc05_state = HC05_STATE_IDLE;
                g_hc05_status = DT_HC05_STATUS_READY; /* 配置完成！ */
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 获取HC-05模块当前状态
 * @return 状态枚举值
 */
dt_hc05_status_t dt_hc05_get_status(void)
{
    return g_hc05_status;
}

/**
 * @brief 简化初始化接口（兼容旧代码）
 *        从时间戳0开始配置流程，调用者须周期性调用dt_hc05_update()
 * @param en_pin HC-05 EN使能引脚
 */
void dt_hc05_init(gpio_pin_enum en_pin)
{
    (void)dt_hc05_begin(en_pin, 0u);
}

#else /* EC_ENABLE_HC05 未定义——提供空实现 */

bool dt_hc05_begin(gpio_pin_enum en_pin, uint32_t now_ms)
{
    (void)en_pin;
    (void)now_ms;
    return false; /* 模块未启用，返回失败 */
}

void dt_hc05_update(uint32_t now_ms)
{
    (void)now_ms; /* 空操作 */
}

dt_hc05_status_t dt_hc05_get_status(void)
{
    return DT_HC05_STATUS_DISABLED; /* 返回禁用状态 */
}

void dt_hc05_init(gpio_pin_enum en_pin)
{
    (void)en_pin; /* 空操作 */
}

#endif /* EC_ENABLE_HC05 */
