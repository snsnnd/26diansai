/**
 * @file    gimbal_transport_zf.c
 * @brief   云台 EMM 传输层 —— 基于逐飞库 UART 的半双工适配器实现
 *
 * 本文件实现了 EmmTransport 接口在逐飞 MCU 库上的移植层。
 * 核心设计要点：
 *
 * 1. 半双工 UART：EMM 总线为半双工通信，发送时需将 TX 引脚配置为输出，
 *    发送完毕后切回输入（高阻），以便同一引脚上接收回显或释放总线。
 *
 * 2. TX 忙等待：发送前等待 UART 空闲，发送后再次等待发送完成，
 *    确保最后一个字节从移位寄存器发出后才切换 TX 引脚方向。
 *
 * 3. 中断驱动 RX：UART 接收中断将字节推入环形缓冲区（SerialRxBuffer），
 *    读取接口通过轮询缓冲区实现，支持超时机制。
 *
 * 4. 共享传输实例：偏航（Yaw）和俯仰（Pitch）两个电机共用一个 UART 总线，
 *    通过协议层地址区分，传输层自身不感知电机地址。
 */
#include "gimbal/gimbal_transport_zf.h"

#include "lib/serial_rx_buffer.h"

/**
 * @def GIMBAL_UART_READ_POLL_MS
 * @brief 读适配器轮询间隔（毫秒），每轮询一次等待 1ms
 */
#define GIMBAL_UART_READ_POLL_MS 1u

/**
 * @def GIMBAL_UART_TX_BUSY_TIMEOUT_LOOPS
 * @brief 等待 UART 发送空闲的最大忙等循环次数
 *
 * 这是一个软件超时保护，防止 UART 硬件异常时死循环。
 * 具体超时时间取决于 CPU 主频和循环指令周期。
 */
#define GIMBAL_UART_TX_BUSY_TIMEOUT_LOOPS 200000u

/**
 * @def EMM_TX_GPIO_PIN
 * @brief 从 UART TX 引脚定义中提取 GPIO 引脚编号
 *
 * 逐飞库的 UART 引脚定义（如 UART_TX_P13_6）包含端口和引脚信息，
 * 使用 UART_PIN_INDEX_MASK 掩码提取纯引脚索引，用于 GPIO 模式切换。
 */
#define EMM_TX_GPIO_PIN ((gpio_pin_enum)(GIMBAL_EMM_UART_TX_PIN & UART_PIN_INDEX_MASK))

/**
 * @struct GimbalUartContext
 * @brief  云台 UART 上下文结构体
 *
 * 在 EmmTransport 的 user_data 中传递，使适配器函数能访问 UART 索引。
 */
typedef struct
{
    uart_index_enum uart;   /**< 逐飞库 UART 索引号 */
} GimbalUartContext;

/** 全局 EMM UART 上下文实例，初始化为 GIMBAL_EMM_UART */
static GimbalUartContext EmmUartContext = { GIMBAL_EMM_UART };

/**
 * @brief EMM UART 接收数据存储区
 *
 * 环形缓冲区的底层数组，容量 128 字节。
 * 对于 EMM 协议（短命令/响应包）而言足够。
 * 若改为堆分配可减少全局变量，但嵌入式场景下静态分配更可靠。
 */
static uint8_t EmmRxStorage[128];

/**
 * @brief EMM UART 串行接收环形缓冲区实例
 *
 * 中断回调将硬件 UART 接收到的字节推入此缓冲区，
 * 读适配器从其中弹出字节，实现中断与轮询的解耦。
 */
static SerialRxBuffer EmmRxBuffer;

/**
 * @brief   根据 UART 索引获取对应的寄存器基地址
 *
 * 逐飞库中 UART 外设通过 UART0~UART3 宏访问寄存器，
 * 此函数将枚举索引映射到实际的寄存器指针。
 *
 * @param   uart  UART 索引枚举值
 * @return  指向 UART_Regs 结构体的指针，无效索引返回 NULL
 */
static UART_Regs *emm_uart_regs(uart_index_enum uart)
{
    switch (uart)
    {
        case UART_0: return UART0;
        case UART_1: return UART1;
        case UART_2: return UART2;
        case UART_3: return UART3;
        default: return NULL;
    }
}

/**
 * @brief   等待 UART 发送器空闲（忙等待）
 *
 * 使用 DL_UART_isBusy() 轮询 UART 忙状态寄存器。
 * 带软件超时保护，防止硬件异常时无限循环。
 * 在 TX 引脚切换方向前必须调用此函数，
 * 确保最后一个字节的移位输出已完成。
 *
 * @param   uart  UART 寄存器基地址指针
 * @retval  true   UART 已空闲或超时后仍返回 true（注：此处超时也返回 true）
 * @retval  false  uart 指针为 NULL 时返回 false
 */
static bool emm_uart_wait_idle(UART_Regs *uart)
{
    uint32_t remaining = GIMBAL_UART_TX_BUSY_TIMEOUT_LOOPS;

    if (uart == NULL)
    {
        return false;
    }
    while (DL_UART_isBusy(uart))
    {
        if (remaining-- == 0u)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief   将 TX 引脚切换为输入模式（高阻浮空）
 *
 * 半双工通信的关键操作：
 * - 发送完成后必须将 TX 引脚切回输入，释放总线
 * - 高阻浮空模式（GPI_FLOATING_IN）让引脚不驱动电平
 * - 初始电平设为 LOW，但在输入模式下无效
 */
static void emm_tx_set_input(void)
{
    gpio_init(EMM_TX_GPIO_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
}

/**
 * @brief   将 TX 引脚切换为输出模式（复用推挽）
 *
 * 半双工通信的关键操作：
 * - 发送前将 TX 引脚配置为复用功能输出
 * - 使用 GPIO_AF2（复用功能 2）对应 UART TX 功能
 * - 推挽输出（GPO_AF_PUSH_PULL）驱动总线
 */
static void emm_tx_set_output(void)
{
    afio_init(EMM_TX_GPIO_PIN, GPO, GPIO_AF2, GPO_AF_PUSH_PULL);
}

/**
 * @brief   EMM UART 接收中断回调函数
 *
 * 在 UART RX 中断触发时被逐飞库调用。
 * 从硬件 UART 接收寄存器中不断读取可用字节（使用 uart_query_byte），
 * 并将每个字节推入环形缓冲区 EmmRxBuffer。
 *
 * 为什么这里用 while 循环而非单次读取？
 * 考虑到逐飞库的中断机制可能在一次调用中传递多个 RX 事件标志，
 * 循环读取可以一次性取完所有已到达的字节，减少中断上下文切换开销。
 *
 * @param   state  中断状态位掩码（来自 UART 中断控制器）
 * @param   ptr    用户参数指针，实际指向 GimbalUartContext 实例
 */
static void emm_uart_rx_callback(uint32 state, void *ptr)
{
    GimbalUartContext *context = (GimbalUartContext *)ptr;
    uint8 byte;

    if (context == NULL || (state & UART_INTERRUPT_STATE_RX) == 0u)
    {
        return;
    }

    while (uart_query_byte(context->uart, &byte) == ZF_TRUE)
    {
        (void)serial_rx_buffer_push(&EmmRxBuffer, byte);
    }
}

/**
 * @brief   毫秒级延时适配函数
 *
 * EmmTransport 接口要求的延时回调。
 * 封装 system_delay_ms()，供 EMM 协议层在重试间隔等场景使用。
 *
 * @param   delay_ms_value  延时毫秒数
 * @param   user_data       EmmTransport user_data（本实现中未使用）
 */
static void delay_ms(uint32_t delay_ms_value, void *user_data)
{
    (void)user_data;
    system_delay_ms(delay_ms_value);
}

/**
 * @brief   UART 写适配函数 —— 发送数据到 EMM 总线
 *
 * 实现 EmmTransport 的 write 回调。发送流程：
 * 1. 参数校验（context/data/length 非空）
 * 2. 将 TX 引脚切换为输出模式（afio_init 复用功能推挽）
 * 3. 逐字节发送：每字节前等待 UART 空闲（emm_uart_wait_idle），
 *    超时则提前返回已发送字节数
 * 4. 所有字节发送完毕后，再次等待发送完成（确保移位寄存器输出完毕）
 * 5. 将 TX 引脚切回输入模式（高阻浮空），释放总线
 *
 * 设计要点：
 * - 半双工总线上，发送完成后立即释放 TX 引脚至关重要，
 *   否则接收回显或从机应答时引脚会持续驱动电平，造成冲突。
 * - TX_BUSY_TIMEOUT_LOOPS 超时机制防止 UART 硬件故障时卡死。
 *
 * @param   data      待发送数据缓冲区
 * @param   length    待发送数据长度（字节）
 * @param   user_data  指向 GimbalUartContext 的指针
 * @return  实际发送的字节数，失败返回 0
 */
static size_t zf_uart_write_adapter(const uint8_t *data, size_t length, void *user_data)
{
    GimbalUartContext *context = (GimbalUartContext *)user_data;
    UART_Regs *uart;
    size_t written = 0u;

    if (context == NULL || data == NULL || length == 0u)
    {
        return 0u;
    }

    uart = emm_uart_regs(context->uart);
    if (uart == NULL)
    {
        return 0u;
    }

    emm_tx_set_output();
    for (written = 0u; written < length; ++written)
    {
        if (!emm_uart_wait_idle(uart))
        {
            emm_tx_set_input();
            return written;
        }
        DL_UART_Main_transmitData(uart, data[written]);
    }

    if (!emm_uart_wait_idle(uart))
    {
        emm_tx_set_input();
        return 0u;
    }

    emm_tx_set_input();
    return length;
}

/**
 * @brief   UART 读适配函数 —— 从环形缓冲区读取数据（带超时）
 *
 * 实现 EmmTransport 的 read 回调。从串行接收缓冲区中弹出指定数量的字节。
 * 如果缓冲区中数据不足，则轮询等待直到超时。
 *
 * 为什么使用轮询而非中断/信号量？
 * 本函数被 EMM 协议层同步调用，调用者期望在返回时获得完整或部分数据。
 * 逐飞库为裸机环境，没有阻塞信号量，故采用轮询 + system_delay_ms 让步。
 *
 * @param[out]  data        读取缓冲区
 * @param       length      期望读取的字节数
 * @param       timeout_ms  超时时间（毫秒），0 表示最小超时（1ms）
 * @param       user_data   EmmTransport user_data（本实现中未使用）
 * @return  实际读取到的字节数（可能小于 length，当超时发生）
 */
static size_t zf_uart_read_adapter(uint8_t *data, size_t length,
    uint32_t timeout_ms, void *user_data)
{
    size_t count = 0u;
    uint32_t elapsed_ms = 0u;

    (void)user_data;
    if (data == NULL)
    {
        return 0u;
    }

    while (count < length)
    {
        if (serial_rx_buffer_pop(&EmmRxBuffer, &data[count]))
        {
            count++;
            continue;
        }
        if (elapsed_ms >= (timeout_ms == 0u ? 1u : timeout_ms))
        {
            break;
        }
        system_delay_ms(GIMBAL_UART_READ_POLL_MS);
        elapsed_ms += GIMBAL_UART_READ_POLL_MS;
    }
    return count;
}

/**
 * @brief   清空输入缓冲区适配函数
 *
 * 实现 EmmTransport 的 flush_input 回调。
 * 同时清除软件环形缓冲区和 UART 硬件 RX 寄存器中的残留字节。
 * 在读操作前调用，确保不会读取到上一帧的残留数据。
 *
 * @param   user_data  指向 GimbalUartContext 的指针
 */
static void zf_uart_flush_input_adapter(void *user_data)
{
    GimbalUartContext *context = (GimbalUartContext *)user_data;
    uint8 byte;

    serial_rx_buffer_clear(&EmmRxBuffer);
    if (context == NULL)
    {
        return;
    }
    while (uart_query_byte(context->uart, &byte) == ZF_TRUE)
    {
    }
}

/**
 * @brief   清空输出缓冲区适配函数
 *
 * 实现 EmmTransport 的 flush_output 回调。
 * EMM 发送为同步阻塞方式，没有软件输出缓冲区需要清空，故为空操作。
 *
 * @param   user_data  EmmTransport user_data（未使用）
 */
static void zf_uart_flush_output_adapter(void *user_data)
{
    (void)user_data;
}

/**
 * @brief   初始化云台 EMM 传输层（基于逐飞库 UART）
 *
 * 初始化流程：
 * 1. 调用 uart_init() 配置 UART 外设（波特率由 EMM_STEPPER_DEFAULT_BAUDRATE 定义，
 *    TX/RX 引脚由 pin_mapping 中的宏指定）。
 * 2. 初始化串行接收环形缓冲区（128 字节存储）。
 * 3. 注册中断回调 emm_uart_rx_callback 并使能 RX 中断。
 * 4. 将 TX 引脚初始置为输入模式（高阻），确保上电后总线释放。
 * 5. 创建 EmmTransport 适配器实例，填充所有回调函数指针，
 *    将 EmmUartContext 作为 user_data 传递。
 * 6. 使用同一个 transport 实例依次初始化偏航（Yaw）和俯仰（Pitch）电机：
 *    - 每个电机通过 emm_init() 获得自己的 EMM 协议层实例
 *    - 地址不同（GIMBAL_YAW_MOTOR_ADDRESS / GIMBAL_PITCH_MOTOR_ADDRESS）
 *    - 共享同一物理 UART 总线，由 EMM 协议层的地址字节区分
 *    - timeout_ms = 80ms，retry_delay_ms = 5ms
 *    - auto_flush_before_read = true（每次读取前自动清空缓冲区）
 *
 * @param   gimbal  指向 Gimbal 结构体的指针
 * @retval  GIMBAL_OK   初始化成功
 * @retval  GIMBAL_ERROR 参数为空指针
 */
GimbalStatus gimbal_transport_zf_init(Gimbal *gimbal)
{
    EmmTransport transport;

    if (gimbal == NULL)
    {
        return GIMBAL_ERROR;
    }

    uart_init(GIMBAL_EMM_UART, EMM_STEPPER_DEFAULT_BAUDRATE,
        GIMBAL_EMM_UART_TX_PIN, GIMBAL_EMM_UART_RX_PIN);
    serial_rx_buffer_init(&EmmRxBuffer, EmmRxStorage, sizeof(EmmRxStorage));
    uart_set_callback(GIMBAL_EMM_UART, emm_uart_rx_callback, &EmmUartContext);
    uart_set_interrupt_config(GIMBAL_EMM_UART, UART_INTERRUPT_CONFIG_RX_ENABLE);
    emm_tx_set_input();

    transport.write = zf_uart_write_adapter;
    transport.read = zf_uart_read_adapter;
    transport.flush_input = zf_uart_flush_input_adapter;
    transport.flush_output = zf_uart_flush_output_adapter;
    transport.delay_ms = delay_ms;
    transport.user_data = &EmmUartContext;

    emm_init(&gimbal->yaw, &transport, GIMBAL_YAW_MOTOR_ADDRESS);
    gimbal->yaw.timeout_ms = 80u;
    gimbal->yaw.retry_delay_ms = 5u;
    gimbal->yaw.auto_flush_before_read = true;

    emm_init(&gimbal->pitch, &transport, GIMBAL_PITCH_MOTOR_ADDRESS);
    gimbal->pitch.timeout_ms = 80u;
    gimbal->pitch.retry_delay_ms = 5u;
    gimbal->pitch.auto_flush_before_read = true;
    return GIMBAL_OK;
}

/**
 * @brief   调试探测函数 —— 发送 EMM 探测指令并打印响应
 *
 * 本函数用于硬件调试，验证 UART 通路是否正常。
 * 流程：
 * 1. 清空接收缓冲区
 * 2. 发送一帧预定义的 EMM 探测命令（地址 0x01，功能码 0xF3 ...）
 * 3. 在 500ms 内轮询接收缓冲区，打印所有收到的字节（十六进制）
 * 4. 同时打印环形缓冲区溢出计数
 *
 * 探测命令 01 F3 AB 01 00 6B 的含义：
 * - 0x01: 电机地址
 * - 0xF3: 特定功能码（厂商自定义）
 * - 后续: 命令参数和校验
 *
 * @note  此函数仅在调试阶段调用，生产代码中应移除或条件编译屏蔽。
 */
void gimbal_debug_probe_emm_uart(void)
{
    const uint8_t command[] = { 0x01u, 0xF3u, 0xABu, 0x01u, 0x00u, 0x6Bu };
    uint8_t byte;
    uint8_t rx_count = 0u;

    printf("[GIMBAL] UART%u probe tx: 01 F3 AB 01 00 6B\r\n",
           (unsigned int)EmmUartContext.uart);
    zf_uart_flush_input_adapter(&EmmUartContext);
    if (zf_uart_write_adapter(command, sizeof(command), &EmmUartContext) != sizeof(command))
    {
        printf("[GIMBAL] probe TX timeout\r\n");
        return;
    }

    printf("[GIMBAL] UART2 irq rx:");
    for (uint32_t elapsed_ms = 0u; elapsed_ms < 500u; elapsed_ms++)
    {
        while (serial_rx_buffer_pop(&EmmRxBuffer, &byte))
        {
            printf(" %X", byte);
            rx_count++;
        }
        system_delay_ms(1u);
    }

    if (rx_count == 0u)
    {
        printf(" none");
    }
    printf(" overflow=%lu\r\n",
        (unsigned long)serial_rx_buffer_overflow_count(&EmmRxBuffer));
}
