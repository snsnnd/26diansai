#include "gimbal/emm_stepper.h"

#include <float.h>
#include <limits.h>

/*
 * ============================================================================
 * EMM 步进电机串口通信协议实现
 *
 * 本文件实现了与 EMM 系列步进电机驱动器的串口通信协议。EMM 协议采用
 * 半双工 UART 通信，数据帧格式如下：
 *
 *   固定长度帧: [地址(1)] [功能码(1)] [数据(N)] [校验和(1)]  总长 = N+3
 *   可变长度帧: [地址(1)] [功能码(1)] [总长度(1)] [数据(M)] [校验和(1)] 总长 >= 5
 *
 * 通信特点：
 *   - 半双工总线，发送数据会被自身 MCU 接收（产生"回显 echo"）
 *   - 支持三种校验模式：固定校验值(0x6B)、XOR 异或校验、CRC8 校验
 *   - 每个命令同时使用 CODE（功能码，标识操作类型）和 PROTOCOL（协议验证字节，
 *     用于帧格式校验和防误触发），两者共同构成命令的唯一标识
 * ============================================================================
 */

#define EMM_STATUS_FIXED_CHECKSUM 0x6Bu
#define EMM_STATUS_SUCCESS 0x02u
#define EMM_STATUS_PARAM_ERROR 0xE2u
#define EMM_STATUS_FORMAT_ERROR 0xEEu

/*
 * ============================================================================
 * EMM 协议命令码 (CODE) 定义
 *
 * CODE 是帧结构中的第二个字节，标识命令的类型（如查询位置、设置参数等）。
 * 每个命令对应的 CODE 值由 EMM 协议规范固定。
 * ============================================================================
 */

#define EMM_CODE_CAL_ENCODER 0x06u
#define EMM_CODE_RESTART 0x08u
#define EMM_CODE_ZERO_POSITION 0x0Au
#define EMM_CODE_CLEAR_PROTECTION 0x0Eu
#define EMM_CODE_FACTORY_RESET 0x0Fu
#define EMM_CODE_GET_VERSION 0x1Fu
#define EMM_CODE_GET_MOTOR_RH 0x20u
#define EMM_CODE_GET_PID 0x21u
#define EMM_CODE_GET_HOME_PARAM 0x22u
#define EMM_CODE_GET_BUS_VOLTAGE 0x24u
#define EMM_CODE_GET_BUS_CURRENT 0x26u
#define EMM_CODE_GET_PHASE_CURRENT 0x27u
#define EMM_CODE_GET_ENCODER 0x31u
#define EMM_CODE_GET_PULSE_COUNT 0x32u
#define EMM_CODE_GET_TARGET_POSITION 0x33u
#define EMM_CODE_GET_REALTIME_SPEED 0x35u
#define EMM_CODE_GET_REALTIME_POSITION 0x36u
#define EMM_CODE_GET_POSITION_ERROR 0x37u
#define EMM_CODE_GET_TEMPERATURE 0x39u
#define EMM_CODE_GET_MOTOR_STATUS 0x3Au
#define EMM_CODE_GET_HOME_STATUS 0x3Bu
#define EMM_CODE_GET_CONFIG 0x42u
#define EMM_CODE_GET_SYS_STATUS 0x43u
#define EMM_CODE_SET_OPEN_LOOP_CURRENT 0x44u
#define EMM_CODE_SET_CLOSED_LOOP_CURRENT 0x45u
#define EMM_CODE_SET_LOOP_MODE 0x46u
#define EMM_CODE_SET_CONFIG 0x48u
#define EMM_CODE_SET_PID 0x4Au
#define EMM_CODE_SET_HOME_PARAM 0x4Cu
#define EMM_CODE_SET_SCALE_INPUT 0x4Fu
#define EMM_CODE_SET_HEARTBEAT_TIME 0x68u
#define EMM_CODE_SET_MICROSTEP 0x84u
#define EMM_CODE_SET_HOME_ZERO 0x93u
#define EMM_CODE_HOME 0x9Au
#define EMM_CODE_STOP_HOME 0x9Cu
#define EMM_CODE_SET_ID 0xAEu
#define EMM_CODE_SET_LOCK_BUTTON 0xD0u
#define EMM_CODE_SET_POSITION_WINDOW 0xD1u
#define EMM_CODE_SET_MOTOR_DIRECTION 0xD4u
#define EMM_CODE_ENABLE 0xF3u
#define EMM_CODE_JOG 0xF6u
#define EMM_CODE_SET_AUTO_RUN 0xF7u
#define EMM_CODE_POSITION 0xFDu
#define EMM_CODE_ESTOP 0xFEu
#define EMM_CODE_SYNC_MOVE 0xFFu
#define EMM_CODE_BROADCAST_GET_ID 0x15u

/*
 * ============================================================================
 * EMM 协议验证字节 (PROTOCOL) 定义
 *
 * PROTOCOL 是部分命令帧中跟在 CODE 之后的固定验证字节。其作用是：
 * 1. 提供额外的帧格式验证，防止误触发
 * 2. 区分 CODE 相同但参数不同的变体命令（如回零与回零参数设置共用 0x22 范围）
 * 3. 作为"命令签名"，确保接收方正确解析后续参数
 *
 * 注意：CODE 和 PROTOCOL 共同构成了命令的唯一标识。CODE 标识操作类型，
 * PROTOCOL 则提供该操作类型下的特定子命令验证。
 * ============================================================================
 */

#define EMM_PROTOCOL_CAL_ENCODER 0x45u
#define EMM_PROTOCOL_RESTART 0x97u
#define EMM_PROTOCOL_ZERO_POSITION 0x6Du
#define EMM_PROTOCOL_CLEAR_PROTECTION 0x52u
#define EMM_PROTOCOL_FACTORY_RESET 0x5Fu
#define EMM_PROTOCOL_ENABLE 0xABu
#define EMM_PROTOCOL_ESTOP 0x98u
#define EMM_PROTOCOL_SYNC_MOVE 0x66u
#define EMM_PROTOCOL_SET_HOME_ZERO 0x88u
#define EMM_PROTOCOL_STOP_HOME 0x48u
#define EMM_PROTOCOL_SET_HOME_PARAM 0xAEu
#define EMM_PROTOCOL_GET_CONFIG 0x6Cu
#define EMM_PROTOCOL_GET_SYS_STATUS 0x7Au
#define EMM_PROTOCOL_SET_MICROSTEP 0x8Au
#define EMM_PROTOCOL_SET_ID 0x4Bu
#define EMM_PROTOCOL_SET_OPEN_LOOP_CURRENT 0x33u
#define EMM_PROTOCOL_SET_CLOSED_LOOP_CURRENT 0x66u
#define EMM_PROTOCOL_SET_LOOP_MODE 0xA6u
#define EMM_PROTOCOL_SET_CONFIG 0xD1u
#define EMM_PROTOCOL_SET_PID 0xC3u
#define EMM_PROTOCOL_SET_AUTO_RUN 0x1Cu
#define EMM_PROTOCOL_SET_SCALE_INPUT 0x71u
#define EMM_PROTOCOL_SET_MOTOR_DIRECTION 0x60u
#define EMM_PROTOCOL_SET_LOCK_BUTTON 0xB3u
#define EMM_PROTOCOL_SET_POSITION_WINDOW 0x07u
#define EMM_PROTOCOL_SET_HEARTBEAT_TIME 0x38u

/*
 * CRC8 查找表，生成多项式为 0x31 (x^8 + x^5 + x^4 + 1)
 *
 * 多项式 0x31 的二进制表示为 0011 0001，对应的代数式为：
 *   x^8 + x^5 + x^4 + 1
 * 这是 CRC-8/MAXIM (Dallas 1-Wire) 使用的标准多项式，在步进电机驱动
 * 通信中广泛使用以确保数据传输的完整性。
 *
 * 该表预先计算了所有 256 个字节值对应的 CRC 余数，将在线计算的时间
 * 复杂度从 O(n) 的逐位运算降低为 O(n) 的查表运算（但常数大幅减小），
 * 适用于 MSPM0G3507 这类资源受限的 MCU。
 */
static const uint8_t EmmCrc8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/*
 * 从大端序字节数组中读取 16 位无符号整数
 * data[0] 为高字节，data[1] 为低字节
 * 返回: 拼接后的 16 位值
 */
static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/*
 * 从大端序字节数组中读取 32 位无符号整数
 * data[0..3] 从高字节到低字节排列
 * 返回: 拼接后的 32 位值
 */
static uint32_t read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
}

/*
 * 读取带符号前缀的变长整数
 *
 * EMM 协议中，符号位以单独字节表示：
 *   data[0] == 1  -> 负数
 *   data[0] != 1  -> 正数
 *   data[1..length-1] 为绝对值（大端序）
 *
 * 参数:
 *   data   - 输入缓冲区，data[0]=符号, data[1..]=绝对值
 *   length - 总字节数（含符号字节）
 * 返回: 解码后的 int32_t 值
 */
static int32_t read_signed_prefix(const uint8_t *data, size_t length)
{
    int32_t sign = (data[0] == 1u) ? -1 : 1;
    uint32_t value = 0u;

    for (size_t i = 1u; i < length; ++i)
    {
        value = (value << 8) | data[i];
    }

    return sign * (int32_t)value;
}

uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode);

/*
 * 将 16 位无符号整数按大端序写入字节数组
 * data[0] = 高字节, data[1] = 低字节
 */
static void write_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

/*
 * 将 32 位无符号整数按大端序写入字节数组
 * data[0..3] 从高字节到低字节排列
 */
static void write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}


/*
 * ============================================================================
 * 环形缓冲区操作
 *
 * 接收缓冲区 (rx_buffer) 是一个环形缓冲区，由头指针 (rx_head) 和尾指针
 * (rx_tail) 管理。数据从 rx_head 写入，从 rx_tail 读取。当 rx_head == rx_tail
 * 时缓冲区为空。环形缓冲区大小由 EMM_STEPPER_RX_BUFFER_SIZE 定义。
 *
 * 使用环形缓冲区的优势：
 * 1. 无需动态内存分配，适合 MCU 环境
 * 2. 读写操作均为 O(1)
 * 3. 支持高效的 peek 操作（不消费数据即可查看）
 * ============================================================================
 */

/*
 * 计算环形缓冲区中下一个位置的索引
 * index - 当前位置
 * 返回: 下一个位置的索引（到达末尾时回绕到 0）
 */
static size_t emm_rx_next_index(size_t index)
{
    return (index + 1u) % EMM_STEPPER_RX_BUFFER_SIZE;
}

/*
 * 获取当前环形缓冲区中可读的字节数
 * device - EMM 设备句柄
 * 返回: 缓冲区中未读取的字节数
 */
size_t emm_rx_available(const EmmDevice *device)
{
    if (device == 0)
    {
        return 0u;
    }
    if (device->rx_head >= device->rx_tail)
    {
        return device->rx_head - device->rx_tail;
    }
    return EMM_STEPPER_RX_BUFFER_SIZE - device->rx_tail + device->rx_head;
}

/*
 * 获取接收缓冲区溢出的累积次数
 * 溢出发生在 rx_push 时缓冲区已满，旧数据被丢弃
 * device - EMM 设备句柄
 * 返回: 溢出计数
 */
size_t emm_rx_overflow_count(const EmmDevice *device)
{
    return (device == 0) ? 0u : device->rx_overflow_count;
}

/*
 * 清空接收缓冲区（重置头尾指针，数据未被实际擦除）
 * device - EMM 设备句柄
 */
void emm_rx_clear(EmmDevice *device)
{
    if (device == 0)
    {
        return;
    }
    device->rx_head = 0u;
    device->rx_tail = 0u;
}

/*
 * 向环形接收缓冲区推入一个字节
 * 当缓冲区满时，丢弃最旧的字节（覆盖尾部），以保留最新的数据。
 * 这是因为在高频轮询场景下，最新的字节更有可能完成当前正在解析的帧。
 *
 * device - EMM 设备句柄
 * byte   - 待推入的字节
 * 返回: 始终返回 true
 */
static bool emm_rx_push(EmmDevice *device, uint8_t byte)
{
    size_t next;
    if (device == 0)
    {
        return false;
    }

    next = emm_rx_next_index(device->rx_head);
    if (next == device->rx_tail)
    {
        /* 丢弃最旧的字节。在高频轮询中，新字节更有可能完成当前帧。 */
        device->rx_tail = emm_rx_next_index(device->rx_tail);
        device->rx_overflow_count++;
    }

    device->rx_buffer[device->rx_head] = byte;
    device->rx_head = next;
    return true;
}

/*
 * 从环形接收缓冲区中查看指定偏移处的字节（不消费数据）
 * 用于在不移除数据的情况下检查缓冲区内容，实现"预解析"。
 *
 * device - EMM 设备句柄
 * offset - 相对于尾指针的偏移量
 * byte   - 输出参数，存储读取到的字节
 * 返回: true 表示成功读取，false 表示偏移超出范围
 */
static bool emm_rx_peek(const EmmDevice *device, size_t offset, uint8_t *byte)
{
    size_t count;
    size_t index;

    if (device == 0 || byte == 0)
    {
        return false;
    }

    count = emm_rx_available(device);
    if (offset >= count)
    {
        return false;
    }

    index = (device->rx_tail + offset) % EMM_STEPPER_RX_BUFFER_SIZE;
    *byte = device->rx_buffer[index];
    return true;
}

/*
 * 从环形接收缓冲区前端丢弃指定长度的数据
 * 用于在成功解析一帧后，将该帧占用的字节从缓冲区移除。
 *
 * device - EMM 设备句柄
 * length - 要丢弃的字节数（超过可用数据时只丢弃全部可用数据）
 */
static void emm_rx_drop(EmmDevice *device, size_t length)
{
    size_t count;

    if (device == 0)
    {
        return;
    }

    count = emm_rx_available(device);
    if (length > count)
    {
        length = count;
    }

    device->rx_tail = (device->rx_tail + length) % EMM_STEPPER_RX_BUFFER_SIZE;
}

/*
 * 从环形接收缓冲区中复制指定范围的数据到外部缓冲区
 * 通过多次调用 peek 实现，不修改头尾指针。
 *
 * device - 源设备（环形缓冲区）
 * offset - 相对于尾指针的起始偏移
 * data   - 目标缓冲区
 * length - 要复制的字节数
 */
static void emm_rx_copy(const EmmDevice *device, size_t offset, uint8_t *data, size_t length)
{
    for (size_t i = 0u; i < length; ++i)
    {
        uint8_t value = 0u;
        (void)emm_rx_peek(device, offset + i, &value);
        data[i] = value;
    }
}

/*
 * 字节匹配函数，支持通配符
 * 当 expected == EMM_STEPPER_MATCH_ANY 时，匹配任意字节。
 * 用于帧解析时根据需要灵活匹配地址或功能码。
 *
 * actual   - 实际接收到的字节
 * expected - 期望的字节值（或 EMM_STEPPER_MATCH_ANY）
 * 返回: true 表示匹配
 */
static bool emm_match_byte(uint8_t actual, uint8_t expected)
{
    return expected == EMM_STEPPER_MATCH_ANY || actual == expected;
}

/*
 * 从 UART 轮询接收数据并推入环形缓冲区
 *
 * 该函数会多次调用 transport.read（最多 device->poll_attempts 次）以
 * 尽可能多地收集数据。首次调用使用完整的 timeout_ms 超时，后续调用
 * 使用 0 超时（即非阻塞）以清空接收 FIFO 中剩余的所有数据。
 *
 * device     - EMM 设备句柄
 * timeout_ms - 首次读取的超时时间（毫秒）
 * 返回: EMM_OK 表示收到至少一个字节，EMM_ERROR_TIMEOUT 表示无数据，
 *        EMM_ERROR_INVALID_ARG 表示参数无效
 */
EmmStatus emm_poll(EmmDevice *device, uint32_t timeout_ms)
{
    uint8_t chunk[EMM_STEPPER_RX_READ_CHUNK];
    size_t total = 0u;
    uint8_t loops = 0u;

    if (device == 0 || device->transport.read == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    do
    {
        size_t count = device->transport.read(chunk, sizeof(chunk), (loops == 0u) ? timeout_ms : 0u, device->transport.user_data);
        if (count == 0u)
        {
            break;
        }
        for (size_t i = 0u; i < count; ++i)
        {
            (void)emm_rx_push(device, chunk[i]);
        }
        total += count;
        loops++;
    } while (loops < device->poll_attempts);

    return (total > 0u) ? EMM_OK : EMM_ERROR_TIMEOUT;
}

/*
 * 从环形缓冲区中尝试解析一帧固定长度的 EMM 响应
 *
 * 固定长度帧格式: [地址(1)] [功能码(1)] [数据(N)] [校验和(1)]  总长 = N+3
 * 其中 N = response_length - 3
 *
 * 本函数会扫描整个缓冲区寻找符合以下条件的帧：
 * 1. 地址字节匹配（支持通配符）
 * 2. 功能码匹配（当 strict_frame_check 启用时）
 * 3. 校验和正确
 *
 * 如果 strict_frame_check 禁用，则只匹配地址（不检查功能码），这
 * 在不确定驱动器返回的精确功能码时使用。
 *
 * 对于不匹配的帧头噪声，如果地址完全不匹配期望值，则将其丢弃以防
 * 缓冲区被垃圾数据填满。
 *
 * device           - EMM 设备句柄
 * expected_address - 期望的驱动器地址（可用 EMM_STEPPER_MATCH_ANY 通配）
 * expected_code    - 期望的功能码（可用 EMM_STEPPER_MATCH_ANY 通配）
 * response         - 输出缓冲区，存储完整的响应帧
 * response_length  - 期望的响应帧总长度（必须 >= 3）
 * 返回: EMM_OK 表示解析成功，EMM_ERROR_TIMEOUT 表示未找到匹配帧，
 *        EMM_ERROR_INVALID_ARG 表示参数无效
 */
static EmmStatus emm_try_parse_fixed_from_rx(EmmDevice *device,
                                             uint8_t expected_address,
                                             uint8_t expected_code,
                                             uint8_t *response,
                                             size_t response_length)
{
    size_t count;

    if (device == 0 || response == 0 || response_length < 3u || response_length > EMM_STEPPER_MAX_FRAME_SIZE)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    count = emm_rx_available(device);
    if (count < response_length)
    {
        return EMM_ERROR_TIMEOUT;
    }

    for (size_t offset = 0u; offset + response_length <= count; ++offset)
    {
        uint8_t address = 0u;
        uint8_t code = 0u;
        uint8_t frame[EMM_STEPPER_MAX_FRAME_SIZE];
        uint8_t expected_checksum;

        (void)emm_rx_peek(device, offset, &address);
        (void)emm_rx_peek(device, offset + 1u, &code);

        if (!emm_match_byte(address, expected_address))
        {
            continue;
        }
        if (device->strict_frame_check && !emm_match_byte(code, expected_code))
        {
            continue;
        }

        emm_rx_copy(device, offset, frame, response_length);
        expected_checksum = emm_calculate_checksum(frame, response_length - 1u, device->checksum_mode);
        if (frame[response_length - 1u] != expected_checksum)
        {
            continue;
        }

        for (size_t i = 0u; i < response_length; ++i)
        {
            response[i] = frame[i];
        }
        emm_rx_drop(device, offset + response_length);
        return EMM_OK;
    }

    /* Discard only obvious noise before the first possible address. Valid but
       currently unmatched frames are retained for emm_read_any_frame(). */
    while (emm_rx_available(device) > 0u)
    {
        uint8_t address = 0u;
        (void)emm_rx_peek(device, 0u, &address);
        if (expected_address == EMM_STEPPER_MATCH_ANY || address == expected_address)
        {
            break;
        }
        if (address == EMM_STEPPER_BROADCAST_ADDRESS || address == EMM_STEPPER_MATCH_ANY)
        {
            break;
        }
        emm_rx_drop(device, 1u);
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 从环形缓冲区中尝试解析一帧可变长度的 EMM 响应
 *
 * 可变长度帧格式: [地址(1)] [功能码(1)] [总长度(1)] [数据(M)] [校验和(1)]
 * 总帧长 >= 5，总长度字段位于索引 2 处。
 *
 * 与固定长度解析相比，可变长度帧多了一个总长度字段，使得帧长度可以在
 * 运行时确定。这使得同一个功能码可以返回不同长度的数据（如系统状态
 * 查询返回多种参数）。
 *
 * 当帧的总长度超过 response_capacity 时返回 EMM_ERROR_BAD_RESPONSE，
 * 因为输出缓冲区无法容纳该帧。
 *
 * device            - EMM 设备句柄
 * expected_address  - 期望的驱动器地址（可用通配符）
 * expected_code     - 期望的功能码（可用通配符）
 * response          - 输出缓冲区，存储完整的响应帧
 * response_capacity - 输出缓冲区的容量
 * response_length   - 输出参数，实际收到的帧长度
 * 返回: EMM_OK 表示解析成功，EMM_ERROR_TIMEOUT 表示未找到匹配帧，
 *        EMM_ERROR_BAD_RESPONSE 表示帧长度超出输出容量
 */
static EmmStatus emm_try_parse_dynamic_from_rx(EmmDevice *device,
                                               uint8_t expected_address,
                                               uint8_t expected_code,
                                               uint8_t *response,
                                               size_t response_capacity,
                                               size_t *response_length)
{
    size_t count;

    if (device == 0 || response == 0 || response_length == 0 || response_capacity < 5u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    count = emm_rx_available(device);
    if (count < 5u)
    {
        return EMM_ERROR_TIMEOUT;
    }

    for (size_t offset = 0u; offset + 5u <= count; ++offset)
    {
        uint8_t address = 0u;
        uint8_t code = 0u;
        uint8_t total_u8 = 0u;
        size_t total;
        uint8_t frame[EMM_STEPPER_MAX_FRAME_SIZE];
        uint8_t expected_checksum;

        (void)emm_rx_peek(device, offset, &address);
        (void)emm_rx_peek(device, offset + 1u, &code);
        (void)emm_rx_peek(device, offset + 2u, &total_u8);
        total = (size_t)total_u8;

        if (!emm_match_byte(address, expected_address))
        {
            continue;
        }
        if (device->strict_frame_check && !emm_match_byte(code, expected_code))
        {
            continue;
        }
        if (total < 5u || total > EMM_STEPPER_MAX_FRAME_SIZE)
        {
            continue;
        }
        if (total > response_capacity)
        {
            return EMM_ERROR_BAD_RESPONSE;
        }
        if (offset + total > count)
        {
            continue;
        }

        emm_rx_copy(device, offset, frame, total);
        expected_checksum = emm_calculate_checksum(frame, total - 1u, device->checksum_mode);
        if (frame[total - 1u] != expected_checksum)
        {
            continue;
        }

        for (size_t i = 0u; i < total; ++i)
        {
            response[i] = frame[i];
        }
        *response_length = total;
        emm_rx_drop(device, offset + total);
        return EMM_OK;
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 读取一帧固定长度的响应
 *
 * 首先尝试从已有的缓冲区数据中解析，如果失败则轮询 UART 获取更多
 * 数据，然后再次尝试解析。重复最多 device->poll_attempts 次。
 *
 * 如果 timeout_ms 为 0，则只尝试一次（从缓冲区解析，不等待新数据）。
 *
 * device           - EMM 设备句柄
 * expected_address - 期望的驱动器地址
 * expected_code    - 期望的功能码
 * response         - 输出缓冲区
 * response_length  - 期望的帧长度
 * timeout_ms       - 每次轮询的超时时间（毫秒）
 * 返回: EMM_OK 表示成功读取，EMM_ERROR_TIMEOUT 表示超时
 */
EmmStatus emm_read_fixed_frame(EmmDevice *device,
                               uint8_t expected_address,
                               uint8_t expected_code,
                               uint8_t *response,
                               size_t response_length,
                               uint32_t timeout_ms)
{
    EmmStatus status;
    uint8_t attempts;

    if (device == 0 || response == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_try_parse_fixed_from_rx(device, expected_address, expected_code, response, response_length);
    if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
    {
        return status;
    }

    attempts = (device->poll_attempts == 0u) ? 1u : device->poll_attempts;
    for (uint8_t i = 0u; i < attempts; ++i)
    {
        status = emm_poll(device, timeout_ms);
        if (status != EMM_OK && status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        status = emm_try_parse_fixed_from_rx(device, expected_address, expected_code, response, response_length);
        if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (timeout_ms == 0u)
        {
            break;
        }
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 读取一帧可变长度的响应
 *
 * 逻辑与 emm_read_fixed_frame 类似，但使用 emm_try_parse_dynamic_from_rx
 * 解析可变长度帧。适用于返回数据长度不固定的命令（如获取系统状态）。
 *
 * device            - EMM 设备句柄
 * expected_address  - 期望的驱动器地址
 * expected_code     - 期望的功能码
 * response          - 输出缓冲区
 * response_capacity - 输出缓冲区容量
 * response_length   - 输出参数，实际收到的帧长度
 * timeout_ms        - 每次轮询的超时时间（毫秒）
 * 返回: EMM_OK 表示成功读取，否则返回错误码
 */
EmmStatus emm_read_dynamic_frame(EmmDevice *device,
                                 uint8_t expected_address,
                                 uint8_t expected_code,
                                 uint8_t *response,
                                 size_t response_capacity,
                                 size_t *response_length,
                                 uint32_t timeout_ms)
{
    EmmStatus status;
    uint8_t attempts;

    if (device == 0 || response == 0 || response_length == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_try_parse_dynamic_from_rx(device, expected_address, expected_code, response, response_capacity, response_length);
    if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
    {
        return status;
    }

    attempts = (device->poll_attempts == 0u) ? 1u : device->poll_attempts;
    for (uint8_t i = 0u; i < attempts; ++i)
    {
        status = emm_poll(device, timeout_ms);
        if (status != EMM_OK && status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        status = emm_try_parse_dynamic_from_rx(device, expected_address, expected_code, response, response_capacity, response_length);
        if (status == EMM_OK || status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (timeout_ms == 0u)
        {
            break;
        }
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 读取任意一帧响应（不限定地址和功能码）
 *
 * 此函数优先尝试解析可变长度帧，如果失败则尝试固定长度（4字节）帧。
 * 这使得接收方可以处理来自总线上任意设备的消息，无需预先知道帧格式。
 *
 * 用于调试、监听模式或处理非预期的主动上报消息。
 *
 * device    - EMM 设备句柄
 * frame     - 输出参数，存储接收到的帧信息（地址、功能码、长度、数据）
 * timeout_ms- 超时时间（毫秒）
 * 返回: EMM_OK 表示成功读取任意一帧
 */
EmmStatus emm_read_any_frame(EmmDevice *device, EmmRxFrame *frame, uint32_t timeout_ms)
{
    uint8_t response[EMM_STEPPER_MAX_FRAME_SIZE];
    size_t response_length = 0u;
    EmmStatus status;

    if (device == 0 || frame == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    status = emm_read_dynamic_frame(device, EMM_STEPPER_MATCH_ANY, EMM_STEPPER_MATCH_ANY,
                                    response, sizeof(response), &response_length, timeout_ms);
    if (status != EMM_OK)
    {
        status = emm_read_fixed_frame(device, EMM_STEPPER_MATCH_ANY, EMM_STEPPER_MATCH_ANY,
                                      response, 4u, timeout_ms);
        if (status != EMM_OK)
        {
            return status;
        }
        response_length = 4u;
    }

    frame->address = response[0];
    frame->code = response[1];
    frame->length = response_length;
    for (size_t i = 0u; i < response_length; ++i)
    {
        frame->bytes[i] = response[i];
    }
    return EMM_OK;
}

/*
 * 等待电机运动到位（接收到指定功能码的响应帧）
 *
 * 在 EMM_RESPONSE_REACHED 模式下，电机运动到位后会发送一帧"到位"
 * 通知。此函数在发送运动命令后调用，等待该通知帧到达。
 *
 * 实际上是对 emm_read_fixed_frame 的封装，固定使用 device->address
 * 作为期望地址。
 *
 * device          - EMM 设备句柄
 * expected_code   - 期望的到位通知功能码（通常就是运动命令的 CODE）
 * response        - 输出缓冲区
 * response_length - 期望的帧长度（至少 4 字节）
 * timeout_ms      - 等待超时（毫秒）
 * 返回: EMM_OK 表示收到到位通知
 */
EmmStatus emm_wait_reached(EmmDevice *device, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms)
{
    if (device == 0 || response == 0 || response_length < 4u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    return emm_read_fixed_frame(device, device->address, expected_code, response, response_length, timeout_ms);
}

/*
 * 发送命令帧（一次写入，不支持重试）
 *
 * 命令帧格式: [命令体(body)] [校验和(1)]
 * 其中命令体包含地址 + 功能码 + 协议字节 + 参数...
 *
 * 核心流程：
 * 1. 如果 auto_flush_before_write 启用，先清空接收缓冲区（避免回显干扰）
 * 2. 刷新发送缓冲区（确保发送完整）
 * 3. 将命令体拷贝到本地数组，计算并追加校验和
 * 4. 通过 transport.write 发送完整帧
 *
 * 注意：校验证计算覆盖整个命令体（包括地址字节），模式由
 * device->checksum_mode 指定（固定值/XOR/CRC8）。
 *
 * device     - EMM 设备句柄
 * body       - 命令体（不含校验和）
 * body_length- 命令体字节数
 * 返回: EMM_OK 表示发送成功，EMM_ERROR_IO 表示写入字节数不匹配
 */
static EmmStatus emm_write_body_once(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t command[EMM_STEPPER_MAX_FRAME_SIZE];

    if (device == 0 || body == 0 || device->transport.write == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    if (body_length + 1u > sizeof(command))
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->auto_flush_before_write)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }
    if (device->transport.flush_output != 0)
    {
        device->transport.flush_output(device->transport.user_data);
    }

    for (size_t i = 0u; i < body_length; ++i)
    {
        command[i] = body[i];
    }
    command[body_length] = emm_calculate_checksum(body, body_length, device->checksum_mode);

    if (device->transport.write(command, body_length + 1u, device->transport.user_data) != body_length + 1u)
    {
        return EMM_ERROR_IO;
    }
    return EMM_OK;
}

/*
 * 从 EMM 响应帧中解析简单的状态值
 *
 * 简单的 EMM 响应帧格式: [地址] [功能码] [状态码] [校验和]
 * 状态码位于索引 response[2] 处：
 *   0x02 = EMM_STATUS_SUCCESS      -> 操作成功
 *   0xE2 = EMM_STATUS_PARAM_ERROR  -> 参数错误
 *   0xEE = EMM_STATUS_FORMAT_ERROR -> 格式错误
 *   其他 -> 未识别的响应
 *
 * response - 完整的响应帧（至少 3 字节）
 * 返回: 对应的 EmmStatus 枚举值
 */
static EmmStatus simple_status_from_response(const uint8_t *response)
{
    if (response[2] == EMM_STATUS_SUCCESS)
    {
        return EMM_OK;
    }
    if (response[2] == EMM_STATUS_PARAM_ERROR)
    {
        return EMM_ERROR_PARAM;
    }
    if (response[2] == EMM_STATUS_FORMAT_ERROR)
    {
        return EMM_ERROR_FORMAT;
    }
    return EMM_ERROR_BAD_RESPONSE;
}

/*
 * 判断命令体是否为广播且不需要响应
 *
 * EMM 协议中，广播地址 (EMM_STEPPER_BROADCAST_ADDRESS) 发送的命令通常
 * 不需要单独的响应，因为所有从站会同时执行。但 EMM_CODE_BROADCAST_GET_ID
 * 是例外，它需要回复。
 *
 * body        - 命令体数据
 * body_length - 命令体长度
 * 返回: true 表示广播且无需等待响应
 */
static bool emm_body_is_broadcast_no_response(const uint8_t *body, size_t body_length)
{
    if (body == 0 || body_length < 2u)
    {
        return false;
    }
    if (body[0] != EMM_STEPPER_BROADCAST_ADDRESS)
    {
        return false;
    }
    return body[1] != EMM_CODE_BROADCAST_GET_ID;
}

/*
 * 发送简单命令并获取状态响应
 *
 "简单"命令指那些只需要成功/失败状态回复，不需要返回额外数据的命令
 *（如使能、重置、归零等）。该函数根据 device->response_mode 决定
 * 是否等待响应：
 *   EMM_RESPONSE_NONE  -> 只发不收，直接返回
 *   其他模式           -> 发送并等待 4 字节状态帧
 *
 * device     - EMM 设备句柄
 * body       - 命令体
 * body_length- 命令体长度
 * 返回: 发送和响应的合并状态
 */
static EmmStatus send_simple(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t response[4];
    EmmStatus status;

    if (device == 0 || body == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    status = emm_send_raw(device, body, body_length, response, sizeof(response));
    if (status != EMM_OK)
    {
        return status;
    }
    return simple_status_from_response(response);
}

/*
 * 发送运动控制命令，并根据响应模式处理回复
 *
 * 运动命令（如 jog、move_pulses 等）的响应处理比简单命令复杂，因为
 * EMM 驱动器支持多种响应模式：
 *
 *   EMM_RESPONSE_NONE    - 只发不收（火灾-遗忘模式）
 *   EMM_RESPONSE_RECEIVE - 等待命令确认帧（表示驱动器已收到）
 *   EMM_RESPONSE_REACHED - 等待电机到位帧（运动完成通知）
 *   EMM_RESPONSE_BOTH    - 先等确认，再等到位（最可靠但最慢）
 *
 * 函数首先发送命令帧，然后根据 response_mode 决定等待何种响应。
 *
 * device     - EMM 设备句柄
 * body       - 命令体
 * body_length- 命令体长度
 * 返回: 根据响应模式返回对应的成功/失败状态
 */
static EmmStatus send_motion(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    uint8_t response[4];
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || body_length < 2u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    status = emm_write_body_once(device, body, body_length);
    if (status != EMM_OK)
    {
        return status;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || body[0] == EMM_STEPPER_BROADCAST_ADDRESS)
    {
        return EMM_OK;
    }

    if (device->response_mode == EMM_RESPONSE_REACHED)
    {
        status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->reached_timeout_ms);
        return (status == EMM_OK) ? simple_status_from_response(response) : status;
    }

    status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->timeout_ms);
    if (status != EMM_OK)
    {
        return status;
    }
    status = simple_status_from_response(response);
    if (status != EMM_OK)
    {
        return status;
    }

    if (device->response_mode == EMM_RESPONSE_BOTH)
    {
        status = emm_read_fixed_frame(device, expected_address, expected_code, response, sizeof(response), device->reached_timeout_ms);
        return (status == EMM_OK) ? simple_status_from_response(response) : status;
    }

    return EMM_OK;
}

/*
 * 发送读取命令并获取响应数据
 *
 * 读取命令只需地址+功能码（无需协议字节或额外参数）。函数构造一个
 * 2 字节命令体，然后调用 emm_send_raw 发送并接收响应。
 *
 * device         - EMM 设备句柄
 * code           - 要读取的功能码（如 EMM_CODE_GET_REALTIME_POSITION）
 * response       - 输出缓冲区，存储完整的响应帧
 * response_length- 期望的响应帧长度
 * 返回: 发送和接收的结果状态
 */
static EmmStatus send_read(EmmDevice *device, uint8_t code, uint8_t *response, size_t response_length)
{
    uint8_t body[2];
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = code;
    return emm_send_raw(device, body, sizeof(body), response, response_length);
}

/*
 * 解析归零（回零）状态字节
 *
 * EMM 驱动器用一个字节编码归零相关的多种状态：
 *   bit 0: 编码器就绪
 *   bit 1: 编码器已校准
 *   bit 2: 正在进行归零
 *   bit 3: 归零失败
 *   bit 4: 过热
 *   bit 5: 过流
 *   bit 6-7: 保留
 *
 * data   - 原始状态字节
 * status - 输出参数，解析后的归零状态结构体
 */
static void parse_homing_status(uint8_t data, EmmHomingStatus *status)
{
    status->encoder_ready = (data & 0x01u) != 0u;
    status->calibrated = (data & 0x02u) != 0u;
    status->is_homing = (data & 0x04u) != 0u;
    status->homing_failed = (data & 0x08u) != 0u;
    status->over_temp = (data & 0x10u) != 0u;
    status->over_current = (data & 0x20u) != 0u;
}

/*
 * 解析电机状态字节
 *
 * EMM 驱动器用一个字节编码电机的运行状态：
 *   bit 0: 电机使能
 *   bit 1: 位置到位
 *   bit 2: 检测到堵转
 *   bit 3: 堵转保护已触发
 *   bit 4: 左限位触发
 *   bit 5: 右限位触发
 *   bit 6: 保留
 *   bit 7: 掉电标志
 *
 * data   - 原始状态字节
 * status - 输出参数，解析后的电机状态结构体
 */
static void parse_motor_status(uint8_t data, EmmMotorStatus *status)
{
    status->enabled = (data & 0x01u) != 0u;
    status->position_reached = (data & 0x02u) != 0u;
    status->stall_detected = (data & 0x04u) != 0u;
    status->stall_protected = (data & 0x08u) != 0u;
    status->left_limit = (data & 0x10u) != 0u;
    status->right_limit = (data & 0x20u) != 0u;
    status->power_off_flag = (data & 0x80u) != 0u;
}

/*
 * 将归零参数结构体编码为字节数组（用于发送给驱动器）
 *
 * 参数布局（15 字节）：
 *   [0]   = 归零模式
 *   [1]   = 归零方向
 *   [2-3] = 归零速度 (RPM)，大端序 uint16
 *   [4-7] = 归零超时时间 (ms)，大端序 uint32
 *   [8-9] = 碰撞检测速度 (RPM)，大端序 uint16
 *   [10-11] = 碰撞检测电流 (mA)，大端序 uint16
 *   [12-13] = 碰撞检测时间 (ms)，大端序 uint16
 *   [14]  = 自动归零标志
 *
 * data   - 目标字节数组
 * params - 归零参数结构体
 */
static void encode_homing_params(uint8_t *data, const EmmHomingParams *params)
{
    data[0] = (uint8_t)params->homing_mode;
    data[1] = (uint8_t)params->homing_direction;
    write_u16_be(&data[2], params->homing_speed_rpm);
    write_u32_be(&data[4], params->homing_timeout_ms);
    write_u16_be(&data[8], params->collision_speed_rpm);
    write_u16_be(&data[10], params->collision_current_ma);
    write_u16_be(&data[12], params->collision_time_ms);
    data[14] = params->auto_home ? 1u : 0u;
}

/*
 * 将字节数组解码为归零参数结构体（用于解析驱动器返回的数据）
 * 字节布局与 encode_homing_params 完全对应。
 *
 * data   - 从驱动器接收到的原始字节
 * params - 输出参数，解码后的归零参数结构体
 */
static void decode_homing_params(const uint8_t *data, EmmHomingParams *params)
{
    params->homing_mode = (EmmHomingMode)data[0];
    params->homing_direction = (EmmDirection)data[1];
    params->homing_speed_rpm = read_u16_be(&data[2]);
    params->homing_timeout_ms = read_u32_be(&data[4]);
    params->collision_speed_rpm = read_u16_be(&data[8]);
    params->collision_current_ma = read_u16_be(&data[10]);
    params->collision_time_ms = read_u16_be(&data[12]);
    params->auto_home = data[14] != 0u;
}

/*
 * 将配置参数结构体编码为字节数组（28 字节，用于发送给驱动器）
 *
 * 配置参数涵盖电机的全部运行参数，包括电机类型、细分、电流、
 * PID、堵转保护、位置窗口等。这是 EMM 协议中最复杂的参数结构。
 *
 * data   - 目标字节数组（至少 28 字节）
 * params - 配置参数结构体
 */
static void encode_config(uint8_t *data, const EmmConfigParams *params)
{
    data[0] = (uint8_t)params->motor_type;
    data[1] = (uint8_t)params->pulse_port_mode;
    data[2] = (uint8_t)params->serial_port_mode;
    data[3] = (uint8_t)params->enable_level;
    data[4] = (uint8_t)params->dir_level;
    data[5] = (params->microstep == 256u) ? 0u : (uint8_t)params->microstep;
    data[6] = params->microstep_interp ? 1u : 0u;
    data[7] = 0u;
    write_u16_be(&data[8], params->open_loop_current_ma);
    write_u16_be(&data[10], params->closed_loop_current_ma);
    write_u16_be(&data[12], params->max_voltage);
    data[14] = (uint8_t)params->baud_rate;
    data[15] = (uint8_t)params->can_rate;
    data[16] = params->motor_id;
    data[17] = (uint8_t)params->checksum_mode;
    data[18] = (uint8_t)params->response_mode;
    data[19] = (uint8_t)params->stall_protect;
    write_u16_be(&data[20], params->stall_speed_rpm);
    write_u16_be(&data[22], params->stall_current_ma);
    write_u16_be(&data[24], params->stall_time_ms);
    write_u16_be(&data[26], params->position_window_x01deg);
}

/*
 * 将字节数组解码为配置参数结构体
 * 字节布局与 encode_config 完全对应。
 *
 * data   - 从驱动器接收到的原始字节（至少 28 字节）
 * params - 输出参数，解码后的配置参数结构体
 * 注意：data[5] 为 0 时表示细分为 256（即最大细分），因为
 *       细分值 0 在实际中无效，所以用作 256 的编码。
 */
static void decode_config(const uint8_t *data, EmmConfigParams *params)
{
    params->motor_type = (EmmMotorType)data[0];
    params->pulse_port_mode = (EmmPulsePortMode)data[1];
    params->serial_port_mode = (EmmSerialPortMode)data[2];
    params->enable_level = (EmmEnableLevel)data[3];
    params->dir_level = (EmmDirLevel)data[4];
    params->microstep = (data[5] == 0u) ? 256u : (uint16_t)data[5];
    params->microstep_interp = data[6] != 0u;
    params->open_loop_current_ma = read_u16_be(&data[8]);
    params->closed_loop_current_ma = read_u16_be(&data[10]);
    params->max_voltage = read_u16_be(&data[12]);
    params->baud_rate = (EmmBaudRate)data[14];
    params->can_rate = (EmmCanRate)data[15];
    params->motor_id = data[16];
    params->checksum_mode = (EmmChecksumMode)data[17];
    params->response_mode = (EmmResponseMode)data[18];
    params->stall_protect = (EmmStallProtect)data[19];
    params->stall_speed_rpm = read_u16_be(&data[20]);
    params->stall_current_ma = read_u16_be(&data[22]);
    params->stall_time_ms = read_u16_be(&data[24]);
    params->position_window_x01deg = read_u16_be(&data[26]);
}

/*
 * 初始化 EMM 设备结构体
 *
 * 设置默认参数：
 *   - 校验模式: EMM_CHECKSUM_FIXED（固定值 0x6B）
 *   - 响应模式: EMM_RESPONSE_NONE（只发不收，适合运动控制场景）
 *   - 超时时间: EMM_STEPPER_DEFAULT_TIMEOUT_MS
 *   - 到位超时: EMM_STEPPER_REACHED_TIMEOUT_MS
 *   - 严格帧检查: 启用
 *   - 清空接收缓冲区
 *
 * 如果 transport 为 NULL，则所有传输函数指针清零（安全初始化）。
 *
 * device    - EMM 设备句柄
 * transport - 传输层接口（read/write/flush/delay 函数指针）
 * address   - 驱动器的通信地址
 */
void emm_init(EmmDevice *device, const EmmTransport *transport, uint8_t address)
{
    if (device == 0)
    {
        return;
    }

    device->address = address;
    device->checksum_mode = EMM_CHECKSUM_FIXED;
    device->timeout_ms = EMM_STEPPER_DEFAULT_TIMEOUT_MS;
    device->retry_delay_ms = 0u;
    device->max_retries = EMM_STEPPER_MAX_RETRIES;
    device->reached_timeout_ms = EMM_STEPPER_REACHED_TIMEOUT_MS;
    device->response_mode = EMM_RESPONSE_NONE;
    device->strict_frame_check = true;
    device->auto_flush_before_write = false;
    device->auto_flush_before_read = false;
    device->poll_attempts = EMM_STEPPER_POLL_ATTEMPTS;
    device->rx_head = 0u;
    device->rx_tail = 0u;
    device->rx_overflow_count = 0u;

    if (transport != 0)
    {
        device->transport = *transport;
    }
    else
    {
        device->transport.write = 0;
        device->transport.read = 0;
        device->transport.flush_input = 0;
        device->transport.flush_output = 0;
        device->transport.delay_ms = 0;
        device->transport.user_data = 0;
    }
}

/*
 * 设置本地存储的目标驱动器地址
 * 后续所有操作（读、写、运动等）都会使用此地址。
 * device - EMM 设备句柄
 * address- 新的驱动器地址
 */
void emm_select_address(EmmDevice *device, uint8_t address)
{
    if (device != 0)
    {
        device->address = address;
    }
}

/*
 * 设置本地响应模式（仅影响本控制器的行为，不写入驱动器）
 * 默认为 EMM_RESPONSE_NONE，即只发送不等待响应。
 *
 * device - EMM 设备句柄
 * mode   - 新的响应模式
 */
void emm_set_response_mode_local(EmmDevice *device, EmmResponseMode mode)
{
    if (device != 0)
    {
        device->response_mode = mode;
    }
}

/*
 * 设置命令超时和到位超时时间
 * command_timeout_ms - 命令等待响应超时
 * reached_timeout_ms - 等待电机到位通知的超时
 */
void emm_set_timeouts(EmmDevice *device, uint32_t command_timeout_ms, uint32_t reached_timeout_ms)
{
    if (device != 0)
    {
        device->timeout_ms = command_timeout_ms;
        device->reached_timeout_ms = reached_timeout_ms;
    }
}

/*
 * 启用/禁用严格的帧检查
 * 启用时：地址和功能码都必须匹配
 * 禁用时：只匹配地址，不校验功能码（用于不确定驱动器返回码的场景）
 */
void emm_set_strict_frame_check(EmmDevice *device, bool enable)
{
    if (device != 0)
    {
        device->strict_frame_check = enable;
    }
}

/*
 * 设置在写入前是否自动清空接收缓冲区
 * 用于半双工场景，防止发送前缓冲区中残留的回显数据干扰后续解析。
 */
void emm_set_auto_flush_before_write(EmmDevice *device, bool enable)
{
    if (device != 0)
    {
        device->auto_flush_before_write = enable;
    }
}

/*
 * 设置校验模式
 * EMM_CHECKSUM_FIXED - 固定校验值 0x6B（最简单，常用于调试）
 * EMM_CHECKSUM_XOR   - 所有字节异或
 * EMM_CHECKSUM_CRC8  - CRC8 校验（多项式 0x31，最可靠）
 */
void emm_set_checksum_mode(EmmDevice *device, EmmChecksumMode mode)
{
    if (device != 0)
    {
        device->checksum_mode = mode;
    }
}

/*
 * 计算 EMM 协议帧的校验和
 *
 * 根据 mode 选择三种校验方式之一：
 * 1. EMM_CHECKSUM_FIXED: 始终返回固定值 0x6B，用于与旧版驱动兼容
 * 2. EMM_CHECKSUM_XOR:   所有字节异或求和
 * 3. EMM_CHECKSUM_CRC8:  CRC8 查表法，多项式 0x31
 *
 * 注意：即使 data 为 NULL 或 length 为 0，在 FIXED 模式下也会返回
 * 0x6B，这是为了在初始化等场景下的健壮性。
 *
 * data    - 输入数据
 * length  - 数据长度
 * mode    - 校验模式
 * 返回: 计算得到的校验和字节
 */
uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode)
{
    uint8_t checksum = 0u;

    if (mode == EMM_CHECKSUM_FIXED || data == 0 || length == 0u)
    {
        return EMM_STATUS_FIXED_CHECKSUM;
    }

    if (mode == EMM_CHECKSUM_XOR)
    {
        for (size_t i = 0u; i < length; ++i)
        {
            checksum ^= data[i];
        }
        return checksum;
    }

    if (mode == EMM_CHECKSUM_CRC8)
    {
        checksum = data[0];
        for (size_t i = 1u; i < length; ++i)
        {
            checksum = EmmCrc8Table[checksum ^ data[i]];
        }
        return checksum;
    }

    return EMM_STATUS_FIXED_CHECKSUM;
}

/*
 * 发送原始命令，不等待响应（火灾-遗忘模式）
 * 直接调用 emm_write_body_once 发送命令体。
 *
 * device     - EMM 设备句柄
 * body       - 命令体
 * body_length- 命令体长度
 * 返回: 发送结果
 */
EmmStatus emm_send_raw_no_response(EmmDevice *device, const uint8_t *body, size_t body_length)
{
    return emm_write_body_once(device, body, body_length);
}

/*
 * 发送原始命令并等待固定长度响应（支持自动重试）
 *
 * 核心机制：
 * 1. 根据命令体和响应模式确定是否需要等待响应
 * 2. 发送命令后，等待符合（地址 + 功能码）的固定长度响应帧
 * 3. 如果超时并且设置了重试延迟，则等待 retry_delay_ms 后重试
 * 4. 最多重试 device->max_retries 次
 *
 * device         - EMM 设备句柄
 * body           - 命令体
 * body_length    - 命令体长度
 * response       - 响应缓冲区
 * response_length- 期望的响应帧长度
 * 返回: 发送结果或响应超时
 */
EmmStatus emm_send_raw(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_length)
{
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0 || body_length < 2u || response_length < 3u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    for (uint8_t tries = 0u; tries < device->max_retries; ++tries)
    {
        status = emm_write_body_once(device, body, body_length);
        if (status != EMM_OK)
        {
            return status;
        }

        status = emm_read_fixed_frame(device, expected_address, expected_code, response, response_length, device->timeout_ms);
        if (status == EMM_OK)
        {
            return EMM_OK;
        }
        if (status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (device->transport.delay_ms != 0 && device->retry_delay_ms != 0u)
        {
            device->transport.delay_ms(device->retry_delay_ms, device->transport.user_data);
        }
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 发送原始命令并等待可变长度响应（支持自动重试）
 * 与 emm_send_raw 类似，但使用 emm_read_dynamic_frame 解析可变长度帧。
 * 适用于返回数据长度不固定的查询命令（如获取系统状态、获取配置等）。
 *
 * device            - EMM 设备句柄
 * body              - 命令体
 * body_length       - 命令体长度
 * response          - 响应缓冲区
 * response_capacity - 响应缓冲区容量
 * response_length   - 输出参数，实际接收到的帧长度
 * 返回: 发送结果或响应超时
 */
EmmStatus emm_send_raw_dynamic(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_capacity, size_t *response_length)
{
    EmmStatus status;
    uint8_t expected_address;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0 || response_length == 0 || body_length < 2u || response_capacity < 5u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    if (device->response_mode == EMM_RESPONSE_NONE || emm_body_is_broadcast_no_response(body, body_length))
    {
        return emm_send_raw_no_response(device, body, body_length);
    }

    expected_address = (body[0] == EMM_STEPPER_BROADCAST_ADDRESS) ? EMM_STEPPER_MATCH_ANY : body[0];
    expected_code = body[1];

    for (uint8_t tries = 0u; tries < device->max_retries; ++tries)
    {
        status = emm_write_body_once(device, body, body_length);
        if (status != EMM_OK)
        {
            return status;
        }

        status = emm_read_dynamic_frame(device, expected_address, expected_code, response, response_capacity, response_length, device->timeout_ms);
        if (status == EMM_OK)
        {
            return EMM_OK;
        }
        if (status != EMM_ERROR_TIMEOUT)
        {
            return status;
        }
        if (device->transport.delay_ms != 0 && device->retry_delay_ms != 0u)
        {
            device->transport.delay_ms(device->retry_delay_ms, device->transport.user_data);
        }
    }

    return EMM_ERROR_TIMEOUT;
}

/*
 * 校准编码器
 * 发送 EMM_CODE_CAL_ENCODER 命令，配合 EMM_PROTOCOL_CAL_ENCODER 验证字节。
 * 驱动器会自动执行编码器校准程序，校准期间电机可能振动。
 * device - EMM 设备句柄
 * 返回: 操作结果
 */
EmmStatus emm_calibrate_encoder(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_CAL_ENCODER, EMM_PROTOCOL_CAL_ENCODER };
    return send_simple(device, body, sizeof(body));
}

/*
 * 重启驱动器
 * 驱动器收到此命令后会执行硬复位，通信会短暂中断。
 */
EmmStatus emm_restart(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_RESTART, EMM_PROTOCOL_RESTART };
    return send_simple(device, body, sizeof(body));
}

/*
 * 将当前位置设为零位
 * 不影响编码器物理零位，仅将当前脉冲计数值清零。
 */
EmmStatus emm_zero_position(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_ZERO_POSITION, EMM_PROTOCOL_ZERO_POSITION };
    return send_simple(device, body, sizeof(body));
}

/*
 * 带验证的零位设置
 *
 * 此函数比 emm_zero_position 更可靠：在发送零位命令后，会通过
 * emm_get_realtime_position_forced 读取实际位置，确认位置确实归零
 *（偏差在 +/-0.1 度以内）。如果多次尝试后仍不成功则返回错误。
 *
 * 适用于对零位精度要求较高的场景（如机械臂校准）。
 */
EmmStatus emm_zero_position_verified(EmmDevice *device)
{
    uint8_t attempts;
    EmmStatus status = EMM_ERROR;

    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    attempts = (device->max_retries == 0u) ? 1u : device->max_retries;
    for (uint8_t attempt = 0u; attempt < attempts; ++attempt)
    {
        float position_deg;
        uint8_t body[3] = {
            device->address, EMM_CODE_ZERO_POSITION, EMM_PROTOCOL_ZERO_POSITION
        };

        status = emm_send_raw_no_response(device, body, sizeof(body));
        if (status == EMM_OK)
        {
            if (device->transport.delay_ms != 0)
            {
                device->transport.delay_ms(10u, device->transport.user_data);
            }
            status = emm_get_realtime_position_forced(device, &position_deg);
            if (status == EMM_OK && position_deg >= -0.1f && position_deg <= 0.1f)
            {
                return EMM_OK;
            }
            if (status == EMM_OK) status = EMM_ERROR_BAD_RESPONSE;
        }
    }
    return status;
}

/*
 * 清除保护状态
 * 当驱动器触发过流、过温或堵转保护后，需要发送此命令才能恢复正常运行。
 */
EmmStatus emm_clear_protection(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_CLEAR_PROTECTION, EMM_PROTOCOL_CLEAR_PROTECTION };
    return send_simple(device, body, sizeof(body));
}

/*
 * 恢复出厂设置
 * 所有参数将被重置为默认值，通信地址也可能改变，之后可能需要重新搜索设备。
 */
EmmStatus emm_factory_reset(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_FACTORY_RESET, EMM_PROTOCOL_FACTORY_RESET };
    return send_simple(device, body, sizeof(body));
}

/*
 * 使能/禁能电机
 * enable   - true 使能（线圈通电保持转矩），false 禁能（线圈断电）
 * sync_flag- 同步标志，控制命令执行时机（立即执行/等待同步信号）
 */
EmmStatus emm_enable(EmmDevice *device, bool enable, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_ENABLE, EMM_PROTOCOL_ENABLE, enable ? 1u : 0u, (uint8_t)sync_flag };
    return send_simple(device, body, sizeof(body));
}

/*
 * 禁能电机（emmm_enable 的快捷封装）
 */
EmmStatus emm_disable(EmmDevice *device, EmmSyncFlag sync_flag)
{
    return emm_enable(device, false, sync_flag);
}

/*
 * 点动运行（连续转动）
 *
 * 电机将按照指定方向和速度持续运行，直到收到停止命令或触发保护。
 * 速度限制为 3000 RPM（超过此值返回参数错误）。
 *
 * 参数布局（7 字节）：
 *   [0] 地址
 *   [1] EMM_CODE_JOG
 *   [2] 方向（CW/CCW）
 *   [3-4] 速度 (RPM)，大端序 uint16
 *   [5] 加速度
 *   [6] 同步标志
 *
 * params - 点动参数（方向、速度、加速度、同步标志）
 * 返回: 发送结果（根据响应模式可能包含到位信息）
 */
EmmStatus emm_jog(EmmDevice *device, const EmmJogParams *params)
{
    uint8_t body[7];
    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_JOG;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    body[6] = (uint8_t)params->sync_flag;
    return send_motion(device, body, sizeof(body));
}

/*
 * 按脉冲数定位运动
 *
 * 电机将按照指定的方向、速度、加速度运行指定的脉冲数后停止。
 * 支持绝对位置模式和相对位置模式。
 *
 * 参数布局（12 字节）：
 *   [0]   地址
 *   [1]   EMM_CODE_POSITION
 *   [2]   方向
 *   [3-4] 速度 (RPM)，大端序 uint16
 *   [5]   加速度
 *   [6-9] 脉冲数，大端序 uint32
 *   [10]  运动模式（绝对/相对）
 *   [11]  同步标志
 *
 * params - 定位参数
 * 返回: 运动发送结果
 */
EmmStatus emm_move_pulses(EmmDevice *device, const EmmPositionParams *params)
{
    uint8_t body[12];
    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_POSITION;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    write_u32_be(&body[6], params->pulse_count);
    body[10] = (uint8_t)params->motion_mode;
    body[11] = (uint8_t)params->sync_flag;
    return send_motion(device, body, sizeof(body));
}

/*
 * 按角度定位运动（角度 -> 脉冲数转换的高层封装）
 *
 * 角度到脉冲数的转换公式：
 *   脉冲数 = 角度 * (200 * 细分) / 360
 * 其中 200 是两相步进电机的标准步距角（1.8 度/步）。
 *
 * 转换后使用四舍五入取整，如果计算结果为 0 但角度不为 0，
 * 说明角度太小无法驱动一个微步，返回 EMM_ERROR_INVALID_ARG。
 *
 * device      - EMM 设备句柄
 * degrees     - 目标角度（度），正数 = CW，负数 = CCW
 * speed_rpm   - 运动速度 (RPM)
 * acceleration- 加速度
 * motion_mode - 运动模式（绝对位置/相对位置）
 * microstep   - 电机细分（1-256），用于角度-脉冲换算
 * sync_flag   - 同步标志
 * 返回: 运动发送结果
 */
EmmStatus emm_move_degrees(EmmDevice *device, float degrees, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag)
{
    float pulses_float;
    int32_t pulses;
    EmmPositionParams params;

    if (device == 0 || microstep == 0u || microstep > 256u ||
        degrees != degrees || degrees > FLT_MAX || degrees < -FLT_MAX)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    pulses_float = degrees * (float)(200u * microstep) / 360.0f;
    if (pulses_float > (float)INT32_MAX || pulses_float < -(float)INT32_MAX)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    pulses = (pulses_float >= 0.0f) ? (int32_t)(pulses_float + 0.5f) : (int32_t)(pulses_float - 0.5f);
    if (pulses == 0 && degrees != 0.0f)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    params.direction = (pulses < 0) ? EMM_DIRECTION_CCW : EMM_DIRECTION_CW;
    params.speed_rpm = speed_rpm;
    params.acceleration = acceleration;
    params.pulse_count = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)pulses;
    params.motion_mode = motion_mode;
    params.sync_flag = sync_flag;
    return emm_move_pulses(device, &params);
}

/*
 * 按圈数定位运动（emmm_move_degrees 的快捷封装，1 圈 = 360 度）
 */
EmmStatus emm_move_revolutions(EmmDevice *device, float revolutions, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag)
{
    if (revolutions != revolutions || revolutions > (FLT_MAX / 360.0f) ||
        revolutions < -(FLT_MAX / 360.0f))
    {
        return EMM_ERROR_INVALID_ARG;
    }
    return emm_move_degrees(device, revolutions * 360.0f, speed_rpm, acceleration, motion_mode, microstep, sync_flag);
}

/*
 * 急停电机
 * 发送 EMM_CODE_ESTOP 指令，配合 EMM_PROTOCOL_ESTOP 验证字节。
 * 电机将立即停止并进入保护状态，需要重新使能才能再次运行。
 * sync_flag - 同步标志
 */
EmmStatus emm_stop(EmmDevice *device, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_ESTOP, EMM_PROTOCOL_ESTOP, (uint8_t)sync_flag };
    return send_simple(device, body, sizeof(body));
}

/*
 * 带验证的急停
 *
 * 相比 emm_stop，此函数会：
 * 1. 发送急停命令
 * 2. 通过 emm_get_system_status_forced 查询电机实际速度
 * 3. 确认速度降为 0（-1 <= RPM <= 1）
 * 4. 如果多次尝试后仍未停稳，强制禁能电机（切断线圈电流）作为安全措施
 *
 * 强制禁能的设计是一个重要的安全特性：即使急停命令因通信异常而失效，
 * 也确保电机不会在未受控状态下继续运行。
 */
EmmStatus emm_stop_verified(EmmDevice *device, EmmSyncFlag sync_flag)
{
    EmmStatus last_status = EMM_ERROR;
    uint8_t attempts;

    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    attempts = (device->max_retries == 0u) ? 1u : device->max_retries;

    for (uint8_t attempt = 0u; attempt < attempts; ++attempt)
    {
        EmmSystemStatusParams status;
        uint8_t body[4] = {
            device->address, EMM_CODE_ESTOP, EMM_PROTOCOL_ESTOP, (uint8_t)sync_flag
        };

        last_status = emm_send_raw_no_response(device, body, sizeof(body));

        if (last_status == EMM_OK)
        {
            if (device->transport.delay_ms != 0)
            {
                device->transport.delay_ms(5u, device->transport.user_data);
            }
            last_status = emm_get_system_status_forced(device, &status);
            if (last_status == EMM_OK && status.realtime_speed_rpm >= -1 &&
                status.realtime_speed_rpm <= 1)
            {
                return EMM_OK;
            }
            if (last_status == EMM_OK)
            {
                last_status = EMM_ERROR_BAD_RESPONSE;
            }
        }

        if (device->transport.delay_ms != 0 && device->retry_delay_ms != 0u)
        {
            device->transport.delay_ms(device->retry_delay_ms,
                                       device->transport.user_data);
        }
    }

    /* A failed stop must not leave the axis intentionally energized. The
     * disable is forced-response too, but the stop failure remains visible. */
    {
        uint8_t disable_body[5] = {
            device->address, EMM_CODE_ENABLE, EMM_PROTOCOL_ENABLE, 0u,
            (uint8_t)EMM_SYNC_IMMEDIATE
        };
        (void)emm_send_raw_no_response(device, disable_body, sizeof(disable_body));
    }

    return last_status;
}

/*
 * 发送同步运动触发信号（广播）
 * 所有配置为 EMM_SYNC_WAIT 模式的驱动器将在收到此信号后同时开始运动，
 * 实现多轴同步控制。
 */
EmmStatus emm_sync_move(EmmDevice *device)
{
    uint8_t body[3] = { EMM_STEPPER_BROADCAST_ADDRESS, EMM_CODE_SYNC_MOVE, EMM_PROTOCOL_SYNC_MOVE };
    return send_simple(device, body, sizeof(body));
}

/*
 * 将当前位置设为归零参考点
 * 驱动器的归零完成后，调用此函数将当前位置记录为机械零点。
 * store - 是否保存到驱动器 flash（掉电保持）
 */
EmmStatus emm_set_home_zero(EmmDevice *device, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_SET_HOME_ZERO, EMM_PROTOCOL_SET_HOME_ZERO, (uint8_t)store };
    return send_simple(device, body, sizeof(body));
}

/*
 * 执行归零（回零）操作
 * mode - 归零模式（如撞限位归零、编码器 Z 信号归零等）
 * sync_flag - 同步标志
 */
EmmStatus emm_home(EmmDevice *device, EmmHomingMode mode, EmmSyncFlag sync_flag)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[4] = { device->address, EMM_CODE_HOME, (uint8_t)mode, (uint8_t)sync_flag };
    return send_motion(device, body, sizeof(body));
}

/*
 * 停止归零操作
 */
EmmStatus emm_stop_home(EmmDevice *device)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_STOP_HOME, EMM_PROTOCOL_STOP_HOME };
    return send_simple(device, body, sizeof(body));
}

/*
 * 获取归零状态
 * 返回驱动器的当前归零进度和相关状态（编码器就绪、归零进行中、故障等）。
 */
EmmStatus emm_get_homing_status(EmmDevice *device, EmmHomingStatus *status)
{
    uint8_t response[4];
    EmmStatus result;
    if (status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_HOME_STATUS, response, sizeof(response));
    if (result == EMM_OK)
    {
        parse_homing_status(response[2], status);
    }
    return result;
}

/*
 * 获取归零参数配置
 * 从驱动器读取当前归零参数的完整配置。
 */
EmmStatus emm_get_homing_params(EmmDevice *device, EmmHomingParams *params)
{
    uint8_t response[18];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_HOME_PARAM, response, sizeof(response));
    if (result == EMM_OK)
    {
        decode_homing_params(&response[2], params);
    }
    return result;
}

/*
 * 设置归零参数并写入驱动器
 * params - 归零参数配置
 * store  - 是否保存到 flash
 */
EmmStatus emm_set_homing_params(EmmDevice *device, const EmmHomingParams *params, EmmStoreFlag store)
{
    uint8_t body[19];
    if (device == 0 || params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_HOME_PARAM;
    body[2] = EMM_PROTOCOL_SET_HOME_PARAM;
    body[3] = (uint8_t)store;
    encode_homing_params(&body[4], params);
    return send_simple(device, body, sizeof(body));
}

/*
 * 获取驱动器固件版本和硬件信息
 *
 * 响应帧解析（7 字节）：
 *   [2-3] 固件版本号
 *   [4-5] 硬件信息（高 4 位=硬件系列，次高 4 位=硬件类型，低 8 位=硬件版本）
 */
EmmStatus emm_get_version(EmmDevice *device, EmmVersionParams *version)
{
    uint8_t response[7];
    EmmStatus result;
    uint16_t hw_info;
    if (version == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_VERSION, response, sizeof(response));
    if (result == EMM_OK)
    {
        version->firmware_version = read_u16_be(&response[2]);
        hw_info = read_u16_be(&response[4]);
        version->hw_series = (uint8_t)((hw_info >> 12) & 0x0Fu);
        version->hw_type = (uint8_t)((hw_info >> 8) & 0x0Fu);
        version->hw_version = (uint8_t)(hw_info & 0xFFu);
    }
    return result;
}

/*
 * 获取电机相电阻和相电感参数
 * 这些参数通常在电机出厂时测得，用于驱动器内部算法调优。
 */
EmmStatus emm_get_motor_rh(EmmDevice *device, EmmMotorRHParams *params)
{
    uint8_t response[7];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_MOTOR_RH, response, sizeof(response));
    if (result == EMM_OK)
    {
        params->phase_resistance_mohm = read_u16_be(&response[2]);
        params->phase_inductance_uh = read_u16_be(&response[4]);
    }
    return result;
}

/*
 * 获取总线电压（单位: mV）
 */
EmmStatus emm_get_bus_voltage(EmmDevice *device, uint16_t *voltage_mv)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_BUS_VOLTAGE, response, sizeof(response));
    if (result == EMM_OK && voltage_mv != 0) { *voltage_mv = read_u16_be(&response[2]); }
    return (voltage_mv == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取总线电流（单位: mA）
 */
EmmStatus emm_get_bus_current(EmmDevice *device, uint16_t *current_ma)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_BUS_CURRENT, response, sizeof(response));
    if (result == EMM_OK && current_ma != 0) { *current_ma = read_u16_be(&response[2]); }
    return (current_ma == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取电机相电流（单位: mA）
 */
EmmStatus emm_get_phase_current(EmmDevice *device, uint16_t *current_ma)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_PHASE_CURRENT, response, sizeof(response));
    if (result == EMM_OK && current_ma != 0) { *current_ma = read_u16_be(&response[2]); }
    return (current_ma == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取编码器原始计数值（0-65535）
 * 编码器一圈对应 65536 个计数。
 */
EmmStatus emm_get_encoder(EmmDevice *device, uint16_t *encoder)
{
    uint8_t response[5];
    EmmStatus result = send_read(device, EMM_CODE_GET_ENCODER, response, sizeof(response));
    if (result == EMM_OK && encoder != 0) { *encoder = read_u16_be(&response[2]); }
    return (encoder == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取编码器角度（度）
 * 将编码器原始计数值(0-65535)映射到 0-360 度。
 */
EmmStatus emm_get_encoder_degrees(EmmDevice *device, float *degrees)
{
    uint16_t encoder;
    EmmStatus result;
    if (degrees == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_get_encoder(device, &encoder);
    if (result == EMM_OK)
    {
        *degrees = ((float)encoder * 360.0f) / 65536.0f;
    }
    return result;
}

/*
 * 获取当前脉冲计数值（带符号）
 * 使用带符号前缀编码（read_signed_prefix），支持正负值。
 * 正值表示 CW 方向脉冲数，负值表示 CCW 方向脉冲数。
 */
EmmStatus emm_get_pulse_count(EmmDevice *device, int32_t *pulse_count)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_PULSE_COUNT, response, sizeof(response));
    if (result == EMM_OK && pulse_count != 0) { *pulse_count = read_signed_prefix(&response[2], 5u); }
    return (pulse_count == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取目标位置（度）
 * 返回驱动器当前正在执行的定位命令的目标位置。
 */
EmmStatus emm_get_target_position(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_TARGET_POSITION, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取实时转速（RPM，带符号）
 * 使用一个字节表示符号（data[2] == 1 为负），后跟 uint16 绝对值。
 * 正值 = CW 方向，负值 = CCW 方向。
 */
EmmStatus emm_get_realtime_speed(EmmDevice *device, int16_t *speed_rpm)
{
    uint8_t response[6];
    int16_t sign;
    EmmStatus result = send_read(device, EMM_CODE_GET_REALTIME_SPEED, response, sizeof(response));
    if (result == EMM_OK && speed_rpm != 0)
    {
        sign = (response[2] == 1u) ? -1 : 1;
        *speed_rpm = (int16_t)(sign * (int16_t)read_u16_be(&response[3]));
    }
    return (speed_rpm == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取实时位置（度）
 * 返回电机的当前实际位置。位置值通过 encoder->pulse 换算为角度，
 * 带符号表示方向（正=CW，负=CCW）。
 */
EmmStatus emm_get_realtime_position(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_REALTIME_POSITION, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取位置误差（度）
 * 返回目标位置与实际位置的差值，用于监测跟随误差。
 */
EmmStatus emm_get_position_error(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result = send_read(device, EMM_CODE_GET_POSITION_ERROR, response, sizeof(response));
    if (result == EMM_OK && degrees != 0) { *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f; }
    return (degrees == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取驱动器温度（摄氏度，带符号）
 * 温度编码：response[2] == 0 表示负值，response[2] == 1 表示正值，
 * response[3] 为温度绝对值。
 */
EmmStatus emm_get_temperature(EmmDevice *device, int16_t *temperature_c)
{
    uint8_t response[5];
    int16_t sign;
    EmmStatus result = send_read(device, EMM_CODE_GET_TEMPERATURE, response, sizeof(response));
    if (result == EMM_OK && temperature_c != 0)
    {
        sign = (response[2] == 0u) ? -1 : 1;
        *temperature_c = (int16_t)(sign * (int16_t)response[3]);
    }
    return (temperature_c == 0) ? EMM_ERROR_INVALID_ARG : result;
}

/*
 * 获取电机状态（使能、到位、堵转、限位等）
 * 解析响应帧中的状态字节，填充到 EmmMotorStatus 结构体。
 */
EmmStatus emm_get_motor_status(EmmDevice *device, EmmMotorStatus *status)
{
    uint8_t response[4];
    EmmStatus result;
    if (status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_MOTOR_STATUS, response, sizeof(response));
    if (result == EMM_OK)
    {
        parse_motor_status(response[2], status);
    }
    return result;
}

/*
 * 获取 PID 参数
 * 从驱动器读取当前的 Kp、Ki、Kd 值，各为 32 位无符号整数。
 * 这些值经过驱动器内部标定，具体物理意义取决于驱动器固件。
 */
EmmStatus emm_get_pid(EmmDevice *device, EmmPIDParams *params)
{
    uint8_t response[15];
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = send_read(device, EMM_CODE_GET_PID, response, sizeof(response));
    if (result == EMM_OK)
    {
        params->kp = read_u32_be(&response[2]);
        params->ki = read_u32_be(&response[6]);
        params->kd = read_u32_be(&response[10]);
    }
    return result;
}

/*
 * 获取完整配置参数
 *
 * 此命令使用可变长度帧（dynamic frame），因为配置数据较多（28 字节）。
 * 发送 EMM_CODE_GET_CONFIG + EMM_PROTOCOL_GET_CONFIG 命令后，
 * 响应帧的第 4 字节开始为配置数据，通过 decode_config 解析。
 *
 * 注意：不同于简单读取命令（只用 CODE），获取配置命令同时需要
 * CODE 和 PROTOCOL 字段，因为此命令由驱动器固件的特定处理函数响应。
 */
EmmStatus emm_get_config(EmmDevice *device, EmmConfigParams *params)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_GET_CONFIG, EMM_PROTOCOL_GET_CONFIG };
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_send_raw_dynamic(device, body, sizeof(body), response, sizeof(response), &response_length);
    if (result == EMM_OK && response_length >= 33u)
    {
        decode_config(&response[4], params);
    }
    return result;
}

/*
 * 获取系统状态（一次获取所有运行参数）
 *
 * 这是最全面的状态查询命令，一帧响应中包含：总线电压、相电流、
 * 编码器值、目标位置、实时速度、实时位置、位置误差、归零状态
 * 和电机状态等全部运行参数。
 *
 * 响应为可变长度帧，数据从 response[4] 开始（前 4 字节为帧头），
 * 至少需要 31 字节的完整数据。
 *
 * 注意：此命令也需要 CODE + PROTOCOL 双重验证，因为返回数据量大、
 * 格式复杂，防止误解析。
 */
EmmStatus emm_get_system_status(EmmDevice *device, EmmSystemStatusParams *params)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[3] = { device->address, EMM_CODE_GET_SYS_STATUS, EMM_PROTOCOL_GET_SYS_STATUS };
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    const uint8_t *data;
    if (params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    result = emm_send_raw_dynamic(device, body, sizeof(body), response, sizeof(response), &response_length);
    if (result == EMM_OK && response_length >= 31u)
    {
        data = &response[4];
        params->bus_voltage_mv = read_u16_be(&data[0]);
        params->phase_current_ma = read_u16_be(&data[2]);
        params->encoder_value = read_u16_be(&data[4]);
        params->target_position = read_signed_prefix(&data[6], 5u);
        params->realtime_speed_rpm = (int16_t)(((data[11] == 1u) ? -1 : 1) * (int16_t)read_u16_be(&data[12]));
        params->realtime_position = read_signed_prefix(&data[14], 5u);
        params->position_error = read_signed_prefix(&data[19], 5u);
        parse_homing_status(data[24], &params->homing_status);
        parse_motor_status(data[25], &params->motor_status);
    }
    return result;
}

/*
 * 设置驱动器通信 ID
 * 成功设置后，device->address 也会同步更新，确保后续通信使用新地址。
 * new_id - 新地址（不能为 0，0 是广播地址）
 * store  - 是否保存到 flash
 */
EmmStatus emm_set_id(EmmDevice *device, uint8_t new_id, EmmStoreFlag store)
{
    uint8_t body[5];
    EmmStatus result;
    if (device == 0 || new_id == 0u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_ID;
    body[2] = EMM_PROTOCOL_SET_ID;
    body[3] = (uint8_t)store;
    body[4] = new_id;
    result = send_simple(device, body, sizeof(body));
    if (result == EMM_OK)
    {
        device->address = new_id;
    }
    return result;
}

/*
 * 设置电机细分
 * microstep - 细分值（1-256），256 时编码为 0 发送
 * store     - 是否保存到 flash
 * 注意：细分值改变后，角度-脉冲换算关系也相应变化。
 */
EmmStatus emm_set_microstep(EmmDevice *device, uint16_t microstep, EmmStoreFlag store)
{
    uint8_t body[5];
    if (device == 0 || microstep == 0u || microstep > 256u)
    {
        return EMM_ERROR_INVALID_ARG;
    }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_MICROSTEP;
    body[2] = EMM_PROTOCOL_SET_MICROSTEP;
    body[3] = (uint8_t)store;
    body[4] = (microstep == 256u) ? 0u : (uint8_t)microstep;
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置控制模式（开环/闭环）
 * mode - EMM_CONTROL_OPEN_LOOP 或 EMM_CONTROL_CLOSED_LOOP
 * store - 是否保存到 flash
 */
EmmStatus emm_set_loop_mode(EmmDevice *device, EmmControlMode mode, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_LOOP_MODE, EMM_PROTOCOL_SET_LOOP_MODE, (uint8_t)store, (uint8_t)mode };
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置开环模式电流（单位: mA）
 * 开环模式下电机相电流固定为此值。最大 5000mA。
 * current_ma - 电流值（0-5000mA）
 * store      - 是否保存到 flash
 */
EmmStatus emm_set_open_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_OPEN_LOOP_CURRENT, EMM_PROTOCOL_SET_OPEN_LOOP_CURRENT, (uint8_t)store, 0u, 0u };
    if (current_ma > 5000u) { return EMM_ERROR_INVALID_ARG; }
    write_u16_be(&body[4], current_ma);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置闭环模式电流（单位: mA）
 * 闭环模式下，驱动器会根据负载自动调节电流，但峰值不超过此设定值。
 * current_ma - 电流值（0-5000mA）
 * store      - 是否保存到 flash
 */
EmmStatus emm_set_closed_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_CLOSED_LOOP_CURRENT, EMM_PROTOCOL_SET_CLOSED_LOOP_CURRENT, (uint8_t)store, 0u, 0u };
    if (current_ma > 5000u) { return EMM_ERROR_INVALID_ARG; }
    write_u16_be(&body[4], current_ma);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置 PID 参数
 * 将 Kp、Ki、Kd 值写入驱动器，影响闭环控制的响应特性。
 * params - PID 参数（kp、ki、kd 各为 32 位）
 * store  - 是否保存到 flash
 */
EmmStatus emm_set_pid(EmmDevice *device, const EmmPIDParams *params, EmmStoreFlag store)
{
    uint8_t body[16];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_PID;
    body[2] = EMM_PROTOCOL_SET_PID;
    body[3] = (uint8_t)store;
    write_u32_be(&body[4], params->kp);
    write_u32_be(&body[8], params->ki);
    write_u32_be(&body[12], params->kd);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置电机运行方向（正转/反转极性）
 * direction - CW 或 CCW
 * store     - 是否保存到 flash
 */
EmmStatus emm_set_motor_direction(EmmDevice *device, EmmDirection direction, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_MOTOR_DIRECTION, EMM_PROTOCOL_SET_MOTOR_DIRECTION, (uint8_t)store, (uint8_t)direction };
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置位置到位窗口（角度）
 * 当实际位置与目标位置的差值小于此窗口时，驱动器认为"到位"。
 * window_deg - 窗口角度（0-6553.5 度），精度 0.1 度
 * store      - 是否保存到 flash
 */
EmmStatus emm_set_position_window(EmmDevice *device, float window_deg, EmmStoreFlag store)
{
    if (device == 0 || window_deg != window_deg || window_deg < 0.0f ||
        window_deg > 6553.5f)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[6] = { device->address, EMM_CODE_SET_POSITION_WINDOW, EMM_PROTOCOL_SET_POSITION_WINDOW, (uint8_t)store, 0u, 0u };
    uint16_t window = (uint16_t)(window_deg * 10.0f);
    write_u16_be(&body[4], window);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置心跳超时时间
 * 驱动器在此时间内未收到任何命令则自动禁能电机（安全功能）。
 * time_ms - 超时时间（毫秒）
 * store   - 是否保存到 flash
 */
EmmStatus emm_set_heartbeat_time(EmmDevice *device, uint32_t time_ms, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[8] = { device->address, EMM_CODE_SET_HEARTBEAT_TIME, EMM_PROTOCOL_SET_HEARTBEAT_TIME, (uint8_t)store, 0u, 0u, 0u, 0u };
    write_u32_be(&body[4], time_ms);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置自动运行模式
 * 驱动器在使能后按预设参数自动运行（无需上位机持续发送运动命令）。
 * params - 自动运行参数（方向、速度、加速度、使能控制等）
 */
EmmStatus emm_set_auto_run(EmmDevice *device, const EmmAutoRunParams *params)
{
    uint8_t body[9];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_AUTO_RUN;
    body[2] = EMM_PROTOCOL_SET_AUTO_RUN;
    body[3] = params->store ? 1u : 0u;
    body[4] = (uint8_t)params->direction;
    write_u16_be(&body[5], params->speed_rpm);
    body[7] = params->acceleration;
    body[8] = params->enable_en_control ? 1u : 0u;
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置完整配置参数
 * 将 EmmConfigParams 中的所有配置项一次性写入驱动器。
 * 这是最复杂的配置命令，包含 28 字节的配置数据。
 * params - 完整配置参数
 * store  - 是否保存到 flash
 */
EmmStatus emm_set_config(EmmDevice *device, const EmmConfigParams *params, EmmStoreFlag store)
{
    uint8_t body[32];
    if (device == 0 || params == 0) { return EMM_ERROR_INVALID_ARG; }
    body[0] = device->address;
    body[1] = EMM_CODE_SET_CONFIG;
    body[2] = EMM_PROTOCOL_SET_CONFIG;
    body[3] = (uint8_t)store;
    encode_config(&body[4], params);
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置模拟量缩放输入使能
 * enable - true 使能模拟量输入（可通过电位器控制速度/位置）
 * store  - 是否保存到 flash
 */
EmmStatus emm_set_scale_input(EmmDevice *device, bool enable, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_SCALE_INPUT, EMM_PROTOCOL_SET_SCALE_INPUT, (uint8_t)store, enable ? 1u : 0u };
    return send_simple(device, body, sizeof(body));
}

/*
 * 设置面板按键锁定
 * lock  - true 锁定驱动器面板按键，false 解锁
 * store - 是否保存到 flash
 */
EmmStatus emm_set_lock_button(EmmDevice *device, bool lock, EmmStoreFlag store)
{
    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    uint8_t body[5] = { device->address, EMM_CODE_SET_LOCK_BUTTON, EMM_PROTOCOL_SET_LOCK_BUTTON, (uint8_t)store, lock ? 1u : 0u };
    return send_simple(device, body, sizeof(body));
}

/*
 * 广播查询驱动器 ID
 *
 * 使用广播地址（0x00）发送 EMM_CODE_BROADCAST_GET_ID，总线上所有
 * 驱动器都会回复自己的 ID。注意：如果总线上有多个驱动器，可能发生
 * 响应冲突。此操作适合单台设备或调试场景。
 *
 * motor_id - 输出参数，存储返回的驱动器 ID
 */
EmmStatus emm_broadcast_get_id(EmmDevice *device, uint8_t *motor_id)
{
    uint8_t body[2] = { EMM_STEPPER_BROADCAST_ADDRESS, EMM_CODE_BROADCAST_GET_ID };
    uint8_t response[4];
    EmmStatus result;
    if (motor_id == 0) { return EMM_ERROR_INVALID_ARG; }
    result = emm_send_raw(device, body, sizeof(body), response, sizeof(response));
    if (result == EMM_OK)
    {
        *motor_id = response[2];
    }
    return result;
}

/*
 * ============================================================================
 * 强制读取机制 (Forced-Read)
 *
 * 背景：
 *   EMM 驱动器在 EMM_RESPONSE_NONE（火灾-遗忘）模式下工作时，发送命令后
 *   不会自动回复确认帧。但某些操作（如查询实时位置、系统状态）需要读取
 *   驱动器返回的数据。此时需要使用"强制读取"机制。
 *
 * 核心问题：半双工 UART 回显 (Echo)
 *   在 RS485 或单线半双工 UART 总线上，MCU 发送的数据会通过收发器的
 *   环路反馈被自身接收。这意味着：
 *   1. MCU 发送命令帧 -> 驱动器接收 -> 驱动器回复
 *   2. MCU 发送的数据也会被自己的 UART 接收（回显 Echo）
 *   3. 因此 RX 缓冲区中先出现自己发送的 Echo，后出现驱动器的真实响应
 *
 * 解决方案：
 *   1. 临时将 device->response_mode 改为 EMM_RESPONSE_RECEIVE
 *   2. 发送命令并等待一段时间（10ms），让回显和响应都到达
 *   3. 将所有原始数据读取到 raw[] 缓冲区
 *   4. 在 raw[] 中搜索：
 *      a. 跳过 Echo 部分（格式：[地址][功能码][固定校验值 0x6B]）
 *      b. 在 Echo 之后查找符合（地址 + 功能码 + 校验和）的真实响应帧
 *   5. 找到后复制到 response 并恢复 response_mode
 *
 * Echo 和真实响应的区别：
 *   - Echo 的校验和始终是 EMM_STATUS_FIXED_CHECKSUM (0x6B)
 *   - 真实响应的校验和根据 checksum_mode 计算（可能是 XOR 或 CRC8）
 *   - 通过校验和验证可以区分 Echo 和真实响应
 *
 * 设计决策：
 *   使用 transport.read 直接读取原始字节到 raw[] 缓冲区，而不是通过
 *   环形缓冲区（rx_buffer）中转。这是因为 Echo 不在协议预期的数据之列，
 *   通过环形缓冲区会增加解析复杂度。
 * ============================================================================
 */

static EmmStatus emm_forced_read(EmmDevice *device,
                                 const uint8_t *body, size_t body_length,
                                 uint8_t *response, size_t response_length)
{
    EmmResponseMode saved_mode;
    uint8_t raw[64];
    size_t raw_len;
    size_t echo_len;
    size_t i;
    uint32_t waited_ms;
    uint8_t expected_addr;
    uint8_t expected_code;

    if (device == 0 || body == 0 || response == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    saved_mode = device->response_mode;
    device->response_mode = EMM_RESPONSE_RECEIVE;

    /* Flush stale RX data. */
    if (device->auto_flush_before_read)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }

    /* 发送命令帧。在半双工总线上，MCU 会收到自己发出的数据（回显 Echo）。
     * Echo 将先于驱动器的真实响应到达 RX 缓冲区。 */
    {
        EmmStatus ws = emm_write_body_once(device, body, body_length);
        if (ws != EMM_OK)
        {
            device->response_mode = saved_mode;
            return ws;
        }
    }

    expected_addr = body[0];
    expected_code = body[1];
    echo_len     = body_length + 1u;  /* 回显长度 = 命令体 + 校验和 */

    /* 等待回显和响应数据到达总线。10ms 的延迟足够 EMM 驱动器处理命令
     * 并生成响应（通常在 1-5ms 内）。 */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(10u, device->transport.user_data);
    }

    /* 从 UART 读取所有可用原始字节到 raw[] 缓冲区。
     * raw[] 中将包含：回显(Echo) + 真实响应(Response)
     * 每次读取使用 5ms 超时，循环直到超时或收到足够数据。 */
    raw_len = 0u;
    waited_ms = 0u;
    while (waited_ms < device->timeout_ms)
    {
        size_t n;
        if (device->transport.read == 0) { break; }
        n = device->transport.read(&raw[raw_len],
                    sizeof(raw) - raw_len,
                    5u,  /* 每次轮询使用短超时 */
                    device->transport.user_data);
        if (n > 0u) { raw_len += n; }
        if (raw_len >= echo_len + response_length) { break; }
        if (device->transport.delay_ms != 0)
        {
            device->transport.delay_ms(1u, device->transport.user_data);
        }
        waited_ms += 6u;  /* 5ms 读取超时 + 1ms 延迟 */
    }

    /* 在 raw[] 缓冲区中搜索真实响应（跳过早先到达的回显数据）。
     *
     * 回显的特征：
     *   - 格式：[地址(1)] [功能码(1)] [数据(N)] [校验和(1)]
     *   - 校验和始终为 EMM_STATUS_FIXED_CHECKSUM (0x6B)
     *     因为回显是 MCU 发出的原始数据，校验方式可能为 FIXED
     *   - 回显长度 = 命令体长度 + 1（校验和）
     *
     * 我们通过以下方式识别回显：
     *   1. 检查 raw[0..2] 是否匹配 [地址, 功能码, 0x6B] 模式
     *   2. 如果匹配，跳过 echo_len 字节开始搜索真实响应
     *   3. 真实响应通过校验和验证（可能使用 XOR 或 CRC8）区分于回显 */
    {
        size_t search_start = 0u;

        /* 检测并跳过回显：检查缓冲区开头是否匹配回显模式 */
        if (raw_len >= echo_len
            && raw[0] == expected_addr
            && raw[1] == expected_code
            && raw[2] == EMM_STATUS_FIXED_CHECKSUM)
        {
            search_start = echo_len;
        }

        /* 从跳过后回显的位置开始扫描，寻找真实的响应帧。
         * 响应帧的地址和功能码必须匹配，且校验和正确。 */
        for (i = search_start; i + response_length <= raw_len; i++)
        {
            if (raw[i] != expected_addr) { continue; }
            if (raw[i + 1u] != expected_code) { continue; }
            if (raw[i + response_length - 1u] !=
                emm_calculate_checksum(&raw[i], response_length - 1u,
                    device->checksum_mode))
            {
                continue;
            }
            /* 找到真实的响应帧，复制到 response 缓冲区 */
        for (size_t j = 0u; j < response_length; j++)
        {
            response[j] = raw[i + j];
        }
        device->response_mode = saved_mode;
        return EMM_OK;
    }

    device->response_mode = saved_mode;
    return EMM_ERROR_TIMEOUT;
}

}

/*
 * 强制读取实时位置（度）
 * 在 EMM_RESPONSE_NONE 模式下，使用 emm_forced_read 机制强制获取位置数据。
 * 适用于需要周期性位置反馈但对实时性要求不高的场景。
 */
EmmStatus emm_get_realtime_position_forced(EmmDevice *device, float *degrees)
{
    uint8_t response[8];
    EmmStatus result;

    if (device == 0 || degrees == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_REALTIME_POSITION;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *degrees = ((float)read_signed_prefix(&response[2], 5u) * 360.0f) / 65536.0f;
    }
    return result;
}

/*
 * 强制读取编码器值
 */
EmmStatus emm_get_encoder_forced(EmmDevice *device, uint16_t *encoder)
{
    uint8_t response[5];
    EmmStatus result;

    if (device == 0 || encoder == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_ENCODER;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *encoder = read_u16_be(&response[2]);
    }
    return result;
}

/*
 * 强制读取电机状态
 */
EmmStatus emm_get_motor_status_forced(EmmDevice *device, EmmMotorStatus *status)
{
    uint8_t response[4];
    EmmStatus result;

    if (device == 0 || status == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_MOTOR_STATUS;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        parse_motor_status(response[2], status);
    }
    return result;
}

/*
 * 强制读取系统状态
 *
 * 注意：不同于其他 forced 函数，此函数没有使用 emm_forced_read 直接操作
 * raw 缓冲区，而是通过临时切换 response_mode 为 EMM_RESPONSE_RECEIVE 后
 * 调用 emm_send_raw_dynamic 实现。这是因为系统状态返回的是可变长度帧，
 * emm_forced_read 目前只支持固定长度帧。
 *
 * 临时修改 response_mode 后，emm_send_raw_dynamic 会等待驱动器响应，
 * 完成后恢复原始模式。
 */
EmmStatus emm_get_system_status_forced(EmmDevice *device, EmmSystemStatusParams *params)
{
    uint8_t body[3];
    uint8_t response[40];
    size_t response_length = 0u;
    EmmStatus result;
    EmmResponseMode saved_mode;
    const uint8_t *data;

    if (device == 0 || params == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    body[0] = device->address;
    body[1] = EMM_CODE_GET_SYS_STATUS;
    body[2] = EMM_PROTOCOL_GET_SYS_STATUS;

    /* System status uses dynamic-length frame; save/restore response_mode. */
    saved_mode = device->response_mode;
    device->response_mode = EMM_RESPONSE_RECEIVE;

    if (device->auto_flush_before_read)
    {
        emm_rx_clear(device);
        if (device->transport.flush_input != 0)
        {
            device->transport.flush_input(device->transport.user_data);
        }
    }

    result = emm_send_raw_dynamic(device, body, sizeof(body),
                                  response, sizeof(response), &response_length);

    device->response_mode = saved_mode;

    if (result == EMM_OK && response_length >= 31u)
    {
        data = &response[4];
        params->bus_voltage_mv = read_u16_be(&data[0]);
        params->phase_current_ma = read_u16_be(&data[2]);
        params->encoder_value = read_u16_be(&data[4]);
        params->target_position = read_signed_prefix(&data[6], 5u);
        params->realtime_speed_rpm = (int16_t)(((data[11] == 1u) ? -1 : 1) * (int16_t)read_u16_be(&data[12]));
        params->realtime_position = read_signed_prefix(&data[14], 5u);
        params->position_error = read_signed_prefix(&data[19], 5u);
        parse_homing_status(data[24], &params->homing_status);
        parse_motor_status(data[25], &params->motor_status);
    }
    return result;
}

/*
 * 强制读取脉冲计数值
 */
EmmStatus emm_get_pulse_count_forced(EmmDevice *device, int32_t *pulse_count)
{
    uint8_t response[8];
    EmmStatus result;

    if (device == 0 || pulse_count == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    {
        uint8_t body[2];
        body[0] = device->address;
        body[1] = EMM_CODE_GET_PULSE_COUNT;
        result = emm_forced_read(device, body, sizeof(body), response, sizeof(response));
    }

    if (result == EMM_OK)
    {
        *pulse_count = read_signed_prefix(&response[2], 5u);
    }
    return result;
}

/*
 * ============================================================================
 * 校准辅助函数
 *
 * 这些函数用于电机的校准和故障恢复流程。主要应用场景：
 * 1. 上电后的自动校准（编码器归零）
 * 2. 运行中的堵转恢复（stall recovery）
 * 3. 保护触发后的状态清除和重新使能
 *
 * 校准流程通常遵循：停止 -> 清除保护 -> 验证状态 -> 重新使能 -> 验证使能
 * ============================================================================
 */

/*
 * 点动运行（不等待响应）
 * 与 emm_jog 功能相同，但始终使用火灾-遗忘模式发送。
 * 适用于需要连续快速发送运动指令的场景（如手动调试面板）。
 */
EmmStatus emm_jog_no_response(EmmDevice *device, const EmmJogParams *params)
{
    uint8_t body[7];

    if (device == 0 || params == 0 || params->speed_rpm > 3000u)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    body[0] = device->address;
    body[1] = EMM_CODE_JOG;
    body[2] = (uint8_t)params->direction;
    write_u16_be(&body[3], params->speed_rpm);
    body[5] = params->acceleration;
    body[6] = (uint8_t)params->sync_flag;

    return emm_send_raw_no_response(device, body, sizeof(body));
}

/*
 * 清除堵转/过流/过温保护状态并恢复电机正常运行
 *
 * 这是本文件中最复杂的高层函数，实现了完整的故障恢复流程：
 *
 * 恢复流程（共 6 步）：
 *   1. 读取系统状态，确认是否真的发生了堵转或保护
 *   2. 发送急停命令并验证电机确实停转（emm_stop_verified）
 *   3. 等待 50ms 让电机稳定
 *   4. 发送清除保护命令 (EMM_CODE_CLEAR_PROTECTION)
 *   5. 等待 50ms 让驱动器处理清除
 *   6. 重新使能电机 (EMM_CODE_ENABLE)
 *   7. 等待 50ms 使使能生效
 *   8. 最终验证：检查电机已使能、无堵转、无过温过流
 *
 * 每一步都有状态验证，任何一步失败都会立即返回错误码。
 * 每一步之间插入 50ms 延迟，确保驱动器有足够时间处理命令。
 *
 * 设计决策：
 * - 使用 emm_send_raw_no_response 而非 send_simple，因为在故障恢复
 *   场景中，驱动器的响应模式可能不正常，使用火灾-遗忘模式更可靠
 * - 使能后立即验证状态，确保恢复真正生效
 * - 如果最终使能不成功，不自动重试以防止电机在故障状态下反复尝试
 *
 * 适用场景：
 * - 电机因堵转触发保护后自动恢复
 * - 过流/过温保护解除后的状态清理
 * - 系统上电后的故障状态清除
 *
 * device - EMM 设备句柄
 * 返回: EMM_OK 表示成功恢复，否则返回具体的错误码
 */
EmmStatus emm_clear_stall_and_recover(EmmDevice *device)
{
    EmmStatus status;
    EmmSystemStatusParams system_status;

    if (device == 0)
    {
        return EMM_ERROR_INVALID_ARG;
    }

    /* 步骤 1: 读取当前系统状态，确认确实存在堵转或保护触发 */
    status = emm_get_system_status_forced(device, &system_status);
    if (status != EMM_OK || system_status.homing_status.over_temp ||
        system_status.homing_status.over_current ||
        (!system_status.motor_status.stall_detected &&
         !system_status.motor_status.stall_protected))
    {
        return (status == EMM_OK) ? EMM_ERROR_BAD_RESPONSE : status;
    }

    /* 步骤 2: 急停电机并验证停转 */
    status = emm_stop_verified(device, EMM_SYNC_IMMEDIATE);
    if (status != EMM_OK)
    {
        return status;
    }

    /* 步骤 3: 等待电机机械稳定 */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    /* 步骤 4: 发送清除保护命令 */
    {
        uint8_t body[3];
        body[0] = device->address;
        body[1] = EMM_CODE_CLEAR_PROTECTION;
        body[2] = EMM_PROTOCOL_CLEAR_PROTECTION;

        if (device->auto_flush_before_read)
        {
            emm_rx_clear(device);
            if (device->transport.flush_input != 0)
            {
                device->transport.flush_input(device->transport.user_data);
            }
        }
        status = emm_send_raw_no_response(device, body, sizeof(body));
    }

    if (status != EMM_OK)
    {
        return status;
    }

    /* 步骤 5: 等待保护状态清除 */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    /* 验证清除是否成功 */
    status = emm_get_system_status_forced(device, &system_status);
    if (status != EMM_OK || system_status.motor_status.stall_detected ||
        system_status.motor_status.stall_protected ||
        system_status.homing_status.over_temp ||
        system_status.homing_status.over_current)
    {
        return (status == EMM_OK) ? EMM_ERROR_BAD_RESPONSE : status;
    }

    /* 步骤 6: 重新使能电机 */
    {
        uint8_t body[5];
        body[0] = device->address;
        body[1] = EMM_CODE_ENABLE;
        body[2] = EMM_PROTOCOL_ENABLE;
        body[3] = 1u;  /* enable = 1 */
        body[4] = (uint8_t)EMM_SYNC_IMMEDIATE;
        status = emm_send_raw_no_response(device, body, sizeof(body));
        if (status != EMM_OK)
        {
            return status;
        }
    }

    /* 步骤 7: 等待使能生效 */
    if (device->transport.delay_ms != 0)
    {
        device->transport.delay_ms(50u, device->transport.user_data);
    }

    /* 步骤 8: 最终验证 - 确认电机已使能且无故障 */
    status = emm_get_system_status_forced(device, &system_status);
    if (status != EMM_OK || !system_status.motor_status.enabled ||
        system_status.motor_status.stall_detected ||
        system_status.motor_status.stall_protected ||
        system_status.homing_status.over_temp ||
        system_status.homing_status.over_current)
    {
        return (status == EMM_OK) ? EMM_ERROR_BAD_RESPONSE : status;
    }

    return EMM_OK;
}
