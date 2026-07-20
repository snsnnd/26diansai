#ifndef EMM_STEPPER_H
#define EMM_STEPPER_H

/*
 * ============================================================================
 * 文件名称: emm_stepper.h
 * 功能描述: EMM系列步进电机驱动器的通信协议层
 *
 * 概要:
 *   本文件实现了与EMM系列智能步进电机驱动器的通信协议。EMM步进电机
 *   使用改进的串行异步通信协议，支持多种校验模式(固定值、XOR、CRC8等)，
 *   可配置响应模式(无响应、接收、到达、两者皆有)以实现灵活的总线管理。
 *
 *   主要功能:
 *   - 多点总线通信(通过地址区分多个电机)
 *   - 位置/速度/力矩控制命令
 *   - 编码器校准与归零
 *   - 归零(Homing)操作(多种模式)
 *   - 参数配置(细分、电流、PID等)
 *   - 堵转检测与保护
 *   - 状态读取(电压、电流、温度、位置等)
 *
 *   协议特点：
 *   每个命令帧结构: [地址(1byte)] [功能码(1byte)] [参数(n bytes)] [校验(1byte)]
 *   每个响应帧结构: [地址(1byte)] [功能码(1byte)] [状态(1byte)] [校验(1byte)]
 *   动态长度帧: 在静态帧基础上增加了长度字段
 *
 *   注意: 本协议层不依赖于特定物理层(支持UART或CAN)，通过EmmTransport
 *         接口抽象实现底层IO。
 * ============================================================================
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ========================== 通信协议常量定义 ==========================
 * 这些常量定义了EMM步进电机驱动器的默认通信参数和行为控制。
 * 用户可以根据实际硬件配置进行调整。
 */

/* 默认波特率: 115200 bps - EMM驱动器的标准通信速率 */
#define EMM_STEPPER_DEFAULT_BAUDRATE 115200u
/* 默认电机地址: 1 - 在多点总线中唯一标识每个电机 */
#define EMM_STEPPER_DEFAULT_ADDRESS 1u
/* 广播地址: 0 - 发送给总线上的所有电机，不做响应 */
#define EMM_STEPPER_BROADCAST_ADDRESS 0u
/* 最大重试次数: 3 - 通信失败时的自动重试次数 */
#define EMM_STEPPER_MAX_RETRIES 3u
/* 最大帧长度: 64字节 - 单次通信帧的最大数据量 */
#define EMM_STEPPER_MAX_FRAME_SIZE 64u
/* 接收缓冲区大小: 256字节 - 用于缓存从电机接收的数据 */
#define EMM_STEPPER_RX_BUFFER_SIZE 256u
/* 每次读取块大小: 32字节 - 轮询时一次性读取的数据量 */
#define EMM_STEPPER_RX_READ_CHUNK 32u
/* 默认命令超时: 20ms - 等待电机响应的最长时间 */
#define EMM_STEPPER_DEFAULT_TIMEOUT_MS 20u
/* 到达判断超时: 3000ms - 等待电机到达目标位置的最长时间 */
#define EMM_STEPPER_REACHED_TIMEOUT_MS 3000u
/* 轮询尝试次数: 4次 - 每次读取操作中尝试轮询的次数 */
#define EMM_STEPPER_POLL_ATTEMPTS 4u
/* 通配匹配符: 0xFF - 用于地址/功能码匹配时表示"任意值" */
#define EMM_STEPPER_MATCH_ANY 0xFFu

/*
 * EMM操作状态枚举
 * 所有API函数均返回此枚举值以指示操作结果。
 * 正值(EMM_OK)表示成功，负值表示各种错误类型。
 */
typedef enum
{
    EMM_OK = 0,                 /* 操作成功 */
    EMM_ERROR = -1,             /* 通用错误 */
    EMM_ERROR_INVALID_ARG = -2, /* 参数无效(如空指针、越界值) */
    EMM_ERROR_IO = -3,          /* IO操作错误(写入/读取失败) */
    EMM_ERROR_TIMEOUT = -4,     /* 通信超时(无响应帧返回) */
    EMM_ERROR_BAD_RESPONSE = -5,/* 响应数据无法解析或内容异常 */
    EMM_ERROR_CHECKSUM = -6,    /* 校验和不匹配(数据可能损坏) */
    EMM_ERROR_PARAM = -7,       /* 电机返回参数错误(命令参数无效) */
    EMM_ERROR_FORMAT = -8,      /* 电机返回格式错误(命令格式不正确) */
    EMM_ERROR_OVERFLOW = -9,    /* 接收缓冲区溢出(数据丢失) */
    EMM_ERROR_NO_RESPONSE = -10,/* 设备配置为无响应模式，无法读取 */
} EmmStatus;

/*
 * 校验和模式枚举
 * EMM协议支持多种校验方式，用于保证数据完整性。
 * 固定校验适用于简单场景，XOR/CRC8/Modbus/DMX512用于更高可靠性要求。
 */
typedef enum
{
    EMM_CHECKSUM_FIXED = 0,   /* 固定校验值 0x6B(最简单，无实际校验) */
    EMM_CHECKSUM_XOR = 1,     /* XOR异或校验(逐字节异或) */
    EMM_CHECKSUM_CRC8 = 2,    /* CRC8校验(使用查表法，多项式x^8+x^5+x^4+1) */
    EMM_CHECKSUM_MODBUS = 3,  /* Modbus CRC16校验 */
    EMM_CHECKSUM_DMX512 = 4,  /* DMX512协议校验 */
} EmmChecksumMode;

/*
 * 电机旋转方向枚举
 * CW = Clockwise (顺时针)
 * CCW = Counter-Clockwise (逆时针)
 */
typedef enum
{
    EMM_DIRECTION_CW = 0,      /* 顺时针方向 */
    EMM_DIRECTION_CCW = 1,     /* 逆时针方向 */
} EmmDirection;

/*
 * 同步标志枚举
 * 控制运动命令是立即执行还是等待sync信号同步执行。
 * 对于多轴协调运动，使用缓冲模式先装载所有轴的运动参数，
 * 再通过sync命令同时启动。
 */
typedef enum
{
    EMM_SYNC_IMMEDIATE = 0,    /* 立即执行，命令发出后电机马上开始运动 */
    EMM_SYNC_BUFFERED = 1,     /* 缓冲模式，等待sync命令触发后再执行 */
} EmmSyncFlag;

/*
 * 参数存储标志枚举
 * 控制配置参数是否写入电机的非易失性存储器(EEPROM/Flash)中。
 * 存储到Flash会写入参数，重启后保持；不存储则仅临时修改。
 * 注意：Flash写入寿命有限，频繁写入会缩短电机驱动器寿命。
 */
typedef enum
{
    EMM_STORE_NO = 0,          /* 不存储(临时修改，断电丢失) */
    EMM_STORE_YES = 1,         /* 存储到Flash(永久保存，但注意写入寿命) */
} EmmStoreFlag;

/*
 * 运动模式枚举
 * 控制电机的位置运动是绝对定位还是相对定位。
 */
typedef enum
{
    EMM_MOTION_RELATIVE_LAST = 0,  /* 相对上一次目标位置 */
    EMM_MOTION_ABSOLUTE = 1,       /* 绝对位置(相对于0点) */
    EMM_MOTION_RELATIVE_CURRENT = 2,/* 相对当前位置 */
} EmmMotionMode;

/*
 * 归零(Homing)模式枚举
 * 不同的归零策略适应不同的机械结构和传感器配置。
 * NEAREST和DIRECTION使用霍尔传感器，COLLISION使用碰撞检测，
 * LIMIT_SWITCH使用限位开关，ABS_ZERO使用编码器绝对零位。
 */
typedef enum
{
    EMM_HOME_NEAREST = 0,         /* 归到最近的传感器位置 */
    EMM_HOME_DIRECTION = 1,       /* 按指定方向归零 */
    EMM_HOME_COLLISION = 2,       /* 碰撞检测归零(撞到机械限位后回退) */
    EMM_HOME_LIMIT_SWITCH = 3,    /* 使用外部限位开关归零 */
    EMM_HOME_ABS_ZERO = 4,        /* 使用编码器绝对零位 */
    EMM_HOME_LAST_POWER_OFF = 5,  /* 归到上次断电时的位置 */
} EmmHomingMode;

/*
 * 控制模式枚举
 * 开环控制无反馈(适用于负载恒定的场景)，闭环控制使用编码器反馈提高精度。
 */
typedef enum
{
    EMM_CONTROL_OPEN_LOOP = 0,   /* 开环控制(无位置反馈) */
    EMM_CONTROL_CLOSED_LOOP = 1, /* 闭环控制(有编码器反馈) */
} EmmControlMode;

/*
 * 电机类型枚举
 * 0.9度步进角电机每转400步，1.8度步进角电机每转200步。
 * 步进角越小，同样细分数下分辨率越高。
 */
typedef enum
{
    EMM_MOTOR_09_DEG = 0x19,     /* 0.9度/步(高精度，每转400步) */
    EMM_MOTOR_18_DEG = 0x32,     /* 1.8度/步(标准，每转200步) */
} EmmMotorType;

/*
 * 固件类型枚举
 */
typedef enum
{
    EMM_FIRMWARE_X = 0,          /* X系列固件(基础版) */
    EMM_FIRMWARE_EMM = 1,        /* EMM标准固件 */
    EMM_FIRMWARE_EMM_TURBO = 2,  /* EMM Turbo固件(高性能版) */
} EmmFirmwareType;

/*
 * 波特率枚举
 * 对应到电机驱动器中固件定义的波特率索引值。
 * 注意：更改波特率后需确保主机与电机的波特率设置一致。
 */
typedef enum
{
    EMM_BAUD_9600 = 0,       /* 9600 bps */
    EMM_BAUD_19200 = 1,      /* 19200 bps */
    EMM_BAUD_25000 = 2,      /* 25000 bps */
    EMM_BAUD_38400 = 3,      /* 38400 bps */
    EMM_BAUD_57600 = 4,      /* 57600 bps */
    EMM_BAUD_115200 = 5,     /* 115200 bps(默认推荐速率) */
    EMM_BAUD_256000 = 6,     /* 256000 bps */
    EMM_BAUD_512000 = 7,     /* 512000 bps(高速) */
    EMM_BAUD_921600 = 8,     /* 921600 bps(最高速) */
} EmmBaudRate;

/*
 * CAN总线速率枚举
 * 仅在使用CAN通信接口时有效。
 */
typedef enum
{
    EMM_CAN_10K = 0,         /* 10 Kbps */
    EMM_CAN_20K = 1,         /* 20 Kbps */
    EMM_CAN_50K = 2,         /* 50 Kbps */
    EMM_CAN_83K = 3,         /* 83.3 Kbps */
    EMM_CAN_100K = 4,        /* 100 Kbps */
    EMM_CAN_125K = 5,        /* 125 Kbps */
    EMM_CAN_250K = 6,        /* 250 Kbps */
    EMM_CAN_500K = 7,        /* 500 Kbps */
    EMM_CAN_800K = 8,        /* 800 Kbps */
    EMM_CAN_1M = 9,          /* 1 Mbps */
} EmmCanRate;

/*
 * 响应模式枚举
 * 控制电机执行命令后的响应行为：
 * NONE(无响应) - 高速模式，适用于批量命令下发
 * RECEIVE(接收) - 收到命令即返回接收确认
 * REACHED(到达) - 到达目标位置后才返回
 * BOTH(两者皆有) - 先返回接收确认，到达后再返回到达确认
 * OTHER(其他) - 自定义响应行为
 */
typedef enum
{
    EMM_RESPONSE_NONE = 0,      /* 无响应模式(高速写，忽略应答) */
    EMM_RESPONSE_RECEIVE = 1,   /* 收到命令即应答 */
    EMM_RESPONSE_REACHED = 2,   /* 到达目标位置后应答 */
    EMM_RESPONSE_BOTH = 3,      /* 收到和到达时都应答 */
    EMM_RESPONSE_OTHER = 4,     /* 其他定制应答模式 */
} EmmResponseMode;

/*
 * 堵转保护模式枚举
 * DISABLE - 关闭堵转检测
 * ENABLE - 检测到堵转时停止电机
 * AUTO_ZERO - 检测到堵转后自动归零并继续
 */
typedef enum
{
    EMM_STALL_DISABLE = 0,      /* 关闭堵转保护 */
    EMM_STALL_ENABLE = 1,       /* 开启堵转保护(堵转时停止) */
    EMM_STALL_AUTO_ZERO = 2,    /* 堵转后自动归零 */
} EmmStallProtect;

/*
 * 脉冲端口模式枚举
 * 用于配置电机的脉冲/方向输入端口功能
 */
typedef enum
{
    EMM_PULSE_PORT_OFF = 0,          /* 关闭脉冲端口 */
    EMM_PULSE_PORT_OPEN = 1,         /* 开环脉冲模式 */
    EMM_PULSE_PORT_FOC = 2,          /* FOC磁场定向控制模式 */
    EMM_PULSE_PORT_ESI_RCO = 3,      /* ESI总线或RCO输出模式 */
    EMM_PULSE_PORT_PLR_ESI = 4,      /* PLR或ESI模式 */
} EmmPulsePortMode;

/*
 * 串口模式枚举
 * 配置电机通信端口的物理层协议
 */
typedef enum
{
    EMM_SERIAL_PORT_OFF = 0,          /* 关闭串口 */
    EMM_SERIAL_PORT_ESI_ALO = 1,      /* ESI总线或模拟量输出 */
    EMM_SERIAL_PORT_UART = 2,         /* UART异步串口通信 */
    EMM_SERIAL_PORT_CAN = 3,          /* CAN总线通信 */
    EMM_SERIAL_PORT_ULR_ESI = 4,      /* ULR或ESI模式 */
} EmmSerialPortMode;

/*
 * 使能电平枚举
 * 定义电机使能信号的有效电平
 */
typedef enum
{
    EMM_ENABLE_LEVEL_LOW = 0,   /* 低电平使能 */
    EMM_ENABLE_LEVEL_HIGH = 1,  /* 高电平使能 */
    EMM_ENABLE_LEVEL_HOLD = 2,  /* 保持当前状态 */
} EmmEnableLevel;

/*
 * 方向电平枚举
 * 定义方向信号电平与旋转方向的对应关系
 */
typedef enum
{
    EMM_DIR_LEVEL_CW = 0,      /* 低电平=顺时针 */
    EMM_DIR_LEVEL_CCW = 1,     /* 高电平=逆时针 */
} EmmDirLevel;

/*
 * IO传输层函数指针类型定义
 * 这些回调函数实现了协议层与物理层之间的解耦。
 * 用户可以基于这些函数指针实现UART、CAN或SPI等不同的物理层通信。
 *
 * EmmWriteFn: 发送数据到电机。返回实际发送的字节数。
 * EmmReadFn: 从电机接收数据。timeout_ms为最长等待时间。
 * EmmFlushFn: 清空输入/输出缓冲区的待处理数据。
 * EmmDelayFn: 毫秒级延时函数(用于重试等待等场景)。
 */
typedef size_t (*EmmWriteFn)(const uint8_t *data, size_t length, void *user_data);
typedef size_t (*EmmReadFn)(uint8_t *data, size_t length, uint32_t timeout_ms, void *user_data);
typedef void (*EmmFlushFn)(void *user_data);
typedef void (*EmmDelayFn)(uint32_t delay_ms, void *user_data);

/*
 * 传输层接口结构体
 *
 * 该结构封装了物理层通信所需的所有函数指针和上下文数据。
 * 通过这种设计，EMM协议栈可以运行在任何物理层之上，
 * 只需填好这些回调函数即可。
 *
 * user_data: 用户自定义上下文指针，在回调函数中传回，
 *            可用于传递UART句柄、设备ID等额外信息。
 */
typedef struct
{
    EmmWriteFn write;             /* 写数据函数指针 */
    EmmReadFn read;               /* 读数据函数指针 */
    EmmFlushFn flush_input;       /* 清空输入缓冲区函数指针 */
    EmmFlushFn flush_output;      /* 清空输出缓冲区函数指针 */
    EmmDelayFn delay_ms;          /* 延时函数指针 */
    void *user_data;              /* 用户数据指针(透传给各回调) */
} EmmTransport;

/*
 * EMM设备实例结构体
 *
 * 该结构体代表一个(或一组)EMM步进电机的通信上下文。
 * 注意：一个EmmDevice实例对应一个物理UART总线。
 * 如果多个电机共享同一条UART总线，只需一个实例；
 * 如果使用多路独立的UART，每路需要一个实例。
 *
 * 关键设计说明：
 * - response_mode设置为EMM_RESPONSE_NONE(默认)可实现"发后即忘"的高速写
 * - forced_read系列函数会临时覆盖response_mode以获取读响应
 * - rx_buffer使用环形缓冲区(FIFO)存储接收数据
 * - auto_flush_before_write/read决定是否在通信前自动清空缓冲区
 */
typedef struct
{
    uint8_t address;                /* 本机通信地址(1~254, 0为广播地址) */
    EmmChecksumMode checksum_mode;  /* 校验和模式 */
    uint32_t timeout_ms;            /* 命令超时时间(ms) */
    uint32_t retry_delay_ms;        /* 重试前延时(ms) */
    uint8_t max_retries;            /* 最大重试次数 */

    EmmTransport transport;         /* 物理层传输接口 */

    /* 注意: 写命令默认使用EMM_RESPONSE_NONE(发后即忘)。
     * 读取命令(emm_get_*_forced)会临时切换到EMM_RESPONSE_RECEIVE
     * 以获取响应，读取完成后恢复原始模式。
     * 当需要读取数据时，每个物理UART总线应使用独立的EmmDevice实例。 */
    uint32_t reached_timeout_ms;    /* 等待"到达"响应的超时时间 */
    EmmResponseMode response_mode;  /* 当前响应模式 */
    bool strict_frame_check;        /* 是否严格检查帧格式(地址+功能码) */
    bool auto_flush_before_write;   /* 写前是否自动清空RX缓冲区 */
    bool auto_flush_before_read;    /* 强制读前是否清空RX缓冲区 */
    uint8_t poll_attempts;          /* 每次读操作的轮询尝试次数 */
    /* 环形接收缓冲区 - 用于暂存从UART读取的原始数据 */
    uint8_t rx_buffer[EMM_STEPPER_RX_BUFFER_SIZE];  /* 环形缓冲区存储空间 */
    size_t rx_head;                 /* 环形缓冲区写指针(生产者位置) */
    size_t rx_tail;                 /* 环形缓冲区读指针(消费者位置) */
    size_t rx_overflow_count;       /* 缓冲区溢出次数统计 */
} EmmDevice;

/*
 * 点动(Jog)参数结构体
 * 用于控制电机持续旋转(无固定目标位置)
 */
typedef struct
{
    EmmDirection direction;      /* 旋转方向(CW顺时针/CCW逆时针) */
    uint16_t speed_rpm;          /* 目标转速, 单位: RPM(转/分钟), 建议≤3000 */
    uint8_t acceleration;        /* 加速度, 值越大加速越快(1~255) */
    EmmSyncFlag sync_flag;       /* 同步标志(立即执行或等待同步) */
} EmmJogParams;

/*
 * 位置运动参数结构体
 * 用于控制电机运动到指定位置
 */
typedef struct
{
    EmmDirection direction;      /* 旋转方向 */
    uint16_t speed_rpm;          /* 目标转速, 单位: RPM */
    uint8_t acceleration;        /* 加速度 */
    uint32_t pulse_count;        /* 脉冲数(每个脉冲对应一个微步) */
    EmmMotionMode motion_mode;   /* 运动模式(相对上一次/绝对/相对当前位置) */
    EmmSyncFlag sync_flag;       /* 同步标志 */
} EmmPositionParams;

/*
 * 归零(Homing)参数结构体
 * 配置电机的归零行为，包括模式、速度、碰撞检测参数等
 */
typedef struct
{
    EmmHomingMode homing_mode;           /* 归零模式选择 */
    EmmDirection homing_direction;       /* 归零搜索方向 */
    uint16_t homing_speed_rpm;           /* 归零搜索速度(RPM) */
    uint32_t homing_timeout_ms;          /* 归零超时时间(ms) */
    uint16_t collision_speed_rpm;        /* 碰撞检测速度(RPM) */
    uint16_t collision_current_ma;       /* 碰撞检测电流(mA) */
    uint16_t collision_time_ms;          /* 碰撞检测时间(ms) */
    bool auto_home;                      /* 是否上电自动归零 */
} EmmHomingParams;

/*
 * 固件版本信息结构体
 */
typedef struct
{
    uint16_t firmware_version;           /* 固件版本号 */
    uint8_t hw_series;                   /* 硬件系列号 */
    uint8_t hw_type;                     /* 硬件类型号 */
    uint8_t hw_version;                  /* 硬件版本号 */
} EmmVersionParams;

/*
 * 电机相电阻/电感参数结构体
 * 用于电机的参数辨识和驱动优化
 */
typedef struct
{
    uint16_t phase_resistance_mohm;      /* 相电阻, 单位: 毫欧(mOhm) */
    uint16_t phase_inductance_uh;        /* 相电感, 单位: 微亨(uH) */
} EmmMotorRHParams;

/*
 * PID控制器参数结构体
 * Kp: 比例系数 - 影响响应速度
 * Ki: 积分系数 - 消除稳态误差
 * Kd: 微分系数 - 抑制超调
 */
typedef struct
{
    uint32_t kp;                         /* 比例增益 */
    uint32_t ki;                         /* 积分增益 */
    uint32_t kd;                         /* 微分增益 */
} EmmPIDParams;

/*
 * 归零状态结构体
 * 反映电机当前的归零进度和状态
 */
typedef struct
{
    bool encoder_ready;                  /* 编码器就绪标志 */
    bool calibrated;                     /* 校准完成标志 */
    bool is_homing;                      /* 正在归零中 */
    bool homing_failed;                  /* 归零失败 */
    bool over_temp;                      /* 过热保护触发 */
    bool over_current;                   /* 过流保护触发 */
} EmmHomingStatus;

/*
 * 电机运行状态结构体
 * 反映电机当前的运行状态和故障标志
 */
typedef struct
{
    bool enabled;                        /* 电机使能状态(是否通电保持) */
    bool position_reached;               /* 是否到达目标位置 */
    bool stall_detected;                 /* 检测到堵转 */
    bool stall_protected;                /* 堵转保护已触发 */
    bool left_limit;                     /* 左限位触发 */
    bool right_limit;                    /* 右限位触发 */
    bool power_off_flag;                 /* 断电标志 */
} EmmMotorStatus;

/*
 * 系统状态参数结构体
 * 电机的综合运行参数，用于监控和调试
 */
typedef struct
{
    uint16_t bus_voltage_mv;             /* 总线电压, 单位: mV */
    uint16_t phase_current_ma;           /* 相电流, 单位: mA */
    uint16_t encoder_value;              /* 编码器原始值(0~65535, 对应0~360度) */
    int32_t target_position;             /* 目标位置(脉冲数) */
    int16_t realtime_speed_rpm;          /* 实时转速, 单位: RPM */
    int32_t realtime_position;           /* 实时位置(脉冲数) */
    int32_t position_error;              /* 位置误差(脉冲数) */
    EmmHomingStatus homing_status;       /* 归零状态 */
    EmmMotorStatus motor_status;         /* 电机运行状态 */
} EmmSystemStatusParams;

/*
 * 电机配置参数结构体
 * 完整的电机配置项，可通过emm_set_config一次性写入
 */
typedef struct
{
    EmmMotorType motor_type;                    /* 电机类型(0.9度或1.8度) */
    EmmPulsePortMode pulse_port_mode;           /* 脉冲端口模式 */
    EmmSerialPortMode serial_port_mode;          /* 串口模式 */
    EmmEnableLevel enable_level;                /* 使能电平 */
    EmmDirLevel dir_level;                      /* 方向电平 */
    uint16_t microstep;                         /* 微步细分数(1~256, 256用0表示) */
    bool microstep_interp;                      /* 微步插值使能(平滑运动) */
    uint16_t open_loop_current_ma;              /* 开环电流(mA) */
    uint16_t closed_loop_current_ma;            /* 闭环电流(mA) */
    uint16_t max_voltage;                       /* 最大电压 */
    EmmBaudRate baud_rate;                      /* 通信波特率 */
    EmmCanRate can_rate;                        /* CAN总线速率 */
    uint8_t motor_id;                           /* 电机ID */
    EmmChecksumMode checksum_mode;              /* 校验模式 */
    EmmResponseMode response_mode;              /* 响应模式 */
    EmmStallProtect stall_protect;              /* 堵转保护模式 */
    uint16_t stall_speed_rpm;                   /* 堵转检测转速阈值 */
    uint16_t stall_current_ma;                  /* 堵转检测电流阈值 */
    uint16_t stall_time_ms;                     /* 堵转检测时间窗口 */
    uint16_t position_window_x01deg;            /* 位置到达窗口(0.1度为单位) */
} EmmConfigParams;

/*
 * 自动运行参数结构体
 * 配置电机上电后的自动运行行为
 */
typedef struct
{
    bool store;                          /* 是否存储到Flash */
    EmmDirection direction;              /* 旋转方向 */
    uint16_t speed_rpm;                  /* 自动运行速度(RPM) */
    uint8_t acceleration;                /* 加速度 */
    bool enable_en_control;              /* 使能EN引脚控制 */
} EmmAutoRunParams;

/*
 * 接收帧结构体
 * 用于保存从总线读取的原始帧数据
 */
typedef struct
{
    uint8_t address;                     /* 源设备地址 */
    uint8_t code;                        /* 功能码 */
    size_t length;                       /* 帧总长度(包含地址和功能码) */
    uint8_t bytes[EMM_STEPPER_MAX_FRAME_SIZE];  /* 帧数据 */
} EmmRxFrame;

/*
 * ======================== EMM协议API函数声明 ========================
 *
 * 函数命名规则：
 * emm_xxx() - 高层封装函数，自动处理响应模式和重试
 * emm_send_raw_xxx() - 底层原始数据发送函数
 * emm_get_xxx_forced() - 强制读取函数(临时切换响应模式)
 *
 * 所有函数返回EmmStatus枚举值表示操作结果。
 */

/*
 * ===================== 设备初始化和配置 =====================
 */

/* 初始化EMM设备实例。设置默认参数并关联传输层接口。 */
void emm_init(EmmDevice *device, const EmmTransport *transport, uint8_t address);

/* 选择/切换电机地址。用于在共享总线上选择不同电机进行通信。 */
void emm_select_address(EmmDevice *device, uint8_t address);

/* 设置校验和模式(固定值/XOR/CRC8等) */
void emm_set_checksum_mode(EmmDevice *device, EmmChecksumMode mode);

/* 设置本地响应模式(仅对当前设备实例生效，不写入电机Flash) */
void emm_set_response_mode_local(EmmDevice *device, EmmResponseMode mode);

/* 同时设置命令超时和到达超时 */
void emm_set_timeouts(EmmDevice *device, uint32_t command_timeout_ms, uint32_t reached_timeout_ms);

/* 启用/禁用严格帧检查(严格模式会检查功能码是否匹配) */
void emm_set_strict_frame_check(EmmDevice *device, bool enable);

/* 启用/禁用写前自动清空缓冲区 */
void emm_set_auto_flush_before_write(EmmDevice *device, bool enable);

/* ===================== 接收缓冲区管理 ===================== */

/* 清空接收环形缓冲区(丢弃所有暂存数据) */
void emm_rx_clear(EmmDevice *device);

/* 查询接收缓冲区中可读的字节数 */
size_t emm_rx_available(const EmmDevice *device);

/* 查询缓冲区溢出次数(用于监控和调试) */
size_t emm_rx_overflow_count(const EmmDevice *device);

/* ===================== 校验和计算 ===================== */

/* 计算指定模式和长度的数据校验和 */
uint8_t emm_calculate_checksum(const uint8_t *data, size_t length, EmmChecksumMode mode);

/* ===================== 轮询和帧读取 ===================== */

/* 轮询UART, 读取数据到环形缓冲区 */
EmmStatus emm_poll(EmmDevice *device, uint32_t timeout_ms);

/* 读取固定长度的响应帧(用于已知应答长度的命令) */
EmmStatus emm_read_fixed_frame(EmmDevice *device, uint8_t expected_address, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms);

/* 读取动态长度的响应帧(用于长度不固定的应答) */
EmmStatus emm_read_dynamic_frame(EmmDevice *device, uint8_t expected_address, uint8_t expected_code, uint8_t *response, size_t response_capacity, size_t *response_length, uint32_t timeout_ms);

/* 读取任意帧(不限制地址和功能码，用于侦听总线) */
EmmStatus emm_read_any_frame(EmmDevice *device, EmmRxFrame *frame, uint32_t timeout_ms);

/* 等待"到达"响应(用于EMM_RESPONSE_REACHED模式) */
EmmStatus emm_wait_reached(EmmDevice *device, uint8_t expected_code, uint8_t *response, size_t response_length, uint32_t timeout_ms);

/* ===================== 原始数据发送 ===================== */

/* 发送原始命令但不等待响应(发后即忘) */
EmmStatus emm_send_raw_no_response(EmmDevice *device, const uint8_t *body, size_t body_length);

/* 发送原始命令并等待固定长度响应 */
EmmStatus emm_send_raw(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_length);

/* 发送原始命令并等待动态长度响应 */
EmmStatus emm_send_raw_dynamic(EmmDevice *device, const uint8_t *body, size_t body_length, uint8_t *response, size_t response_capacity, size_t *response_length);

/* ===================== 电机控制命令 ===================== */

/* 编码器校准 - 自动检测电机相位和编码器零位 */
EmmStatus emm_calibrate_encoder(EmmDevice *device);

/* 重启电机驱动器(软重启) */
EmmStatus emm_restart(EmmDevice *device);

/* 清零当前位置为0点 */
EmmStatus emm_zero_position(EmmDevice *device);

/* 清零位置并验证(读取位置确认确实为0) */
EmmStatus emm_zero_position_verified(EmmDevice *device);

/* 清除保护状态(堵转/过流/过温等) */
EmmStatus emm_clear_protection(EmmDevice *device);

/* 恢复出厂设置(所有参数恢复默认) */
EmmStatus emm_factory_reset(EmmDevice *device);

/* 使能/禁用电机(通电保持或断电自由) */
EmmStatus emm_enable(EmmDevice *device, bool enable, EmmSyncFlag sync_flag);

/* 禁用电机的便捷宏(调用emm_enable(device, false, sync_flag)) */
EmmStatus emm_disable(EmmDevice *device, EmmSyncFlag sync_flag);

/* 点动控制(持续旋转直到收到停止命令或堵转) */
EmmStatus emm_jog(EmmDevice *device, const EmmJogParams *params);

/* 按脉冲数运动(精确控制位置) */
EmmStatus emm_move_pulses(EmmDevice *device, const EmmPositionParams *params);

/* 按角度运动(度数), 自动转换为脉冲数 */
EmmStatus emm_move_degrees(EmmDevice *device, float degrees, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag);

/* 按转数运动(圈数), 1圈=360度 */
EmmStatus emm_move_revolutions(EmmDevice *device, float revolutions, uint16_t speed_rpm, uint8_t acceleration, EmmMotionMode motion_mode, uint16_t microstep, EmmSyncFlag sync_flag);

/* 紧急停止电机 */
EmmStatus emm_stop(EmmDevice *device, EmmSyncFlag sync_flag);

/* 安全停止: 强制执行停止并验证速度归零。失败则禁用电机。 */
EmmStatus emm_stop_verified(EmmDevice *device, EmmSyncFlag sync_flag);

/* 同步启动所有缓冲的运动命令(多轴协调) */
EmmStatus emm_sync_move(EmmDevice *device);

/* ===================== 归零(Homing) ===================== */

/* 设置当前位置为归零参考点 */
EmmStatus emm_set_home_zero(EmmDevice *device, EmmStoreFlag store);

/* 执行归零操作 */
EmmStatus emm_home(EmmDevice *device, EmmHomingMode mode, EmmSyncFlag sync_flag);

/* 停止归零 */
EmmStatus emm_stop_home(EmmDevice *device);

/* 读取归零状态 */
EmmStatus emm_get_homing_status(EmmDevice *device, EmmHomingStatus *status);

/* 读取归零参数 */
EmmStatus emm_get_homing_params(EmmDevice *device, EmmHomingParams *params);

/* 设置归零参数 */
EmmStatus emm_set_homing_params(EmmDevice *device, const EmmHomingParams *params, EmmStoreFlag store);

/* ===================== 状态读取 ===================== */

/* 读取版本信息 */
EmmStatus emm_get_version(EmmDevice *device, EmmVersionParams *version);

/* 读取电机相电阻/电感 */
EmmStatus emm_get_motor_rh(EmmDevice *device, EmmMotorRHParams *params);

/* 读取总线电压(mV) */
EmmStatus emm_get_bus_voltage(EmmDevice *device, uint16_t *voltage_mv);

/* 读取总线电流(mA) */
EmmStatus emm_get_bus_current(EmmDevice *device, uint16_t *current_ma);

/* 读取相电流(mA) */
EmmStatus emm_get_phase_current(EmmDevice *device, uint16_t *current_ma);

/* 读取编码器值(0~65535, 对应0~360度) */
EmmStatus emm_get_encoder(EmmDevice *device, uint16_t *encoder);

/* 读取编码器角度(度) */
EmmStatus emm_get_encoder_degrees(EmmDevice *device, float *degrees);

/* 读取脉冲计数 */
EmmStatus emm_get_pulse_count(EmmDevice *device, int32_t *pulse_count);

/* 读取目标位置(度) */
EmmStatus emm_get_target_position(EmmDevice *device, float *degrees);

/* 读取实时转速(RPM) */
EmmStatus emm_get_realtime_speed(EmmDevice *device, int16_t *speed_rpm);

/* 读取实时位置(度) */
EmmStatus emm_get_realtime_position(EmmDevice *device, float *degrees);

/* 读取位置误差(度) */
EmmStatus emm_get_position_error(EmmDevice *device, float *degrees);

/* 读取温度(摄氏度) */
EmmStatus emm_get_temperature(EmmDevice *device, int16_t *temperature_c);

/* 读取电机运行状态 */
EmmStatus emm_get_motor_status(EmmDevice *device, EmmMotorStatus *status);

/* 读取PID参数 */
EmmStatus emm_get_pid(EmmDevice *device, EmmPIDParams *params);

/* 读取完整配置参数 */
EmmStatus emm_get_config(EmmDevice *device, EmmConfigParams *params);

/* 读取系统综合状态 */
EmmStatus emm_get_system_status(EmmDevice *device, EmmSystemStatusParams *params);

/* ===================== 参数设置 ===================== */

/* 修改电机ID地址 */
EmmStatus emm_set_id(EmmDevice *device, uint8_t new_id, EmmStoreFlag store);

/* 设置微步细分数(1~256) */
EmmStatus emm_set_microstep(EmmDevice *device, uint16_t microstep, EmmStoreFlag store);

/* 设置控制模式(开环/闭环) */
EmmStatus emm_set_loop_mode(EmmDevice *device, EmmControlMode mode, EmmStoreFlag store);

/* 设置开环电流(mA) */
EmmStatus emm_set_open_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store);

/* 设置闭环电流(mA) */
EmmStatus emm_set_closed_loop_current(EmmDevice *device, uint16_t current_ma, EmmStoreFlag store);

/* 设置PID参数 */
EmmStatus emm_set_pid(EmmDevice *device, const EmmPIDParams *params, EmmStoreFlag store);

/* 设置电机旋转方向 */
EmmStatus emm_set_motor_direction(EmmDevice *device, EmmDirection direction, EmmStoreFlag store);

/* 设置位置到达窗口(度) */
EmmStatus emm_set_position_window(EmmDevice *device, float window_deg, EmmStoreFlag store);

/* 设置心跳超时时间(ms)：超时无通信则电机自动停止 */
EmmStatus emm_set_heartbeat_time(EmmDevice *device, uint32_t time_ms, EmmStoreFlag store);

/* 设置自动运行参数 */
EmmStatus emm_set_auto_run(EmmDevice *device, const EmmAutoRunParams *params);

/* 批量设置全部配置参数 */
EmmStatus emm_set_config(EmmDevice *device, const EmmConfigParams *params, EmmStoreFlag store);

/* 使能/禁用缩放输入 */
EmmStatus emm_set_scale_input(EmmDevice *device, bool enable, EmmStoreFlag store);

/* 锁定/解锁按钮 */
EmmStatus emm_set_lock_button(EmmDevice *device, bool lock, EmmStoreFlag store);

/* 广播查询电机ID */
EmmStatus emm_broadcast_get_id(EmmDevice *device, uint8_t *motor_id);

/* ===================== 强制读取函数(Forced-Read) =====================
 *
 * 这些函数临时将response_mode改为EMM_RESPONSE_RECEIVE，
 * 执行读取后恢复原始模式。用于设备配置为无响应模式时获取数据。
 */
EmmStatus emm_get_realtime_position_forced(EmmDevice *device, float *degrees);
EmmStatus emm_get_encoder_forced(EmmDevice *device, uint16_t *encoder);
EmmStatus emm_get_motor_status_forced(EmmDevice *device, EmmMotorStatus *status);
EmmStatus emm_get_system_status_forced(EmmDevice *device, EmmSystemStatusParams *params);
EmmStatus emm_get_pulse_count_forced(EmmDevice *device, int32_t *pulse_count);

/* ===================== 校准辅助函数 ===================== */

/* 发送jog命令但不等待响应(用于校准时的快速轮询) */
EmmStatus emm_jog_no_response(EmmDevice *device, const EmmJogParams *params);
/* 清除堵转保护并重新使能电机(从锁定状态恢复) */
EmmStatus emm_clear_stall_and_recover(EmmDevice *device);

#ifdef __cplusplus
}
#endif

#endif
