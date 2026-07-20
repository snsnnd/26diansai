#include "app/hardware_test.h"

#include "config.h"
#include "driver/dt_gyro_z.h"
#include "gimbal/gimbal.h"
#include "gimbal/maixcam2_protocol.h"
#include "gimbal/position_control.h"
#include "gimbal/speed_control.h"
#include "framework/ec_time.h"

/* ==================== 模块内部常量 ==================== */
#define HW_TEST_PRINT_PERIOD_MS 500u    /* 调试信息打印周期(毫秒) */

/* ==================== 模块内部全局变量 ==================== */
static uint32_t g_hw_test_ms = 0;                          /* 当前系统时间戳(毫秒) */
static uint32_t g_hw_test_last_print_ms = 0;                /* 上次打印时间戳 */
static GimbalStatus g_hw_test_gimbal_status = GIMBAL_ERROR; /* 云台初始化状态 */
static uint8_t g_hw_test_gimbal_enabled = 0;                /* 云台使能标志 */
static bool g_hw_test_emergency_latched = true;             /* 急停锁定标志(默认锁定) */
static PositionController g_hw_test_position_ctrl;          /* 位置控制器实例 */
static SpeedController g_hw_test_speed_ctrl;                /* 速度控制器实例 */
static GimbalControlMode g_hw_test_control_mode = GIMBAL_DEFAULT_CONTROL_MODE;  /* 当前控制模式 */

/**
 * @brief 获取视觉控制状态名称(用于调试输出)
 * @param state 视觉控制状态枚举值
 * @return 对应的状态字符串
 */
static const char *hw_test_vision_ctrl_state_name(VisionControlState state)
{
    switch (state)
    {
        case CTRL_STATE_IDLE:        return "IDLE";    /* 空闲 */
        case CTRL_STATE_SEARCH:      return "SEARCH";  /* 搜索目标 */
        case CTRL_STATE_COARSE_TRACK:return "COARSE";  /* 粗跟踪 */
        case CTRL_STATE_FINE_TRACK:  return "FINE";    /* 精跟踪 */
        case CTRL_STATE_LOST_HOLD:   return "LOST";    /* 目标丢失保持 */
        case CTRL_STATE_FAILSAFE:    return "FAIL";    /* 故障安全 */
        default:                     return "?";       /* 未知 */
    }
}

/**
 * @brief 获取云台状态名称(用于调试输出)
 * @param status 云台状态枚举值
 * @return 对应的状态字符串
 */
static const char *hw_test_gimbal_status_name(GimbalStatus status)
{
    switch (status)
    {
        case GIMBAL_OK: return "ok";             /* 正常 */
        case GIMBAL_ERROR_MOTOR: return "motor"; /* 电机通信错误 */
        case GIMBAL_ERROR_SENSOR: return "sensor"; /* 传感器错误 */
        case GIMBAL_ERROR_CALIB: return "calib"; /* 校准错误 */
        case GIMBAL_ERROR_NOT_HOMED: return "not-homed"; /* 未找零位 */
        case GIMBAL_ERROR_SAFETY_LATCHED: return "safety-latched"; /* 安全锁定 */
        case GIMBAL_ERROR:
        default: return "error";                 /* 通用错误 */
    }
}

/**
 * @brief 浮点数转毫单位整型(保留3位小数精度)
 * 例如：1.2345f -> 1235(四舍五入到毫单位)
 * @param value 输入浮点数
 * @return 毫单位整型值
 */
static int32_t hw_test_float_to_milli(float value)
{
    if (value >= 0.0f) return (int32_t)(value * 1000.0f + 0.5f);   /* 正数四舍五入 */
    return (int32_t)(value * 1000.0f - 0.5f);                        /* 负数四舍五入 */
}

/**
 * @brief 以"值.三位小数"格式打印毫单位数值
 * 例如：hw_test_print_milli("yaw", 12345, "deg") 输出 "yaw=12.345deg"
 * @param name 变量名称字符串
 * @param milli 毫单位整型值
 * @param unit 单位字符串
 */
static void hw_test_print_milli(const char *name, int32_t milli, const char *unit)
{
    int32_t integer = milli / 1000;        /* 整数部分 */
    int32_t decimal = milli % 1000;        /* 小数部分(毫) */
    char dec_str[4];

    if (decimal < 0) decimal = -decimal;   /* 取绝对值 */
    /* 逐位提取百位、十位、个位 */
    dec_str[0] = (char)('0' + (decimal / 100) % 10);
    dec_str[1] = (char)('0' + (decimal / 10) % 10);
    dec_str[2] = (char)('0' + decimal % 10);
    dec_str[3] = '\0';

    printf("%s=%d.%s%s", name, (int)integer, dec_str, unit);
}

/**
 * @brief 初始化陀螺仪(UART 或 I2C，由 GYRO_Z_TRANSPORT 决定)
 */
static void hw_test_init_gyro(void)
{
    dt_gyro_z_config_t gc;

#if GYRO_Z_TRANSPORT == 1
    gc.uart   = GYRO_Z_UART;
    gc.baud   = GYRO_Z_BAUD;
    gc.tx_pin = GYRO_Z_TX_PIN;
    gc.rx_pin = GYRO_Z_RX_PIN;
#else
    gc.uart = 0; gc.baud = 0; gc.tx_pin = 0; gc.rx_pin = 0;
#endif

    dt_gyro_z_init(&gc);
    system_delay_ms(200);
#if GYRO_Z_TRANSPORT == 1
    dt_gyro_z_zero_yaw();
    system_delay_ms(200);
#endif

#if GYRO_Z_TRANSPORT == 1
    printf("[HW][GYRO] UART0 A10/A11 baud=%u ok (zeroed)\r\n", (unsigned int)GYRO_Z_BAUD);
#else
    printf("[HW][GYRO] I2C addr=0x%02x ok (zeroed)\r\n", (unsigned int)GYRO_Z_IIC_ADDR);
#endif
}

/**
 * @brief 打印陀螺仪调试信息
 * 输出帧计数、校验错误、溢出计数、偏航角、角速度等
 */
static void hw_test_print_gyro(void)
{
    const dt_gyro_z_data_t *g = dt_gyro_z_get_data();
    printf("[HW][GYRO] frames=%u chk=%u ovf=%u ",
           (unsigned int)g->frame_count,         /* 接收到的数据帧数 */
           (unsigned int)g->checksum_error_count,/* 校验错误计数 */
           (unsigned int)g->rx_overflow);        /* 接收溢出计数 */
    hw_test_print_milli("yaw", hw_test_float_to_milli(g->yaw_deg), "deg ");  /* 偏航角(度) */
    hw_test_print_milli("wz",  hw_test_float_to_milli(g->wz_dps),   "dps");  /* Z轴角速度(度/秒) */
    printf(" raw=(%d,%d)\r\n", (int)g->yaw_raw, (int)g->wz_raw);            /* 原始数据 */
}

/**
 * @brief 打印云台调试信息
 * 输出云台使能状态、急停、找零、位置有效性、反馈状态、故障锁、
 * 控制模式、视觉跟踪状态、控制次数、实际位置和接收溢出等
 */
static void hw_test_print_gimbal(void)
{
    float yaw_pos = 0.0f, pitch_pos = 0.0f;
    VisionControlState st;

    /* 云台初始化失败时只打印错误信息 */
    if (g_hw_test_gimbal_status != GIMBAL_OK)
    {
        printf("[HW][GIMBAL] init=%s(%d)\r\n",
               hw_test_gimbal_status_name(g_hw_test_gimbal_status),
               (int)g_hw_test_gimbal_status);
        return;
    }

    /* 读取云台实际角度 */
    gimbal_read_actual_position(&g_gimbal, &yaw_pos, &pitch_pos);
    /* 根据控制模式获取对应的视觉控制状态 */
    if (g_hw_test_control_mode == GIMBAL_CONTROL_SPEED)
        st = speed_control_get_state(&g_hw_test_speed_ctrl);
    else
        st = position_control_get_state(&g_hw_test_position_ctrl);

    printf("[HW][GIMBAL] arm=%u estop=%u home=%u pos=%u fb=%u fault=%u %s ctrl=%s cnt=%u ",
            (unsigned int)g_hw_test_gimbal_enabled,   /* 使能标志 */
            (unsigned int)g_hw_test_emergency_latched,/* 急停标志 */
            (unsigned int)g_gimbal.homed,              /* 已找零位 */
            (unsigned int)g_gimbal.position_valid,     /* 位置有效 */
            (unsigned int)g_gimbal.feedback_valid,     /* 反馈有效 */
            (unsigned int)g_gimbal.safety_fault_latched, /* 安全故障锁 */
           (g_hw_test_control_mode == GIMBAL_CONTROL_SPEED) ? "SPD" : "POS",  /* 控制模式 */
           hw_test_vision_ctrl_state_name(st),         /* 视觉跟踪状态 */
           (unsigned int)((g_hw_test_control_mode == GIMBAL_CONTROL_SPEED)
               ? g_hw_test_speed_ctrl.control_count       /* 速度控制次数 */
               : g_hw_test_position_ctrl.control_count)); /* 位置控制次数 */
    hw_test_print_milli("Y", hw_test_float_to_milli(yaw_pos),   "deg ");   /* 偏航角 */
    hw_test_print_milli("P", hw_test_float_to_milli(pitch_pos), "deg");    /* 俯仰角 */
    printf(" ovf=%u\r\n",
           (unsigned int)(g_gimbal.yaw.rx_overflow_count + g_gimbal.pitch.rx_overflow_count));
}

/**
 * @brief 打印视觉模块调试信息
 * 输出MaixCam2视觉传感器的统计数据：帧接收、CRC错误、溢出、
 * 畸形帧、语义错误、间字节超时，以及最新的目标检测信息
 */
static void hw_test_print_vision(void)
{
    const MaixProtocolStats *s = maixcam2_get_stats();
    MaixVisionTarget t;
    uint32_t rxm = 0u;

    if (maixcam2_get_latest_target(&t, &rxm))
    {
        /* 有目标数据 */
        printf("[HW][VISION] frames=%u crc=%u ovf=%u bad=%u sem=%u ito=%u age=%u valid=%u st=%u err=(%d,%d)\r\n",
                (unsigned int)s->frames_received,       /* 接收帧数 */
                (unsigned int)s->crc_errors,            /* CRC错误数 */
                (unsigned int)s->ring_overflows,        /* 环形缓冲区溢出 */
                (unsigned int)s->malformed_frames,      /* 畸形帧数 */
                (unsigned int)s->semantic_errors,       /* 语义错误数 */
                (unsigned int)s->interbyte_timeouts,    /* 字节间超时 */
               (unsigned int)(g_hw_test_ms - rxm),     /* 数据年龄(毫秒) */
               (unsigned int)t.target_valid,            /* 目标是否有效 */
               (unsigned int)t.vision_state,            /* 视觉状态 */
               (int)t.error_x, (int)t.error_y);        /* 跟踪误差(x,y) */
    }
    else
    {
        /* 无目标数据 */
        printf("[HW][VISION] frames=%u crc=%u ovf=%u bad=%u sem=%u ito=%u no_target\r\n",
                (unsigned int)s->frames_received,
                (unsigned int)s->crc_errors,
                (unsigned int)s->ring_overflows,
                (unsigned int)s->malformed_frames,
                (unsigned int)s->semantic_errors,
                (unsigned int)s->interbyte_timeouts);
    }
}

/**
 * @brief 硬件测试初始化
 *
 * 初始化流程：
 * 1. 打印UART引脚映射信息
 * 2. 初始化按键(K3用于急停)
 * 3. 初始化陀螺仪(串口型)
 * 4. 初始化云台电机(SH1006系列)
 * 5. 配置云台参数(速度、加速度、超时等)
 * 6. 初始化位置和速度控制器
 * 7. 可选择接受开机参考位置(跳过找零位)
 * 8. 初始化视觉模块(MaixCam2)
 */
void hardware_test_init(void)
{
    GimbalStatus reference_status = GIMBAL_ERROR_NOT_HOMED;

    printf("[HW] UART: D=U1(A8/A9) G=U0(A10/A11) B=U2(B15/B16) V=U3(B2/B3)\r\n");
    /* 初始化K3按键(上拉输入)，用于硬件测试中的急停 */
    gpio_init(KEY3_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    hw_test_init_gyro();

    /* 初始化云台电机 */
    g_hw_test_gimbal_status = gimbal_init(&g_gimbal);
    printf("[HW][GIMBAL] init=%s(%d) yaw=%u pitch=%u\r\n",
           hw_test_gimbal_status_name(g_hw_test_gimbal_status),
           (int)g_hw_test_gimbal_status,
           (unsigned int)GIMBAL_YAW_MOTOR_ADDRESS, (unsigned int)GIMBAL_PITCH_MOTOR_ADDRESS);

    if (g_hw_test_gimbal_status == GIMBAL_OK)
    {
        /* 配置云台运行参数 */
        g_gimbal.speed_rpm    = 1200u;    /* 电机运行速度(RPM) */
        g_gimbal.acceleration = 255u;     /* 加速度 */
        g_gimbal.yaw.timeout_ms   = 5u;   /* 偏航电机通信超时(毫秒) */
        g_gimbal.pitch.timeout_ms = 5u;   /* 俯仰电机通信超时(毫秒) */
        g_gimbal.yaw.poll_attempts   = 2u;/* 偏航电机轮询尝试次数 */
        g_gimbal.pitch.poll_attempts = 2u;/* 俯仰电机轮询尝试次数 */
        g_gimbal.yaw_min_deg   = -90.0f;  /* 偏航角最小限制(度) */
        g_gimbal.yaw_max_deg   = 90.0f;   /* 偏航角最大限制(度) */

        /* 初始化控制器 */
        position_control_init(&g_hw_test_position_ctrl);
        speed_control_init(&g_hw_test_speed_ctrl);

        /* 默认进入急停状态(安全启动) */
        (void)hardware_test_emergency_stop();

#if HW_TEST_GIMBAL_ACCEPT_STARTUP_REFERENCE
        /* 如果配置了使用开机参考位置，跳过找零位过程 */
        reference_status = gimbal_accept_known_reference(
            &g_gimbal, HW_TEST_GIMBAL_STARTUP_YAW_DEG,
            HW_TEST_GIMBAL_STARTUP_PITCH_DEG);
        if (reference_status == GIMBAL_OK)
        {
            reference_status = hardware_test_rearm();
        }
#endif
        printf("[HW][GIMBAL] startup-reference=%s(%d) armed=%u rpm=%u acc=%u\r\n",
                hw_test_gimbal_status_name(reference_status), (int)reference_status,
                (unsigned int)g_hw_test_gimbal_enabled,
                (unsigned int)g_gimbal.speed_rpm,
                (unsigned int)g_gimbal.acceleration);
    }

    /* 初始化视觉模块 */
    maixcam2_init();
    printf("[HW][VISION] init ok\r\n");
}

/**
 * @brief 硬件测试主循环(需周期性调用)
 *
 * 每周期执行的操作：
 * 1. 检查K3按键是否被按下，触发急停
 * 2. 更新陀螺仪数据
 * 3. 更新视觉模块数据
 * 4. 如果云台已就绪且使能，更新对应的控制器(PID位置或速度控制)
 * 5. 检测到安全故障时自动急停
 * 6. 按固定周期打印各模块调试信息
 */
void hardware_test_run(void)
{
    g_hw_test_ms = ec_time_ms();

    /* 按键K3急停检测：非锁定状态且K3被按下时触发急停 */
    if (!g_hw_test_emergency_latched && gpio_get_level(KEY3_PIN) == GPIO_LOW)
    {
        (void)hardware_test_emergency_stop();
    }

    /* 更新传感器数据 */
    dt_gyro_z_update(g_hw_test_ms);           /* 更新陀螺仪 */
    maixcam2_update(g_hw_test_ms);/* 更新视觉模块 */

    /* 云台控制更新 */
    if (g_hw_test_gimbal_status == GIMBAL_OK && g_hw_test_gimbal_enabled &&
        !g_hw_test_emergency_latched && !g_gimbal.safety_fault_latched)
    {
        if (g_hw_test_control_mode == GIMBAL_CONTROL_SPEED)
            speed_control_update(&g_hw_test_speed_ctrl, &g_gimbal, g_hw_test_ms);
        else
            position_control_update(&g_hw_test_position_ctrl, &g_gimbal, g_hw_test_ms);
    }

    /* 安全故障自动急停 */
    if (g_hw_test_gimbal_enabled && g_gimbal.safety_fault_latched)
    {
        (void)hardware_test_emergency_stop();
    }

    /* 周期打印调试信息 */
    if ((uint32_t)(g_hw_test_ms - g_hw_test_last_print_ms) >= HW_TEST_PRINT_PERIOD_MS)
    {
        g_hw_test_last_print_ms = g_hw_test_ms;
        hw_test_print_gyro();     /* 打印陀螺仪数据 */
        hw_test_print_gimbal();   /* 打印云台数据 */
        hw_test_print_vision();   /* 打印视觉数据 */
    }

}

/**
 * @brief 设置云台控制模式
 *
 * 切换控制模式时需要先停止云台，然后重新初始化控制器。
 * 支持两种模式：
 * - GIMBAL_CONTROL_POSITION：位置控制(云台跟踪指定角度)
 * - GIMBAL_CONTROL_SPEED：速度控制(云台以指定速度旋转)
 *
 * @param mode 目标控制模式
 */
void hardware_test_set_gimbal_control_mode(GimbalControlMode mode)
{
    if (mode != GIMBAL_CONTROL_POSITION && mode != GIMBAL_CONTROL_SPEED) return;

    /* 如果云台正在运行，先安全停止 */
    if (g_hw_test_gimbal_enabled && gimbal_stop(&g_gimbal) != GIMBAL_OK)
    {
        (void)hardware_test_emergency_stop();
        return;
    }

    /* 重新初始化控制器 */
    position_control_init(&g_hw_test_position_ctrl);
    speed_control_init(&g_hw_test_speed_ctrl);
    g_hw_test_control_mode = mode;
}

/**
 * @brief 获取当前云台控制模式
 * @return 当前控制模式
 */
GimbalControlMode hardware_test_get_gimbal_control_mode(void)
{
    return g_hw_test_control_mode;
}

/**
 * @brief 执行紧急停止
 *
 * 急停操作：
 * 1. 清除云台使能标志
 * 2. 设置急停锁定标志
 * 3. 重置控制器
 * 4. 发送停止命令到云台电机
 * 5. 禁用云台电机输出
 * 6. 如果停止失败，触发安全故障锁存
 *
 * @return GIMBAL_OK 成功；GIMBAL_ERROR_MOTOR 失败
 */
GimbalStatus hardware_test_emergency_stop(void)
{
    GimbalStatus stop_status;
    GimbalStatus disable_status;

    g_hw_test_gimbal_enabled = 0u;            /* 清除使能标志 */
    g_hw_test_emergency_latched = true;       /* 设置急停锁 */
    position_control_init(&g_hw_test_position_ctrl);  /* 重置位置控制器 */
    speed_control_init(&g_hw_test_speed_ctrl);         /* 重置速度控制器 */

    stop_status = gimbal_stop(&g_gimbal);              /* 发送停止命令 */
    disable_status = gimbal_enable(&g_gimbal, false);  /* 禁用电机 */
    if (stop_status != GIMBAL_OK || disable_status != GIMBAL_OK)
    {
        /* 停止失败，触发安全故障锁存 */
        gimbal_latch_safety_fault(&g_gimbal);
        return GIMBAL_ERROR_MOTOR;
    }
    return GIMBAL_OK;
}

/**
 * @brief 重新准备云台(解除急停)
 *
 * 解除急停需要满足以下条件：
 * 1. 云台初始化成功
 * 2. 已找到零位(homed)
 * 3. 位置有效(position_valid)
 * 4. 安全按键K3未被按下(松开状态)
 *
 * 满足条件后：
 * 1. 清除安全故障锁
 * 2. 使能云台电机
 * 3. 重置控制器
 * 4. 清除急停锁定标志
 *
 * @return GIMBAL_OK 就绪成功；其他 就绪失败
 */
GimbalStatus hardware_test_rearm(void)
{
    GimbalStatus status;

    g_hw_test_gimbal_enabled = 0u;  /* 先清除使能(安全操作) */

    /* 检查云台初始化状态 */
    if (g_hw_test_gimbal_status != GIMBAL_OK)
    {
        return g_hw_test_gimbal_status;
    }
    /* 检查云台是否已找零位且位置有效 */
    if (!g_gimbal.homed || !g_gimbal.position_valid)
    {
        return GIMBAL_ERROR_NOT_HOMED;
    }
    /* 检查急停按键是否被按下(按下时不允许解除急停) */
    if (gpio_get_level(KEY3_PIN) == GPIO_LOW)
    {
        return GIMBAL_ERROR_SAFETY_LATCHED;
    }

    /* 清除安全故障并重新使能 */
    status = gimbal_clear_safety_fault(&g_gimbal);
    if (status == GIMBAL_OK)
    {
        status = gimbal_enable(&g_gimbal, true);
    }
    if (status != GIMBAL_OK)
    {
        (void)hardware_test_emergency_stop();
        return status;
    }

    /* 重置控制器并解除急停锁 */
    position_control_init(&g_hw_test_position_ctrl);
    speed_control_init(&g_hw_test_speed_ctrl);
    g_hw_test_emergency_latched = false;
    g_hw_test_gimbal_enabled = 1u;
    return GIMBAL_OK;
}

/**
 * @brief 检查云台是否已就绪(使能且无急停)
 * @return true 已就绪；false 未就绪
 */
bool hardware_test_is_gimbal_armed(void)
{
    return g_hw_test_gimbal_enabled != 0u;
}

/**
 * @brief 检查急停是否锁定
 * @return true 急停锁定中；false 未锁定
 */
bool hardware_test_is_emergency_latched(void)
{
    return g_hw_test_emergency_latched;
}
