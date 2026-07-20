#ifndef _DT_ENCODER_H_
#define _DT_ENCODER_H_

#include "zf_common_headfile.h"

/*
 * GPIO 编码器解码器限制说明：
 *
 * - 单相模式计数 a_pin 的双边沿，忽略 b_pin。正交模式使用两相。
 *   GPIO 挂起位不是事件 FIFO，因此在中断被屏蔽或 CPU 忙碌时重复边沿可能合并。
 * - invalid_transition_count 可检测非法 A/B 状态跳变，但无法检测所有丢失。
 *   两次丢失的跳变可能返回之前的状态而不会增加错误计数。QERR==0 并不保证没有丢失。
 * - 单相模式无法推断方向；direction_sign 是固定的假设方向。
 *   在使用距离阈值前请测量每转脉冲数（CPR）和轮周长。
 * - 仅在车轮停止时调用 dt_encoder_reset_odometry()。
 *   复位后到达的边沿将作为新测量的一部分。
 * - 对于高速、安全关键或需要权威绝对定位的场景，请使用定时器/外部正交解码硬件。
 *   本驱动适用于短距离竞速小车的里程计和运行时错误监控。
 */

/**
 * @brief 编码器驱动结构体
 *        支持 GPIO 中断方式和硬件定时器正交解码方式，
 *        可配置单相模式或正交模式。
 */
typedef struct {
    gpio_pin_enum    a_pin;                    /**< A相GPIO引脚 */
    gpio_pin_enum    b_pin;                    /**< B相GPIO引脚（正交模式时使用） */
    uint16_t         counts_per_rev;           /**< 编码器每转脉冲数（CPR） */
    float            wheel_circumference_mm;   /**< 车轮周长，单位：毫米 */
    volatile uint32_t edge_count;              /**< 有效脉冲边沿总数（绝对值） */
    volatile int32_t  signed_edge_count;       /**< 带符号的脉冲边沿计数（方向相关） */
    volatile uint32_t invalid_transition_count; /**< 非法状态跳变计数（诊断用） */
    volatile uint8_t  last_ab_state;           /**< 上次中断触发的AB状态值 (bit1=A, bit0=B) */
    volatile uint32_t sampled_transition_count; /**< 通过采样方式独立检测到的边沿变化数 */
    uint8_t           sampled_ab_state;        /**< 最近一次采样的AB引脚电平状态 */
    bool              quadrature_enabled;      /**< 是否启用正交解码模式 */
    bool              hardware_quadrature;     /**< 是否使用硬件定时器正交解码（而非GPIO中断） */
    bool              initialized;             /**< 配置有效且硬件初始化成功 */
    timer_index_enum  timer_index;             /**< 硬件正交解码使用的定时器索引 */
    encoder_channel1_enum channel_a;            /**< 硬件编码器A通道配置 */
    encoder_channel2_enum channel_b;            /**< 硬件编码器B通道配置 */
    int8_t           direction_sign;           /**< 方向符号（1或-1），决定正方向 */
    uint32_t         last_edge_count;          /**< 上一次读取时的边沿计数（用于计算增量） */
    int32_t          last_signed_edge_count;   /**< 上一次读取时的带符号边沿计数 */
    float            rpm_lpf_alpha;            /**< RPM低通滤波系数（0~1），默认0.35 */
    float            rpm;                      /**< 最近一次计算的转速值（RPM） */
    float            rpm_signed;               /**< 带符号转速（正=前进方向） */
} dt_encoder_t;

/**
 * @brief 初始化编码器驱动
 *        根据配置选择 GPIO 中断方式或硬件正交解码方式，
 *        初始化引脚并注册中断回调。
 * @param enc 编码器结构体指针
 * @return true 配置受支持且初始化完成；false 配置无效
 */
bool    dt_encoder_init(dt_encoder_t *enc);

/**
 * @brief 查询编码器配置是否已成功初始化
 */
bool    dt_encoder_is_ready(const dt_encoder_t *enc);

/**
 * @brief 读取编码器 A/B 引脚的实时电平状态
 * @param enc 编码器结构体指针
 * @return 实时AB电平，bit1=A，bit0=B
 */
uint8_t dt_encoder_get_ab_state(const dt_encoder_t *enc);

/**
 * @brief 独立于中断的采样函数，通过轮询检测AB引脚状态变化
 *        不会影响里程计数据，仅用于诊断监测
 * @param enc 编码器结构体指针
 */
void    dt_encoder_sample_inputs(dt_encoder_t *enc);

/**
 * @brief 获取通过采样函数检测到的状态变化总次数（诊断用）
 * @param enc 编码器结构体指针
 * @return 采样检测到的边沿跳变计数
 */
uint32_t dt_encoder_get_sampled_transitions(dt_encoder_t *enc);

/**
 * @brief 复位里程计数据（脉冲计数、符号计数、RPM和错误计数）
 *        注意：仅应在车轮停止时调用，否则累积数据将丢失
 * @param enc 编码器结构体指针
 */
void    dt_encoder_reset_odometry(dt_encoder_t *enc);

/**
 * @brief 获取自复位以来有效的正交边沿总数（无方向信息）
 * @param enc 编码器结构体指针
 * @return 有效边沿总数
 */
uint32_t dt_encoder_get_edges(dt_encoder_t *enc);

/**
 * @brief 获取应用 direction_sign 后的符号化边沿计数
 *        正数为正方向移动，负数为反方向移动
 * @param enc 编码器结构体指针
 * @return 符号化边沿计数
 */
int32_t dt_encoder_get_signed_edges(dt_encoder_t *enc);

/**
 * @brief 获取非法状态跳变的累计次数（诊断用）
 *        零值不能保证没有丢失脉冲
 * @param enc 编码器结构体指针
 * @return 非法跳变计数
 */
uint32_t dt_encoder_get_invalid_transitions(dt_encoder_t *enc);

/**
 * @brief 获取自上次调用后新增的脉冲增量
 * @param enc 编码器结构体指针
 * @return 脉冲增量（绝对值）
 */
uint32_t dt_encoder_get_delta(dt_encoder_t *enc);

/**
 * @brief 获取自上次调用后新增的带符号脉冲增量
 * @param enc 编码器结构体指针
 * @return 带符号脉冲增量（正=前进方向）
 */
int32_t dt_encoder_get_signed_delta(dt_encoder_t *enc);

/**
 * @brief 计算带符号转速（RPM），使用 signed_edge_count 保留方向信息
 * @param enc 编码器结构体指针
 * @param dt_ms 距离上次计算的时间间隔（毫秒）
 * @return 带符号滤波转速（RPM，正=前进方向）
 */
float   dt_encoder_compute_signed_rpm(dt_encoder_t *enc, uint32_t dt_ms);

/**
 * @brief 根据脉冲增量计算当前转速（RPM），内部进行低通滤波
 * @param enc 编码器结构体指针
 * @param dt_ms 距离上次计算的时间间隔（毫秒）
 * @return 滤波后的转速（转/分钟）
 */
float   dt_encoder_compute_rpm(dt_encoder_t *enc, uint32_t dt_ms);

/**
 * @brief 根据 edge_count 和车轮周长计算总行驶路径长度（绝对值，毫米）
 * @param enc 编码器结构体指针
 * @return 总行驶距离（毫米）
 */
float   dt_encoder_get_travel_mm(dt_encoder_t *enc);

/**
 * @brief 根据 signed_edge_count 计算有符号的位移量（毫米）
 *        正值表示正方向移动，负值表示反方向移动
 * @param enc 编码器结构体指针
 * @return 有符号位移（毫米）
 */
float   dt_encoder_get_distance_mm(dt_encoder_t *enc);

#endif
