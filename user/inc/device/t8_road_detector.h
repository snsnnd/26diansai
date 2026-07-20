#ifndef T8_ROAD_DETECTOR_H
#define T8_ROAD_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief T8 路段类型枚举
 *
 * 通过对 T8 灰度传感器 8 位二值化数据 (bit=0 黑线, bit=1 白色)
 * 按左右半区统计黑点数量，分类当前所处路段类型。
 *
 * bit[7:4] = 右半区, bit[3:0] = 左半区
 */
typedef enum
{
    T8_ROAD_STRAIGHT = 0,
    T8_ROAD_LEFT_TURN,
    T8_ROAD_RIGHT_TURN,
    T8_ROAD_CROSS,
    T8_ROAD_LOST,
} t8_road_type_t;

/**
 * @brief T8 路段检测器实例
 */
typedef struct
{
    uint8_t         confirm_samples;
    uint8_t         turn_half_black_min;
    uint8_t         turn_other_half_max;
    uint8_t         confirm_count;
    t8_road_type_t  last_classified;
    t8_road_type_t  confirm_target;
} t8_road_detector_t;

void           t8_road_detector_init(t8_road_detector_t *det,
                 uint8_t confirm_samples, uint8_t half_black_min,
                 uint8_t other_half_max);
t8_road_type_t t8_road_classify(uint8_t line_bits);
bool           t8_road_detector_confirm(t8_road_detector_t *det,
                 uint8_t line_bits);
t8_road_type_t t8_road_detector_type(const t8_road_detector_t *det);
void           t8_road_detector_reset(t8_road_detector_t *det);
uint8_t        t8_road_count_black_half(uint8_t bits, bool right_half);

#endif
