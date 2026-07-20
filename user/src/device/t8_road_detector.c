#include "device/t8_road_detector.h"

uint8_t t8_road_count_black_half(uint8_t bits, bool right_half)
{
    uint8_t i;
    uint8_t shift;
    uint8_t count = 0u;

    shift = right_half ? 4u : 0u;
    for (i = 0u; i < 4u; i++)
    {
        if ((bits & (1u << (shift + i))) == 0u)
        {
            count++;
        }
    }
    return count;
}

t8_road_type_t t8_road_classify(uint8_t line_bits)
{
    uint8_t left_black  = t8_road_count_black_half(line_bits, false);
    uint8_t right_black = t8_road_count_black_half(line_bits, true);
    uint8_t total_black = left_black + right_black;

    if (total_black == 0u)
    {
        return T8_ROAD_LOST;
    }

    if (total_black == 8u)
    {
        return T8_ROAD_CROSS;
    }

    return T8_ROAD_STRAIGHT;
}

void t8_road_detector_init(t8_road_detector_t *det,
    uint8_t confirm_samples, uint8_t half_black_min, uint8_t other_half_max)
{
    if (det == NULL)
    {
        return;
    }

    det->confirm_samples      = confirm_samples;
    det->turn_half_black_min  = half_black_min;
    det->turn_other_half_max  = other_half_max;
    det->confirm_count        = 0u;
    det->last_classified      = T8_ROAD_STRAIGHT;
    det->confirm_target       = T8_ROAD_STRAIGHT;
}

bool t8_road_detector_confirm(t8_road_detector_t *det, uint8_t line_bits)
{
    t8_road_type_t classified;
    uint8_t        left_black;
    uint8_t        right_black;
    bool           is_turn;

    if (det == NULL)
    {
        return false;
    }

    classified = t8_road_classify(line_bits);

    left_black  = t8_road_count_black_half(line_bits, false);
    right_black = t8_road_count_black_half(line_bits, true);

    is_turn = false;
    if (classified == T8_ROAD_STRAIGHT || classified == T8_ROAD_LOST
        || classified == T8_ROAD_CROSS)
    {
        is_turn = (right_black >= det->turn_half_black_min
                && left_black <= det->turn_other_half_max)
               || (left_black >= det->turn_half_black_min
                && right_black <= det->turn_other_half_max);

        if (is_turn)
        {
            if (right_black >= det->turn_half_black_min
                && left_black <= det->turn_other_half_max)
            {
                classified = T8_ROAD_RIGHT_TURN;
            }
            else
            {
                classified = T8_ROAD_LEFT_TURN;
            }
        }
    }

    if (classified == det->confirm_target)
    {
        det->confirm_count++;
    }
    else
    {
        det->confirm_target = classified;
        det->confirm_count  = 1u;
    }

    det->last_classified = classified;

    if (det->confirm_target == T8_ROAD_LEFT_TURN
        || det->confirm_target == T8_ROAD_RIGHT_TURN)
    {
        return det->confirm_count >= det->confirm_samples;
    }

    return false;
}

t8_road_type_t t8_road_detector_type(const t8_road_detector_t *det)
{
    if (det == NULL)
    {
        return T8_ROAD_STRAIGHT;
    }
    return det->confirm_target;
}

void t8_road_detector_reset(t8_road_detector_t *det)
{
    if (det == NULL)
    {
        return;
    }
    det->confirm_count   = 0u;
    det->last_classified = T8_ROAD_STRAIGHT;
    det->confirm_target  = T8_ROAD_STRAIGHT;
}
