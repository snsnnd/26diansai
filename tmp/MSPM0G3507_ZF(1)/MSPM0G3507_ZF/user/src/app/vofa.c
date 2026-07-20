#include "app/vofa.h"

static const uint8_t tail[] = VOFA_TAIL;

void vofa_send(const float *data, uint8_t count)
{
    uint8_t buf[64];
    uint8_t i;

    if (data == 0)
    {
        return;
    }

    if (count > 15u)
    {
        count = 15u;
    }

    for (i = 0; i < count; i++)
    {
        const uint8_t *p = (const uint8_t *)(&data[i]);
        buf[i * 4 + 0] = p[0];
        buf[i * 4 + 1] = p[1];
        buf[i * 4 + 2] = p[2];
        buf[i * 4 + 3] = p[3];
    }
    memcpy(buf + count * 4, tail, 4);

    uart_write_buffer(DEBUG_UART_INDEX, buf, count * 4 + 4);
}
