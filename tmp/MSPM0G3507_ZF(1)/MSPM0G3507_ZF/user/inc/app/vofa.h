#ifndef _VOFA_H_
#define _VOFA_H_

#include "zf_common_headfile.h"

/* JustFloat 帧尾: 0x00 0x00 0x80 0x7F */
#define VOFA_TAIL  {0x00, 0x00, 0x80, 0x7F}

void vofa_send(const float *data, uint8_t count);

#endif
