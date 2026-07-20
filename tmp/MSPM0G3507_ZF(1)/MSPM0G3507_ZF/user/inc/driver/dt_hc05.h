#ifndef _DT_HC05_H_
#define _DT_HC05_H_

#include "zf_common_headfile.h"

#define HC05_NAME    "MSPM0_Car"
#define HC05_PIN     "1234"
#define HC05_BAUD    115200

void dt_hc05_init(gpio_pin_enum en_pin);

#endif
