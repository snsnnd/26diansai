#ifndef XV7001_H
#define XV7001_H

#include "ti_msp_dl_config.h"

void XV7001_Init(void);
double XV7001_GetData(void);
float XV7001_GetTemperature(void);
#endif