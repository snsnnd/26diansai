#ifndef FLASH_H
#define FLASH_H

#include "ti_msp_dl_config.h"

/* Flash存储配置 */
// 最后4KB起始地址 (128KB - 4KB = 124KB = 0x1F000)
#define FLASH_USER_START_ADDR   (0x0001F000)        // 物理地址（用于编程/擦除）
#define FLASH_USER_READ_ADDR    (0x0041F000)        // 内存映射地址（用于读取）


void SaveRegs(short sBuf[], unsigned short usSize);
void ReadRegs(short sBuf[], unsigned short usSize);

#endif