#include "flash.h"
#include <string.h>

/**
 * @brief   保存寄存器值到Flash存储器
 * @param   sBuf    指向要保存的16位有符号整数数组的指针
 * @param   usSize  数组中元素的个数
 * @retval  无
 * @note    每个寄存器值占用8字节Flash空间
 *          写入前会先擦除对应的Flash扇区（每个扇区1024字节）
 *          操作期间会暂时关闭全局中断
 */
void SaveRegs(short sBuf[], unsigned short usSize)
{
    unsigned char i = 0;
    uint32_t baseAddr = FLASH_USER_START_ADDR;
    uint32_t endAddr = baseAddr + (usSize * 8);
    uint32_t sectorAddr;
    uint32_t currentAddr;
    
    __disable_irq();
    
    for (sectorAddr = baseAddr; sectorAddr < endAddr; sectorAddr += 1024) 
		{
        DL_FlashCTL_unprotectSector(FLASHCTL, sectorAddr, DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, sectorAddr, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
        DL_FlashCTL_waitForCmdDone(FLASHCTL);
    }
    
    for (i = 0; i < usSize; i++) 
		{
        currentAddr = baseAddr + (i * 8);
        DL_FlashCTL_unprotectSector(FLASHCTL, currentAddr, DL_FLASHCTL_REGION_SELECT_MAIN);
        
        if (DL_FlashCTL_programMemoryFromRAM16WithECCGenerated(FLASHCTL,
                currentAddr, (uint16_t*)&sBuf[i]) == DL_FLASHCTL_COMMAND_STATUS_FAILED) 
				{
            __enable_irq();
            return;
        }
        DL_FlashCTL_waitForCmdDone(FLASHCTL);
    }
    
    __enable_irq();
}

/**
 * @brief   从Flash存储器读取寄存器值
 * @param   sBuf    指向用于存储读取数据的16位有符号整数数组的指针
 * @param   usSize  需要读取的寄存器数量（数组元素个数）
 * @retval  无
 * @note    每个寄存器从Flash地址 FLASH_USER_READ_ADDR + (i * 8) 读取
 *          假设Flash中的数据已由 SaveRegs 函数正确写入
 */
void ReadRegs(short sBuf[], unsigned short usSize)
{
    for (unsigned short i = 0; i < usSize; i++) 
		{
        sBuf[i] = *(short*)(FLASH_USER_READ_ADDR + (i * 8));
    }
}

