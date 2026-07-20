#ifndef I2C_H
#define I2C_H

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include <stdint.h>  
#include <stdbool.h> 


#define SDA_OUT()   {                                                \
                        DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);    \
                        DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);      \
                        DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); \
                    }

// 设置 SDA 为输入模式
#define SDA_IN()    { DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); }

// 获取 SDA 引脚电平
#define SDA_GET()   ( ( ( DL_GPIO_readPins(I2C_PORT, I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 )

// SDA 与 SCL 输出
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN)) : (DL_GPIO_clearPins(I2C_PORT, I2C_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT, I2C_SCL_PIN)) : (DL_GPIO_clearPins(I2C_PORT, I2C_SCL_PIN)) )

// 基础 I2C 函数
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Ack(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Send_Byte(uint8_t dat);
unsigned char IIC_Read_Byte(void);

// 寄存器读写函数
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

// 单字节读写
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

// 命令发送
uint8_t I2C_SendCmd(uint8_t dev_addr, uint8_t cmd);

// 简单数据读写
void I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint8_t len);
void I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t len);


#endif 