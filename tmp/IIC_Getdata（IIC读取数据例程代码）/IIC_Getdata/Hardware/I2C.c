/******************************************************************
 * @file    I2C.c
 * @brief   I2C 软件模拟驱动，支持标准模式(100kHz)
 * @note    使用 GPIO 模拟 I2C 时序，适用于 MSPM0 系列
 * @version 1.0
 * @date    2026
 ******************************************************************/

#include "I2C.h"
#include "Delay.h"

/**
 * @brief  产生 I2C 起始信号
 * @note   在 SCL 高电平时，SDA 从高到低变化
 * @param  无
 * @return 无
 */
void IIC_Start(void)
{
    SDA_OUT();              /* 设置 SDA 为输出模式 */
    
    SDA(1);                 /* SDA 拉高 */
    SCL(1);                 /* SCL 拉高 */
    delay_us(5);            /* 建立时间 tSU;STA ≥ 4.7us */
    
    SDA(0);                 /* SCL 高时，SDA 拉低产生起始条件 */
    delay_us(5);            /* 保持时间 tHD;STA ≥ 4us */
    
    SCL(0);                 /* SCL 拉低，准备发送数据 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
}

/**
 * @brief  产生 I2C 停止信号
 * @note   在 SCL 高电平时，SDA 从低到高变化
 * @param  无
 * @return 无
 */
void IIC_Stop(void)
{
    SDA_OUT();              /* 设置 SDA 为输出模式 */
    
    SCL(0);                 /* SCL 拉低准备 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    SDA(0);                 /* SDA 拉低 */
    delay_us(5);            /* 建立时间 tSU;STO ≥ 4us */
    
    SCL(1);                 /* SCL 拉高 */
    delay_us(5);            /* 时钟高电平时间 tHIGH ≥ 4us */
    
    SDA(1);                 /* SCL 高时，SDA 拉高产生停止条件 */
    delay_us(5);            /* 总线释放时间 tBUF ≥ 4.7us */
}

/**
 * @brief  主机发送应答或非应答信号
 * @param  ack: 应答信号
 *         @arg 0: 发送应答信号 (ACK) - SDA 拉低
 *         @arg 1: 发送非应答信号 (NACK) - SDA 拉高
 * @return 无
 */
void IIC_Send_Ack(uint8_t ack)
{
    SDA_OUT();              /* 设置 SDA 为输出模式 */
    
    SCL(0);                 /* SCL 拉低，准备 SDA 数据 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    /* 设置 SDA 电平 */
    if (ack) {
        SDA(1);             /* NACK: SDA 拉高 */
    } else {
        SDA(0);             /* ACK: SDA 拉低 */
    }
    delay_us(5);            /* 数据建立时间 tSU;DAT ≥ 250ns */
    
    SCL(1);                 /* SCL 拉高，从机在第9个时钟采样 ACK/NACK */
    delay_us(5);            /* 时钟高电平时间 tHIGH ≥ 4us */
    delay_us(5);            /* 额外保持时间，确保从机正确采样 */
    
    SCL(0);                 /* SCL 拉低，结束应答周期 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    SDA(1);                 /* 释放 SDA 总线 */
}

/**
 * @brief  等待从机应答信号
 * @param  无
 * @return uint8_t: 应答状态
 *         @retval 0: 收到应答 (ACK) - 从机拉低 SDA
 *         @retval 1: 未收到应答 (NACK) - 从机未响应，发送停止信号
 */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t timeout = 0;    /* 超时计数器 */
    
    SDA_IN();               /* 设置 SDA 为输入模式，释放总线 */
    SDA(1);                 /* 内部上拉，释放 SDA */
    delay_us(5);            /* 总线释放时间 */
    
    SCL(1);                 /* SCL 拉高，在第9个时钟读取从机应答 */
    delay_us(5);            /* 时钟高电平时间 tHIGH ≥ 4us */
    
    /* 等待从机拉低 SDA (应答) 或超时 */
    while (SDA_GET() && (timeout < 255)) {
        timeout++;          /* 超时计数增加 */
        delay_us(1);        /* 每1us检测一次 */
    }
    
    SCL(0);                 /* SCL 拉低，结束应答检测 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    SDA_OUT();              /* 恢复 SDA 为输出模式 */
    
    /* 检查是否超时 */
    if (timeout >= 255) {
        IIC_Stop();         /* 超时，发送停止信号 */
        return 1;           /* 返回无应答 */
    }
    
    return 0;               /* 返回有应答 */
}

/**
 * @brief  通过 I2C 发送一个字节数据
 * @param  dat: 要发送的字节数据
 * @return 无
 * @note   发送时高位 (MSB) 在前
 */
void IIC_Send_Byte(uint8_t dat)
{
    uint8_t i;              /* 循环计数器 */
    
    SDA_OUT();              /* 设置 SDA 为输出模式 */
    SCL(0);                 /* SCL 拉低准备 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    /* 循环发送 8 位数据，高位在前 */
    for (i = 0; i < 8; i++) {
        /* 根据当前位设置 SDA 电平 */
        if (dat & 0x80) {
            SDA(1);         /* 数据位为 1，SDA 拉高 */
        } else {
            SDA(0);         /* 数据位为 0，SDA 拉低 */
        }
        delay_us(5);        /* 数据建立时间 tSU;DAT ≥ 250ns */
        
        SCL(1);             /* SCL 拉高，从机采样数据 */
        delay_us(5);        /* 时钟高电平时间 tHIGH ≥ 4us */
        
        SCL(0);             /* SCL 拉低，准备下一位数据 */
        delay_us(5);        /* 时钟低电平时间 tLOW ≥ 4.7us */
        
        dat <<= 1;          /* 左移准备下一位 */
    }
}

/**
 * @brief  通过 I2C 读取一个字节数据
 * @param  无
 * @return uint8_t: 读取到的字节数据
 * @note   读取时高位 (MSB) 在前
 */
uint8_t IIC_Read_Byte(void)
{
    uint8_t i;              /* 循环计数器 */
    uint8_t receive = 0;    /* 接收数据缓存 */
    
    SDA_IN();               /* 设置 SDA 为输入模式 */
    SDA(1);                 /* 释放 SDA，让从机控制 */
    delay_us(5);            /* 总线释放时间 */
    
    /* 循环读取 8 位数据，高位在前 */
    for (i = 0; i < 8; i++) {
        SCL(0);             /* SCL 拉低，从机准备数据 */
        delay_us(5);        /* 时钟低电平时间 tLOW ≥ 4.7us */
        
        SCL(1);             /* SCL 拉高，主机读取数据 */
        delay_us(5);        /* 时钟高电平时间 tHIGH ≥ 4us */
        
        receive <<= 1;      /* 左移接收寄存器 */
        if (SDA_GET()) {    /* 读取 SDA 电平 */
            receive |= 1;   /* 如果 SDA 为高，置位最低位 */
        }
        
        delay_us(5);        /* 数据保持时间 tHD;DAT ≥ 0ns */
    }
    
    SCL(0);                 /* SCL 拉低，结束读取 */
    delay_us(5);            /* 时钟低电平时间 tLOW ≥ 4.7us */
    
    SDA_OUT();              /* 恢复 SDA 为输出模式 */
    
    return receive;         /* 返回读取到的数据 */
}

/**
 * @brief  向 I2C 设备指定寄存器写入一个字节
 * @param  dev_addr: 设备地址 (7位)
 * @param  reg_addr: 寄存器地址 (8位)
 * @param  data:     要写入的数据
 * @return uint8_t: 操作状态
 *         @retval 1: 写入成功
 *         @retval 0: 写入失败
 * @note   调用 I2C_WriteReg 实现
 */
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C_WriteReg(dev_addr, reg_addr, &data, 1);
}

/**
 * @brief  从 I2C 设备指定寄存器读取一个字节
 * @param  dev_addr: 设备地址 (7位)
 * @param  reg_addr: 寄存器地址 (8位)
 * @param  data:     接收数据指针
 * @return uint8_t: 操作状态
 *         @retval 1: 读取成功
 *         @retval 0: 读取失败
 * @note   调用 I2C_ReadReg 实现
 */
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return I2C_ReadReg(dev_addr, reg_addr, data, 1);
}

/**
 * @brief  向 I2C 设备发送命令 (无数据)
 * @param  dev_addr: 设备地址 (7位)
 * @param  cmd:      命令码 (8位)
 * @return uint8_t: 操作状态
 *         @retval 1: 发送成功
 *         @retval 0: 发送失败
 * @note   I2C 命令发送: START + 设备地址(W) + 命令码 + STOP
 */
uint8_t I2C_SendCmd(uint8_t dev_addr, uint8_t cmd)
{
    IIC_Start();            /* 发送起始信号 */
    
    /* 发送设备地址 (写方向) */
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
    
    /* 发送命令码 */
    IIC_Send_Byte(cmd);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
    
    IIC_Stop();             /* 发送停止信号 */
    return 1;               /* 发送成功 */
}

/**
 * @brief  向 I2C 设备写入数据 (简化版)
 * @param  dev_addr: 设备地址 (7位)
 * @param  data:     要写入的数据缓冲区指针
 * @param  len:      数据长度 (字节数)
 * @return 无
 * @note   无寄存器地址，直接写入数据
 *         I2C 流程: START + 设备地址(W) + 数据 + STOP
 */
void I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;              /* 循环计数器 */
    
    IIC_Start();            /* 发送起始信号 */
    
    /* 发送设备地址 (写方向) */
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return;
    }
    
    /* 循环发送数据 */
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack()) {   /* 等待应答 */
            IIC_Stop();
            return;
        }
    }
    
    IIC_Stop();             /* 发送停止信号 */
}

/**
 * @brief  从 I2C 设备读取数据 (简化版)
 * @param  dev_addr: 设备地址 (7位)
 * @param  data:     接收数据缓冲区指针
 * @param  len:      数据长度 (字节数)
 * @return 无
 * @note   无寄存器地址，直接读取数据
 *         I2C 流程: START + 设备地址(R) + 读取数据 + STOP
 */
void I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;              /* 循环计数器 */
    
    IIC_Start();            /* 发送起始信号 */
    
    /* 发送设备地址 (读方向) */
    IIC_Send_Byte((dev_addr << 1) | 0x01);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return;
    }
    
    /* 循环读取数据 */
    for (i = 0; i < len; i++) {
        data[i] = IIC_Read_Byte();      /* 读取一个字节 */
        if (i < len - 1) {
            IIC_Send_Ack(0);            /* 还有数据，发送 ACK */
        } else {
            IIC_Send_Ack(1);            /* 最后一个字节，发送 NACK */
        }
    }
    
    IIC_Stop();             /* 发送停止信号 */
}



/**
 * @brief  向 I2C 设备指定寄存器写入数据
 * @param  dev_addr: 设备地址 (7位)
 * @param  reg_addr: 寄存器地址 (8位)
 * @param  data:     要写入的数据缓冲区指针
 * @param  len:      数据长度 (字节数)
 * @return uint8_t: 操作状态
 *         @retval 1: 写入成功
 *         @retval 0: 写入失败
 * @note   I2C 写流程: START + 设备地址(W) + 寄存器地址 + 数据 + STOP
 */
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;              /* 循环计数器 */
    
    IIC_Start();            /* 发送起始信号 */
    
    /* 发送设备地址 (写方向) */
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;           /* 无应答，返回失败 */
    }
    
    /* 发送寄存器地址 */
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
    
    /* 循环发送数据 */
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack()) {   /* 等待应答 */
            IIC_Stop();
            return 0;
        }
    }
    
    IIC_Stop();             /* 发送停止信号 */
    return 1;               /* 写入成功 */
}

/**
 * @brief  从 I2C 设备指定寄存器读取数据
 * @param  dev_addr: 设备地址 (7位)
 * @param  reg_addr: 寄存器地址 (8位)
 * @param  data:     接收数据缓冲区指针
 * @param  len:      数据长度 (字节数)
 * @return uint8_t: 操作状态
 *         @retval 1: 读取成功
 *         @retval 0: 读取失败
 * @note   I2C 读流程: START + 设备地址(W) + 寄存器地址 + 
 *         Repeated START + 设备地址(R) + 读取数据 + STOP
 */
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;              /* 循环计数器 */
    
    IIC_Start();            /* 发送起始信号 */
    
    /* 发送设备地址 (写方向) */
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
		
		
    /* 发送寄存器地址 */
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
		
    IIC_Stop();
		
    IIC_Start();            /* 发送 Repeated START 信号 */
    
    /* 发送设备地址 (读方向) */
    IIC_Send_Byte((dev_addr << 1) | 0x01);
    if (IIC_Wait_Ack()) {   /* 等待应答 */
        IIC_Stop();
        return 0;
    }
    
    /* 循环读取数据 */
    for (i = 0; i < len; i++) {
        data[i] = IIC_Read_Byte();      /* 读取一个字节 */
        if (i < len - 1) {
            IIC_Send_Ack(0);            /* 还有数据，发送 ACK */
        } else {
            IIC_Send_Ack(1);            /* 最后一个字节，发送 NACK */
        }
    }
    
    IIC_Stop();             /* 发送停止信号 */
    return 1;               /* 读取成功 */
}
