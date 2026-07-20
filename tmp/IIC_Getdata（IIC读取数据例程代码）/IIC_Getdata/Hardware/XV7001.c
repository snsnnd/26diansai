#include "XV7001.h"
#include "I2C.h"
#include "board.h"
#include <math.h>
#include "lcd_init.h"
#include "lcd.h"

/* 寄存器/命令定义 */
#define REG_DSP_CTL2        0x02    // 低通滤波器配置寄存器
#define REG_OUT_CTL1        0x0B    // 输出控制寄存器
#define REG_TS_DATA_FORMAT  0x1C    // 温度数据格式寄存器

#define CMD_SOFT_RESET      0x09    // 软件复位
#define CMD_SLEEP_OUT       0x06    // 唤醒
#define CMD_DATA_ACC        0x0A    // 角速度数据读

/* 滤波器阶数定义 */
#define LPF_ORDER_2         (0x00 << 4)    // 0x00 - 2阶滤波器
#define LPF_ORDER_3         (0x01 << 4)    // 0x10 - 3阶滤波器
#define LPF_ORDER_4         (0x02 << 4)    // 0x20 - 4阶滤波器

/* 滤波器截止频率定义 */
#define LPF_FC_10           0x00    // 10Hz
#define LPF_FC_35           0x01    // 35Hz
#define LPF_FC_45           0x02    // 45Hz
#define LPF_FC_50           0x03    // 50Hz
#define LPF_FC_70           0x04    // 70Hz
#define LPF_FC_85           0x05    // 85Hz
#define LPF_FC_100          0x06    // 100Hz
#define LPF_FC_140          0x07    // 140Hz
#define LPF_FC_175          0x08    // 175Hz
#define LPF_FC_200          0x09    // 200Hz
#define LPF_FC_285          0x0a    // 285Hz
#define LPF_FC_345          0x0b    // 345Hz
#define LPF_FC_400          0x0c    // 400Hz
#define LPF_FC_500          0x0d    // 500Hz

/* 设备地址 - SA0接VDDI */
#define XV7011_ADDR         0x6A

/* 转换系数 - 修正版 */
// 灵敏度: 280 LSB/(°/s)
// 1 LSB = 1/280 = 0.0035714 °/s = 6.233e-5 rad/s   2.4932e-4
#define LSB_TO_RAD           2.4932e-4


/******************************************************************
 * 函 数 名 称：XV7001_Init
 * 函 数 说 明：XV7011BB陀螺仪初始化
 * 函 数 返 回：1=成功 0=失败
******************************************************************/
void XV7001_Init(void)
{
    delay_ms(100);                      // 等待电源稳定
    
    /* 软件复位 */
    I2C_SendCmd(XV7011_ADDR, CMD_SOFT_RESET);
    delay_ms(10);                       // 等待复位完成
    
    /* 唤醒传感器 */
    I2C_SendCmd(XV7011_ADDR, CMD_SLEEP_OUT);
    delay_ms(10);
    
    /* 设置输出格式：16bit角速度输出 */
    // REG_OUT_CTL1: bit2 DataFormat=0(16bit), bit1-0 OutCtrl=01(角速度输出)
    I2C_WriteByte(XV7011_ADDR, REG_OUT_CTL1, 0x01);

    
    /* 配置低通滤波器：4阶，截止频率10Hz */
    // LPF_ORDER_4 = 0x20, LPF_FC_10 = 0x00
    I2C_WriteByte(XV7011_ADDR, REG_DSP_CTL2, LPF_ORDER_4 | LPF_FC_10);

}

/******************************************************************
 * 函 数 名 称：XV7001_GetData
 * 函 数 说 明：读取角速度数据
 * 函 数 返 回：角速度值(弧度/秒)
******************************************************************/
double XV7001_GetData(void)
{
    unsigned char buffer[2];
    short raw_data;
    
    I2C_ReadReg(XV7011_ADDR, CMD_DATA_ACC, buffer, 2);
    /* 大端模式组合成16位有符号整数 */
    raw_data = (short)(((unsigned short)buffer[0] << 8) | buffer[1]);
    
    /* 转换为弧度/秒 */
    return (double)raw_data * LSB_TO_RAD;
}
