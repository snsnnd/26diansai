#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "lcd_init.h"
#include "lcd.h"
#include "I2C.h"

/*==================== 宏定义 ====================*/
#define SLAVE_ADDR  0x48   // I2C从设备地址（传感器地址）
#define YAW_ADDR    0x1B   // 偏航角度寄存器地址
#define GZ_ADDR     0x09   // 偏航角速度寄存器地址

/*==================== 全局变量定义 ====================*/
uint8_t YAW_data[2] = {0};  // 角度数据缓存（2字节）
uint8_t GZ_data[2] = {0};   // 角速度数据缓存（2字节）

/* 传感器控制指令 */
uint8_t KEY[3] = {0x13, 0x8E, 0x5F};        // 解锁指令（写入传感器前需先解锁）
uint8_t SAVE[3] = {0x00, 0x00, 0x00};       // 保存指令（保存当前配置）
uint8_t CALIYAW[3] = {0x15, 0x00, 0x00};    // 偏航角归零指令
uint8_t BIAS_CAL[3] = {0x0A, 0x01, 0x00};   // 校准指令（需时约21秒）

volatile float Yaw = 0.0f;  // 偏航角度值（单位：度）
volatile float GZ = 0.0f;   // 偏航角速度值（单位：度/秒）

/*==================== 函数声明 ====================*/
void sendCaliYawCommand(void);
void performCaliBias(void);
void Get_senserdata(void);

/**
 * @brief   主函数 - 系统初始化与主循环
 * @param   无
 * @retval  无
 */
int main(void)
{
    /* 系统时钟与外设初始化 */
    SYSCFG_DL_init();
    
    /* 清除定时器中断挂起标志 */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    
    /* 使能定时器中断 */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    
    /* LCD显示屏初始化 */
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    
    /* 上电后执行一次角度归零 */
    sendCaliYawCommand();
   
    while(1)
    {
        /* 在LCD上显示偏航角度和角速度 */
        LCD_ShowFloatNumEx(0, 20, Yaw, 4, WHITE, BLACK, 16);
        LCD_ShowFloatNumEx(0, 40, GZ, 4, WHITE, BLACK, 16);
        
        /* 延时5ms，控制显示刷新率 */
        delay_ms(5);
    }
}

/**
 * @brief   读取传感器数据
 * @param   无
 * @retval  无
 * @note    通过I2C读取角度和角速度数据，并更新全局变量Yaw和GZ
 */
void Get_senserdata(void)
{
    /* 读取偏航角度数据（2字节） */
    I2C_ReadReg(SLAVE_ADDR, YAW_ADDR, YAW_data, 2);
    /* 组合成16位有符号数（传感器数据为16位补码） */
    short rawYaw = (short)((YAW_data[1] << 8) | YAW_data[0]);
    /* 转换为浮点角度值（-180° ~ +180°） */
    Yaw = (float)rawYaw / 32768.0f * 180.0f;

    /* 读取偏航角速度数据（2字节） */
    I2C_ReadReg(SLAVE_ADDR, GZ_ADDR, GZ_data, 2);
    /* 组合成16位有符号数 */
    short rawGZ = (short)((GZ_data[1] << 8) | GZ_data[0]);
    /* 转换为浮点角速度值 */
    GZ = (float)rawGZ / 32768.0f * 180.0f;
}

/**
 * @brief   定时器0中断服务函数
 * @param   无
 * @retval  无
 * @note    定时器周期为5ms，每次中断触发时读取传感器数据
 */
void TIMER_0_INST_IRQHandler(void)
{
    /* 获取并处理定时器中断类型 */
    switch(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:  /* 定时器溢出中断 */
            /* 读取传感器数据 */
            Get_senserdata();
            break;

        default:  /* 其他类型中断（未使用） */
            break;
    }
}

/**
 * @brief   发送Z轴角度归零命令
 * @param   无
 * @retval  无
 * @note    操作步骤：解锁 → 发送归零指令 → 保存配置
 */
void sendCaliYawCommand(void)
{
    /* 步骤1：发送解锁指令（传感器需解锁后才能写入配置） */
    I2C_WriteData(SLAVE_ADDR, KEY, 3);
    delay_ms(10);  /* 等待传感器响应 */
    
    /* 步骤2：发送偏航角归零指令 */
    I2C_WriteData(SLAVE_ADDR, CALIYAW, 3);
    delay_ms(10);  /* 等待指令执行 */
    
    /* 步骤3：保存配置到传感器内部存储器 */
    I2C_WriteData(SLAVE_ADDR, SAVE, 3);
}

/**
 * @brief   发送传感器校准指令
 * @param   无
 * @retval  无
 * @note    校准过程中请勿移动传感器，否则会影响校准效果
 *         整个过程约需21秒
 */
void performCaliBias(void)
{
    /* 步骤1：发送解锁指令 */
    I2C_WriteData(SLAVE_ADDR, KEY, 3);
    delay_ms(10);
    
    /* 步骤2：发送校准指令（传感器开始自动校准） */
    I2C_WriteData(SLAVE_ADDR, BIAS_CAL, 3);
    delay_ms(21000);  /* 等待校准完成（约21秒） */
    
    /* 步骤3：保存校准结果 */
    I2C_WriteData(SLAVE_ADDR, SAVE, 3);
}