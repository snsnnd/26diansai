#include "Gyro.h"
#include <math.h>  // 添加：用于 fabs() 函数
#include "flash.h" // 添加：如果SaveRegs/ReadRegs在此头文件中

/**
 * @brief 初始化角度计算状态结构体
 * @param state 角度计算状态结构体指针
 * @return 1-加载了保存的零偏, 0-未找到有效零偏数据
 */
char Angle_Init(AngleState_t* state)
{
    state->dYaw = 0.0;
    state->dBiasSum = 0.0;
    state->uiBiasCnt = 0;
    state->cBiasCalibrated = 0;
    state->usStaticCnt = 0;
    state->dGyroBias = 0.0;
    
    if (Angle_LoadBiasFromFlash(state))
    {
        return 1;  // 成功加载已保存的零偏
    }
    
    return 0;  // 未找到有效零偏，需要校准
}

/**
 * @brief 开始零偏校准
 * @param state 角度计算状态结构体指针
 * @note 调用此函数后，传感器需保持静止2秒完成校准
 */
void Angle_StartBiasCalibInit(AngleState_t* state)
{
    state->dBiasSum = 0.0;          // 清空累加和
    state->uiBiasCnt = 0;           // 清空计数
    state->cBiasCalibrated = 0;     // 标记未校准状态
}


/**
 * @brief 保存零偏数据到Flash
 * @param state 角度计算状态结构体指针
 * @retval 1-成功, 0-失败
 */
char Angle_SaveBiasToFlash(AngleState_t* state)
{
    short biasBuffer[5];  
    
    if (!state->cBiasCalibrated) {
        return 0;  // 未校准，不保存
    }
    
    // 将double拆分成4个short（16位数据）存储
    // 方法1: 使用联合体
    union {
        double dVal;
        short sVal[4];
    } converter;
    converter.dVal = state->dGyroBias;
    
    // 将4个short存入buffer，再加上有效标志位
    biasBuffer[0] = converter.sVal[0];
    biasBuffer[1] = converter.sVal[1];
    biasBuffer[2] = converter.sVal[2];
    biasBuffer[3] = converter.sVal[3];
    biasBuffer[4] = 0x5A5A;  // 有效标志
    
    // 保存到Flash（需要5个short，占用40字节）
    SaveRegs(biasBuffer, 5);
    
    return 1;
}

/**
 * @brief 从Flash加载零偏数据
 * @param state 角度计算状态结构体指针
 * @retval 1-加载成功, 0-无有效数据
 */
char Angle_LoadBiasFromFlash(AngleState_t* state)
{
    short biasBuffer[5];
    
    // 从Flash读取数据
    ReadRegs(biasBuffer, 5);
    
    // 检查有效标志
    if (biasBuffer[4] == 0x5A5A)
    {
        // 重建double值
        union {
            double dVal;
            short sVal[4];
        } converter;
        
        converter.sVal[0] = biasBuffer[0];
        converter.sVal[1] = biasBuffer[1];
        converter.sVal[2] = biasBuffer[2];
        converter.sVal[3] = biasBuffer[3];
        
        state->dGyroBias = converter.dVal;
        state->cBiasCalibrated = 1;
        return 1;  // 加载成功
    }
    
    return 0;  // 无有效数据
}


/**
 * @brief 更新零偏校准(每次中断调用一次)
 * @param state 角度计算状态结构体指针
 * @param dWz 当前角速度值(rad/s)
 * @return 1-校准中, 0-校准完成
 */
char Angle_UpdateBiasCalib(AngleState_t* state, double dWz)
{
    if (state->cBiasCalibrated) return 0;
    
    state->dBiasSum += dWz;
    state->uiBiasCnt++;
    
    if (state->uiBiasCnt >= 2000u)  // 使用2000u保持类型一致
    {
        // 计算平均值作为零偏
        state->dGyroBias = state->dBiasSum / state->uiBiasCnt;
        state->cBiasCalibrated = 1;
        
        // 自动保存到Flash
        Angle_SaveBiasToFlash(state);
        
        return 0;  // 校准完成
    }
    return 1;  // 校准中
}

/**
 * @brief 静态检测(带防抖)
 * @param state 角度计算状态结构体指针
 * @param dWz 当前角速度值(rad/s)
 * @param dStaticThre 静态阈值(rad/s)，小于此值认为是静态
 * @param usStaticTime 静态判定时间(ms)，连续满足阈值时间
 * @return 1-静态状态, 0-动态状态
 * @note 需连续usStaticTime毫秒角速度小于阈值才判定为静态
 */
static char IsStatic(AngleState_t* state, double dWz, double dStaticThre, unsigned short usStaticTime)
{
    if (fabs(dWz) < dStaticThre)           // 角速度小于阈值
    {
        state->usStaticCnt++;              // 静态计时增加
        if (state->usStaticCnt >= usStaticTime)  // 达到判定时间
        {
            state->usStaticCnt = usStaticTime;   // 限制最大值
            return 1;                      // 返回静态状态
        }
    }
    else                                    // 角速度大于阈值
    {
        state->usStaticCnt = 0;             // 复位静态计时
    }
    return 0;                               // 返回动态状态
}

/**
 * @brief 角度更新函数(每次中断调用一次)
 * @param dWz 原始角速度(rad/s)
 * @param state 角度计算状态结构体指针
 * @return 当前偏航角(度)，范围 -180° ~ +180°
 * @note 此函数在5ms中断中调用，内部完成零偏扣除和积分
 */
double Angle_Update(double dWz, AngleState_t* state)
{
    // ========== 1. 扣除零偏 ==========
    if (state->cBiasCalibrated)            // 如果零偏已校准
    {
        dWz -= state->dGyroBias;           // 减去零偏值
    }
    
    // ========== 2. 角度积分 ==========
    // 5ms = 0.005秒，积分公式: 角度(弧度) = 角速度(rad/s) × 时间(s)
    state->dYaw += dWz * 0.005;
    
    // ========== 3. 角度归一化(弧度) ==========
    // 将弧度角度限制在 [-PI, PI] 范围内
    if (state->dYaw > PI)                  // 大于180度
    {
        state->dYaw -= (PI * 2);           // 减去360度
    }
    else if (state->dYaw < -PI)            // 小于-180度
    {
        state->dYaw += (PI * 2);           // 加上360度
    }
    
    // ========== 4. 返回角度值(度) ==========
    return (state->dYaw * 180.0 / PI);
}

/**
 * @brief 重置偏航角为零
 * @param state 角度计算状态结构体指针
 * @note 不影响零偏值，只是将当前角度清零
 */
void Angle_ResetYaw(AngleState_t* state)
{
    state->dYaw = 0.0;                     // 偏航角归零
}