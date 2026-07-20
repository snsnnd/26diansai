#ifndef Gyro_H
#define Gyro_H

#include "ti_msp_dl_config.h"
#include <math.h>

// 在头文件中定义
#define BIAS_INDEX 0              // 使用第0个寄存器位置存储零偏

typedef struct {
    double dGyroBias;             // 陀螺仪零偏值
    uint16_t uiValidFlag;         // 数据有效标志(0x5A5A)
} BiasData_t;

#define PI 3.14159265358979f

typedef struct
{
    double dYaw;                // 偏航角 (rad)
    double dGyroBias;           // 零偏值 (rad/s)
    double dBiasSum;            // 零偏累加和
    unsigned int uiBiasCnt;     // 零偏采样计数
    char cBiasCalibrated;       // 零偏校准完成标志
    unsigned short usStaticCnt; // 静态计时
} AngleState_t;

char Angle_Init(AngleState_t* state);
void Angle_StartBiasCalibInit(AngleState_t* state);
char Angle_SaveBiasToFlash(AngleState_t* state);
char Angle_LoadBiasFromFlash(AngleState_t* state);
char Angle_UpdateBiasCalib(AngleState_t* state, double dWz);
static char IsStatic(AngleState_t* state, double dWz, double dStaticThre, unsigned short usStaticTime);
double Angle_Update(double dWz, AngleState_t* state);
//double Angle_Update(double dWz, AngleState_t* state, double dStaticThre, unsigned short usStaticTime, double dDeadzone);
double Angle_GetYawDeg(const AngleState_t* state);
double Angle_GetYawRad(const AngleState_t* state);
void Angle_ResetYaw(AngleState_t* state);


#endif