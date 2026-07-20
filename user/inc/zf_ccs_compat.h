/*********************************************************************************************************************
 * zf_ccs_compat.h — 逐飞库 CCS SDK 兼容层
 *
 * 问题背景：
 *   逐飞科技的开源库基于较旧的 MSPM0 SDK 2.04 版本开发，
 *   而 CCS（Code Composer Studio）IDE 自带的 MSPM0 SDK 版本较新（2.10）。
 *   不同 SDK 版本之间存在 API 差异——部分逐飞库使用的函数在新版 SDK 中
 *   不存在、已改名或声明位置发生了变化。
 *
 * 解决方案：
 *   此头文件补充声明新版 SDK 中缺失的逐飞自定义 API，
 *   确保逐飞库可以在新的 CCS SDK 环境下正常编译。
 *   这些函数的具体实现在逐飞库的 dl_timer.c 等文件中。
 *
 * 维护提示：
 *   如果升级 CCS SDK 后出现链接错误（undefined reference），
 *   很可能需要在此文件中补充缺失的 API 声明。
 ********************************************************************************************************************/

#ifndef _ZF_CCS_COMPAT_H_
#define _ZF_CCS_COMPAT_H_

#include <ti/driverlib/driverlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===================================================
 * DL_Timer_Count_CCP
 *
 * 逐飞在 driverlib 中自定义的函数, 用于将定时器配置为
 * CCP 计数模式 (编码器脉冲计数)。
 *
 * 用途：
 *   编码器输入捕获模式：TIMG 定时器的 CCP 引脚被配置为
 *   计数器模式，对外部脉冲（编码器输出）进行计数。
 *   每个 CCP 通道独立计数，因此可以同时捕获速度 + 方向信息。
 *
 * 在新 CCS SDK 中此函数不存在, 由 逐飞版 dl_timer.c 提供实现。
 * 此处仅声明, 实现在逐飞库的 dl_timer.c 中。
 *
 * @gptimer: 通用定时器（TIMG）寄存器基地址
 *===================================================*/
extern void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);

#ifdef __cplusplus
}
#endif

#endif /* _ZF_CCS_COMPAT_H_ */
