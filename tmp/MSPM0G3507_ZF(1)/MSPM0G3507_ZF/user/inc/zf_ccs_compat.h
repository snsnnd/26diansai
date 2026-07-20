/*********************************************************************************************************************
 * zf_ccs_compat.h — 逐飞库 CCS SDK 兼容层
 *
 * 解决逐飞库在较新 CCS SDK 上编译的 API 差异。
 * 逐飞库基于旧版 driverlib (SDK 2.04), CCS 自带新版 (SDK 2.10)。
 * 本文件补充新版 SDK 中缺失或改名了的 逐飞自定义 API。
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
 * 在新 CCS SDK 中此函数不存在, 由 逐飞版 dl_timer.c 提供实现。
 * 此处仅声明, 实现在逐飞库的 dl_timer.c 中。
 *===================================================*/
extern void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);

#ifdef __cplusplus
}
#endif

#endif /* _ZF_CCS_COMPAT_H_ */
