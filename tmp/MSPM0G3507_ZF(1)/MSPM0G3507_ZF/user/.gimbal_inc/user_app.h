/*********************************************************************************************************************
 * user_app.h - 用户应用任务头文件
 *
 * 在此定义用户裸机应用函数
 ********************************************************************************************************************/

#ifndef _user_app_h_
#define _user_app_h_

#include "zf_common_headfile.h"
#include "gimbal.h"
#include "maixcam2_protocol.h"
#include "vision_gimbal_control.h"

void user_app_init(void);
void user_app_loop(void);

#endif
