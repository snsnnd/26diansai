/*********************************************************************************************************************
 * user_app.c - 裸机用户应用
 *
 * 上电 → 使能电机 → 自动运行齿轮比标定 → 进入追踪就绪状态
 ********************************************************************************************************************/

#include "gimbal/user_app.h"

static uint32_t LastStatusMs = 0;
static uint32_t LastBlinkMs = 0;
static uint32_t AppTickMs = 0;
static bool CalibrationDone = false;
static VisionGimbalController VisionController;

void user_app_init(void)
{
    GimbalStatus status;

    gpio_init(PIN_LED, GPO, GPIO_LOW, GPO_PUSH_PULL);

    status = gimbal_init(&g_gimbal);
    if (status == GIMBAL_OK)
    {
        printf("[GIMBAL] init ok. yaw addr=%u pitch addr=%u\r\n",
            (unsigned int)GIMBAL_YAW_MOTOR_ADDRESS,
            (unsigned int)GIMBAL_PITCH_MOTOR_ADDRESS);

        /* Enable motors so encoder/position reads work */
        gimbal_enable(&g_gimbal, true);
        system_delay_ms(200u);

        printf("[APP] auto-starting geared calibration...\r\n");
        printf("[APP] 请确保 PITCH/YAW 在安全范围内可自由运动\r\n");
#if GIMBAL_USE_PRECALIB_PITCH
        printf("[APP] pre-calib mode, skip calibration. Moving to 0,0...\r\n");
        gimbal_move_to(&g_gimbal, 0.0f, 0.0f);
        system_delay_ms(500u);
        CalibrationDone = true;
#else
        status = gimbal_calibrate_geared(&g_gimbal);
        if (status == GIMBAL_OK)
        {
            printf("[APP] calibration OK, gimbal ready\r\n");
            CalibrationDone = true;
        }
        else
        {
            printf("[APP] calibration failed: %d\r\n", status);
        }
#endif
    }
    else
    {
        printf("[GIMBAL] init failed: %d\r\n", status);
    }
}

void user_app_loop(void)
{
    /* ---- LED blink ---- */
    if ((AppTickMs - LastBlinkMs) >= 500u)
    {
        LastBlinkMs = AppTickMs;
        gpio_toggle_level(PIN_LED);
    }

    /* ---- Status print ---- */
    if (CalibrationDone && (AppTickMs - LastStatusMs) >= 1000u)
    {
        float yaw_pos = 0.0f, pitch_pos = 0.0f;

        LastStatusMs = AppTickMs;

        gimbal_read_actual_position(&g_gimbal, &yaw_pos, &pitch_pos);

        {
            int32_t y_01 = (int32_t)(yaw_pos * 10.0f);
            int32_t p_01 = (int32_t)(pitch_pos * 10.0f);
            printf("[APP] t=%lu ms, Yaw=%ld.%ld, Pitch=%ld.%ld\r\n",
                (unsigned long)AppTickMs,
                (long)(y_01 / 10), (long)((y_01 < 0 ? -y_01 : y_01) % 10),
                (long)(p_01 / 10), (long)((p_01 < 0 ? -p_01 : p_01) % 10));
        }
    }

    system_delay_ms(50u);
    AppTickMs += 50u;
}
