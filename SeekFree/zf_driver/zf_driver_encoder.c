/*********************************************************************************************************************
* MSPM0G3507 Opensource Library ����MSPM0G3507 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� MSPM0G3507 ��Դ���һ����
* 
* MSPM0G3507 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_driver_encoder
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          MDK 5.37
* ����ƽ̨          MSPM0G3507
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_common_interrupt.h"

#include "zf_driver_encoder.h"

static const DL_TimerA_ClockConfig gCAPTURE_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static GPTIMER_Regs* const timer_list[] = {TIMA0, TIMA1, TIMG0, TIMG6, TIMG7, TIMG8, TIMG12};

static gpio_pin_enum encoder_dir[TIM_MAX] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint16 encoder_last_count[TIM_MAX] = {0};

// 逐飞扩展，实现在 SeekFree/dl_timer.c。
extern void DL_Timer_Count_CCP(GPTIMER_Regs *gptimer);

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ��ȡ����ֵ
// ����˵��     index           TIMER ����ģ���
// ���ز���     void
// ʹ��ʾ��     encoder_get_count(index);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
int16 encoder_get_count (timer_index_enum index)
{
    int16 count;
    
    count = DL_Timer_getTimerCount(timer_list[index]);
    if(0xff != encoder_dir[index])
    {
        count = gpio_get_level(encoder_dir[index]) == 1 ?  count : -count;
    }
    // else
    // {
        // // Ϊ���뷽������ı���������һ�£��������/4�������Ҫ�߾��ȿ�������ȥ��/4
        // count = count / 4;
    // }

    return count;
}

int16 encoder_get_delta (timer_index_enum index)
{
    uint16 current = (uint16)DL_Timer_getTimerCount(timer_list[index]);
    int16 delta = (int16)(current - encoder_last_count[index]);

    encoder_last_count[index] = current;
    if(0xff != encoder_dir[index] && gpio_get_level(encoder_dir[index]) == 0)
    {
        delta = -delta;
    }
    return delta;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER �������ֵ
// ����˵��     index           TIMER ����ģ���
// ���ز���     void
// ʹ��ʾ��     encoder_clear_count(index);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void encoder_clear_count (timer_index_enum index)
{
    DL_Timer_stopCounter(timer_list[index]);
    DL_Timer_setTimerCount(timer_list[index], 0);
    encoder_last_count[index] = 0;
    DL_Timer_startCounter(timer_list[index]);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ��������ģʽ��ʼ��
// ����˵��     index           TIMER ����ģ���
// ����˵��     ch1_pin         ͨ��1����
// ����˵��     ch2_pin         ͨ��2����
// ���ز���     void
// ʹ��ʾ��     encoder_quad_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, TIMG8_ENCODER1_CH2_B11);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void encoder_quad_init (timer_index_enum index, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin)
{
    /* MSPM0G3507 TIMG12 does not implement QEI; accepting it produces a free-running count. */
    zf_assert(TIM_G8 == index);
    zf_assert(((ch1_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);
    zf_assert(((ch2_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);
    zf_assert(timer_funciton_check(index, TIMER_FUNCTION_ENCODER));
    
    afio_init(ch1_pin & ENCODER_PIN_INDEX_MASK, GPI, (ch1_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);
    afio_init(ch2_pin & ENCODER_PIN_INDEX_MASK, GPI, (ch2_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);
    
    DL_Timer_setClockConfig(timer_list[index], &gCAPTURE_0ClockConfig);

    DL_TimerG_configQEI(timer_list[index], DL_TIMER_QEI_MODE_2_INPUT, DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(timer_list[index], DL_TIMER_QEI_MODE_2_INPUT, DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_Timer_setLoadValue(timer_list[index], 65535);
    DL_Timer_enableClock(timer_list[index]);
    DL_Timer_startCounter(timer_list[index]);
    encoder_last_count[index] = (uint16)DL_Timer_getTimerCount(timer_list[index]);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ���������ģʽ��ʼ��
// ����˵��     index           TIMER ����ģ���
// ����˵��     lsb_pin         ������������
// ����˵��     dir_pin         ������������
// ���ز���     void
// ʹ��ʾ��     encoder_dir_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, B11);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void encoder_dir_init (timer_index_enum index, encoder_channel1_enum lsb_pin, gpio_pin_enum dir_pin)
{
    zf_assert(((lsb_pin >> ENCODER_INDEX_OFFSET) & ENCODER_INDEX_MASK) == index);
    zf_assert(timer_funciton_check(index, TIMER_FUNCTION_ENCODER));
    
    // ��ʼ������
    afio_init(lsb_pin & ENCODER_PIN_INDEX_MASK, GPI, (lsb_pin >> ENCODER_PIN_AF_OFFSET) & ENCODER_PIN_AF_MASK, GPI_PULL_UP);
    
    // ���淽������
    encoder_dir[index] = dir_pin;
    gpio_init(dir_pin, GPI, 0, GPI_PULL_UP);
    
    DL_Timer_setClockConfig(timer_list[index], &gCAPTURE_0ClockConfig);
    DL_Timer_Count_CCP(timer_list[index]);
    DL_Timer_setCaptureCompareInputFilter(timer_list[index], DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER, DL_TIMER_CC_INPUT_FILT_FP_PER_8, DL_TIMER_CC_0_INDEX);
    DL_Timer_enableCaptureCompareInputFilter(timer_list[index], DL_TIMER_CC_0_INDEX);
    DL_Timer_enableClock(timer_list[index]);
    encoder_last_count[index] = (uint16)DL_Timer_getTimerCount(timer_list[index]);
}
