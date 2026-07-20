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

#ifndef _zf_driver_encoder_h_
#define _zf_driver_encoder_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"


// ��������Ҫ�õĺ궨�� ���ݸ���Ƭ����ͬ �������ɶ�����Ҫʲô��Ϣ
// bit[11:0 ] �̶�Ϊ��������
// bit[15:12] �̶�Ϊ���Ÿ���
// bit[19:16] �� ENCODER ģ����Ϊ ENCODER ����

#define     ENCODER_PIN_INDEX_OFFSET    ( 0      )                                  // bit[11:0 ] �洢 GPIO ��������
#define     ENCODER_PIN_INDEX_MASK      ( 0x0FFF )                                  // ���� 12bit �������Ϊ 0x0FFF

#define     ENCODER_PIN_AF_OFFSET       ( 12     )                                  // bit[15:12] �洢 GPIO �ĸ��ù�������
#define     ENCODER_PIN_AF_MASK         ( 0x0F   )                                  // ���� 4bit �������Ϊ 0x0F

#define     ENCODER_INDEX_OFFSET        ( 16     )                                  // bit[19:16] �洢 SPI ����
#define     ENCODER_INDEX_MASK          ( 0x0F   )                                  // ���� 4bit �������Ϊ 0x0F


typedef enum
{
    TIMA0_ENCODER1_CH1_A0   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMA0_ENCODER1_CH1_A8   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A8 ),
    TIMA0_ENCODER1_CH1_A21  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMA0_ENCODER1_CH1_B8   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B8 ),
    TIMA0_ENCODER1_CH1_B14  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B14),
    
    TIMA1_ENCODER1_CH1_A10  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A10),
    TIMA1_ENCODER1_CH1_A15  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A15),
    TIMA1_ENCODER1_CH1_A17  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMA1_ENCODER1_CH1_A28  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A28),
    TIMA1_ENCODER1_CH1_B0   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B0 ),
    TIMA1_ENCODER1_CH1_B2   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMA1_ENCODER1_CH1_B4   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B4 ),
    TIMA1_ENCODER1_CH1_B17  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B17),
    TIMA1_ENCODER1_CH1_B26  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (B26),
    
    TIMG0_ENCODER1_CH1_A5   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG0_ENCODER1_CH1_A12  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A12),
    TIMG0_ENCODER1_CH1_A23  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG0_ENCODER1_CH1_B10  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (B10),
    
    TIMG6_ENCODER1_CH1_A5   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG6_ENCODER1_CH1_A21  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMG6_ENCODER1_CH1_A29  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A29),
    TIMG6_ENCODER1_CH1_B2   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMG6_ENCODER1_CH1_B6   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B6 ),
    TIMG6_ENCODER1_CH1_B10  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B10),
    TIMG6_ENCODER1_CH1_B26  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B26),
    
    TIMG7_ENCODER1_CH1_A3   = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMG7_ENCODER1_CH1_A17  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMG7_ENCODER1_CH1_A23  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG7_ENCODER1_CH1_A26  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A26),
    TIMG7_ENCODER1_CH1_A28  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A28),
    TIMG7_ENCODER1_CH1_B15  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (B15),
    
    TIMG8_ENCODER1_CH1_A1   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A1 ),
    TIMG8_ENCODER1_CH1_A3   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMG8_ENCODER1_CH1_A5   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG8_ENCODER1_CH1_A7   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A7 ),
    TIMG8_ENCODER1_CH1_A21  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMG8_ENCODER1_CH1_A23  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG8_ENCODER1_CH1_A26  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A26),
    TIMG8_ENCODER1_CH1_A29  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A29),
    TIMG8_ENCODER1_CH1_B6   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B6 ),
    TIMG8_ENCODER1_CH1_B10  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (B10),
    TIMG8_ENCODER1_CH1_B15  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B15),
    TIMG8_ENCODER1_CH1_B21  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (B21),
    
    TIMG12_ENCODER1_CH1_A10 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A10),
    TIMG12_ENCODER1_CH1_A14 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A14),
    TIMG12_ENCODER1_CH1_B13 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B13),
    TIMG12_ENCODER1_CH1_B20 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B20),
}encoder_channel1_enum;             
                                    
typedef enum                        
{                                   
    TIMA0_ENCODER1_CH2_A1   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A1 ),
    TIMA0_ENCODER1_CH2_A3   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMA0_ENCODER1_CH2_A7   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A7 ),
    TIMA0_ENCODER1_CH2_A9   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A9 ),
    TIMA0_ENCODER1_CH2_A22  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A22),
    TIMA0_ENCODER1_CH2_B9   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B9 ),
    TIMA0_ENCODER1_CH2_B12  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B12),
    TIMA0_ENCODER1_CH2_B20  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B20),

    TIMA1_ENCODER1_CH2_A11  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A11),
    TIMA1_ENCODER1_CH2_A16  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A16),
    TIMA1_ENCODER1_CH2_A18  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A18),
    TIMA1_ENCODER1_CH2_A24  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A24),
    TIMA1_ENCODER1_CH2_A31  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A31),
    TIMA1_ENCODER1_CH2_B1   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B1 ),
    TIMA1_ENCODER1_CH2_B3   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (B3 ),
    TIMA1_ENCODER1_CH2_B5   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B5 ),
    TIMA1_ENCODER1_CH2_B18  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B18),
    TIMA1_ENCODER1_CH2_B27  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (B27),

    TIMG0_ENCODER1_CH2_A6   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A6 ),
    TIMG0_ENCODER1_CH2_A13  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A13),
    TIMG0_ENCODER1_CH2_A24  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A24),
    TIMG0_ENCODER1_CH2_B11  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (B11),

    TIMG6_ENCODER1_CH2_A6   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A6 ),
    TIMG6_ENCODER1_CH2_A22  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF8 << ENCODER_PIN_AF_OFFSET) | (A22),
    TIMG6_ENCODER1_CH2_A30  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A30),
    TIMG6_ENCODER1_CH2_B3   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B3 ),
    TIMG6_ENCODER1_CH2_B7   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (B7 ),
    TIMG6_ENCODER1_CH2_B11  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B11),
    TIMG6_ENCODER1_CH2_B27  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B27),

    TIMG7_ENCODER1_CH2_A2   = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A2 ),
    TIMG7_ENCODER1_CH2_A4   = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A4 ),
    TIMG7_ENCODER1_CH2_A7   = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A7 ),
    TIMG7_ENCODER1_CH2_A18  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A18),
    TIMG7_ENCODER1_CH2_A24  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A24),
    TIMG7_ENCODER1_CH2_A27  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A27),
    TIMG7_ENCODER1_CH2_A31  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF7 << ENCODER_PIN_AF_OFFSET) | (A31),
    TIMG7_ENCODER1_CH2_B16  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (B16),
    TIMG7_ENCODER1_CH2_B19  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (B19),

    TIMG12_ENCODER1_CH2_A25 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A25),
    TIMG12_ENCODER1_CH2_A31 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (A31),
    TIMG12_ENCODER1_CH2_B14 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B14),
    TIMG12_ENCODER1_CH2_B24 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B24),

    TIMG8_ENCODER1_CH2_A0   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF6 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMG8_ENCODER1_CH2_A2   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (A2 ),
    TIMG8_ENCODER1_CH2_A4   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (A4 ),
    TIMG8_ENCODER1_CH2_A6   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF2 << ENCODER_PIN_AF_OFFSET) | (A6 ),
    TIMG8_ENCODER1_CH2_A22  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (A22),
    TIMG8_ENCODER1_CH2_A27  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A27),
    TIMG8_ENCODER1_CH2_A30  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (A30),
    TIMG8_ENCODER1_CH2_B7   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B7 ),
    TIMG8_ENCODER1_CH2_B11  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (B11),
    TIMG8_ENCODER1_CH2_B16  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF5 << ENCODER_PIN_AF_OFFSET) | (B16),
    TIMG8_ENCODER1_CH2_B19  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF4 << ENCODER_PIN_AF_OFFSET) | (B19),
    TIMG8_ENCODER1_CH2_B22  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (GPIO_AF3 << ENCODER_PIN_AF_OFFSET) | (B22),
}encoder_channel2_enum;

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ��ȡ����ֵ
// ����˵��     index           TIMER ����ģ���
// ���ز���     void
// ʹ��ʾ��     encoder_get_count(index);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
int16   encoder_get_count       (timer_index_enum index);

// 返回自上次调用后的 16 位模计数差，不停止或清零硬件计数器。
int16   encoder_get_delta       (timer_index_enum index);

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER �������ֵ
// ����˵��     index           TIMER ����ģ���
// ���ز���     void
// ʹ��ʾ��     encoder_clear_count(index);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_clear_count     (timer_index_enum index);

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ��������ģʽ��ʼ��
// ����˵��     index           TIMER ����ģ���
// ����˵��     ch1_pin         ͨ��1����
// ����˵��     ch2_pin         ͨ��2����
// ���ز���     void
// ʹ��ʾ��     encoder_quad_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, TIMG8_ENCODER1_CH2_B11);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_quad_init       (timer_index_enum index, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin);

//-------------------------------------------------------------------------------------------------------------------
// �������     ENCODER ���������ģʽ��ʼ��
// ����˵��     index           TIMER ����ģ���
// ����˵��     lsb_pin         ������������
// ����˵��     dir_pin         ������������
// ���ز���     void
// ʹ��ʾ��     encoder_dir_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, B11);
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_dir_init        (timer_index_enum index, encoder_channel1_enum lsb_pin, gpio_pin_enum dir_pin);

#endif

