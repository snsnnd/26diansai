#include "Delay.h"

volatile unsigned int delay_times = 0;

void delay_us(int __us) 
{ 
	delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); 
}


//搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while( delay_times != 0 );
}



//滴答定时器中断服务函数
void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}