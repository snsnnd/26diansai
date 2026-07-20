#include "zf_common_headfile.h"
#include "app/line_car.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();
    system_delay_ms(500);

    printf("\r\n===== Line Follower Car =====\r\n");

    line_car_init();

    printf("===== Init done =====\r\n\r\n");

    while (1)
    {
        line_car_run();
    }
}
