#include "driver/dt_hc05.h"

void dt_hc05_init(gpio_pin_enum en_pin)
{
    gpio_init(en_pin, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    /* EN 拉高, 重配 UART 为 38400 进 AT 模式 */
    gpio_high(en_pin);
    system_delay_ms(200);
    uart_init(DEBUG_UART_INDEX, 38400, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
    system_delay_ms(300);

    uart_write_string(DEBUG_UART_INDEX, "AT\r\n");
    system_delay_ms(100);
    uart_write_string(DEBUG_UART_INDEX, "AT+NAME=" HC05_NAME "\r\n");
    system_delay_ms(100);
    uart_write_string(DEBUG_UART_INDEX, "AT+PSWD=" HC05_PIN "\r\n");
    system_delay_ms(100);

    {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+UART=%d,0,0\r\n", HC05_BAUD);
        uart_write_string(DEBUG_UART_INDEX, cmd);
    }
    system_delay_ms(100);

    /* EN 拉低, 切回 115200 透传 */
    gpio_low(en_pin);
    system_delay_ms(200);
    uart_init(DEBUG_UART_INDEX, HC05_BAUD, DEBUG_UART_TX_PIN, DEBUG_UART_RX_PIN);
}
