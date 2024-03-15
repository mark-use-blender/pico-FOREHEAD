/*
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program pico-FOREHEAD.elf verify reset exit"
*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"


bool ball_attach()
{
    int data = adc_read();
    if (data>1300)
        return true;
    else
        return false;
}

void detach()
{
    while (ball_attach()==true)
    {    
        gpio_put(22,1);
        sleep_ms(50);
        gpio_put(22,0);
        sleep_ms(50);
    }
}

void attach()
{
    while (ball_attach()==false)
    {
        gpio_put(22,0);
        uart_putc(uart0,'0');
        uart_putc(uart0,'\n');
        sleep_ms(10);
    }
    uart_putc(uart0,'1');
    uart_putc(uart0,'0');
    uart_putc(uart0,'0');
    uart_putc(uart0,'\n');
}


void main()
{
    char s[10];
    int radix=10;
    int a;
    int i;
    char c;
    adc_init();
    adc_gpio_init(28);
    gpio_init(22);
    gpio_set_dir(22, 1);
    gpio_put(22, 0);
    stdio_init_all();
    uart_init(uart0, 2400);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    
    sleep_ms(10);
    adc_select_input(2);
    
    gpio_put(22,1);
    sleep_ms(500);
    gpio_put(22,0);
    
    while(false)
    {
        a = adc_read();
        itoa(a,s,radix);
        uart_puts(uart0,s);
        uart_putc(uart0,'\n');
        sleep_ms(100);
    }
    while (true)
    {
        sleep_ms(100);
        // gpio_put(22,0);
        if (/*uart_is_readable(uart0)*/true)
        {
            c = uart_getc(uart0);
            i = (int)c;
            itoa(i,s,radix);
            switch(i)
            {
                case 49:
                    uart_putc(uart0,'1');
                    uart_putc(uart0,'\n');
                    multicore_reset_core1();
                    multicore_launch_core1(detach);
                    break;
                case 50:
                    uart_putc(uart0,'2');
                    uart_putc(uart0,'\n');
                    multicore_reset_core1();
                    multicore_launch_core1(attach);
                    break;
                case 10:
                    break;
                default:
                    
                    uart_puts(uart0,c);
                    uart_putc(uart0,'\n');


            }
        }


    }
    
}