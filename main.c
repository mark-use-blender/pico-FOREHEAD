/*
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program pico-tail-controller.elf verify reset exit"
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

#define ONB_UART_TX 20
#define ONB_UART_RX 21
#define ONB_LED_0 11
#define ONB_LED_1 13
#define ONB_ADC_0 27
#define ONB_ADC_1 26
#define ONB_DO_0 3
#define ONB_DO_1 17
#define ONB_PWM_0 2
#define ONB_PWM_1 16
#define ONB_SIG_IN 23
#define ONB_SIG_OT 22


#define BUF_LEN 4
#define SBUF_LEN 1
uint8_t in_buf0[BUF_LEN];
uint8_t chin_buf0[BUF_LEN];
uint8_t sin_buf0[SBUF_LEN];
uint8_t ddid[2];
uint8_t in_bufid[2];
uint16_t adcinr0;
uint16_t adcinr1;
uint16_t temadcinr0;
uint16_t temadcinr1;
uint16_t delta0;
uint16_t delta1;
int curpos0 = 0;
int curpos1 = 0;
int tarpos0 = 0;
int tarpos1 = 0;
int difpos0 = 0;
int difpos1 = 0;
uint8_t cmid;
uint16_t msdel;
bool cmdir;
bool cmaxi;
int cmdel;
char scmdel[13];
char uin;
int suin;
char* sresult;
char int_str[50];
float fresult;
bool in_ok;

int posdiff(int st, int nd ){
    int delta = nd - st;
    bool big = 2047 < abs(delta);
    bool pos = 0 < delta;
    if (big){
        if (pos){
            delta = -4096 + delta;
        }
        else{
            delta = 4096 + delta;
        }
    }
    return delta;
}
void core1(){
    while(true){
        gpio_put(ONB_LED_0, 0);
        adc_select_input(0);
        temadcinr0 = adc_read();
        curpos0 = curpos0 + posdiff(adcinr0, temadcinr0);
        adcinr0 = temadcinr0;
        difpos0 = tarpos0 - curpos0;
        if (difpos0 < -10){
            pwm_set_gpio_level(ONB_PWM_0, (uint16_t)fmax(0, (105 + (difpos0 / 2) )));
            gpio_put(ONB_DO_0, 1);
            sleep_ms(1);
            pwm_set_gpio_level(ONB_PWM_0, 0);
            gpio_put(ONB_DO_0, 0);
        }
        else if (difpos0 > 10){
            pwm_set_gpio_level(ONB_PWM_0, (uint16_t)fmin(100, ( (difpos0 / 2) - 5)));
            gpio_put(ONB_DO_0, 0);
            sleep_ms(1);
            pwm_set_gpio_level(ONB_PWM_0, 0);
            gpio_put(ONB_DO_0, 0);
        }
        gpio_put(ONB_LED_0, 1);
        adc_select_input(1);
        temadcinr1 = adc_read();
        curpos1 = curpos1 + posdiff(adcinr1, temadcinr1);
        adcinr1 = temadcinr1;
        difpos1 = tarpos1 - curpos1;
        if (difpos1 < -10){
            pwm_set_gpio_level(ONB_PWM_1, (uint16_t)fmax(0, (105 + (difpos1 / 2) )));
            gpio_put(ONB_DO_1, 1);
            sleep_ms(1);
            pwm_set_gpio_level(ONB_PWM_1, 0);
            gpio_put(ONB_DO_1, 0);
        }
        else if (difpos1>10){
            pwm_set_gpio_level(ONB_PWM_1, (uint16_t)fmin(100, ( (difpos1 / 2) - 5)));
            gpio_put(ONB_DO_1, 0);
            sleep_ms(1);
            pwm_set_gpio_level(ONB_PWM_1, 0);
            gpio_put(ONB_DO_1, 0);
        }
        gpio_put(ONB_LED_0, 0);
        //sleep_ms(1);
    }
}
void main() {
    INIT:
    gpio_init(ONB_LED_0);
    gpio_init(ONB_DO_1);
    gpio_init(ONB_DO_0);
    gpio_init(ONB_LED_1);
    gpio_init(ONB_SIG_OT);
    gpio_init(ONB_SIG_IN);
    adc_init();
    adc_gpio_init(ONB_ADC_1);
    adc_gpio_init(ONB_ADC_0);
    gpio_set_dir(ONB_LED_0, 1);
    gpio_set_dir(ONB_SIG_OT, 1);
    gpio_set_dir(ONB_SIG_IN, 0);
    gpio_set_function(ONB_PWM_1, 4);
    pwm_set_wrap(0, 100);
    pwm_set_chan_level(0, PWM_CHAN_A, 0);
    pwm_set_enabled(0, true);
    gpio_set_dir(ONB_DO_1, 1);
    gpio_set_function(ONB_PWM_0, 4);
    pwm_set_wrap(1, 100);
    pwm_set_chan_level(1, PWM_CHAN_A, 0);
    pwm_set_enabled(1, true);
    gpio_set_dir(ONB_DO_0, 1);
    gpio_set_dir(ONB_LED_1, 1);
    gpio_put(ONB_DO_1, 0);
    gpio_put(ONB_DO_0, 0);
    gpio_put(ONB_LED_1, 1);
    gpio_put(ONB_LED_0, 1);
    gpio_put(ONB_SIG_OT, 0);
    stdio_init_all();
    if (false){
        in_ok = gpio_get(ONB_SIG_IN);
        while (in_ok == 0){
            in_ok = gpio_get(ONB_SIG_IN);
        }
        // init uart
        // Enable UART for comunicating
        uart_init(uart1, 38400);
        gpio_set_function(ONB_UART_TX, 2);
        gpio_set_function(ONB_UART_RX, 2);
        sleep_ms(10);
        while(uart_is_enabled(uart1)){
            if (uart_is_readable(uart1)){
                uart_read_blocking(uart1, ddid, 2);
                break;
            }
        }
        uart_puts(uart1, "ok");
        gpio_put(ONB_SIG_OT, 1);
    }
    adc_select_input(0);
    adcinr0 = adc_read();
    adc_select_input(1);
    adcinr1 = adc_read();
    gpio_put(ONB_LED_0, 0);
    gpio_put(ONB_LED_1, 0);
    multicore_launch_core1(core1);
    while(true){ 
        if(true){
            gpio_put(ONB_LED_1, 0);
            if (uart_is_readable(uart1)){
                uart_read_blocking(uart1, in_buf0, BUF_LEN);
                in_bufid[0] = in_buf0[0];
                in_bufid[1] = in_buf0[1];
                if (in_bufid == ddid){
                    gpio_put(ONB_LED_1, 1);
                    cmaxi = (in_buf0[2] >> 7) & 0x1;
                    cmdir = (in_buf0[2] >> 6) & 0x1;
                    msdel = (in_buf0[2] & 0x3f) << 8;
                    msdel |= in_buf0[3];
                    if (cmaxi){
                        if (cmdir){
                            tarpos0 = (int)msdel;
                        }
                        else{
                            tarpos0 = ((int)msdel) * -1 ;
                        }
                    }
                    else{
                        if (cmdir){
                            tarpos1 = (int)msdel;
                        }
                        else{
                            tarpos1 = ((int)msdel) * -1 ;
                        }
                    }
                }
                if (in_bufid == 0){
                    multicore_reset_core1();
                    goto INIT;
                }
            }
        }
    }
}
