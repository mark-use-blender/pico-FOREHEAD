#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#define BUF_LEN 3
#define SBUF_LEN 1
uint8_t in_buf0[BUF_LEN];
uint8_t chin_buf0[BUF_LEN];
uint8_t sin_buf0[SBUF_LEN];
uint8_t ddid;
//ddid[0] = '0';
uint16_t adcinr0;
uint16_t adcinr1;
uint16_t temadcinr0;
uint16_t temadcinr1;
uint16_t delta0;
uint16_t delta1;
int curpos0 =0;
int curpos1 =0;
int tarpos0 =0;
int tarpos1 =0;
int difpos0 =0;
int difpos1 =0;
uint8_t cmid;
uint16_t msdel;
bool cmdir;
bool cmaxi;
int cmdel;
char scmdel[13];
char uin;
int suin;

char* sresult;
//uint16_t result = adc_read();
char int_str[50];
float fresult;




int posdiff(int st,int nd ){
    int delta = nd-st;
    bool big = 2047 < abs(delta);
    bool pos = 0 < delta;
    if (big){
        if (pos){
            delta = -4096+delta;
        }
        else{
            delta = 4096+delta;
        }
    }
    return delta;
        
}

void core1(){
    while(true){
        
        
        
        adc_select_input(0);
        temadcinr0 = adc_read();
        curpos0 = curpos0+posdiff(adcinr0,temadcinr0);
        adcinr0 = temadcinr0;
        difpos0 =tarpos0-curpos0;
        if (difpos0<-10){
            //uart_puts (uart0,"right");
            pwm_set_gpio_level(2, (uint16_t)fmax(0,(105+(difpos0/2))));
            
            gpio_put(3,1);
            sleep_ms(1);
            pwm_set_gpio_level(2, 0);
            gpio_put(3,0);
        }
        else if (difpos0>10){
            //uart_puts (uart0,"left"); 
            pwm_set_gpio_level(2, (uint16_t)fmin(100,((difpos0/2)-5)));
                  
            gpio_put(3,0);
            sleep_ms(1);
            pwm_set_gpio_level(2, 0);
            gpio_put(3,0);
        }
        
        
        gpio_put(11,1);
        
        adc_select_input(1);
        temadcinr1 = adc_read();
        curpos1 = curpos1+posdiff(adcinr1,temadcinr1);
        adcinr1 = temadcinr1;
        difpos1 =tarpos1-curpos1;
        if (difpos1<-10){
            //uart_puts (uart0,"right");
            pwm_set_gpio_level(26, (uint16_t)fmax(0,(105+(difpos1/2))));
            
            gpio_put(27,1);
            sleep_ms(1);
            pwm_set_gpio_level(26, 0);
            gpio_put(27,0);
        }
        else if (difpos1>10){
            //uart_puts (uart0,"left"); 
            pwm_set_gpio_level(26, (uint16_t)fmin(100,((difpos1/2)-5)));
                  
            gpio_put(27,0);
            sleep_ms(1);
            pwm_set_gpio_level(26, 0);
            gpio_put(27,0);
        }
        
        
        
        gpio_put(11,0);
        
        
    }

}
void main() {
    gpio_init(11);
    //gpio_init(16);
    gpio_init(17);
    //gpio_init(2);
    gpio_init(3);
    gpio_init(13);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    gpio_set_dir(11,1);
    gpio_set_function(16, 4);
    pwm_set_wrap(0, 100);
    pwm_set_chan_level(0, PWM_CHAN_A, 0);
    pwm_set_enabled(0, true);
    //gpio_set_dir(16,1);
    gpio_set_dir(17,1);
    //gpio_set_dir(2,1);
    gpio_set_function(2, 4);
    pwm_set_wrap(1, 100);
    pwm_set_chan_level(1, PWM_CHAN_A, 0);
    pwm_set_enabled(1, true);
    gpio_set_dir(3,1);
    gpio_set_dir(13,1);
    //gpio_put(16,0);
    gpio_put(17,0);
    //gpio_put(2,0);
    gpio_put(3,0);
    gpio_put(13,1);
    gpio_put(11,1);
    //stdio_usb_init();	


    
    while(false){
        gpio_put(11,1);
        gpio_put(2,1);
        gpio_put(3,0);
        gpio_put(16,1);
        gpio_put(17,0);

        sleep_ms(1000);

        gpio_put(11,0);
        gpio_put(2,0);
        gpio_put(3,1);
        gpio_put(16,0);
        gpio_put(17,1);
        sleep_ms(1000);

    }
    
    //uint8_t did[SBUF_LEN];
    // Enable SPI 
    stdio_init_all();
    spi_init(spi0, 1000 * 1000);
    spi_set_slave(spi0, true);
    gpio_set_function(20, 1);
    gpio_set_function(21, 1);
    gpio_set_function(22, 1);
    gpio_set_function(23, 1);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(20, 23, 22, 21, 1));


    // init uart
    // Enable UART for debug
    uart_init (uart0,38400);
    gpio_set_function(0, 2);
    gpio_set_function(1, 2);
    sleep_ms(10);
    
    /*
    while(false){
        if(uart_is_enabled(uart0)){
            adc_select_input(0);
            fresult = adc_read();
            fresult = fresult/4048;
            fresult = sin(fresult * M_PI);
            sprintf(int_str, "%g", fresult);
            uart_puts (uart0,int_str);
            uart_puts (uart0,"\n");
            adc_select_input(1);
            fresult = adc_read();
            fresult = fresult/4048;
            fresult = sin(fresult * M_PI);
            sprintf(int_str, "%g", fresult);
            uart_puts (uart0,int_str);
            uart_puts (uart0,"\n");
            sleep_ms(500);
        }
    }
    */
    
    
    
    while (false){
    in_buf0[0] = 0x0a;
    in_buf0[1] = 0xc1;
    in_buf0[2] = 0x2d;
    cmaxi = (in_buf0[1] >> 7) & 0x1;
    cmdir = (in_buf0[1] >> 6) & 0x1;
    msdel = (in_buf0[1] & 0x3f) << 8;
    msdel = msdel + in_buf0[2];
    cmdel = msdel;
    sprintf (scmdel,"%d", cmdel);
    uart_puts (uart0,scmdel);
    uart_puts (uart0,"\nd:");
    sleep_ms(500);
    }
    
    adc_select_input(0);
    adcinr0 = adc_read();
    adc_select_input(1);
    adcinr1 = adc_read();
    if (false){
        while(ddid=='0'){
            if (spi_is_readable(spi0)){
                spi_read_blocking(spi0,0,sin_buf0,1);
                ddid=sin_buf0[0];
                
             
            }
            if (uart_is_enabled(uart0)){
                //uart_puts (uart0,"\nddid:");
                //uart_puts (uart0,ddid);
            }

        }
    }
    
    gpio_put(11,0);
    gpio_put(13,0);
    //spi_write_blocking(spi0,ddid,BUF_LEN);
    multicore_launch_core1(core1);
    while(true){
        //to do:    ok close loop control*2(X+Y) pid / inching / compairator
        //          indexing protocol   [id][/|\(x/y)][-|+(dir)][int(amount)][~(end)]
        //          ok calibrate 0   
        //          ok wrapping angle  using expected rotation dir/ full rot counting/ sine - cosine filter
        //          ok analogue in    using adc
        //          soft e stop
        //          
        
        
        if (false){
            if (spi_is_readable(spi0)){
                spi_read_blocking(spi0,0x00,in_buf0,BUF_LEN);
                //chin_buf0 = in_buf0;
                if (in_buf0[0] == ddid){
                    cmaxi = (in_buf0[1] >> 7) & 0x1;
                    cmdir = (in_buf0[1] >> 6) & 0x1;
                    msdel = (in_buf0[1] & 0x3f) << 8;
                    msdel |= in_buf0[2];
                    if (cmaxi){
                        if (cmdir){
                            tarpos0 = (int)msdel;
                        }
                        else{
                            tarpos0 = ((int)msdel)* -1 ;
                        }
                    }
                    
                    else{
                        if (cmdir){
                            tarpos1 = (int)msdel;
                        }
                        else{
                            tarpos1 = ((int)msdel)* -1 ;
                        }
                    }
                    /*
                    cmdel = msdel;
                    sprintf (scmdel,"%d", cmdel);
                    uart_puts (uart0,scmdel);
                    uart_puts (uart0,"\nd:");
                    */
                }
            }
        }
        
        
        
        
        if(true){
            if (uart_is_enabled(uart0)){
                if (uart_is_writable(uart0)){
                    sprintf (sresult,"ad0:%d  ad1:%d  ta0:%d  ta1:%d  cp0:%d  cp1:%d  tp0:%d  tp1:%d \n",adcinr0 , adcinr1 , temadcinr0 , temadcinr1 , curpos0 ,curpos1,tarpos0,tarpos1);
                    uart_puts (uart0,sresult);
                    sprintf (sresult,"0b%d\n",in_buf0);
                    uart_puts (uart0,sresult);
                }
                /*
                if (uart_is_readable(uart0)){
                    uin = uart_getc (uart0);
                    while('\n' != uin){
                        suin = suin*10 + ((int)uin);
                        uart_puts (uart0,"read\n");
                        uin = uart_getc (uart0);
                    }
                }
                */
                //uart_puts (uart0,"\n");
                //uart_puts (uart0,ddid);
                //gpio_put(11,1);
                //sleep_ms(3);
                //gpio_put(11,0);
            }
        }
        
        //printf ("ad0:%d  ad1:%d  ta0:%d  ta1:%d  cp0:%d  cp1:%d  tp0:%d  tp1:%d \n",adcinr0 , adcinr1 , temadcinr0 , temadcinr1 , curpos0 ,curpos1,tarpos0,tarpos1);
        //printf ("0b%d\n",in_buf0);
        //tarpos0 = suin;
        /**
        gpio_put(11,1);
        sleep_ms(3);
        gpio_put(11,0);
        **/
        //sleep_ms(10);
        /*
        tarpos0 = 5094;
        sleep_ms(1500);
        tarpos0 = -5094;
        sleep_ms(1500);
        */
		/*
        //sprintf (sresult,"0b%d\n",in_buf0);
        //uart_puts (uart0,"hi\n");
        //sleep_ms(100);
        //sprintf (sresult,"ad0:%d  ad1:%d  ta0:%d  ta1:%d  cp0:%d  cp1:%d  tp0:%d  tp1:%d \n",adcinr0 , adcinr1 , temadcinr0 , temadcinr1 , curpos0 ,curpos1,tarpos0,tarpos1);
        //sprintf(sresult,"sjfiewafji");
        uart_puts (uart0, sresult );
        uart_puts (uart0, "\n" );
        sleep_ms(1000);
		*/
    }
}
