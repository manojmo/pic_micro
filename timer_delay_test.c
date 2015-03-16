/*
 * File:   timer_delay_test.c
 * Author: manoj
 * Creating and measuring long delays
 * Created on March 5, 2015, 6:01 PM
 */

#include <xc.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <generic_uart.h> // needs to be after the defines

// PLL with pre-scaler and post-scale options is a way to derive multiple
// freq from the same source, e.g. for low=6Mhz/high=48Mhz speed USB and for MCU clock
// Input to PLL has to be 4 Mhz and its output is 96 MHz
// So, for e.g. if we are using exernal 20 MHz osc, its o/p has to be
// divided by 5 to get 4 Mhz input for PLL
// PLLDIV is prescaler, o/p has to be 4MHz
// CPUDIV and USBDIV are postscalers, input is 96Mhz

#pragma config PLLDIV   = 5         // (20 MHz crystal on PICDEM FS USB board)
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS

#pragma config IESO = OFF
#pragma config WDT = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config BOR = ON
#pragma config MCLRE = ON
#pragma config PWRT = OFF
#pragma config PBADEN = OFF

#define PIN_OUT LATD4 // output

// printf uses this method
void putch(char data) {
    uart_write(data);
}

const uint16_t MAX_TIMER1_CNT = 21845;
void delay_uS( unsigned long times){
    while( times > 0){
        // See timer setup. 3 instruction cycles = 3 timer counts = 1uS.
        // Max uS without overflow = 65535/3 = 21845
        TMR1IF = 0;
        TMR1H=0;
        TMR1L=0;
        uint16_t curr_times = min( times, MAX_TIMER1_CNT);
        uint16_t end_count = (curr_times * 3) -3; // -3 : compensate for code exec
        uint8_t end_count_lo = end_count & 0xFF;
        uint8_t end_count_hi = end_count >>8;
        times = times - curr_times;
        //printf( "c:%u", curr_times); // printf has issue printing together.
        //printf( "e:%u", end_count);
        //printf( "l:%u", end_count_lo);
        //printf( "h:%u", end_count_hi);
        // we need to take care of overflow too
        while( TMR1IF == 0 && (TMR1L < end_count_lo || TMR1H < end_count_hi) );
    }
}

// milli sec delay
void delay_mS( uint24_t times){
    while( times >= 10){
        __delay_ms(10);
        times = times - 10;
    }
    while( times > 0){
        __delay_ms(1);
        times--;
    }
}

uint8_t t0_expired_times = 0;

// Interrupt service Routine
void interrupt myIsr(void)
{
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        t0_expired_times++;
        INTCONbits.TMR0IF = 0; // clear this interrupt condition
    }
}


void reset_timer(){
    TMR0ON = 1; // enable/start
    t0_expired_times = 0;
    TMR0H = 0;
    TMR0L = 0;
}

unsigned long get_timer_value(){
    uint16_t tmr = TMR0H << 8 | TMR0L;
    unsigned long value = (t0_expired_times * (0x010000)) + tmr;
    return value;
}
/*
 *
 */
int main(int argc, char** argv)
{
    ei();
    TRISDbits.RD4 = 0; // output
    // Use timer0 for debug/performance
    T0CON = 0b10000001; // internal clock FOSC/4 , 16bit, prescaler = 1/4, Timer enabled
    TMR0IE = 1; // interupt enabled
    // Use timer1 for delays
    T1CON = 0b10100001; // internal clock FOSC/4 , prescaler = 1/4, Timer enabled
    TMR1IE = 1; // interupt enabled

    uart_init(1);
    uart_start_tx();
    printf( "ST!");
    uint8_t num_exp = 0;
    unsigned long timer_val;
    reset_timer();
    __delay_us(6);
    unsigned long timer_val = get_timer_value();
    num_exp = t0_expired_times;
    printf( "6ACT:%ld,t0et:%d!", timer_val, num_exp );
    reset_timer();
    __delay_us(600);
    timer_val = get_timer_value();
    num_exp = t0_expired_times;
    printf( "600ACT:%ld,t0et:%d!", timer_val, num_exp );
    reset_timer();
    __delay_us(10000);__delay_us(10000);__delay_us(10000);
    timer_val = get_timer_value();
    num_exp = t0_expired_times;
    printf( "30000ACT:%ld,t0et:%d!", timer_val, num_exp );

    unsigned long arr_testnums[] = {5, 50, 100, 1000, 10000, 100000};
    uint8_t i;
    unsigned long test_dur;
    for( i=0; i< 6; i++){
        test_dur = arr_testnums[i];
        printf( "test_dur:%ld!", test_dur);
        reset_timer();
        delay_uS(test_dur);
        num_exp = t0_expired_times;
        timer_val = get_timer_value()/3;
        printf( "Aft delay,t0et:%d!,x:%ld", num_exp, num_exp * 0x010000);
        // Seems to be a bug n printf, cannot accept multiple/large uint32 args ?
        printf( "ACT:%ld!", timer_val );
    }
    while(1);
}
