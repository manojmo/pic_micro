/*
 * File:   tata_sky_remote_serial.c
 * Author: manoj
 * A remote for tata-sky settop box, the buttons are selected by sending a code over uart
 * Created on March 7, 2015, 6:01 PM
 */

#include <xc.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <conio.h>

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

#include <generic_uart.h> // needs to be after the defines

#define PIN_OUT LATD4 // output

//factor out common part to save space
uint16_t cmn_durations[] = {2686,889,444,444,444,444,444,889,444,889,889,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444,444};
uint8_t num_cmn_durations = sizeof(cmn_durations) / sizeof(cmn_durations[0]);
// part specific to each code. Use 0 to detect end
uint16_t all_durations[][14] = {
    {889,889,889,444,444,889,444,444,444,444,444,0}, // up
    {889,889,889,444,444,889,444,444,889,0}, //down
    {889,889,889,444,444,889,889,444,444,0}, //right
    {889,889,889,444,444,889,889,889,444,0}, //left
    {444,444,889,889,444,444,444,444,444,444,444,444,444,0}, //ch+
    {444,444,889,889,444,444,444,444,444,444,889,0}, //ch-
    {444,444,444,444,889,889,444,444,444,444,444,444,444,0}, //v+
    {444,444,444,444,889,889,444,444,444,444,889,0}, //v-
    {889,889,889,444,444,444,444,889,444,444,444,0}//select
};
// working copy
uint16_t durations[50];
uint16_t num_durations = sizeof(durations) / sizeof(durations[0]);

// printf uses this method
void putch(char data) {
    uart_write(data);
}

// upto 21845 us only
void delay_uS( uint16_t times){
    if( times <= 0){
        return;
    }
    //TMR1IE = 1; // enable timer0 interrupt
    // See timer1 setup. With FOSC = 48Mhz, Ftimer = 12Mhz, prescaler = 4,
    // 3 instruction cycles = 3 timer counts = 1uS.
    // Max uS without overflow = 65535/3 = 21845
    TMR1H=0;
    TMR1L=0;
    if( times > 21845){
        times = 21845;
    }
    uint16_t end_count = (times * 3) -3; // -3 : compensate for other code exec
    uint8_t end_count_lo = (uint8_t) end_count;
    uint8_t end_count_hi = end_count >>8;

    // note : if end_count is very near overflow, i.e FFFF, we need to check for overflow too
    while( TMR1L < end_count_lo || TMR1H < end_count_hi){
    }
    //t1_expired_times = 0; // reset counter for next delay
    //TMR1IE = 0; // Disable timer0 interrupt
}

// milli sec delay. Not very accurate
void delay_mS( uint16_t times){
    while( times >= 10){
        __delay_ms(10);
        times = times - 10;
    }
    while( times > 0){
        __delay_ms(1);
        times--;
    }
}

// keep as signed int, as the subtraction can lead to -ve number
// The accuracy of timings is important here, the overhead of other instructions
// should be factored in.
void IOPulsedHigh( int duration){
    //printf("IOPH,D:%u", duration );
    while( duration > 0){
        //TMR0H =0;
        //TMR0L = 0;
        PIN_OUT = 1;
        __delay_us(11);
        PIN_OUT = 0;
        __delay_us(13);
        duration = duration - 26;
        //printf( "ACT:%u", (TMR0H<<8|TMR0L)/3 );
    }
}

// populate the durations for the specific code
void set_code_durations( char code[], uint16_t result[], uint8_t size){
    uint8_t i;
    // common durations
    for( i=0; i < num_cmn_durations; i++){
        result[i] = cmn_durations[i];
    }
    // durations for code
    uint8_t code_idx = ((uint8_t) code[0] ) - 48; // 48 is ascii zero
    uint16_t * code_durations = all_durations[ code_idx];
    uint8_t j = 0;
    while( code_durations[j] != 0){
        result[i] = code_durations[j];
        i++;
        j++;
    }
    // clear the rest
    for( ; i < size; i++){
        result[i] = 0;
    }

}
/*
 *
 */
int main(int argc, char** argv)
{
    TRISDbits.RD4 = 0; // output
    // Use timer0 for debug/performance
    T0CON = 0b10000001; // internal clock FOSC/4 , 16bit, prescaler = 1/4, Timer enabled
    TMR0IE = 0; // No interupt on overflow

    // Use timer1 for delays
    T1CON = 0b10100001; // internal clock FOSC/4 , prescaler = 1/4, Timer enabled
    TMR1IE = 0; // No interupt on overflow

    char code[2];
    uint16_t duration = 0;
    int idx;

    uart_init(1); // invert
    uart_start_tx(); // all printfs should be after this
    //printf("Start", 5);
    // Wait in loop awaiting command
    while(1) {
        printf("Enter code");
        uart_read( 1, code);
        printf("Uart read:%c", code[0]);
        set_code_durations( code, durations, num_durations);
        //printf("1st code dur:%u", durations[36]);
        // send each code twice
        uint8_t j;
        for( j=0; j< 2; j++ ){

            bool state = 1; // start with HIGH

            for( idx = 0;  idx < num_durations; idx++ ) {
                TMR0H =0;
                TMR0L = 0;
                duration = durations[idx];

                if( duration > 0 ){
                    // If we need a pulsed high state, e.g for IR remote
                    if( state == 1 ){
                        IOPulsedHigh( duration);
                    }
                    else {
                        PIN_OUT = state;
                        delay_uS(duration);
                    }
                }
                else {
                    // 0 duration means end
                    break;
                }
                state = ! state;
                //printf( "S:%u,D:%uACT:%u", !state, duration, (TMR0H<<8|TMR0L)/3 );
            }

            PIN_OUT = 0;

            // wait for 65 ms before resending
            for( idx = 0; idx< 13; idx++){
                __delay_ms(5);
            }
            //printf( "after resend delay" );
        }

        // wait a sec before accepting another key ?
        for( idx = 0; idx< 1000; idx++){
            __delay_ms(1);
        }


    }
}
