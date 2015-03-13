/*
 * File:   read_pin_durations.c
 * Author: manoj
 * reading durations of ON/OFFs on a pin
 * Created on March 5, 2015, 6:01 PM
 */

#include <xc.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <conio.h>
#include <generic_uart.h>


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

#define PIN_SENSOR PORTDbits.RD2 // input
#define IDLE_STATE 1
#define NUM_DURATIONS 100

// printf uses this method
void putch(char data) {
    uart_write(data);
}

/*
 *
 */
int main(int argc, char** argv)
{
    TRISDbits.RD2 = 1; // Sensor pin. set as input
    TRISDbits.RD1 = 0; // indicator. output

    // Use timer0 for debug/performance
    T0CON = 0b10000001; // internal clock FOSC/4 , 16bit, prescaler = 1/4, Timer enabled
    TMR0IE = 0; // No interupt on overflow

    uint8_t pin_state = 0;
    uart_init(1); // invert
    uart_start_tx();
    uint16_t durations[ NUM_DURATIONS];
    uint8_t states[ NUM_DURATIONS];
    uint16_t curr_sample_duration = 0;
    uint8_t curr_sample_num = 0;
    uint8_t max_delay_lapsed = 0;
    uint8_t i = 0;
    // Flash to indicate start
    LATD1 = 1;
    for( i=0; i< 200; i++ ){
        __delay_ms(1);
    }
    LATD1 = 0;
    // Let the system stabilize ?
    // TODO : Give it separate trigger on another pin to start ?
    for( i=0; i< 200; i++ ){
        __delay_ms(10);
    }

    printf( "Starting..\n");

    // Wait till the state changes from idle state
    while( PIN_SENSOR == IDLE_STATE);

    while( curr_sample_num < NUM_DURATIONS) {

        // reset timer
        TMR0H = 0;
        TMR0L = 0;

        pin_state = PIN_SENSOR;
        states[curr_sample_num] = pin_state;
        while( PIN_SENSOR == pin_state) {
            __delay_us( 5);
            curr_sample_duration = curr_sample_duration + 5;
            if( curr_sample_duration >= 0xFFFF){
                // Max duration reported is 0xFFFF
                curr_sample_duration = 0xFFFF;
                max_delay_lapsed = 1;
                break;
            }
        }
        // Using the timer-value is most accurate; just the delay does not count other code-exec.
        durations[curr_sample_num] = max_delay_lapsed ? 0xFFFF : (TMR0H << 8 | TMR0L)/3;

        curr_sample_duration = 0;
        curr_sample_num ++;
        max_delay_lapsed = 0;

    }
    for( i=0; i< NUM_DURATIONS; i++){
        printf( "S:%u,D:%u,", states[i], durations[i] ); // u for unsigned
    }
    printf( "Done..\n");

}
