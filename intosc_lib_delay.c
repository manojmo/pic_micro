/* 
 * File:   intosc_lib_delay.c
 * Author: manoj
 * A simple LED delay, using library delay routines and internal oscillator
 * and reading from EEPROM memory
 * Created on September 27, 2014, 6:01 PM
 */

#include <xc.h>
#include <stdint.h>

// PLL with pre-scaler and post-scale options is a way to derive multiple
// freq from the same source, e.g. for low=6Mhz/high=48Mhz speed USB and for MCU clock
// Input to PLL has to be 4 Mhz and its output is 96 MHz
// So, for e.g. if we are using exernal 20 MHz osc, its o/p has to be
// divided by 5 to get 4 Mhz input for PLL
// PLLDIV is prescaler, o/p has to be 4MHz
// CPUDIV and USBDIV are postscalers, input is 96Mhz
//#pragma config PLLDIV   = 5
//#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2 = 48MHz
#pragma config FOSC     = INTOSC_HS // Internal OSC
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config WDT = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config BOR = ON
#pragma config MCLRE = ON
#pragma config PWRT = OFF
#pragma config PBADEN = OFF

// Effective CPU Freq, considering PLL and CPUDIV. Needed for the delay routines
#define _XTAL_FREQ 8000000


//DATA to be written to EEPROM
__EEPROM_DATA(5,0,0,0,0,0,0,0);

void delay_10ms_times( unsigned int times) {
    int cnt;
    for( cnt=0; cnt< times; cnt++){
        __delay_ms(10);
    }
}
/*
 * 
 */
int main(int argc, char** argv)
{
    // read from EEPROM
    unsigned char loopTimes = eeprom_read (0);

    TRISDbits.TRISD1 = 0; // Configure RD1 as output
    SCS1=1; // Internal Oscillator
    IRCF2 = 1; IRCF1 = 1; IRCF0 = 1;// IRCF2:0 = 111, i.e. no scaler to internal clock

    uint8_t i;
    for( i=0; i< loopTimes; i++ )
    {
        LATDbits.LATD1 = 1; // Turn RD1 ON
        delay_10ms_times(100);
        LATDbits.LATD1 = 0; // Turn RD1 OFF
        delay_10ms_times(100);
    }
    // A longer delay to indicate end of loop
    LATDbits.LATD1 = 1;
    delay_10ms_times(400);
    LATDbits.LATD1 = 0;
    delay_10ms_times(100);
    #asm
    sleep;
    #endasm
    return 0;
    
}
