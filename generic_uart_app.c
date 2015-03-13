/*
 * File:   main.c
 * Author: manoj
 * App to read a 2 byte command from uart, execute it, and echo back the command
 * Created on March 1, 2015, 6:01 PM
 */

#include <xc.h>
#include <string.h>
#include <stdint.h>
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

// process the command we received
void process_cmd(char cmd[] ){
    if( strncmp( cmd, "TG", 2) == 0){
        LATD1 ^= 1;
    }
}

/*
 *
 */
int main(int argc, char** argv)
{
    TRISDbits.RD1 = 0; // command output

    uart_init(1); // invert
    uart_start_tx();
    char cmd[2]; // array to hold received command.
    while(1){
       uart_read( 2, cmd); // read a command, specified bytes long
       process_cmd( cmd);
       uart_writestr( cmd, 2); // echo it back
    }
}
