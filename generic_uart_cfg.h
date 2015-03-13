/* 
 * File:   generic_uart_cfg.h
 * Author: manoj
 * This file should be with the project files, and not common includes,
 * as the config can change with each project
 * Created on 12 March, 2015, 5:05 PM
 */

// Effective CPU Freq, considering PLL and CPUDIV. Needed for the delay routines
// ideally , this should be defined as project level globals
//#define _XTAL_FREQ

#define UART_BIT_TIME 104 // us. inverse gives the bit-rate
#define PIN_UART_TX LATC6 // output
#define PIN_UART_RX PORTCbits.RC7 // input
#define PIN_UART_TX_MODE TRISCbits.RC6 // ouput
#define PIN_UART_RX_MODE TRISCbits.RC7 // input

