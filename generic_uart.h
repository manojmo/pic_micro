/*
 * File:   generic_uart.h
 * Author: manoj
 * generic uart read/write
 * Created on March 11, 2015, 6:01 PM
 */

#include <stdint.h>

// flexible way to set some params.
void uart_init( uint8_t is_inverted);

// start of transmission
void uart_start_tx();

// end of transmission
void uart_end_tx();

// write a char
void uart_write( char c);

// Read specified number of chars and put them into holder array
void uart_read( uint8_t len, char holder[] );

// write a string, optionally of specified len.
// if null terminated, len can be -1
void uart_writestr( const char str[], int len );
