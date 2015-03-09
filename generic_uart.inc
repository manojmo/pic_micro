/*
 * File:   generic_uart.c
 * Author: manoj
 * generic uart read/write
 * Created on September 27, 2014, 6:01 PM
 */

uint8_t UART_INVERTED = 0; // If inverted, low and high states will be inverted
uint8_t UART_HIGH_STATE = 1;
uint8_t UART_LOW_STATE = 0;

// flexible way to set some params.
void uart_init( uint8_t is_inverted){
    UART_INVERTED = is_inverted;
    if( is_inverted){
        UART_HIGH_STATE = 0;
        UART_LOW_STATE = 1;
    }
    PIN_UART_TX_MODE = 0;//output
    PIN_UART_RX_MODE = 1;//input
}

// start of transmission
void uart_start_tx(){
    PIN_UART_TX = UART_HIGH_STATE;
}

// end of transmission
void uart_end_tx(){
    PIN_UART_TX = UART_HIGH_STATE;
}

// write a char
void uart_write( char c){

    // start bit
    PIN_UART_TX = UART_LOW_STATE;
    __delay_us( UART_BIT_TIME);

    uint8_t bit_pos = 0;
    // transmit bits, LSB first
    while (bit_pos < 8) {
        uint8_t currbit =  c & 0x01;
        PIN_UART_TX = UART_INVERTED ? ! currbit : currbit;
        __delay_us( UART_BIT_TIME);
        bit_pos++;
        c = c >> 1;
    }
    // stop bit
    PIN_UART_TX = UART_HIGH_STATE;
    __delay_us( UART_BIT_TIME);
}

// Read specified number of chars and put them into holder array
void uart_read( uint8_t len, char holder[] ){
    uint8_t i;
    uint8_t bit_pos;

    // loop and read len number of chars.
    for( i=0; i< len; i++){

        // Wait for idle state to change, i.e. start bit
        while( PIN_UART_RX == UART_HIGH_STATE);

        // start bit
        __delay_us( UART_BIT_TIME);

        // read bits, LSB first
        bit_pos = 0;
        char c = 0;
        while (bit_pos < 8) {
            uint8_t currbit = UART_INVERTED ? ! PIN_UART_RX : PIN_UART_RX;
            c = c | (currbit << bit_pos);
            __delay_us( UART_BIT_TIME);
            bit_pos++;
        }
        holder[i] = c;


        // stop bit
        __delay_us( UART_BIT_TIME);
    }
}

// write a string, optionally of specified len.
// if null terminated, len can be -1
void uart_writestr( char str[], int len ){
    int i;
    for( i=0; i< len || len == -1; i++){
        char curr = str[i];
        if( curr == '\0'){
            break;
        }
        uart_write( curr);
    }
}
