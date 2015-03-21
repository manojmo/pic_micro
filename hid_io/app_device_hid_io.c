/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

/** INCLUDES *******************************************************/
#include <usb/usb.h>
#include <usb/usb_device_hid.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <system.h>
#include <generic_uart.h>
#include "app_device_hid_io.h"


/** VARIABLES ******************************************************/
/* Some processors have a limited range of RAM addresses where the USB module
 * is able to access.  The following section is for those devices.  This section
 * assigns the buffers that need to be used by the USB module into those
 * specific areas.
 */
#if defined(FIXED_ADDRESS_MEMORY)
    #if defined(COMPILER_MPLAB_C18)
        #pragma udata HID_CUSTOM_OUT_DATA_BUFFER = HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS
        unsigned char ReceivedDataBuffer[64];
        #pragma udata HID_CUSTOM_IN_DATA_BUFFER = HID_CUSTOM_IN_DATA_BUFFER_ADDRESS
        unsigned char ToSendDataBuffer[64];
        #pragma udata

    #else defined(__XC8)
        unsigned char ReceivedDataBuffer[64] @ HID_CUSTOM_OUT_DATA_BUFFER_ADDRESS;
        unsigned char ToSendDataBuffer[64] @ HID_CUSTOM_IN_DATA_BUFFER_ADDRESS;
    #endif
#else
    unsigned char ReceivedDataBuffer[64];
    unsigned char ToSendDataBuffer[64];
#endif

volatile USB_HANDLE USBOutHandle;    
volatile USB_HANDLE USBInHandle;
// Buffer to hold extended requests i.e. >64 and <128 bytes.
// we could probably increase ReceivedDataBuffer itself, but it has fixed address in mem
unsigned char ReceivedDataBuffer_Ext[128];
uint8_t incomplete_command = 0;
uint8_t t1_expired_times = 0;
bool max_delay_lapsed = 0;

typedef enum
{ 
    PIN_MODE_OUTPUT = 0,
    PIN_MODE_INPUT = 1
} PIN_MODES;

/** DEFINITIONS ****************************************************/
typedef enum
{
    COMMAND_IO_INIT = 0x01,
    COMMAND_IO_WRITE_PIN = 0x11,
    COMMAND_IO_READ_PIN = 0x12,
    COMMAND_IO_READ_PIN_DURATIONS = 0x13,
    COMMAND_IO_READ_ADC = 0x14
} CUSTOM_HID_DEMO_COMMANDS;

volatile uint8_t t0_expired_times = 0;

#define T0_TICKS_PER_US 3

/** FUNCTIONS ******************************************************/

/*********************************************************************
* Function: void APP_DeviceCustomHIDInitialize(void);
*
* Overview: Initializes the Custom HID demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDInitialize()
{
    //initialize the variable holding the handle for the last
    // transmission
    USBInHandle = 0;

    //enable the HID endpoint
    USBEnableEndpoint(CUSTOM_DEVICE_HID_EP, USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = (volatile USB_HANDLE)HIDRxPacket(CUSTOM_DEVICE_HID_EP,(uint8_t*)&ReceivedDataBuffer,64);
}

// printf uses this method
void putch(char data) {
    TMR0IE = 0; TMR0ON = 0; // Timer intereferes with this, so disable.
    uart_write(data);
}

/*
void delay_uS( unsigned long times){
    while( times >= 1000){
        __delay_us(1000);
        times = times - 1000;
    }
    while( times >= 100){
        __delay_us(100);
        times = times - 100;
    }
    while( times >= 10){
        __delay_us(10);
        times = times - 10;
    }
    while( times > 0){
        __delay_us(1);
        times--;
    }
}
*/

/*
// upto 21845 us only
void delay_uS( unsigned long times){
    if( times <= 0){
        return;
    }
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

    while( TMR1L < end_count_lo || TMR1H < end_count_hi){
    }
}
*/


// we deliberately keep MAX_TIMER1_CNT a bit low, so that we don't have to check for a timer overflow
const uint16_t MAX_TIMER1_CNT = 20000;
void delay_uS( unsigned long times){
    TMR1ON = 1; // enable timer
    while( times > 0){
        // See timer setup. T0_TICKS_PER_US instruction cycles = T0_TICKS_PER_US timer counts = 1uS.
        // Max uS without overflow = 65535/T0_TICKS_PER_US
        TMR1H=0;
        TMR1L=0;
        TMR1IF=0;
        uint16_t curr_times = min( times, MAX_TIMER1_CNT);
        uint16_t end_count = (curr_times * T0_TICKS_PER_US) -3; // -3 : compensate for code exec
        uint8_t end_count_lo = end_count & 0xFF;
        uint8_t end_count_hi = end_count >>8;
        times = times - curr_times;
        //printf( "c:%u", curr_times); // printf has issue printing together.
        //printf( "e:%u", end_count);
        //printf( "l:%u", end_count_lo);
        //printf( "h:%u", end_count_hi);
        // we need to take care of overflow too
        while( TMR1IF == 0 && ( TMR1L < end_count_lo || TMR1H < end_count_hi) );
    }
    TMR1ON = 0; // disable timer
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

// Interrupt service Routine. Should take as less time as possible.
// system.c already has defined a high-priority isr for usb,
// use a low priority isr for others
void interrupt low_priority myLoISR(void)
{
    // only process timer-triggered interrupts
    if( INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        t0_expired_times++;
        INTCONbits.TMR0IF = 0; // clear this interrupt condition
    }
}


void start_timer(){
    TMR0ON = 1; // enable/start
    TMR0IE = 1;
    t0_expired_times = 0;
    TMR0H = 0;
    TMR0L = 0;
}

void stop_timer(){
    TMR0ON = 0;
    TMR0IE = 0;
}

unsigned long get_timer_value(){
    //printf("t0et:%lu", t0_expired_times);
    uint16_t tmr = TMR0H << 8 | TMR0L;
    unsigned long value = (t0_expired_times * (0x010000)) + tmr;
    return value;
}


uint16_t bytes2_to_int( unsigned char msb, unsigned char lsb){
    uint16_t result = (msb << 8) | lsb;
    return result;
}

void IOSetPinMode( uint8_t pin_num, uint8_t mode) {
    switch( pin_num) //
    {
        case 0xd1:
            TRISDbits.TRISD1 = mode;
            break;
        case 0xd2:
            TRISDbits.TRISD2 = mode;
            break;
        case 0xd3:
            TRISDbits.TRISD3 = mode;
            break;
        case 0xd4:
            TRISDbits.TRISD4 = mode;
            break;
    }

}


void IOSetPinState( uint8_t pin_num, bool on_off) {
    switch( pin_num) //
    {
        case 0xd1:
            LATD1 = on_off;
            break;
        case 0xd2:
            LATD2 = on_off;
            break;
        case 0xd3:
            LATD3 = on_off;
            break;
        case 0xd4:
            LATD4 = on_off;
            break;

    }
}

uint8_t IOGetPinState( uint8_t pin_num) {
    uint8_t ret = 0;
    switch( pin_num) //
    {
        case 0xd1:
            ret = PORTDbits.RD1;
            break;
        case 0xd2:
            ret = PORTDbits.RD2;
            break;
       case 0xd3:
            ret = PORTDbits.RD3;
            break;
       case 0xd4:
            ret = PORTDbits.RD4;
            break;

    }
    return ret;
}


// Pulsed high state, like that used for Infrared remote etc
// The actual times in this code are much more : 17 for 9 and 26 for 18 !!
// so an overhead of 8us in the loop
// By keeping hi + lo < pulse duration, we can compensate for this delay
// keep duration as long, since it may go -ve
void IOPulsedHigh( uint8_t pin_num, long duration, uint16_t pulse_duration,
        uint16_t pulse_high_duration, uint16_t pulse_low_duration, bool is_millis){
    //printf("IOPH,D:%u", duration );
    while( duration > pulse_high_duration){
        IOSetPinState( pin_num, 1);
        is_millis ? delay_mS( pulse_high_duration) : delay_uS(pulse_high_duration);
        IOSetPinState( pin_num, 0);
        is_millis ? delay_mS( pulse_low_duration) : delay_uS(pulse_low_duration);
        duration = duration - pulse_duration;
    }
    //printf("IOPHEND");
}

void IOWritePattern( uint8_t* cmd_args) {
    start_timer();
    delay_uS(60);
    uint16_t tmv = TMR0H << 8|TMR0L;
    printf( "60ACT:%u\r\n",tmv);

    uint8_t pin_num = cmd_args[1];
    IOSetPinMode( pin_num, PIN_MODE_OUTPUT);
    uint8_t flags = cmd_args[2];
    uint8_t carrier_interval = cmd_args[3]; // If set, the Highs are actually on-offs at carrier freq
    uint8_t carrier_duty_cycle_on = cmd_args[4]; //part of carrierInterval in hi state
    uint8_t carrier_duty_cycle_off = cmd_args[5];
    bool is_extended = flags & 1; // 2 packets to be received
    bool is_repeat = flags & 2; // repeat pattern
    uint8_t num_repeats = is_repeat ? cmd_args[6] : 1; // a value of 0 means infinite repeats
    bool is_millis = flags & 4; // durations are in milli seconds
    bool is_reset = flags & 8; // should we reset the pin after the pattern is written ?
    uint16_t duration = 0;
    uint8_t curr_duration_num = 0;
    bool state;
    int idx;
    const uint8_t DURATION_START_INDEX = 7;
    uint8_t j;
    printf( "Carrier:t:%u,h:%u,l:%u\r\n", carrier_interval,  carrier_duty_cycle_on, carrier_duty_cycle_off);
    // repeat pattern loop
    for( j=0; j< num_repeats || num_repeats == 0; j++ ){
        // Command ends with a duration of 0
        // Now each 2 bytes will have the on/off durations, starting with on.
        // the duration will be in uS. A duration of 0 will be ignored if at the start.
        state = 1; // start with HIGH
        for( idx = DURATION_START_INDEX;  idx < 128-1; idx=idx+2 ) {
            start_timer();
            // combine 2 bytes to form the duration
            duration = cmd_args[idx] << 8;
            duration = duration | cmd_args[idx+1];
            if( duration == 0 && idx != DURATION_START_INDEX ){
                // end of command data
                break;
            }
            else if( duration > 0 ){
                // If we need a pulsed high state, e.g for IR remote
                if( state == 1 && carrier_interval > 0){
                    IOPulsedHigh( pin_num, duration, carrier_interval, carrier_duty_cycle_on,
                            carrier_duty_cycle_off, is_millis);
                }
                else {
                    IOSetPinState( pin_num, state);
                    is_millis ? delay_mS(duration) : delay_uS(duration);
                }
            }
            state = ! state;
            curr_duration_num ++;
            unsigned long timer_value_us = get_timer_value()/T0_TICKS_PER_US;
            stop_timer();
            printf( "DUR:%u,ACT:%lu,S:%d\r\n", duration, timer_value_us, !state);
        }
        printf( "Rpt\r\n");
    }
    if ( is_reset){
        IOSetPinState( pin_num, 0);
    }
}


/*********************************************************************
* Function: void APP_DeviceCustomHIDTasks(void);
*
* Overview: Keeps the Custom HID demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCustomHIDInitialize() and APP_DeviceCustomHIDStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCustomHIDTasks()
{
    //uart_write('1');
    //Check if we have received an OUT data packet from the host
    if(HIDRxHandleBusy(USBOutHandle) == false)
    {
        uint8_t eff_cmd = ReceivedDataBuffer[0];
        //uart_write('C');
        //uart_write(eff_cmd);

        if( incomplete_command > 0){
            eff_cmd = incomplete_command;
        }
        //We just received a packet of data from the USB host.
        //Check the first uint8_t of the packet to see what command the host
        //application software wants us to fulfill.
        switch( eff_cmd ) //Look at the data the host sent, to see what kind of application specific command it sent.
        {
            case COMMAND_IO_WRITE_PIN:  // Write 0,1s pattern to the specified pin
                /*LATD1 = 1;
                delay_mS(100);
                LATD1 = 0;*/
                //uart_writestr("WP", -1);

                // Check if more data is to come, i.e extended flag
                if( (ReceivedDataBuffer[2] & 1) == 1 && incomplete_command == 0){
                    incomplete_command = COMMAND_IO_WRITE_PIN;
                    // copy buffer
                    uint8_t i;
                    for( i=0; i< 64; i++){
                        ReceivedDataBuffer_Ext[i] = ReceivedDataBuffer[i];
                    }
                    //uart_writestr("ic1", -1);
                }
                else if ( incomplete_command > 0 ){
                    // We have the 2nd packet now
                    incomplete_command = 0;
                    uint8_t i;
                    // copy buffer
                    for( i=0; i< 64; i++){
                        ReceivedDataBuffer_Ext[i+64] = ReceivedDataBuffer[i];
                    }
                    //uart_writestr("ic2", -1);
                    // execute with multiple-packets-data
                    IOWritePattern(ReceivedDataBuffer_Ext);
                }
                else if ( incomplete_command == 0 ){
                    //uart_writestr("ic3", -1);
                    // Single packet execution
                    IOWritePattern(ReceivedDataBuffer);
                }
                break;

            case COMMAND_IO_READ_PIN:  // read and report state of pin
                ToSendDataBuffer[0] = COMMAND_IO_READ_PIN; //Echo back to the host the command we are fulfilling
                uint8_t pin_num = ReceivedDataBuffer[1];
                uint8_t flags = ReceivedDataBuffer[2];
                uint8_t idle_state = flags & 1;
                bool is_debug = flags & 2; // send back some debug data
                bool is_millis = flags & 4; // time is in millis
                IOSetPinMode( pin_num, PIN_MODE_INPUT);
                uint16_t num_samples = bytes2_to_int( ReceivedDataBuffer[3], ReceivedDataBuffer[4]);
                uint16_t sample_interval = bytes2_to_int( ReceivedDataBuffer[5], ReceivedDataBuffer[6]);
                uint16_t curr_sample_num = 0;
                uint8_t timer_val_lo = 0;
                uint8_t timer_val_hi = 0;
                uint8_t send_buff_pos = 2; // 1st 2 bytes reserved

                if( is_debug){
                    TMR0ON = 1;
                }
                // Wait till the state changes from idle state
                while( IOGetPinState( pin_num) == idle_state){
                }
                
                while( curr_sample_num < num_samples ) {
                    // Reset timer
                    TMR0H = 0;
                    TMR0L = 0;
                    
                    ToSendDataBuffer[send_buff_pos] = IOGetPinState(pin_num);
                    is_millis ? delay_mS( sample_interval) : delay_uS( sample_interval);
                    send_buff_pos ++;
                    curr_sample_num ++;

                    timer_val_lo = TMR0L + 0; // Capture low first as this will buffer the high value
                    timer_val_hi = TMR0H + 0;

                    // Check if we have filled the buffer. If yes, transmit
                    if( send_buff_pos >= 64 || curr_sample_num >= num_samples) {
                        //Wait till the endpoint/buffer is free
                        while( HIDTxHandleBusy(USBInHandle)){
                        }

                        ToSendDataBuffer[1] = send_buff_pos; // how many bytes are relevant
                        if( is_debug){
                            ToSendDataBuffer[3] = 1; // Testing only !!
                            ToSendDataBuffer[2] = 1; // The actual timer count
                            TMR0H = 0; //reset
                            TMR0L = 0;
                            delay_uS(sample_interval);
                            ToSendDataBuffer[5] = TMR0L; // Testing only !!
                            ToSendDataBuffer[4] = TMR0H; // The actual timer count
                        }

                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ToSendDataBuffer[0],64);
                        send_buff_pos = 2; // reset pos
                    }
                }
                TMR0ON = 0;
                break;

            case COMMAND_IO_READ_PIN_DURATIONS:  // read and report durations of highs and lows
                ToSendDataBuffer[0] = COMMAND_IO_READ_PIN_DURATIONS; //Echo back to the host the command we are fulfilling
                uint8_t pin_num = ReceivedDataBuffer[1];
                uint8_t flags = ReceivedDataBuffer[2];
                uint8_t idle_state = flags & 1;
                bool is_debug = flags & 2; // send back some debug data
                bool is_millis = flags & 4; // time is in millis
                IOSetPinMode( pin_num, PIN_MODE_INPUT);
                uint24_t curr_sample_duration = 0; // This needs to be > 16b, so that we don't overflow
                uint16_t num_samples = bytes2_to_int( ReceivedDataBuffer[3], ReceivedDataBuffer[4]);
                uint16_t sample_interval = bytes2_to_int( ReceivedDataBuffer[5], ReceivedDataBuffer[6]);
                uint16_t curr_sample_num = 0;
                uint8_t timer_val_lo = 0;
                uint8_t timer_val_hi = 0;
                uint8_t send_buff_pos = 2; // 1st 2 bytes reserved
                uint8_t pin_state = 0;

                if( is_debug){
                    TMR0ON = 1;
                }
                // Wait till the state changes from idle state
                while( IOGetPinState( pin_num) == idle_state){
                }

                while( curr_sample_num < num_samples ) {
                    // Reset timer
                    TMR0H = 0;
                    TMR0L = 0; // Write low last, as the high is captured then

                    pin_state = IOGetPinState( pin_num);
                    while( pin_state == IOGetPinState( pin_num)){
                        is_millis ? delay_mS( sample_interval) : delay_uS( sample_interval);
                        curr_sample_duration = curr_sample_duration + sample_interval;
                        if( curr_sample_duration >= 0xFFFF){
                            // Max duration reported is 0xFFFF
                            curr_sample_duration = 0xFFFF;
                            max_delay_lapsed = 1;
                            break;
                        }
                    }

                    timer_val_lo = TMR0L + 0; // Capture low first as this will buffer the high value
                    timer_val_hi = TMR0H + 0;

                    ToSendDataBuffer[send_buff_pos] =  curr_sample_duration >> 8;     //MSB
                    ToSendDataBuffer[send_buff_pos+1] = (uint8_t) curr_sample_duration; //LSB
                    send_buff_pos = send_buff_pos + 2;
                    curr_sample_num ++;
                    curr_sample_duration = 0;


                    // Check if we have filled the buffer. If yes, transmit
                    if( max_delay_lapsed || send_buff_pos > 63 || curr_sample_num == num_samples) {
                        //Wait till the endpoint/buffer is free
                        while( HIDTxHandleBusy(USBInHandle)){
                        }

                        ToSendDataBuffer[1] = send_buff_pos; // how many bytes are relevant
                        if( is_debug){
                            ToSendDataBuffer[2] = 1; // Testing only !!
                            ToSendDataBuffer[3] = 1; // The actual timer count
                            TMR0H = 0;
                            TMR0L = 0;
                            delay_uS(sample_interval);
                            ToSendDataBuffer[5] = TMR0L; // Testing only !!
                            ToSendDataBuffer[4] = TMR0H; // The actual timer count
                        }

                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ToSendDataBuffer[0],64);
                        send_buff_pos = 2; // reset pos
                        max_delay_lapsed = 0;
                    }
                }
                TMR0ON = 0;
                break;
                
            case COMMAND_IO_READ_ADC: //Uses ADC to measure an analog voltage on one of the ANxx I/O pins, and returns the result to the host

                ToSendDataBuffer[0] = COMMAND_IO_READ_ADC; //Echo back to the host the command we are fulfilling
                uint16_t pot;
                uint8_t  chnls = ReceivedDataBuffer[1];
                uint8_t  flags = ReceivedDataBuffer[2];
                bool is_millis = flags & 4; // time is in millis
                bool is_debug = flags & 2; // send back some debug data
                uint16_t num_samples = bytes2_to_int( ReceivedDataBuffer[3], ReceivedDataBuffer[4]);
                uint16_t sample_interval = bytes2_to_int( ReceivedDataBuffer[5], ReceivedDataBuffer[6]);
                // value that should be reached before we start sampling. Useful for manual triggering like pressing a remote key
                uint16_t minStartingValue = bytes2_to_int( ReceivedDataBuffer[7], ReceivedDataBuffer[8]);
                uint16_t curr_sample_num = 0;
                uint8_t send_buff_pos = 2; // 1st 2 bytes reserved

                if( is_debug){
                    TMR0ON = 1;
                }

                // Wait till ADC reading reaches the min starting value.
                while( 1){
                    if( ADC_Read10bit(ADC_CHANNEL_POTENTIOMETER) >= minStartingValue){
                        break;
                    }
                }

                // Loop for samples
                while( curr_sample_num < num_samples ) {
                    // Reset timer
                    TMR0H = 0;
                    TMR0L = 0; // Write low last, as the high is captured then

                    pot = ADC_Read10bit(ADC_CHANNEL_POTENTIOMETER);
                    ToSendDataBuffer[send_buff_pos] =  pot >> 8;     //MSB
                    ToSendDataBuffer[send_buff_pos+1] = (uint8_t) pot; //LSB
                    send_buff_pos = send_buff_pos + 2;
                    curr_sample_num ++;
                    is_millis ? delay_mS( sample_interval) : delay_uS( sample_interval);

                    timer_val_lo = TMR0L; // Capture low first as this will buffer the high value
                    timer_val_hi = TMR0H;
                    
                    // Check if we have filled the buffer. If yes, transmit
                    if( send_buff_pos > 63 || curr_sample_num == num_samples) {
                        //Wait till the endpoint/buffer is free
                        while( HIDTxHandleBusy(USBInHandle)){
                        }
                        
                        ToSendDataBuffer[1] = send_buff_pos; // how many bytes are relevant
                        if( is_debug){
                            ToSendDataBuffer[2] = timer_val_hi; // Testing only !!
                            ToSendDataBuffer[3] = timer_val_lo; // The actual timer count
                            TMR0H = 0;
                            TMR0L = 0;
                            delay_uS(sample_interval);
                            ToSendDataBuffer[5] = TMR0L; // Testing only !!
                            ToSendDataBuffer[4] = TMR0H; // The actual timer count
                        }

                        //Prepare the USB module to send the data packet to the host
                        USBInHandle = HIDTxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ToSendDataBuffer[0],64);
                        send_buff_pos = 2; // reset pos
                    }
                }
                TMR0ON = 0;
                break;

        }
        //Re-arm the OUT endpoint, so we can receive the next OUT data packet 
        //that the host may try to send us.
        USBOutHandle = HIDRxPacket(CUSTOM_DEVICE_HID_EP, (uint8_t*)&ReceivedDataBuffer, 64);
    }
}
