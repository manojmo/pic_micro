/*
 * File:   pot_minutes_calc.c
 * Author: manoj
 *
 * Created on September 27, 2014, 6:01 PM
 */

#include <xc.h>
#include <stdint.h>
#pragma config WDTE = OFF
#pragma config MCLRE = ON
#pragma config FOSC = INTRCIO // Internal Oscillator, GP4,GP5 as IOs

// For 12f675, Timer0 is 8 bit and Timer1 is 16-bit timer.
// Only Timer0 has prescaler ?
unsigned int t0_expired_times; // our counter variable

// Interrupt service Routine
void interrupt myIsr(void)
{
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        t0_expired_times++;
        INTCONbits.TMR0IF = 0; // clear this interrupt condition
    }
}

void delay_times_1s( unsigned int times){
    TMR0 = 0; // !!! Need to reset timer first
    INTCONbits.TMR0IE = 1; // enable timer0 interrupt
    // T = (4/Fosc) * Prescaler * Precision. e.g. 4/4 * 256 * 256 = 65536
    unsigned int end_count = times * 15; // why is this zero ? (times * 1000*1000)/65536 ;
    while( t0_expired_times < end_count);
    t0_expired_times = 0; // reset counter for next delay
    INTCONbits.TMR0IE = 0; // Disable timer0 interrupt
}

void delay_times_1ms( unsigned int times){
    // TODO with interrupt
    int i= 0;
    for( i=0; i< 1000; i++);// 1000 uS = 1ms approx.
    // Actually 4MHz and how many clock cycles per insruction
}

// returns the delay required, in seconds
unsigned int calc_delay_from_ADC( void){
    VCFG = 0; // VDD based reference
    CHS0=0;CHS1=1; ANS2=1;// channel AN2
    ADCS0=0;ADCS1=0;ADCS2=0; // FOSC/2
    ADFM = 1; // Right justified result
    ADON = 1; // turn on converter;
    delay_times_1ms( 1); // Time to charge CHOLD capacitor
    GO_DONE =1;// start conversion
    while( GO_DONE == 1); // wait till the conversion is over
    ADON = 0; // turn off converter module
    unsigned int result = (ADRESH<<8)|ADRESL;
    // Assuming a 10K resistor to limit current, in series with a 86K pot connected to ADC,
    // the voltage range for ADC will be 0 -> VDD * 86/96. i.e 0-0.9VDD. The ADC values range
    // from 0-1023, so we have arange from 0-917. 917 corresponds to 86K,
    // approximately 1.06 K corresponds to 10 points. We can then have a range of 43 mins,
    // each min corresponding to 2K, or 20 points.
    result = (10 * result)/(1000 - result); // pot-val excluding 10K
    result = result/2; // in minutes
    return result;
}

/*
 *
 */
int main(int argc, char** argv)
{
    // Timer settings
    PSA = 0; PS0 = 1; PS1= 1; PS2= 1;// Timer0, prescaler=1:256
    T0CS = 0; // internal clock
    ei();
    INTCONbits.TMR0IE = 1; // disable Timer 0 interrupt, enable only in delay routine
    TRISIO5 = 0; // Configure GP5 as output
    unsigned char loopTimes = calc_delay_from_ADC();
    uint8_t i;
    for( i=0; i< loopTimes; i++ )
    {
        GP5 = 1; // Turn GP5 ON
        delay_times_1s(1);
        GP5 = 0; // Turn GP5 OFF
        delay_times_1s(1);
    }
    delay_times_1s(5);
    #asm
    sleep;
    #endasm
    return 0;

}
