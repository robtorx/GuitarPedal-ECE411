/*
 * GuitarPedal.c
 *
 * Created: 12/01/2023
 * Author : Lynn
 * 
 * NOTE : ADC operates in single conversion, manual mode.
 */ 


#ifndef F_CPU
#define F_CPU 16000000UL    // 16MHz Clock Speed
#endif

 #include <avr/io.h>
// #include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

// defining the output PWM parameters
#define PWM_FREQ 0x00FF             // pwm frequency - 31.3KHz
#define PWM_MODE 0                  // Fast (1) or Phase Correct (0)
#define PWM_QTY 2                   // 2 PWMs in parallel

// Function definitions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void switch_adc(void);
void pin_setup();
void timer_setup();
void adc_setup();

int main(void) {
    cli();
    pin_setup();
    timer_setup();
    adc_setup();
    sei(); // Enable global interrupts

    while (1) {}; // prevent program from ending
}
////////////////////////////////////////////////////////////////////////////
//                      Interrupt Service Routines                        //
////////////////////////////////////////////////////////////////////////////


// Timer1 interrupt
ISR(TIMER1_COMPA_vect) {
  36   // get ADC data
  37   unsigned int temp1 = ADCL; // you need to fetch the low byte first
  38   unsigned int temp2 = ADCH;
  50   
  51   // output high byte on OC1A
  52   OCR1AH = temp2 >> 8; // takes top 8 bits
  53   OCR1AL = temp2; // takes bottom 8 bits
  54   
  55   // output low byte on OC1B
  56   OCR1BH = temp1 >> 8;
  57   OCR1BL = temp1;
}

////////////////////////////////////////////////////////////////////////////
//                          HELPER FUNCTIONS                              //
////////////////////////////////////////////////////////////////////////////

void pin_setup(void){
    // Pins PD4 PD5 as output with PWM
    DDRD |= ((1<<DDB4) | (1<<DDB5));
    // Pins PA0 PA1 as input pins
    DDRA &= ~((1<<DDB0) | (1<<DDB1));
}

// Used to generate PWM signals
void timer_setup(void) {
  // setup PWM
  TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); //
  TCCR1B = ((PWM_MODE << 3) | 0x11);                        // ck/1
  TIMSK1 = 0x20;                                            // interrupt on capture interrupt
  ICR1H = (PWM_FREQ >> 8);
  ICR1L = (PWM_FREQ & 0xff);
}

void adc_setup(void){   
  // setup ADC
  ADMUX = 0x60;   // left adjust, adc0, internal vcc
  ADCSRA = 0xe5;  // turn on adc, ck/32, auto trigger
  ADCSRB = 0x07;  // t1 capture for trigger
  DIDR0 = 0x01;   // turn off digital inputs for adc0
}

// Maps input value to one in given range, used to digitally amplify signal
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// // Bit manipulation -- reference
// DDRB |= (1 << DDB3);     // set pin 3 of Port B as output
// PORTB |= (1 << PB3);     // set pin 3 of Port B high
// PORTB &= ~(1 << PB3);    // set pin 3 of Port B low
// PORTB ^= (1 << PB3);   // toggles the state of the bit (XOR)m1111111
