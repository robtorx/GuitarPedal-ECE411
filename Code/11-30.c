/*
 * GuitarPedal.c
 *
 * Created: 11/13/23 12:47:54
 * Author : Rob
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL    // 16MHz Clock Speed
#endif

#include <avr/io.h>
#include <util/delay.h>

#define PWM_FREQ 0x00FF     // pwm frequency - 31.3KHz
#define PWM_MODE 0          // Fast (1) or Phase Correct (0)
#define PWM_QTY 2           // 2 PWMs in parallel

// Function definitions
long map(long x, long in_min, long in_max, long out_min, long out_max);

int main(void)
{
  int pot;
  int upper_threshold, lower_threshold;
  
  // SETUP
  // Pins PD1 PD2 as output with PWM
  TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); // Timer 1
  TCCR1B = ((PWM_MODE << 3) | 0x11);                        // CLK/1 = 16 MHz
  TIMSK1 = 0x20;    // interrupt on capture interrupt
  ICR1H = (PWM_FREQ >> 8);
  ICR1L = (PWM_FREQ & 0xff);
  DDRD |= 0x06;     // Turn on PWM outputs PD1 and PD2
  sei();            // Turn on interrupts  

  while(1) {};
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void adc_init(void)
{
  // ADMUX |= (0 << REFS1) | (1 << REFS0); // Ref voltage AVCC with external cap at AREF?
  // ADMUX |= (1 << ADLAR);                // ADC left justified
  // ADMUX |= (1 << ADC0);
  ADMUX   = 0x60;   // left adjust, ADC0, internal vcc
  ADCSRA  = 0xE5;   // turn on ADC, CK/32 = , auto trigger
  ADCSRB  = 0x07;   // T1 capture for trigger
  DIDR0   = 0x01;   // turn off digital inputs for ADC0
}

ISR(TIMER1_CAPT_vect) 
{
  // get ADC data
  ADC_low = 0; // ADC_low always 0 to save space
  ADC_high = ADCH;
  
    // amplifies signal based on pot value
  upper_threshold=map(pot,0,4095,4095,2047);
  lower_threshold=map(pot,0,4095,0000,2047);
  
  // allows "clipping" of signal necessary for distortion effect
  if(in_ADC0>=upper_threshold) in_ADC0=upper_threshold;
  else if(in_ADC0<lower_threshold)  in_ADC0=lower_threshold;
 
  if(in_ADC1>=upper_threshold) in_ADC1=upper_threshold;
  else if(in_ADC1<lower_threshold)  in_ADC1=lower_threshold;

}