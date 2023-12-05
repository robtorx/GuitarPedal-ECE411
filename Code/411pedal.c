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

unsigned int ADC_low, ADC_high, POT;
unsigned int upper_threshold, lower_threshold;
int input;
int counter = 0;
bool activeADC = 0;

// Function definitions
long map(long x, long in_min, long in_max, long out_min, long out_max);
void switch_adc(void);

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

/**
// Timer0 interrupt
ISR(TIMER0_COMPA_vect) {
    counter++;

    // if (counter >= 100) {
    //     switch_adc();
    // }
    
}
*/

// ADC Conversion Complete
ISR(ADC_vect) {
    if (!activeADC) {           // Update ADC_high/low (guitar line) if active channel is ADC0
        ADC_low = 0;            // Always 0 to save space
        ADC_high = ADCH;
    } else if (activeADC) {     // Update POT if active channel is ADC1
        POT = ADCH;
    }

    // Determines clipping levels based on POT
    upper_threshold=map(POT,0,4095,4095,2047);
    lower_threshold=map(POT,0,4095,0000,2047);
    
    // Clips signal based on threshholds
    if (ADC_high>=upper_threshold) {
        ADC_high=upper_threshold;
    }
    else if (ADC_high<lower_threshold) {
        ADC_high=lower_threshold;
    }

    // Output
    input = ((ADC_high << 8) | ADC_low) + 0x8000; // make a signed 16b value

    OCR1AL = ((input + 0x8000) >> 8);       // convert to unsigned, send out high byte
    OCR1BL = input; // send out low byte    // PortD for the ATMega1284
    
    // Conversion takes 13 seconds -> approx. 0.1 ms per conversion at 125KHz ADC freq.
    // Every 100 conversions = 1 ms/potentiometer reading
    if (++counter >= 100) {
        switch_adc();           // switch to ADC1 to read POT next conversion
        counter = 0;            // reset counter
    } else if (counter == 0) {
        switch_adc();           // switch back to ADC0
    }

    counter++;
    ADCSRA |= (1<<ADSC);        // Start next ADC conversion
}

////////////////////////////////////////////////////////////////////////////
//                          HELPER FUNCTIONS                              //
////////////////////////////////////////////////////////////////////////////

void pin_setup(void){
    // Pins PD1 PD2 as output with PWM
    DDRD |= ((1<<DDB1) | (1<<DDB2));
    // Pins PA0 PA1 as input pins
    DDRA &= ~((1<<DDB0) | (1<<DDB1));
}

// Used to generate PWM signals
void timer_setup(void) {
    PRR0 &= ~(1<<PRTIM1);               // Enable TIMER1 module 
 
    // TCNT1 = 0x00;
    TCCR1B = (1<<WGM13) | (0<<WGM12);   // Phase correct, PWM waveform generation
    TCCR1A = (1<<WGM11) | (0<<WGM10);   // TOP = ICR1         
    TCCR1B = (1<<CS10);                 // CLK/1 = 16 MHz TIMER1
    TCCR1A = (1<<COM1A1);               // Set output to low level.
    TCCR1B = (1<<COM1B1);               //

    ICR1L = 0xFF                        // PWM frequency = TIMER1/(1*PRE*ICR1)
                                        // PWM Resolution = log2
    // ICR1H = (0xFF >> 8);                

    TIMSK1 = (1<<TICIE1);               // Enable TIMER1 capture interrupt
   
    // TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); //
    // TCCR1B = ((PWM_MODE << 3) | 0x11);                        // CLK/1
    // ICR1H = (PWM_FREQ >> 8);             // PWM Frequency = 16 MHz/256 = 31.3 KHz
    // ICR1L = (PWM_FREQ & 0xFF);
}

void adc_setup(void){   
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);     // CLK/128 = 125 KHz ADC frequency
    ADMUX |= (0<<REFS1) | (1<<REFS0);                   // AVCC with external capacitor at AREF pin
    ADMUX |= (1<<ADLAR);                                // Left aligned for 8 bit resolution
	ADMUX |= (0<<MUX0) | (0<<MUX1) | (0<<MUX2) | (0<<MUX3) | (0<<MUX4); // Select port ADC1
    PRR &= ~(1<<PRADC);     // Clear power reduction bit
    ADCSRA |= (1<<ADEN);    // Enable ADC
	ADCSRA |= (1<<ADSC);    // Start first conversion
	// SFIOR |= (0<<ADTS0) | (0<<ADTS1) | (0<<ADTS2); // Free running mode
}

// Toggles LSB of ADMUX.MUX (switches between ADC0 and ADC1)
void switch_adc(void) {
    ADMUX ^= (1 << MUX0);
    activeADC = !activeADC;
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