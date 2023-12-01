
#ifndef F_CPU
#define F_CPU 16000000UL    // 16MHz Clock Speed
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PWM_FREQ 0x00FF     // pwm frequency - 31.3KHz
#define PWM_MODE 0          // Fast (1) or Phase Correct (0)
#define PWM_QTY 2           // 2 PWMs in parallel

// Function prototypes
long map(long x, long in_min, long in_max, long out_min, long out_max);
void adc_init(void);
void switch_adc_channel(void);

volatile int pot; // Potentiometer value
volatile int ADC_high; // High byte of ADC

int main(void)
{
    adc_init(); // Initialize ADC

    // PWM setup
    TCCR1A = (((PWM_QTY - 1) << 5) | 0x80 | (PWM_MODE << 1)); 
    TCCR1B = ((PWM_MODE << 3) | 0x11);
    TIMSK1 = 0x20;    
    ICR1H = (PWM_FREQ >> 8);
    ICR1L = (PWM_FREQ & 0xff);
    DDRD |= 0x06;     
    sei();            // Enable global interrupts

    while(1) {
        // Main loop code here (if needed)
    }
}

void adc_init(void)
{
    // ADC configuration
    ADMUX = 0x60;   // Left adjust, ADC0, internal Vcc
    ADCSRA = 0xE5;  // Enable ADC, CK/32, auto trigger
    ADCSRB = 0x07;  // T1 capture for trigger
    DIDR0 = 0x01;   // Disable digital input on ADC0

    ADCSRA |= (1 << ADEN);    // Enable the ADC
    ADCSRA |= (1 << ADIE);    // Enable ADC Interrupt
    sei();                    // Enable global interrupts
}

ISR(TIMER1_CAPT_vect) 
{
    // ADC complete logic
    ADC_high = ADCH; // Read high byte of ADC

    // Update potentiometer value if ADC0 is active
    if ((ADMUX & 0x0F) == 0) {
        pot = ADC_high; // Read potentiometer value
        switch_adc_channel(); // Switch to next ADC channel
    }

    // Apply signal processing logic here based on pot value and ADC_high
    // ...

    // Start next ADC conversion
    ADCSRA |= (1 << ADSC);
}

void switch_adc_channel(void) {
    if ((ADMUX & 0x0F) == 0) {
        ADMUX = (ADMUX & 0xF0) | 1; // Switch to ADC1
    } else {
        ADMUX = (ADMUX & 0xF0) | 0; // Switch back to ADC0
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
