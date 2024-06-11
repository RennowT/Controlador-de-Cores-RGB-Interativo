#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

/* ATMega328 pin definition */
#define RED_COLOR_PIN (1 << PD6)         // Definition of the pin referring to the red color of the RGB LED
#define GREEN_COLOR_PIN (1 << PD5)       // Definition of the pin referring to the green color of the RGB LED
#define BLUE_COLOR_PIN (1 << PB3)        // Definition of the pin referring to the blue color of the RGB LED
#define ON_OFFF_BUTTON_PIN (1 << PD2)    // Definition of the pin for the on and off button
#define SAVE_BUTTON_PIN (1 << PD3)       // Definition of the pin for the color saving button
#define RIGHT_BUTTON_PIN (1 << PD4)      // Definition of the pin for the color change button to the right
#define LEFT_BUTTON_PIN (1 << PB0)       // Definition of the pin for the color change button to the left
#define BUZZER_PIN (1 << PB2)            // definition of the pin related to the buzzer
#define POTENCIOMETER_ADC_CHANNEL ADC0D  // Definition of the ADC referring to the potentiometer pin

/* Samples */
#define SAMPLE_ADC_READ 8

/* Declaration of functions */
void GPIO_init();
void INTx_init();
void PCINT_init();
void PWM_init();
void ADC_init();
int ADC_read(uint8_t ch);

unsigned int adc_result0;

float tensao;

// Interrupt routine for the power on/off button
ISR(INT0_vect) {
  
}

// Interrupt routine for saving colors to EEPROM
ISR(INT1_vect) {

}

// Interrupt routine for changing color to the left
ISR(PCINT0_vect) {

}

// Interrupt routine for changing color to the right
ISR(PCINT2_vect) {

}

// main
int main() {
  GPIO_init();  // Configure the GPIO pins
  INTx_init();  // Configure external interrupts
  PCINT_init(); // Configure PCINT
  PWM_init();   // Configures timers for PWM
  ADC_init();   // Configure the ADCs pins    
  sei();        // Enable global interrupt
 
  while (1) {
    
  }
  return 0;
}

// Starts the GPIO pins
void GPIO_init() {
  // Declaring pins as output
  DDRB |= BLUE_COLOR_PIN | BUZZER_PIN; // 0b00001100
  DDRD |= RED_COLOR_PIN | GREEN_COLOR_PIN; // 0b01100000

  // Declaring pins as input
  DDRB &= ~LEFT_BUTTON_PIN; // 0b00000001
  DDRD &= ~(ON_OFFF_BUTTON_PIN | SAVE_BUTTON_PIN | RIGHT_BUTTON_PIN); // 0b00011100

  // Put the button pins in pull-up
  PORTB |= LEFT_BUTTON_PIN; // 0b00000001
  PORTD |= ON_OFFF_BUTTON_PIN | SAVE_BUTTON_PIN | RIGHT_BUTTON_PIN; // 0b00011100
}

// Configure external interrupts
void INTx_init() {
  // Falling edge on INT1 or INT0 generates an interrupt
  EICRA |= (1 << ISC11) | (1 << ISC01); // 0b00001010

  // Enables the externally configured setting
  EIMSK |= (1 << INT0) | (1 << INT1); // 0b00000011
}

// Configure PCINT
void PCINT_init() {
  // Enables the PCINT interrupt for PORTB and PORTD
  PCICR |= (1 << PCIE2) | (1 << PCIE0); // 0b00000101

  // Enables the PCINT interrupt for pins PCINT23 and PCINT20
  PCMSK0 |= (1 << PCINT0); // 0b00000001
  PCMSK2 |= (1 << PCINT20); // 0b00010000
}

// Configures timers for PWM
void PWM_init() {
  // Configure Timer 0 for Compare Match Output A and B Mode
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // Compare Output Mode and Fast PWM Mode (0b11000011)
  TCCR0B |= (1 << CS00); // No prescaling (0b00000001)

  // Configure Timer 2 for Compare Match Output A Mode
  TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Compare Output Mode and Fast PWM Mode (ob10000011)
  TCCR2B |= (1 << CS20); // No prescaling (0b00000001)
}

// Configure the ADCs pins
void ADC_init(void) {
  // Configuring Vref for VCC = 5V
  ADMUX = (1 << REFS0); 
  // Enables the ADC and configures the 128 prescaler
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}


// Reads the ADC channel value
int ADC_read(uint8_t ch) {
  char i;
  int ADC_temp = 0;  // Temporary ADC to handle read
  int ADC_read = 0;  // ADC_read
  ch &= 0x07;        // 0b00000111

  ADMUX = (ADMUX & 0xF8) | ch;     // Select the input channel
  ADCSRA |= (1 << ADSC);           // Make a conversion
  while (!(ADCSRA & (1 << ADIF))); // Wait for the signal to convert

  for (i = 0; i < SAMPLE_ADC_READ; i++) {  // Doing the conversion a few times for greater accuracy
    ADCSRA |= (1 << ADSC);                 // Make a conversion
    while (!(ADCSRA & (1 << ADIF)));       // Wait for the signal to convert
    ADC_temp = ADCL;                       // reads the ADCL register
    ADC_temp += (ADCH << 8);               // reads the ADCH register
    ADC_read += ADC_temp;                  // Accumulate the result to average
  }

  ADC_read = ADC_read / SAMPLE_ADC_READ; // Calculates the average
  return ADC_read;
}
