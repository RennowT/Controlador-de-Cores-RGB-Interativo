#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

/* ATMega328 pin definition */
#define RED_COLOR_PIN (1 << PD5)      // Definition of the pin referring to the red color of the RGB LED
#define GREEN_COLOR_PIN (1 << PD6)    // Definition of the pin referring to the green color of the RGB LED
#define BLUE_COLOR_PIN (1 << PB3)     // Definition of the pin referring to the blue color of the RGB LED
#define ON_OFFF_BUTTON_PIN (1 << PD2) // Definition of the pin for the on and off button
#define SAVE_BUTTON_PIN (1 << PD3)    // Definition of the pin for the color saving button
#define RIGHT_BUTTON_PIN (1 << PD4)   // Definition of the pin for the color change button to the right
#define LEFT_BUTTON_PIN (1 << PB0)    // Definition of the pin for the color change button to the left
#define POTENCIOMETER_PIN (1 << PC0)  // Definition of the pin referring to the ADC reading of the potentiometer
#define BUZZER_PIN (1 << PB2)         // definition of the pin related to the buzzer

/* Declaration of functions */
void GPIO_init();
void INTx_init();
void PCINT_init();
void PWM_init();
void ADC_init();

unsigned int adc_result0;
unsigned int adc_result1;
unsigned int adc_result2;

float tensao;

int ADC_read(uint8_t ch) {
  char i;
  int ADC_temp = 0;  // ADC temporário, para manipular leitura
  int ADC_read = 0;  // ADC_read
  ch &= 0x07;        // 0b00000111
  // Zerar os 3 primeiros bits e manter o resto
  ADMUX = (ADMUX & 0xF8) | ch;  // 0b11111000
  // ADSC (ADC Start Conversion)
  ADCSRA |= (1 << ADSC);  // Faça uma conversão
  // ADIF (ADC Interrupt Flag) é setada quando o ADC pede interrupção
  // e resetada quando o vetor de interrupção
  // é tratado.
  while (!(ADCSRA & (1 << ADIF)))
    ;                      // Aguarde a conversão do sinal
  for (i = 0; i < 8; i++)  // Fazendo a conversão 8 vezes para maior precisão
  {
    ADCSRA |= (1 << ADSC);  // Faça uma conversão
    while (!(ADCSRA & (1 << ADIF)))
      ;                       // Aguarde a conversão do sinal
    ADC_temp = ADCL;          // lê o registro ADCL
    ADC_temp += (ADCH << 8);  // lê o registro ADCH
    ADC_read += ADC_temp;     // Acumula o resultado (8 amostras) para média
  }
  ADC_read = ADC_read >> 3;  // média das 8 amostras ( >> 3 é o mesmo que /8)
  return ADC_read;
}

void setRGB(float r, float g, float b) {
  OCR0A = r / 4;  // Red
  OCR0B = g / 4;  // Green
  //OCR0C = b / 4;  // Blue
}

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
