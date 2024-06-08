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
#define LEFT_BUTTON_PIN (1 << PD7)    // Definition of the pin for the color change button to the left
#define POTENCIOMETER_PIN (1 << PC0)  // Definition of the pin referring to the ADC reading of the potentiometer
#define BUZZER_PIN (1 << PB2)         // definition of the pin related to the buzzer

/* Declaration of functions */
void GPIO_init();
void INTx_init();

unsigned int adc_result0;
unsigned int adc_result1;
unsigned int adc_result2;

float tensao;

void ADC_init(void) {
  // Configurando Vref para VCC = 5V
  ADMUX = (1 << REFS0);
  /*
    ADC ativado e preescaler de 128
    16MHz / 128 = 125kHz
    ADEN = ADC Enable, ativa o ADC
    ADPSx = ADC Prescaler Select Bits
    1 1 1 = clock / 128
  */
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

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

int main() {
  GPIO_init(); // Starts the GPIO pins
  INTx_init(); // Configure external interrupts

  //ADC_init();  // Inicializa ADC     

  //TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);
  //TCCR0B |= (1 << CS00);
  //OCR0A = 0;

  sei();

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
  DDRD &= ~(ON_OFFF_BUTTON_PIN | SAVE_BUTTON_PIN | RIGHT_BUTTON_PIN | LEFT_BUTTON_PIN); // 0b10011100

  // Put the button pins in pull-up
  PORTD |= ON_OFFF_BUTTON_PIN | SAVE_BUTTON_PIN | RIGHT_BUTTON_PIN | LEFT_BUTTON_PIN; // 0b10011100
}

// Configure external interrupts
void INTx_init() {
  // Falling edge on INT1 or INT0 generates an interrupt
  EICRA |= ISC11 | ISC01; // 0b00001010

  // Enables the externally configured setting
  EIMSK |= INT0 | INT1; // 0b00000011
}