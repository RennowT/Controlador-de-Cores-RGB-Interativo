#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/eeprom.h>

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

/* Samples definition */
#define SAMPLE_ADC_READ 8

/* // Definition of addresses in EEPROM */
#define RED_COLOR_ADDRESS 0x00   // Address in EEPROM associated with red color
#define GREEN_COLOR_ADDRESS 0x01 // Address in EEPROM associated with green color
#define BLUE_COLOR_ADDRESS 0x02  // Address in EEPROM associated with blue color

/* State machine definitions */
#define OFF 0
#define ON 1
#define RED 0
#define GREEN 1
#define BLUE 2
#define RGB_SAVED 3

/* Declaration of functions */
void GPIO_init();
void INTx_init();
void PCINT_init();
void PWM_init();
void set_PWM(unsigned char r, unsigned char g, unsigned char b);
void ADC_init();
int ADC_read(unsigned char ch);
void EEPROM_read();

/* Global variables */
char state = OFF; // Regarding the state of the state machine
char color = RGB_SAVED; // Regarding the state of the color of the state machine
unsigned char red;   // Red color intensity 
unsigned char green; // Green color intensity
unsigned char blue;  // Blue color intensity
char left_button_pressed = 0;  // Checks if the left button was pressed
char right_button_pressed = 0; // Checks if the right button was pressed

// Interrupt routine for the power on/off button
ISR(INT0_vect) {
  if (state == ON) {
    state = OFF;
  } else {
    state = ON;
  }

  PORTB |= BUZZER_PIN;
  _delay_ms(50);
  PORTB &= ~BUZZER_PIN;
  _delay_ms(25);
  PORTB |= BUZZER_PIN;
  _delay_ms(25);
  PORTB &= ~BUZZER_PIN;
}

// Interrupt routine for saving colors to EEPROM
ISR(INT1_vect) {
  if (state == ON) {
    // Writes the PWM value for the colors in the EEPROM
    eeprom_write_byte((unsigned char*)RED_COLOR_ADDRESS, red);
    eeprom_write_byte((unsigned char*)GREEN_COLOR_ADDRESS, green);
    eeprom_write_byte((unsigned char*)BLUE_COLOR_ADDRESS, blue);

    PORTB |= BUZZER_PIN;
    _delay_ms(50);
    PORTB &= ~BUZZER_PIN;
    _delay_ms(100);
    PORTB |= BUZZER_PIN;
    _delay_ms(50);
    PORTB &= ~BUZZER_PIN;
    _delay_ms(100);
    PORTB |= BUZZER_PIN;
    _delay_ms(50);
    PORTB &= ~BUZZER_PIN;
  }
}

// Interrupt routine for changing color to the left
ISR(PCINT0_vect) {
  if (state == ON) {
    _delay_ms(50);

    if (!(PINB & LEFT_BUTTON_PIN)) {
      left_button_pressed = 1;
    }
  }
}

// Interrupt routine for changing color to the right
ISR(PCINT2_vect) {
  if (state == ON) {
    _delay_ms(50);

    if (!(PIND & RIGHT_BUTTON_PIN)) {
      right_button_pressed = 1;
    }
  }
}

// main
int main() {
  GPIO_init();   // Configure the GPIO pins
  INTx_init();   // Configure external interrupts
  PCINT_init();  // Configure PCINT
  PWM_init();    // Configures timers for PWM
  ADC_init();    // Configure the ADCs pins    
  sei();         // Enable global interrupt
  EEPROM_read(); // Retrieves the PWM value of the LED colors in the EEPROM

  while (1) {
    switch (state) {
      case OFF:
        set_PWM(255, 255, 255); // Set all color PWM values ​​to 255 to turn off the LED
        break;

      case ON:
        switch (color) {
          case RED:
            red = ADC_read(ADC0D) / 4;
            break;
          
          case GREEN:
            green = ADC_read(ADC0D) / 4;
            break;

          case BLUE:
            blue = ADC_read(ADC0D) / 4;
            break;

          case RGB_SAVED:
            EEPROM_read();
            break;
          
          default:
            break;
        }

        set_PWM(red, green, blue);
        break;
      
      default:
        break;
    }

    if (left_button_pressed) {
      switch (color) {
        case RED:
          color = RGB_SAVED;
          break;
        
        case GREEN:
          color = RED;
          break;

        case BLUE:
          color = GREEN;
          break;

        case RGB_SAVED:
          color = BLUE;
          break;

        default:
          break;
      }

      PORTB |= BUZZER_PIN;
      _delay_ms(100);
      PORTB &= ~BUZZER_PIN;

      left_button_pressed = 0;
    }

    if (right_button_pressed) {
      switch (color) {
      case RED:
        color = GREEN;
        break;

      case GREEN:
        color = BLUE;
        break;

      case BLUE:
        color = RGB_SAVED;
        break;

      case RGB_SAVED:
        color = RED;
        break;
      
      default:
        break;
      }

      PORTB |= BUZZER_PIN;
      _delay_ms(100);
      PORTB &= ~BUZZER_PIN;

      right_button_pressed = 0;
    }
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

// Sets the PWM values ​​for each LED color
void set_PWM(unsigned char r, unsigned char g, unsigned char b) {
  OCR0A = r;
  OCR0B = g;
  OCR2A = b;
}

// Configure the ADCs pins
void ADC_init(void) {
  // Configuring Vref for VCC = 5V
  ADMUX = (1 << REFS0); 
  // Enables the ADC and configures the 128 prescaler
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

// Reads the ADC channel value
int ADC_read(unsigned char ch) {
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

// Retrieves the PWM value of the LED colors in the EEPROM
void EEPROM_read() {
  red = eeprom_read_byte((const unsigned char*)RED_COLOR_ADDRESS);   
  green = eeprom_read_byte((const unsigned char*)GREEN_COLOR_ADDRESS);
  blue = eeprom_read_byte((const unsigned char*)BLUE_COLOR_ADDRESS);
}