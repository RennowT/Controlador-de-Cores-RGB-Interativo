# Controlador-de-Cores-RGB-Interativo
O Controlador de Cores RGB Interativo é um projeto eletrizante que traz o poder da customização de cores para a palma da sua mão. Este sistema inovador utiliza um microcontrolador ATMega328, LEDs RGB, botões de seleção e um potenciômetro, proporcionando uma experiência de controle de iluminação intuitiva e versátil.

## Componentes
1. LED RGB - 1
2. Botões - 4
3. Potenciometro - 1
4. Display LCD - 1

## Periféricos
1. GPIO
2. Interrupções externas
3. PWM
4. ADC
5. EEPROM

## Pinagem
- PB3 - PWM com azul do LED
- PC0 - ADC para mudança de intensidade das cores
- PD2 - INT0 para o botão de desligar/ligar
- PD3 - INT1 para o botão de salvar cor do LED RGB
- PD4 - PCINT20 para o botão de mudança de cor para a direita <-
- PD5 - PWM cor vermelha do LED
- PD6 - PWM cor verde do LED
- PD7 - PCINT23 para o botão de mudança de cor para a esquerda ->
