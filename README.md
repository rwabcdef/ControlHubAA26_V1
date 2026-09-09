# ControlHubAA26_V1
#
# Main board: NUCLEO-F439ZI

#--------------------------------------------------------------------
######## NUCLEO-F439ZI configuration

# Programming cable
Std usb A to micro usb B

#### On dev kit hardware

## button
B1 USER: the user button is connected to the I/O PC13 by default (Tamper support, SB173
ON and SB180 OFF) 

## Leds
Green LED (LD1): PB0
Blue LED (LD2): PB7
Red LED (LD3): PB14

# USB to Serial (uart) converter cable
FTDI   STM32                           CN9 (female hdr lower left of board)
TXD	-> PD6 (USART2_RX) - orange        Pin 4
RXD	-> PD5 (USART2_TX) - yellow        Pin 6
GND	-> GND on STM32 board - black      Pin 12
#--------------------------------------------------------------------
