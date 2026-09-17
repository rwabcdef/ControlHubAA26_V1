# ControlHubAA26_V1
#
# Main board: NUCLEO-F439ZI

#--------------------------------------------------------------------
######## NUCLEO-F439ZI configuration

# Programming cable
Std usb A to micro usb B

#### On dev kit hardware

## External Power (external 5V power supply)
JP3: the jumper must be in the EV5 position (left hand side most position)
external 5V power supply: 5V0 to CN11 pin 6
                          GND to CN11 pin 8


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

seems to come up as COM5

## nRF24L01
SPI5
  PF7: SPI5_SCK
  PF8: SPI5_MISO
  PF9: SPI5_MOSI
  PF10: GPIO_Output - nRF24L01 SPI (SPI5) SS (CSN)
  PF6: GPIO_Output - nRF24L01 CE (for controlling Rx/Tx operation)
  PF5: GPIO_EXTI5 - nRF24L01 nINT
#--------------------------------------------------------------------
## MQTT

#1) Norton 360

# when windows machine is the MQTT broker
Norton 360 -> Security -> Advanced -> Smart Firewall

More -> Create rule

  Name: Mosquitto MQTT 1883 STM32
  Action: allow
  Protocol: TCP
  Direction: In
  Address: 192.168.0.200
  Local port: 1883
  remote port: 1883

#2) Make sure ethernet cable is in NUCLEO-F439ZI
#--------------------------------------------------------------------
## Motor (with TC78H611FNG and TC78H611FNG_Standby)

TC78H611FNG IC is on ControlHubAA26 Peripheral Board A

PC6: TIM8_CH1 - CN12 pin4  -> IN1B (J10 pin 10)
PC7: TIM8_CH2 - CN12 pin19 -> IN2B (J10 pin 8)
PB8: GPIO     - CN12 pin3  -> nSTBY (J10 pin 6)

#--------------------------------------------------------------------
