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
  PF7: SPI5_SCK                                                        CN11 - pin 11  grey
  PF8: SPI5_MISO                                                       CN11 - pin 54  blue
  PF9: SPI5_MOSI                                                       CN11 - pin 56  purple
  PF10: GPIO_Output - nRF24L01 SPI (SPI5) SS (CSN)                     CN12 - pin 42  yellow
  PF6: GPIO_Output - nRF24L01 CE (for controlling Rx/Tx operation)     CN11 - pin 9   white
  PF5: GPIO_EXTI5 - nRF24L01 nINT                                      CN12 - pin 36  green
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

#3) Open app (In Vs code): C:\Users\rwabc\Software\Development\MQTT\mqtt_dev1\
In terminal:
npm run pubsub    (this will send a ndreceive mqtt messages between dev kit and PC)

#4) SerLink over MQTT (SerLink2)
Broker: MQTT_BROKER_IP in Core/Src/main_tasks.cpp (currently 192.168.0.196:1883)
  hub/aa26/serlink/down   PC -> controlHub  (publish frames here)
  hub/aa26/serlink/up     controlHub -> PC  (acks and status frames)
Payloads are frames exactly as typed on the serial console, e.g.
  mosquitto_pub -h 192.168.0.196 -t hub/aa26/serlink/down -m "DBG00T349002R2"
  mosquitto_sub -h 192.168.0.196 -t hub/aa26/serlink/up

#--------------------------------------------------------------------
## MotorB (with TC78H611FNG and TC78H611FNG_Standby)

TC78H611FNG IC is on ControlHubAA26 Peripheral Board A

PC6: TIM8_CH1  - CN12 pin4    -> IN1B (J10 pin 10)
PC7: TIM8_CH2  - CN12 pin19   -> IN2B (J10 pin 8)
PB8: GPIO      - CN12 pin3    -> nSTBY (J10 pin 6)
PA6: TIM8_BKIN - CN12 pin13   -> (unconnected - reserved for an e-stop)

PA6 note: the TIM8 break is armed (active high). A high on PA6 latches both
motor PWM outputs (PC6/PC7) off until software restarts the PWM or the board
is reset. PA6 has an internal pull-down (set in HAL_TIM_PWM_MspInit, USER CODE
block), so leaving it unconnected is safe - but do not wire anything to CN12
pin 13 that can drive it high. The TC78H611FNG has no fault output.

motorB current sense goes to PA3 (see Analog inputs below).

#--------------------------------------------------------------------
## TachoB

PF4 (CN12 pin 38) # tachoB input pin (3144 Hall switch via level shifter, 2200pF to GND)
GND (CN12 pin 39) # sensor return

TIM5 is the tacho timebase. PF4 is not claimed in the .ioc - Tachometer::init()
configures it at runtime.
#--------------------------------------------------------------------
## Analog inputs (ADC1, scanned continuously, paced by TIM2)

PA0: ADC1_IN0  (index 0)
PA3: ADC1_IN3  (index 1) - motorB current sense
PA4: ADC1_IN4  (index 2)
PA5: ADC1_IN5  (index 3)
#--------------------------------------------------------------------
## Buttons

PB9 (CN12 pin 5)
PB12 (CN12 pin 16)

Both are GPIO inputs with no internal pull (set in the .ioc), so each button
needs an external pull-up or pull-down. Not yet read by the firmware.
#--------------------------------------------------------------------
## SD card (SDIO, 4 bit, FatFs) - 10 pin card header to CN8

Card header pin -> CN8 pin:

1  DAT3 (~CS)   -> CN8 8    PC11  SDIO_D3
2  CMD  (MOSI)  -> CN8 12   PD2   SDIO_CMD
3  DAT0 (MISO)  -> CN8 2    PC8   SDIO_D0
4  SCK          -> CN8 10   PC12  SDIO_CK
7  DAT1         -> CN8 4    PC9   SDIO_D1
8  DAT2         -> CN8 6    PC10  SDIO_D2
9  CD           -> CN8 14   PG2   card detect (GPIO input, pulled up)
5/11 GND        -> CN8 11 or 13
6/12 VCC 3.3V   -> CN8 7    +3.3V

!! CN8 pin 9 is +5V - do NOT use it to power the card. !!

PG2 is not claimed in the .ioc - SdCard::init() configures it at runtime. CD is
taken as active low (card in = GND); if the status reads A with a card in, flip
SDCARD_DETECT_ACTIVE in main_tasks.cpp. The SDIO bus runs at 8 MHz (ClockDiv 4,
main.c USER CODE SDIO_Init 2) while on jumper wires. The card must be FAT/FAT32
(exFAT is off). Driven by the SDC00 socket over uart2 - see main_tasks.cpp.
#--------------------------------------------------------------------
