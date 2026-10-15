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
PA6: TIM8_BKIN - CN12 pin13   -> motorB overcurrent trip, from the current
                                 measurement board's comparator (planned -
                                 see MotorB current measurement below)
PA15: GPIO out - CN11 pin17   -> motorB direction, to the current measurement
                                 hardware (high = forward, low = reverse)

PA6 note: the TIM8 break is armed (active high). A high on PA6 latches the
TIM8 PWM outputs - both motorB pins (PC6/PC7) and any other TIM8 channel - off
until software restarts the PWM or the board is reset. PA6 has an internal
pull-down (set in HAL_TIM_PWM_MspInit, USER CODE block), so leaving it
unconnected is safe. Nothing but the overcurrent comparator should drive CN12
pin 13. The TC78H611FNG has no fault output.

motorB current sense goes to PA5 (see Analog inputs below).

#--------------------------------------------------------------------
## MotorB current measurement (bespoke veroboard)

IoutB is the output of the INA240APWR (U7, main board): R12 (50 mR) sense
resistor, gain 50 -> 2.5 V/A. Its REF pins put it at mid-rail (~1.65 V) at
zero current, and it swings up or down from there with motor direction.
This board folds both directions to one side for the ADC (PA5) and drives
the overcurrent trip (PA6). Everything runs on 3.3 V; all op-amps are
MCP6042 and all FETs 2N7000:

  U1A, U1B   inverting / non-inverting buffers      1st MCP6042
  U2A        x5.26 gain stage -> Output             2nd MCP6042
  U2B        overcurrent comparator -> PA6          2nd MCP6042 (planned)
  Q_A, Q_B   direction mux
  Q_C        dir inverter
  Q_D        trip-level select (planned)

1) dir inverter (Q_C):  /dir = NOT dir

                     3V3
                      │
                    /\/\/  10k
                      │
                      ├──────── /dir   (to Q_B gate)
                      │
                    Q_C (drain)
 dir (PA15) ────────┤ gate
                      │ source=GND
                      ⏚

2) Inverting buffer (U1A):  inv = 3.3 V - IoutB

                         Rf 10k
                   ┌─────/\/\/─────┐
          Ri 10k   │               │
 IoutB ───/\/\/────┴──── (–) ┐     │
                             ├─────┴───── inv
 3V3 ───/\/\/───┬─────── (+) ┘
        10k     │          U1A
              /\/\/ 10k
                │
                ⏚        (+) held at 1.65 V by the 10k/10k divider

3) Non-inverting buffer (U1B, gain 1):  non-inv = IoutB

 IoutB ───────────────── (+) ┐
                             ├─────┬───── non-inv
                   ┌──── (–) ┘     │
                   │       U1B     │
                   └───────────────┘

4) Direction mux (Q_A/Q_B) and x5.26 gain stage (U2A)

   The FET that is ON shorts its own path to GND, so U2A sees the other
   one: dir high -> inv, dir low -> non-inv.

             Rs 10k           R 10k
 non-inv ───/\/\/───┬─────────/\/\/──────┐
                    │                    │
                  Q_A (drain)            │
 dir ─────────────┤ gate                 ├───────── (+) ┐
                    │ source=GND         │              ├───────┬──── Output
                    ⏚                    │      ┌── (–) ┘       │
             Rs 10k           R 10k      │      │     U2A       │
 inv ───────/\/\/───┬─────────/\/\/──────┘      ├────/\/\/──────┘
                    │                           │    Rf 20k
                  Q_B (drain)                 /\/\/ Rg 4.7k
 /dir ────────────┤ gate                        │
                    │ source=GND                ⏚
                    ⏚

 Output ──┬──> PA5  ADC1_IN5 (CN12 pin 11) - monitoring, see Analog inputs
          └──> U2B (+) via Rin - overcurrent trip, below

5) Overcurrent comparator (U2B) -> TIM8_BKIN   (planned)

                                Rh 330k (hysteresis)
                    ┌────────────/\/\/────────────┐
           Rin 10k  │                             │
 Output ───/\/\/────┴───────── (+) ┐              │
                                   ├──────────────┴──── PA6 TIM8_BKIN (CN12 pin 13)
                     ┌──────── (–) ┘
                     │           U2B
       Rt 10k        │ Vref
 3V3 ───/\/\/────────┤
                     │
             ┌───────┴───────┐
             │               │
           /\/\/ Rb 82k    /\/\/ Rsel 100k
             │               │
             ⏚             Q_D (drain)
 trip_sel ───────────────┤ gate     (100k gate-to-GND pull-down)
                             │ source=GND
                             ⏚

   trip_sel is a spare GPIO, not yet assigned. Q_D off (low, or unpowered
   thanks to the pull-down) = 0.5 A trip; Q_D on puts Rsel across Rb and
   lowers Vref to the 0.4 A trip. Rsel is 100k, so Q_D's on-resistance
   (high for a 2N7000 at 3.3 V gate drive) does not matter.

   Output rises above Vref -> U2B output goes high -> BKIN latches the PWM
   off. Rh adds 3.3 V x Rin/Rh = 100 mV of hysteresis at Output (~38 mA),
   so the trip does not chatter at the threshold. The MCP6042 is an op-amp,
   not a comparator: it slews at ~3 V/ms, so the trip takes ~0.5-1 ms -
   fine for a motor, and it also swallows the PWM ripple on Output.

   Trip levels. The board's gain is not calibrated yet (see
   MOTORB_CURRENT_MA_PER_MV in main_tasks.cpp), so these are worked from the
   measured idle: Output 1730 mV for IoutB 1650 mV -> 1.048 V/V, so
   2.62 V/A at Output.

     trip    Vref     Output rising (trip)   Output falling (reset)
     0.5 A   2.941 V  3.030 V   ~0.50 A      2.930 V   ~0.46 A
     0.4 A   2.701 V  2.782 V   ~0.40 A      2.682 V   ~0.36 A

   Re-pick Rb / Rsel once the gain (k, V/A at Output) is measured:

     V_trip = V_zero + I_trip x k                (V_zero ~1.73 V)
     Vref   = V_trip / (1 + Rin/Rh)
     Rb     = Rt x Vref / (3.3 - Vref)           (and Rb || Rsel for 0.4 A)

   On the same estimate, the ~2300 mV seen at stall is only ~0.22 A, so
   both trips sit above stall: they catch a short, not a stalled motor.

#--------------------------------------------------------------------
## TachoB

PF4 (CN12 pin 38) # tachoB input pin (3144 Hall switch via level shifter, 2200pF to GND)
GND (CN12 pin 39) # sensor return

TIM5 is the tacho timebase. PF4 is GPIO_EXTI4 (TACHO_B) in the .ioc, with its
NVIC line left unticked - Tachometer::init() enables the interrupt at runtime.
#--------------------------------------------------------------------
## Analog inputs (ADC1, scanned continuously, paced by TIM2)

PA0: ADC1_IN0  (index 0)
PA3: ADC1_IN3  (index 1) - not used: driven (~0.6 V) by the main board's old sense path
PA4: ADC1_IN4  (index 2)
PA5: ADC1_IN5  (index 3) - motorB current sense, from the rectifier board's MCP6042 (CN12 pin 11)
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

PG2 is GPIO_Input (SD_CD) in the .ioc; SdCard::init() applies the pull-up. CD is
taken as active low (card in = GND); if the status reads A with a card in, flip
SDCARD_DETECT_ACTIVE in main_tasks.cpp. The SDIO bus runs at 8 MHz (ClockDiv 4,
main.c USER CODE SDIO_Init 2) while on jumper wires. The card must be FAT/FAT32
(exFAT is off). Driven by the SDC00 socket over uart2 - see main_tasks.cpp.
#--------------------------------------------------------------------
