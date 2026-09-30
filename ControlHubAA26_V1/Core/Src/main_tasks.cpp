/*
 File: main_tasks.cpp
 Description: This file contains the implementation of the main tasks for the ControlHubAA26_V1
 Author: Rob Woodhouse
 Created: 2026-09-08

 #---------------------
 # Teraterm (windows)
 #   serial: baud rate: 115200
 #   terminal: line end: CR+LF
 #   terminal: local echo: on
 #
 # debugSocket
 DBG00T347005hello   # will produce std ack

 DBG00T349002R2      # will produce ack with data "OK" (from: debugSockInstantHandler())
 DBG00T349003S00     # task 00 stack: name and lowest free bytes, e.g. "writer0Task:0412"
 DBG00T349002SL      # the task with the least stack free, same format
 DBG00T349001M       # MQTT link (SerLink2): C/D connected, rx, tx, tx dropped, other topic

 # ping - any socket, any link ('S' = system frame, see Socket.hpp). Answered by
 # SerLink itself; never reaches the socket's handlers.
 MOTORS045004PING    # -> MOTORA045008PINGBACK  socket exists on this link
 XXXXXS045004PING    # -> XXXXXA045900          no such socket: plain ACK_OK
 LIFT0S045004PING    # -> LIFT0A045008PINGBACK over MQTT, but LIFT0A045900
                     #    on serial - LIFT0 is on MQTT only

 # led socket - relayed to the radio, and ultimatelty to the remote hub (arduino uno r4)
LED01U492002A1
LED01U492002A0

LED01T492002A1
LED01T492002A0

# Motor socket - the TC78H611FNG dual H-bridge on TIM8, channel B (IN1B/IN2B on J10 pins 10 and 8)
MOTORT516005BP050   # open loop: 50% duty (also turns closed loop off)
MOTORT516005BP010
MOTORT516006BS0300  # closed loop: hold 300 RPM (controllerB takes over)
MOTORT529003BGS     # read the required RPM back, 4 digits

# Control socket - speed controllers. controllerB drives motorB from tachoB.
# On serial AND on MQTT (SerLink2) - two sockets sharing the same handlers,
# driving the same controllerB, so the last command from either link wins.
#
# Safety limits (see Unresponsive tachometer in Controller.hpp): the duty is
# capped at CONTROLB_OUTPUT_MAX_PERCENT (50%), and if the tacho reads 0 for
# CONTROLB_TACHO_TIMEOUT_S (10 s) with a speed demanded and the duty at or
# above CONTROLB_TACHO_CHECK_MIN_PERCENT (20%), the controller stops
# the motor (0%, direction D) and latches a tacho fault. The next speed set
# (BR<dddd>, MOTOR BS<dddd> or a lift start) clears it.

# start closed loop motor run
CTRL0T523003BDF     # controllerB: direction forward
CTRL0T516006BR0120  # controllerB: hold 120 RPM (enables closed loop)

# end closed loop motor run
CTRL0T516006BR0000
CTRL0T523003BDD

# sets - plain ACK_OK; a malformed set is dropped silently, so read it back
CTRL0T516006BR0120  # controllerB: hold 120 RPM (enables closed loop)
CTRL0T523003BDF     # controllerB: direction forward
CTRL0T523003BDR     # controllerB: direction reverse (F forward, D disabled)
CTRL0T516008BI002000  # integral gain = 0.002 (6 digits, millionths: 000000..999999).
                      # Next pass, bumpless; lasts until reset (boot value is
                      # CONTROLB_INTEGRAL_GAIN)

# reads - answered on the ack
CTRL0T529003BGR     # required RPM              -> CTRL0A5290040120
CTRL0T529003BGD     # direction, F/R/D          -> CTRL0A529001R
CTRL0T529003BGF     # tacho fault, 1/0          -> CTRL0A5290011
CTRL0T529003BGI     # integral gain, millionths -> CTRL0A529006002000
CTRL0T529003BGA     # all: gain.required.measured RPM
                    #                           -> CTRL0A529016002000.0150.0148

# Sent by the board every TACHO_PUBLISH_PERIOD_MS, ONLY while liftB is moving
# (on both links, 'U' so no ack):
CTRL0U001008030.0350  # duty 30%, measured 350 RPM

# Lift socket - liftB, over MQTT only (SerLink2: publish to hub/aa26/serlink/down).
# Direction and distance only - distance in tachoB edges (2 per rev). The speed
# is controllerB's: set it first with CTRL0 BR<dddd> (it can be changed
# mid-move); it boots at CONTROLB_BOOT_RPM (100). A start with the required
# speed at 0 is refused silently, so nothing moves.
CTRL0T516006BR0020  # speed for the moves that follow: 20 RPM
LIFT0U645006BSF234  # start liftB forward for 234 edges (1..6 digits)
LIFT0U645006BSR234  # start liftB reverse for 234 edges
LIFT0U645006BG2000  # liftB down to the ground sensor (PB9), 2000 edges max
LIFT0U645002BX      # stop liftB now (coasts)
LIFT0T645002BT      # liftB status -> LIFT0A645014M000120.000234
                    #   M moving / I idle / G idle on the ground sensor,
                    #   edges travelled, target
# Sent by the board when a move ends (arrived, stopped, ground, or tacho fault):
LIFT0U001015BI000234.000234   # liftB idle, 234 of 234 edges travelled
LIFT0U001015BG000180.002000   # liftB ground move found the ground after 180
LIFT0U001015BI000000.000234   # ended short - with CTRL0 BGF reading 1, the
                              # tacho timeout stopped it

# Adc socket - ADC1, ranks IN0/IN3/IN4/IN5 (PA0/PA3/PA4/PA5). Channel 1 is the motorB current sense.
ADC00T529002G1      # raw count, 4 digits
ADC00T529002V1      # millivolts, 4 digits
ADC00T529002GA      # all four channels, raw

# Tacho socket - motorB speed in RPM. Transmit only, so there is nothing to
# type. CURRENTLY OFF: the send in startControlBTask is commented out (the
# CTRL0 status frame, and CTRL0 BGA, carry the measured RPM instead).
# Uncommented, it would appear every TACHO_PUBLISH_PERIOD_MS as:
TACHOU001011R0432.00017   # 432 RPM, 17 glitches rejected since boot.
                          # 'U', so the board expects no ack back

 */

#include <cstdio>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
#include "main_tasks.h"
#include "queue.h"
#include "Frame.hpp"
#include "Reader.hpp"
#include "Transport.hpp"
#include "SerlinkRelay.hpp"
#include "uart2.h"
#include "Button.hpp"
#include "Led.hpp"
#include "PWM.hpp"
#include "TC78H611FNG.hpp"
#include "TC78H611FNG_Standby.hpp"
#include "Adc.hpp"
#include "Tachometer.hpp"
#include "Controller.hpp"
#include "Lift.hpp"
#include "nRF24L01.hpp"
#include "spi5.h"
#include "Radio.hpp"
#include "MqttPubSub.hpp"
#include "SerLinkMqttAdapter.hpp"
#include "lwip/netif.h"

//--------------------------------------------------------------
// Task stacks are 256 words (1 KB) except where noted. The exceptions:
//
// MQTT_TASK_STACK_SIZE - every task that calls into lwIP's MQTT client:
// mqttTask, mqttRxTask, mqtt2Task, writer2Task and reader2Task (the last
// two through SerLinkMqttAdapter::write()). MqttPubSub takes the tcpip
// core lock and runs lwIP on the CALLER's stack, not the tcpip thread's,
// so a publish drags the whole send path along with it: mqtt_publish ->
// tcp_write -> tcp_output -> ip4_output -> etharp -> ethernet_output ->
// low_level_output -> HAL ETH, about 700 bytes at -O0 before the caller's
// own frames, and an interrupt's saved FPU context, are added. 1 KB
// overflowed within seconds of reset; this is the 2 KB they had before.
//
// Check with the DBG00 stack query (DBG00T349002SL) before trimming.
#define MQTT_TASK_STACK_SIZE (512 * 4)

//--------------------------------------------------------------
/* Definitions for writer0Task */
osThreadId_t writer0TaskHandle;
const osThreadAttr_t writer0Task_attributes = {
  .name = "writer0Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for reader0Task */
osThreadId_t reader0TaskHandle;
const osThreadAttr_t reader0Task_attributes = {
  .name = "reader0Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for serLink0Task */
osThreadId_t serLink0TaskHandle;
const osThreadAttr_t serLink0Task_attributes = {
  .name = "serLink0Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for radioRxTask */
osThreadId_t radioRxTaskHandle;
const osThreadAttr_t radioRxTask_attributes = {
  .name = "radioRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for radioTxTask */
osThreadId_t radioTxTaskHandle;
const osThreadAttr_t radioTxTask_attributes = {
  .name = "radioTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for writer1Task */
osThreadId_t writer1TaskHandle;
const osThreadAttr_t writer1Task_attributes = {
  .name = "writer1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for reader1Task */
osThreadId_t reader1TaskHandle;
const osThreadAttr_t reader1Task_attributes = {
  .name = "reader1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for serLink1Task */
osThreadId_t serLink1TaskHandle;
const osThreadAttr_t serLink1Task_attributes = {
  .name = "serLink1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for radio1Task */
osThreadId_t radio1TaskHandle;
const osThreadAttr_t radio1Task_attributes = {
  .name = "radio1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mqttTask */
osThreadId_t mqttTaskHandle;
const osThreadAttr_t mqttTask_attributes = {
  .name = "mqttTask",
  .stack_size = MQTT_TASK_STACK_SIZE,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mqttRxTask */
osThreadId_t mqttRxTaskHandle;
const osThreadAttr_t mqttRxTask_attributes = {
  .name = "mqttRxTask",
  .stack_size = MQTT_TASK_STACK_SIZE,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for relayTask */
osThreadId_t relayTaskHandle;
const osThreadAttr_t relayTask_attributes = {
  .name = "relayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for adcTask */
// Supervision only - the sampling itself is timer, DMA and interrupt, so
// this task just wakes twice a second to check it is still running.
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for motorTask */
// One shot, and it only touches motorB, so configMINIMAL_STACK_SIZE is
// ample - the same 128 words ledTask runs in.
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for controlBTask */
// Drains the tachometer queue and recomputes speed every
// CONTROLB_PERIOD_MS. Given more stack than the other small tasks because
// this is where the motorB control loop will go.
osThreadId_t controlBTaskHandle;
const osThreadAttr_t controlBTask_attributes = {
  .name = "controlB",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mqtt2Task */
// Owns mqtt2 - the receive side of the SerLink2 link and its connection.
// The transmit side does not come through here: SerLinkMqttAdapter::write()
// publishes from whichever task called it.
osThreadId_t mqtt2TaskHandle;
const osThreadAttr_t mqtt2Task_attributes = {
  .name = "mqtt2Task",
  .stack_size = MQTT_TASK_STACK_SIZE,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for writer2Task, reader2Task and serLink2Task - SerLink2,
   the same stack as SerLink0/1, carried by mqtt2. */
osThreadId_t writer2TaskHandle;
const osThreadAttr_t writer2Task_attributes = {
  .name = "writer2Task",
  .stack_size = MQTT_TASK_STACK_SIZE,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t reader2TaskHandle;
const osThreadAttr_t reader2Task_attributes = {
  .name = "reader2Task",
  .stack_size = MQTT_TASK_STACK_SIZE,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t serLink2TaskHandle;
const osThreadAttr_t serLink2Task_attributes = {
  .name = "serLink2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//--------------------------------------------------------------
void startWriter0Task(void *argument);
void startReader0Task(void *argument);
void startSerLink0Task(void *argument);
void StartLedTask(void *argument);
void startRadioRxTask(void *argument);
void startRadioTxTask(void *argument);
void startWriter1Task(void *argument);
void startReader1Task(void *argument);
void startSerLink1Task(void *argument);
void startRadio1Task(void *argument);
void startMqttTask(void *argument);
void startMqttRxTask(void *argument);
void startRelayTask(void *argument);
void startMotorTask(void *argument);
void startAdcTask(void *argument);
void startControlBTask(void *argument);
void startMqtt2Task(void *argument);
void startWriter2Task(void *argument);
void startReader2Task(void *argument);
void startSerLink2Task(void *argument);

bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// MOTOR socket handlers - the command set is documented above their
// implementations, below startMotorTask().
void motorSockReceiveHandler(const char* data, uint16_t dataLen);
bool motorSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// CTRL0 socket handlers - speed controller commands, documented above
// their implementations, below the MOTOR socket's.
void controlSockReceiveHandler(const char* data, uint16_t dataLen);
bool controlSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// LIFT0 socket handlers - lift commands, documented above their
// implementations, below the CTRL0 socket's.
void liftSockReceiveHandler(const char* data, uint16_t dataLen);
bool liftSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// ADC00 socket handler - reads only, documented above its implementation.
bool adcSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// This is called by transport0 when a frame is received.
void transport0ReceiveCallback(const char* data, uint16_t dataLen){ 
    //ledOrange.flash(1, 1, 0, true);
}

// This is called by transport0 when an ack frame is received for a frame that was sent with ack=true.
void transport0AckCallback(const char* data, uint16_t dataLen){ 
    //ledBlue.flash(1, 1, 0, true);
}

//--------------------------------------------------------------
// uart2Queue is created in uart2.c and used by the uart2 ISR to
// pass received frames to the reader0Task.

// Uart2 SerLink writer and reader
SerLink::Writer writer0(WRITER_CONFIG__WRITER0_ID);
SerLink::Reader reader0(READER_CONFIG__READER0_ID);

// SerLink0 transport dispatch queue - created here (rather than owned
// internally by Transport) and handed in via transport0.init()
#define TRANSPORT0_QUEUE_LENGTH 5
StaticQueue_t transport0StaticQueue;
uint8_t transport0QueueStorageArea[TRANSPORT0_QUEUE_LENGTH * sizeof(SerLink::FrameMsg)];
QueueHandle_t transport0Queue;

SerLink::Transport transport0(&writer0, &reader0);

SerLink::Socket* ledSerialSocket = nullptr;

//--------------------------------------------------------------
// SerLink1 - the same stack again, over the nRF24L01 (radio1) instead of
// uart2. writer1 and reader1 post serialised frames to radio1.eventQueue,
// and reader1 takes received frames from radio1.rxDataQueue.
SerLink::Writer writer1(WRITER_CONFIG__WRITER1_ID);
SerLink::Reader reader1(READER_CONFIG__READER1_ID);

#define TRANSPORT1_QUEUE_LENGTH 5
StaticQueue_t transport1StaticQueue;
uint8_t transport1QueueStorageArea[TRANSPORT1_QUEUE_LENGTH * sizeof(SerLink::FrameMsg)];
QueueHandle_t transport1Queue;

SerLink::Transport transport1(&writer1, &reader1);

Radio radio1(nRF24L01_CE_GPIO_Port, nRF24L01_CE_Pin,
             nRF24L01_SS_GPIO_Port, nRF24L01_SS_Pin);

// A TX_DATA RadioMsg has to hold the longest string Frame::toString() writes,
// NUL included - writer1 and reader1 both send serialised frames that way.
static_assert(RADIOMSG__FRAME_LEN_MAX >= SerLink::Frame::MAX_FRAME_LEN,
  "RADIOMSG__FRAME_LEN_MAX is too small for a serialised SerLink frame");

//--------------------------------------------------------------
// LED01 relay (ledRelay): ledSerialSocket (transport0, uart2) <-> ledRadioSocket
// (transport1, radio1), in both directions - as the serlink_nrf24_brg sketch.
// Relayed frames keep their roll code, e.g.
//   PC -> LED01U492002A1          (uart2)
//   radio -> LED01U492002A1
//
// For a 'T' frame, the far end's ack is returned as a relay ack ('B'), e.g.
//   PC -> LED01T492002A1          (uart2)
//   PC <- LED01A492900            (ack from reader0)
//   radio -> LED01T492002A1
//   radio <- LED01A492900         (ack from the far end)
//   PC <- LED01B492900            (relay ack)
SerLink::Socket* ledRadioSocket = nullptr;
SerLink::SerlinkRelay ledRelay;

//--------------------------------------------------------------
// Board LEDs (GPIOB)
Led ledBoardGreen(GPIOB, GPIO_PIN_0);

//--------------------------------------------------------------
// Motor drive - channel B of a TC78H611FNG dual H-bridge, on TIM8.
// Board pins are CN12 (ST morpho, UM1974 Table 21); J10 is the 2x5
// header on the driver board:
//
//   PC6  TIM8_CH1   CN12 pin 4   -> J10 pin 10 -> IN1B   (outA)
//   PC7  TIM8_CH2   CN12 pin 19  -> J10 pin 8  -> IN2B   (outB)
//   PB8  GPIO out   CN12 pin 3   -> J10 pin 6  -> /STBY
//   PA6  TIM8_BKIN  CN12 pin 13  -> break input, see stm32f4xx_hal_msp.c
//
// CN12 pin 20 is GND, opposite PC7, for the ground link to J10.
//
// TC78H611FNG takes (IN1, IN2) whichever bridge it drives, so outA is
// IN1B and outB is IN2B here. Per the datasheet's Input/Output table
// that puts the PWM on IN2B (PC7) for forward and IN1B (PC6) for
// reverse.
//
// Both IN pins are PWM channels rather than one PWM and one GPIO, which
// is what TC78H611FNG.hpp asks for - see the comment there. That is why
// PC7 had to be added to pwmPinMappings[] in PWM.cpp.
//
// /STBY is device-wide, so this same TC78H611FNG_Standby also covers
// channel A (IN1A/IN2A, J10 pins 4 and 2) if a second TC78H611FNG is
// added for it later.
//
// PB8 is ZIO D15 on CN7, next to PC6 (D16). It is not claimed in the
// .ioc at all, so CubeMX never touches it and TC78H611FNG_Standby
// configures it itself - but for the same reason nothing stops a future
// CubeMX edit handing PB8 to a peripheral, so claim it there if this
// becomes permanent.
//
// 1 kHz is well inside the TC78H611FNG's 500 kHz input rating and above
// the audible whine of the slower options. Note that TIM8's period is
// shared, so this also sets the frequency of any other PWM channel on
// TIM8 - PWM::setFrequency() has the details.
#define MOTORB_PWM_FREQ       PWM_FREQ_1KHZ
#define MOTORB_START_PERCENT  20
#define MOTOR_START_DELAY_MS   2000

TC78H611FNG_Standby motorStandby(GPIOB, GPIO_PIN_8);

TC78H611FNG motorB(GPIOC, GPIO_PIN_6,   // IN1B, TIM8_CH1
                   GPIOC, GPIO_PIN_7,   // IN2B, TIM8_CH2
                   MOTORB_PWM_FREQ);

// Acquired on transport0 (uart2), so motor commands arrive over the
// serial link rather than the radio.
SerLink::Socket* motorSocket = nullptr;

//--------------------------------------------------------------
// motorB tachometer - a wheel carrying two magnets, read by a 3144 Hall
// switch. See Tachometer.hpp for how a speed is got out of it.
//
// The 3144 is open collector and pulls low as a magnet passes; a
// non-inverting level shifter brings the swing up to 3 V, comfortably
// clear of the 1.8 V VIH of a 3.3 V input. The line idles high and each
// magnet gives one falling edge, so two per revolution.
//
//   PF4  GPIO EXTI4 (both)     CN12 pin 38  <- level shifter output
//                                            2200 pF to GND at the pin
//   GND                        CN12 pin 39  <- sensor return
//
// PF4 puts the sensor on the same morpho connector as the motor it
// measures (PC6/PC7/PB8 on CN12 pins 4/19/3), with GND immediately next
// to it. What actually picked it is the pin NUMBER: pin 4 means EXTI line
// 4, and lines 0-4 have a vector each, so this shares nothing with the
// radio nINT on EXTI5 or USER_Btn on EXTI13 - no demux, no pending bit
// belonging to someone else.
//
// Like PB8, PF4 is not claimed in the .ioc, so CubeMX never touches it
// and Tachometer::init() configures the pin and the NVIC at runtime. The
// EXTI4_IRQHandler vector is hand written in stm32f4xx_it.c for the same
// reason. Claim PF4 in the .ioc if this becomes permanent - and until
// then nothing stops a future CubeMX edit handing it to a peripheral.
//
// TIM5 is the timebase: 32 bit, free running at Tachometer::TICK_HZ, with
// no interrupt of its own - the edge ISR just reads CNT. It is otherwise
// unused, and its being 32 bit is what keeps every interval a plain
// unsigned subtraction. TIM2, the only other 32 bit timer, is already the
// ADC trigger.
#define TACHOB_PIN  GPIO_PIN_4

// The controlB period. 20 Hz is quick enough for speed control and slow
// enough that several revolutions land in one update at working speed,
// which is where the averaging in Tachometer::update() earns its keep.
#define CONTROLB_PERIOD_MS 50

Tachometer tachoB(TIM5, GPIOF, TACHOB_PIN);

// Speed goes out unsolicited rather than being polled: nothing on the PC
// has to ask for it, and a terminal left open shows the motor spinning up
// and slowing down on its own. Sent as 'U' (no ack) because a telemetry
// frame that went missing is better dropped than retried - the next one is
// only TACHO_PUBLISH_PERIOD_MS away, and waiting on an ack would stall the
// control loop.
#define TACHO_PUBLISH_PERIOD_MS 500  // 2000
#define TACHO_RPM_FIELD_WIDTH   4U

// Glitch count, after a '.' separator. getGlitchCount() is a uint16_t, so
// five digits hold its whole range and no clamp is needed.
#define TACHO_GLITCH_FIELD_WIDTH 5U

// controlBTask publishes on a whole number of its own passes rather than
// keeping a second timebase, so the two periods have to divide.
static_assert((TACHO_PUBLISH_PERIOD_MS % CONTROLB_PERIOD_MS) == 0,
  "TACHO_PUBLISH_PERIOD_MS must be a whole number of controlB periods");

// Acquired on transport0 (uart2), alongside the motor and ADC sockets.
SerLink::Socket* tachoSocket = nullptr;

//--------------------------------------------------------------
// motorB speed controller, run by controlBTask - see Controller.hpp.
//
// Idle until the MOTOR socket's set speed command (<sel>S<dddd>) enables
// it; a set percent command (<sel>P<ddd>) disables it again and puts the
// motor back in open loop. While it is enabled it owns motorB's duty
// cycle, and a percent written by anything else is overwritten on the
// next pass.
//
// CONTROLB_INTEGRAL_GAIN is duty cycle percent per RPM of error, per
// pass. At 20 Hz, 0.002 moves the output 4%/s for a 100 RPM error. That
// is deliberately slow: the tachometer's reading lags by up to a
// revolution-average (TACHO__AVG_REVS revolutions - 2.4 s at 50 RPM), and
// this law keeps integrating through the lag. Raise it once the response has been seen.
//
// CONTROLB_MAX_PLAUSIBLE_RPM only has to catch sensor faults, so it sits
// well above anything this gearbox reaches - lower it if a real top
// speed is known.
//
// CONTROLB_OUTPUT_MAX_PERCENT caps the duty cycle the controller can
// ever write. With a dead or unplugged tacho the reading is zero and the
// law climbs straight to this limit, so it is also how hard the motor is
// driven until the timeout below stops it. Raise it if a demand ever
// needs more than 50% to reach.
//
// CONTROLB_TACHO_TIMEOUT_S: how long the tacho may read zero, with a
// non-zero demand and the duty at or above CONTROLB_TACHO_CHECK_MIN_PERCENT,
// before controllerB stops the motor and latches a fault (CTRL0T529003BGF
// reads it). 0 turns the check off.
//
// CONTROLB_TACHO_CHECK_MIN_PERCENT is the duty at which the motor ought to
// be turning faster than tachoB can measure (~17 RPM with 2 magnets and
// a 2000 ms TACHO__STALL_TIMEOUT_MS - see there). The timer only runs from there, so the ramp up
// from 0% never counts, however slowly a low gain and a low demand make it
// climb (gain 0.0015 at 40 RPM is 1.2%/s - 10 s only reached ~12%). The
// timeout then only has to cover the first revolution or two. 20% held
// ~150 RPM on the bench; lower it if the motor is seen turning well below
// that, and it must stay <= CONTROLB_OUTPUT_MAX_PERCENT.
#define CONTROLB_INTEGRAL_GAIN      0.002f
#define CONTROLB_OUTPUT_MIN_PERCENT 0U
#define CONTROLB_OUTPUT_MAX_PERCENT 50U
#define CONTROLB_MAX_PLAUSIBLE_RPM  3000U
#define CONTROLB_TACHO_TIMEOUT_S    10U
#define CONTROLB_TACHO_CHECK_MIN_PERCENT 20U

// The required speed controllerB boots with, set in initTasks(). Setting
// it does not enable the controller, so nothing moves at boot - it is the
// speed a lift move runs at until CTRL0 BR<dddd> sets another.
#define CONTROLB_BOOT_RPM           100U

// Controller has its own direction type so it does not depend on the
// motor driver; these translate for the TC78H611FNG. Written out for
// every value rather than cast, so reordering either enum cannot quietly
// swap forward and reverse.
static TC78H611FNG::direction toMotorDirection(ControllerDirection direction)
{
  switch(direction)
  {
    case ControllerDirection::forward: return TC78H611FNG::forward;
    case ControllerDirection::reverse: return TC78H611FNG::reverse;
    default:                           return TC78H611FNG::idle;
  }
}

static ControllerDirection fromMotorDirection(TC78H611FNG::direction direction)
{
  switch(direction)
  {
    case TC78H611FNG::forward: return ControllerDirection::forward;
    case TC78H611FNG::reverse: return ControllerDirection::reverse;
    default:                   return ControllerDirection::idle;
  }
}

// Captureless lambdas, so they convert to the plain function pointers
// ControllerConfig takes (see the note on its constructor).
static const ControllerConfig controllerBConfig =
{
  []() -> uint16_t { return tachoB.getRpm(); },           // getRpm
  []() -> uint8_t  { return motorB.getPercent(); },       // getPwmPercent
  [](uint8_t percent) { motorB.setPercent(percent); },    // setPwmPercent
  [](ControllerDirection direction)                       // setDirection
    { motorB.setDirection(toMotorDirection(direction)); },
  []() -> ControllerDirection                             // getDirection
    { return fromMotorDirection(motorB.getDirection()); },
  CONTROLB_INTEGRAL_GAIN,
  CONTROLB_OUTPUT_MIN_PERCENT,
  CONTROLB_OUTPUT_MAX_PERCENT,
  CONTROLB_MAX_PLAUSIBLE_RPM,
  CONTROLB_TACHO_TIMEOUT_S,
  CONTROLB_TACHO_CHECK_MIN_PERCENT,
  CONTROLB_PERIOD_MS
};

Controller controllerB(controllerBConfig);

//--------------------------------------------------------------
// liftB - moves by distance on top of controllerB. See Lift.hpp.
//
// liftB owns controllerB: controlBTask calls liftB.run(), which runs the
// controller. Lift runs it whether or not a move is in progress, so the
// CTRL0 socket still works while the lift is idle.
//
// liftB is direction and distance only - the speed is controllerB's, set
// through CTRL0 (BR<dddd>) before the move and changeable during it. A
// start with a zero demand is refused. Mid-move, leave CTRL0's direction
// alone and do not set the speed to zero - either can stall the move or
// cut it short; stop the lift instead (LIFT0 BX). See Speed in Lift.hpp.
//
// Distance is tachoB edges, PULSES_PER_REV (2) per revolution - see
// Tachometer::getEdges() for why not revolutions.
//
// LIFTB_FORWARD_DIRECTION is the controller direction that moves the
// lift forward. Swap it if the lift is mounted the other way round.
#define LIFTB_FORWARD_DIRECTION  ControllerDirection::forward

// The ground sensor: a pressure plate microswitch on PB9, pulled up on
// the board, so low while the lift is on the ground. PB9 is a plain
// GPIO_Input in the .ioc (no pull - the board's pull-up does it), set
// up by MX_GPIO_Init(). Only ground moves (LIFT0 BG) look at it - see
// Ground level in Lift.hpp.
#define LIFTB_GROUND_PORT        GPIOB
#define LIFTB_GROUND_PIN         GPIO_PIN_9
#define LIFTB_GROUND_ACTIVE      GPIO_PIN_RESET

Lift liftB(&controllerB, LIFTB_FORWARD_DIRECTION,
           []() -> uint32_t { return tachoB.getEdges(); },
           []() -> bool { return HAL_GPIO_ReadPin(LIFTB_GROUND_PORT,
                            LIFTB_GROUND_PIN) == LIFTB_GROUND_ACTIVE; });

// A lift command, parsed by liftSockReceiveHandler() in serLink2Task and
// carried to controlBTask, which owns liftB. One queue item per command,
// so a command's fields always arrive together - they cannot be mixed
// with the next command's the way separate shared variables could.
struct LiftCmd
{
  enum op_t : uint8_t { start, ground, stop };

  Lift*           lift;
  op_t            op;
  Lift::direction dir;        // start only
  uint32_t        distance;   // start, and ground's maximum
};

// Commands arriving within one CONTROLB_PERIOD_MS. A command sent with
// the queue full is dropped - over a 'T' frame the ack has already gone,
// so check with the status read.
#define LIFT_CMD_QUEUE_LENGTH 4
StaticQueue_t liftCmdStaticQueue;
uint8_t liftCmdQueueStorageArea[LIFT_CMD_QUEUE_LENGTH * sizeof(LiftCmd)];
QueueHandle_t liftCmdQueue;

// Acquired on transport2 (SerLink2, MQTT) only.
SerLink::Socket* liftMqttSocket = nullptr;

// CTRL0 - speed demands for the controllers. The MOTOR socket keeps its
// own speed set for now, but this is the one to use. Two sockets, one
// per link, sharing the same handlers (see the CTRL0 section):
//
//   controlSocket      transport0 - SerLink0, serial (uart2)
//   controlMqttSocket  transport2 - SerLink2, MQTT (mqtt2)
//
// Both drive the same controllerB, so the last command from either link
// wins. controlBTask sends its status frame on both.
SerLink::Socket* controlSocket = nullptr;
SerLink::Socket* controlMqttSocket = nullptr;

//--------------------------------------------------------------
// Analog inputs - ADC1, four ranks, scanned continuously into a circular
// DMA buffer. See Adc.hpp for how the sampling is arranged.
//
// The rank order below has to match the sequence MX_ADC1_Init() builds.
// Rank 1 is channel index 0 here, which is what the ADC00 socket takes:
//
//   index  rank  ADC channel  pin
//   0      1     IN0          PA0
//   1      2     IN3          PA3   motorB current sense
//   2      3     IN4          PA4
//   3      4     IN5          PA5
//
// PA3 carries the TC78H611FNG current sense through a 10k/0.47u RC on the
// board (fc ~34 Hz), so the 1 kHz PWM chop is already filtered out and the
// ADC sees the average. That filter settles in about 40 ms, which is fine
// for monitoring but far too slow for protection - fast overcurrent trip
// belongs on TIM8_BKIN (PA6), not here.
//
// TIM2 paces the scans at 320 Hz (84 MHz / 2625 / 100). Each half buffer is
// 32 scans, so a set of averages is published every 100 ms - 10 Hz out of
// 32x oversampling.
#define ADC1_NUM_CHANNELS   4

extern ADC_HandleTypeDef hadc1;   // main.c
extern TIM_HandleTypeDef htim2;   // main.c, the 320 Hz TRGO source

/* C++ does not mangle plain global variable names, so these link against
   main.c's definitions without needing extern "C" - unlike a function. */

Adc adc1(&hadc1, &htim2, ADC1_NUM_CHANNELS);

// Acquired on transport0 (uart2), alongside the motor socket.
SerLink::Socket* adcSocket = nullptr;

//--------------------------------------------------------------
// nRF24L01 radio on SPI5. The driver owns CE (PF6) and CSN (PF10); spi5
// itself handles only SCK/MISO/MOSI, so the bus stays free for other slaves.
// These three must agree with the transmitter. The values below are the
// Arduino RF24 library's own defaults, so a sketch that only calls begin(),
// openWritingPipe() and stopListening() needs no changes:
//
//   channel    76        RF24 begin() default
//   data rate  1 Mbps    RF24 begin() default  (radio.init() sets this)
//   payload    32 bytes  RF24 default; with dynamic payloads off - also the
//                        default - RF24 zero-pads every write() up to it
//
// If the sketch calls radio.setChannel(), setDataRate() or setPayloadSize(),
// match it here. A mismatch on any of them means silence, not corruption.
#define RADIO_CHANNEL     76   // 2476 MHz - above most WiFi traffic
#define RADIO_PAYLOAD_LEN 32   // fixed width, both ends. 32 -> 64 hex chars,
                               // which is exactly SerLink's Frame::MAX_DATALEN
#define RADIO_SERLINK     1      // 1 = SerLink1 over radio1; 0 = the test task RADIO_TEST_TX picks
#define RADIO_TEST_TX     1      // 1 = run startRadioTxTask, 0 = startRadioRxTask
#define RADIO_MODE        nRF24L01::Mode::Interrupt   // radioRxTask: or nRF24L01::Mode::Polled
#define RADIO_POLL_MS     10     // Mode::Polled: delay between FIFO drains
#define RADIO_IRQ_TIMEOUT_MS 1000   // Mode::Interrupt: backstop wake if an nINT
                                    // edge is ever missed
#define RADIO_TX_PERIOD_MS   3000   // radioTxTask: one count packet per period
#define RADIO1_DETECT_TIMEOUT_MS 500 // radio1Task: how long the boot-time detect
                                     // check waits for the module (Tpor is 100 ms)

nRF24L01 radio(nRF24L01_CE_GPIO_Port,  nRF24L01_CE_Pin,
               nRF24L01_SS_GPIO_Port,  nRF24L01_SS_Pin);

// Acquired in initTasks() rather than inside startRadioRxTask(), so it is
// guaranteed valid before startSerLink0Task() can start calling
// transport0.run() against the socket table.
SerLink::Socket* radioSocket = nullptr;

static uint16_t bytesToHex(const uint8_t* src, uint8_t srcLen, char* dst);


SerLink::Socket* mqttSocket = nullptr;

//--------------------------------------------------------------
// MQTT over lwIP. mqttTask keeps the connection up and publishes a count;
// mqttRxTask handles messages arriving on MQTT_SUB_TOPIC.
#define MQTT_BROKER_IP          "192.168.0.196"
#define MQTT_BROKER_PORT        1883
#define MQTT_CLIENT_ID          "stm32-controlhub"   // must be unique on the broker
#define MQTT_TOPIC              "test/hello"         // published to
#define MQTT_SUB_TOPIC          "test/stm32/cmd"     // subscribed to. Not MQTT_TOPIC,
                                                     // or the board hears its own publishes
#define MQTT_PUBLISH_PERIOD_MS  3000

extern struct netif gnetif;   // lwip.c

// The constructor only stores its arguments, so a global is safe here - the
// lwIP client itself is allocated on the first connect().
MqttPubSub mqtt(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID);

//--------------------------------------------------------------
// SerLink2 link layer - SerLink over MQTT, for the PC.
//
// mqtt2Client is a SECOND connection to the same broker, reserved for
// SerLink and read by nothing but mqtt2. MqttPubSub has one rxQueue for
// all of its subscriptions and receive() does not filter by topic, so two
// tasks receiving on one client would steal each other's messages - which
// is why this is a separate instance rather than another subscription on
// mqtt. See the ownership note in SerLinkMqttAdapter.hpp.
//
// MQTT2_CLIENT_ID must differ from MQTT_CLIENT_ID: a second connection
// with the same id kicks the first one off, and the two would sit there
// disconnecting each other in a loop.
//
// The topics are a pair, not one topic. A broker delivers to every
// subscriber including the publisher, so a single topic would feed every
// frame and every ack straight back into our own Reader - the same trap
// the MQTT_SUB_TOPIC comment above warns about.
//
//   down   PC -> controlHub    (subscribed to here)
//   up     controlHub -> PC    (published here)
//
// The payload is the serialised frame exactly as it would appear on
// uart2, so the strings at the top of this file can be pasted straight
// into mosquitto_pub, and mosquitto_sub reads the link like a terminal.
#define MQTT2_CLIENT_ID    "stm32-serlink"          // NOT MQTT_CLIENT_ID
#define MQTT2_TOPIC_DOWN   "hub/aa26/serlink/down"  // subscribed to
#define MQTT2_TOPIC_UP     "hub/aa26/serlink/up"    // published to

MqttPubSub mqtt2Client(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT2_CLIENT_ID);

SerLinkMqttAdapter mqtt2(&mqtt2Client, MQTT2_TOPIC_UP, MQTT2_TOPIC_DOWN);

//--------------------------------------------------------------
// SerLink2 - the same stack as SerLink0 and SerLink1, over mqtt2.
//
//   reader2 takes received frames from mqtt2.rxDataQueue and sends its
//           acks with mqtt2.write() (Reader::setAckWriteFunc()).
//   writer2 sends frames with mqtt2.write().
//
// Both call write() from their own tasks, which SerLinkMqttAdapter allows:
// the publish takes lwIP's core lock itself. See SerLinkMqttAdapter.hpp.
//
// Sockets: DBG00 (link check, stack query) and CTRL0 (speed controller),
// sharing their handlers with SerLink0 - a socket belongs to one
// transport, a handler does not. So CTRL0 over MQTT and CTRL0 over uart2
// drive the same controllerB, and the last command from either wins.
//
// The whole stack is idle until mqtt2 connects: until then nothing
// arrives on rxDataQueue, and write() refuses, so the CTRL0 status frame
// is dropped rather than queued up.
SerLink::Writer writer2(WRITER_CONFIG__WRITER2_ID);
SerLink::Reader reader2(READER_CONFIG__READER2_ID);

#define TRANSPORT2_QUEUE_LENGTH 5
StaticQueue_t transport2StaticQueue;
uint8_t transport2QueueStorageArea[TRANSPORT2_QUEUE_LENGTH * sizeof(SerLink::FrameMsg)];
QueueHandle_t transport2Queue;

SerLink::Transport transport2(&writer2, &reader2);

//--------------------------------------------------------------
void initTasks()
{
  // Created synchronously here (rather than inside startSerLink0Task) so
  // transport0.queue is guaranteed valid before any task - including
  // startReader0Task, which passes it to reader0.init() - can run.
  transport0Queue = xQueueCreateStatic(TRANSPORT0_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport0QueueStorageArea, &transport0StaticQueue);
  transport0.init(transport0Queue, transport0ReceiveCallback, transport0AckCallback);

  radioSocket = transport0.acquireSocket("RAD00");
  ledSerialSocket = transport0.acquireSocket("LED01");

  // Motor drive. setPercent() is what brings the hardware up: PWM
  // configures its timer and GPIO on first use, so this is where PC6/PC7
  // stop being inputs and TIM8 starts running.
  //
  // The direction is still idle, so applyOutputs() drives both channels
  // to 0% - IN1B and IN2B both low, which the datasheet's Input/Output
  // table calls Stop (outputs high impedance). MOTORB_START_PERCENT is
  // stored and takes effect on the first setDirection(); nothing turns
  // until then.
  //
  // enable() comes last, so /STBY only goes high with both IN pins
  // already parked low - the state the datasheet asks for across a
  // standby transition. Call motorStandby.disable() to coast the
  // bridge without disturbing the PWM settings.
  //motorB.setPercent(MOTORB_START_PERCENT);
  motorB.setPercent(0);          // motor is idle - as it is now controlled by controllerB
  motorStandby.enable();

  motorTaskHandle = osThreadNew(startMotorTask, NULL, &motorTask_attributes);

  /* motorB tachometer. init() creates the timestamp queue and starts TIM5
     before it enables the EXTI line, so the first edge always has
     somewhere to go - which matters here because interrupts are already
     on and the wheel may still be turning from a previous run.

     Before controlBTask exists, so that task cannot reach update() while
     the queue is still being built. */
  tachoB.init();

  /* Transmit only: no receive callback and no instant handler, because
     nothing is ever sent to this socket. Acquired here rather than from
     controlBTask so the socket table is complete before the scheduler
     starts. */
  tachoSocket = transport0.acquireSocket("TACHO");

  /* Only validates the config - the controller stays disabled until a
     set speed command arrives. Before controlBTask exists, like tachoB. */
  controllerB.init();

  /* A demand without enable(): the motor stays still, but a lift start
     now has a speed to run at from boot (Lift::start() refuses a zero
     demand). CTRL0 BR<dddd> replaces it. */
  controllerB.setRequiredRpm(CONTROLB_BOOT_RPM);

  /* Only validates - the lift is idle until a start command. Its command
     queue is created here, with it, so both exist before controlBTask
     can drain one or the LIFT0 socket (acquired below, with SerLink2)
     can post to it. */
  liftB.init();
  liftCmdQueue = xQueueCreateStatic(LIFT_CMD_QUEUE_LENGTH, sizeof(LiftCmd),
    liftCmdQueueStorageArea, &liftCmdStaticQueue);

  controlBTaskHandle = osThreadNew(startControlBTask, NULL, &controlBTask_attributes);

  writer0.init(uart2_writeBlocking);
  reader0.init(uart2Queue, &writer0, transport0.queue);

  /* Sets are handled by motorSockReceiveHandler (in serLink0Task), reads
     by motorSockInstantHandler (in reader0Task, so the answer rides back
     on the ack).

     Order against reader0.init() does not matter - the socket keeps its
     own instant handler, and reader0 finds it through transport0 (see
     Instant handling in Socket.hpp) - but this stays next to the
     reader/writer setup it depends on. Still before the scheduler
     starts, so no task can see the socket half-registered.

     transport0 holds eight of the SERLINK_CONFIG__MAX_SOCKETS slots -
     RAD00, LED01, MOTOR, CTRL0, ADC00 and TACHO here, DBG00 and MQTT0
     later, from their own tasks. An acquire past the limit returns a
     silent nullptr, which is why every socket pointer is checked before
     use. */
  motorSocket = transport0.acquireSocket("MOTOR", motorSockReceiveHandler,
    motorSockInstantHandler);

  /* Same split as MOTOR: sets in serLink0Task, reads on the ack. */
  controlSocket = transport0.acquireSocket("CTRL0", controlSockReceiveHandler,
    controlSockInstantHandler);

  /* Analog inputs. init() only builds the queue and registers adc1 for the
     HAL callbacks; start() arms the DMA and starts TIM2, after which the
     sampling free runs in hardware. Both happen here, before the scheduler,
     so the first half buffer cannot complete while the socket table or the
     supervising task is still half built.

     Reads only, so there is no receive callback - the instant handler puts
     the answer on the ack instead. */
  adcSocket = transport0.acquireSocket("ADC00", nullptr, adcSockInstantHandler);

  adc1.init();
  adc1.start();

  adcTaskHandle = osThreadNew(startAdcTask, NULL, &adcTask_attributes);

  writer0TaskHandle = osThreadNew(startWriter0Task, NULL, &writer0Task_attributes);

  reader0TaskHandle = osThreadNew(startReader0Task, NULL, &reader0Task_attributes);

  serLink0TaskHandle = osThreadNew(startSerLink0Task, NULL, &serLink0Task_attributes);

   /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of mqttTask and mqttRxTask */
  // Creates mqtt.rxQueue, before mqttRxTask can block on it. Subscribing has
  // to wait for lwIP, so that happens in mqttTask.
  mqtt.init();

  /* The SerLink2 link layer. init() creates mqtt2's frame queue and
     mqtt2Client's rxQueue and touches no lwIP, so it belongs here; the
     connection itself cannot start until MX_LWIP_Init() has run, and so
     happens in startMqtt2Task. */
  mqtt2.init();

  mqtt2TaskHandle = osThreadNew(startMqtt2Task, NULL, &mqtt2Task_attributes);

  /* SerLink2, on top of mqtt2. Same order as SerLink0: the transport's
     queue first, then its sockets, then the reader and writer, all before
     the scheduler - so no task sees a half built socket table.

     After mqtt2.init(), which is what creates the rxDataQueue reader2 is
     handed. The sockets are acquired here rather than from serLink2Task
     (as SerLink0/1 do with DBG00) for the same reason. */
  transport2Queue = xQueueCreateStatic(TRANSPORT2_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport2QueueStorageArea, &transport2StaticQueue);
  transport2.init(transport2Queue);

  transport2.acquireSocket("DBG00", nullptr, debugSockInstantHandler);
  controlMqttSocket = transport2.acquireSocket("CTRL0", controlSockReceiveHandler,
    controlSockInstantHandler);

  /* Commands in serLink2Task, posted on to controlBTask; the status read
     on the ack, from reader2Task. */
  liftMqttSocket = transport2.acquireSocket("LIFT0", liftSockReceiveHandler,
    liftSockInstantHandler);

  writer2.init([](char* buffer) -> uint8_t { return mqtt2.write(buffer); });
  reader2.init(mqtt2.rxDataQueue, &writer2, transport2.queue);
  reader2.setAckWriteFunc([](char* buffer) -> uint8_t { return mqtt2.write(buffer); });

  writer2TaskHandle = osThreadNew(startWriter2Task, NULL, &writer2Task_attributes);
  reader2TaskHandle = osThreadNew(startReader2Task, NULL, &reader2Task_attributes);
  serLink2TaskHandle = osThreadNew(startSerLink2Task, NULL, &serLink2Task_attributes);

  mqttTaskHandle = osThreadNew(startMqttTask, NULL, &mqttTask_attributes);

  mqttRxTaskHandle = osThreadNew(startMqttRxTask, NULL, &mqttRxTask_attributes);

  // The nRF24L01 has one owner at a time - SerLink1 through radio1, or one of
  // the raw test tasks - since the driver is not thread-safe. Select with
  // RADIO_SERLINK and RADIO_TEST_TX.
#if RADIO_SERLINK
  transport1Queue = xQueueCreateStatic(TRANSPORT1_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport1QueueStorageArea, &transport1StaticQueue);
  transport1.init(transport1Queue);

  // Registered before the scheduler starts, so the Transport tasks never see
  // either LED01 socket unrelayed.
  ledRadioSocket = transport1.acquireSocket("LED01");
  ledRelay.init();
  ledRelay.registerPair(ledSerialSocket, ledRadioSocket);

  // Before writer1/reader1: init() creates the queues they are handed.
  radio1.init((const uint8_t*)"00001");

  // Queued now, applied once radio1Task has brought the device up. SerLink
  // needs the radio listening between transmissions or no ack ever arrives.
  radio1.startListening();

  writer1.init([](char* buffer) -> uint8_t {
    return radio1.write(buffer);
  });
  reader1.init(radio1.rxDataQueue, &writer1, transport1.queue, radio1.eventQueue);

  writer1TaskHandle = osThreadNew(startWriter1Task, NULL, &writer1Task_attributes);

  reader1TaskHandle = osThreadNew(startReader1Task, NULL, &reader1Task_attributes);

  serLink1TaskHandle = osThreadNew(startSerLink1Task, NULL, &serLink1Task_attributes);

  radio1TaskHandle = osThreadNew(startRadio1Task, NULL, &radio1Task_attributes);

  relayTaskHandle = osThreadNew(startRelayTask, NULL, &relayTask_attributes);

#elif RADIO_TEST_TX
  radioTxTaskHandle = osThreadNew(startRadioTxTask, NULL, &radioTxTask_attributes);
#else
  radioRxTaskHandle = osThreadNew(startRadioRxTask, NULL, &radioRxTask_attributes);
#endif
}

void startWriter0Task(void *argument)
{
  /* USER CODE BEGIN startWriter0Task */
  //writer0.init();

  for(;;)
  {
    writer0.run();
  }
  /* USER CODE END startWriter0Task */
}

void startReader0Task(void *argument)
{
  /* USER CODE BEGIN startReader0Task */
  //reader0.init(uart2Queue, &writer0, transport0.queue);

  for(;;)
  {
    reader0.run();
  }
  /* USER CODE END startReader0Task */
}

void startSerLink0Task(void *argument)
{
  /* USER CODE BEGIN startSerLink0Task */
  SerLink::Socket* debugSocket = transport0.acquireSocket("DBG00", nullptr, debugSockInstantHandler);



  for(;;)
  {
    transport0.run();
  }
  /* USER CODE END startSerLink0Task */
}

// Owns ledRelay: all relaying happens here (see SerlinkRelay.hpp).
void startRelayTask(void *argument)
{
  for(;;)
  {
    ledRelay.run();
  }
}

//--------------------------------------------------------------
// SerLink1: the same stack as SerLink0, carried by radio1 instead of uart2.
void startWriter1Task(void *argument)
{
  for(;;)
  {
    writer1.run();
  }
}

void startReader1Task(void *argument)
{
  for(;;)
  {
    reader1.run();
  }
}

void startSerLink1Task(void *argument)
{
  // Same debug socket as SerLink0, so the far end can test the link with
  // DBG00T349002R2 and expect "OK" back in the ack.
  transport1.acquireSocket("DBG00", nullptr, debugSockInstantHandler);

  for(;;)
  {
    transport1.run();
  }
}

// Owns the nRF24L01 while RADIO_SERLINK is set: all SPI to it happens here.
//
// The detect check first gives a definite answer on whether the module was
// there at boot - radio1.run() on its own just retries INIT silently
// forever. The result is in radio1Detected for the debugger, and on LD2
// (blue): lit if the radio answered. A miss is not fatal: run() still
// brings the module up if it is connected later, but LD2 only reports the
// boot-time check. radio1DetectStatus is the STATUS byte the module
// returned - see Radio::detect() for what each value points at.
volatile bool    radio1Detected     = false;
volatile uint8_t radio1DetectStatus = 0;

void startRadio1Task(void *argument)
{
  uint8_t status = 0;
  radio1Detected     = radio1.detect(RADIO1_DETECT_TIMEOUT_MS, &status);
  radio1DetectStatus = status;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, radio1Detected ? GPIO_PIN_SET : GPIO_PIN_RESET);

  for(;;)
  {
    radio1.run();
  }
}

void StartLedTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(Led::PERIOD_MS);

  ledBoardGreen.flash(0, 2, 2, false); // continuous blink: 500 ms on / 500 ms off

  /* Infinite loop */
  for(;;)
  {
    ledBoardGreen.run();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartLedTask */
}

//--------------------------------------------------------------
// Motor task
//
// One shot: lets the board settle for MOTOR_START_DELAY_MS, then starts
// motorB at MOTORB_START_PERCENT and exits. This is the bring-up kick -
// once it has run, the MOTOR socket drives everything from here (see
// motorSockReceiveHandler below).
//
// setDirection() is what actually starts the motor. initTasks() has
// already stored the percent and taken the driver out of standby, but it
// leaves the direction at idle - both IN pins low, which the datasheet
// calls Stop - so nothing turns until this runs.
//
// reverse puts the PWM on IN1B (PC6) and holds IN2B (PC7) low; forward
// is the other way round, and idle coasts.
void startMotorTask(void *argument)
{
  osDelay(MOTOR_START_DELAY_MS);

  // motorB.setPercent(MOTORB_START_PERCENT);
  // motorB.setDirection(TC78H611FNG::reverse);

  /* Nothing further to do. INCLUDE_vTaskDelete is on, so give the stack
     back rather than parking the task in an empty loop for ever. The
     motor keeps running: the PWM is generated by TIM8 in hardware and
     does not need this task alive to hold it up. */
  osThreadTerminate(osThreadGetId());
}

//--------------------------------------------------------------
// MOTOR socket - motor control over SerLink0 (uart2).
//
// Frame data is <selector><command><args>, on top of SerLink's usual
// 12 character header (5 protocol, 1 type, 3 roll code, 3 data length):
//
//   Sets. Handled by motorSockReceiveHandler(), acked with a plain
//   ACK_OK - the ack says the frame arrived, not that the motor moved:
//
//     MOTORT516005AP030    percent = 30%   (always 3 digits, zero padded)
//                          - open loop: also disables the speed controller
//     MOTORT516006AS0300   speed = 300 RPM (always 4 digits, zero padded)
//                          - closed loop: enables the speed controller
//     MOTORT523003ADF      direction = forward
//     MOTORT523003ADR      direction = reverse
//     MOTORT523003ADD      direction = disabled
//
//   Reads. Handled by motorSockInstantHandler(), which piggybacks the
//   answer onto the ack instead of sending a frame of its own:
//
//     MOTORT529003AGP  ->  MOTORA529003030    percent,   3 digits
//     MOTORT529003AGF  ->  MOTORA5290041000   frequency, 4 digits (Hz)
//     MOTORT529003AGD  ->  MOTORA529001F      direction, one of F/R/D
//     MOTORT529003AGS  ->  MOTORA5290040300   required speed, 4 digits (RPM)
//
// The speed set only takes effect while there is a direction: with it
// idle the controller holds (see startControlBTask). Set a direction as
// well as a speed.
//
// <selector> is the TC78H611FNG bridge channel. Only channel B is wired
// (IN1B/IN2B on J10 pins 10 and 8), so for now 'A' and 'B' both reach
// motorB - see motorForSelector().
//
// 'D' for disabled means direction idle: both IN pins low, which the
// datasheet's Input/Output table calls Stop, so the motor coasts. It
// deliberately does not drop /STBY, because /STBY is device-wide and
// would take channel A down with it once that exists.

// Selector + command. Everything past this is command specific.
#define MOTOR_CMD_MIN_LEN   2U
#define MOTOR_CMD_SET_PERCENT_LEN  5U   // <sel>P<ddd>
#define MOTOR_CMD_SET_SPEED_LEN    6U   // <sel>S<dddd>
#define MOTOR_RPM_FIELD_WIDTH      4U
#define MOTOR_CMD_DIRECTION_LEN    3U   // <sel>D<F|R|D> and <sel>G<P|F|D>

// Only channel B of the TC78H611FNG is built, so both selectors resolve
// to motorB. When channel A is wired, give it its own TC78H611FNG on
// IN1A/IN2A (J10 pins 4 and 2) and return that for 'A'.
static TC78H611FNG* motorForSelector(char selector)
{
  switch(selector)
  {
    case 'A':   // -> &motorA once channel A hardware exists
    case 'B':
      return &motorB;

    default:
      return nullptr;
  }
}

// The speed controller for a selector, on the same terms as
// motorForSelector(): both reach motorB's controller for now.
static Controller* controllerForSelector(char selector)
{
  switch(selector)
  {
    case 'A':   // -> &controllerA once channel A hardware exists
    case 'B':
      return &controllerB;

    default:
      return nullptr;
  }
}

static bool motorDirectionFromChar(char value, TC78H611FNG::direction* direction)
{
  switch(value)
  {
    case 'F': *direction = TC78H611FNG::forward; return true;
    case 'R': *direction = TC78H611FNG::reverse; return true;
    case 'D': *direction = TC78H611FNG::idle;    return true;
    default:  return false;
  }
}

static char motorDirectionToChar(TC78H611FNG::direction direction)
{
  switch(direction)
  {
    case TC78H611FNG::forward: return 'F';
    case TC78H611FNG::reverse: return 'R';
    default:                   return 'D';   // idle - reported as disabled
  }
}

/* Frame::int3dToStr() does this job, but only at three digits wide, and
   the frequency read needs four. Frame::str3dToInt() is no use for the
   inbound direction either: it maps any non-digit silently onto 0, so
   "APxyz" would be read as 0% rather than rejected. */
static void writeUintField(uint32_t value, uint8_t width, char* dst)
{
  for(uint8_t i = width; i > 0U; i--)
  {
    dst[i - 1U] = (char)('0' + (value % 10U));
    value /= 10U;
  }
}

static bool readUintField(const char* src, uint8_t width, uint32_t* value)
{
  uint32_t result = 0U;

  for(uint8_t i = 0U; i < width; i++)
  {
    if((src[i] < '0') || (src[i] > '9'))
    {
      return false;
    }
    result = (result * 10U) + (uint32_t)(src[i] - '0');
  }

  *value = result;
  return true;
}

// The sets. Runs in serLink0Task, from Transport::run(), which for a 'T'
// frame is after the ack has already gone out - so a malformed command is
// dropped silently rather than reported. Use the reads to confirm what
// actually landed.
void motorSockReceiveHandler(const char* data, uint16_t dataLen)
{
  if(dataLen < MOTOR_CMD_MIN_LEN)
  {
    return;
  }

  TC78H611FNG* motor = motorForSelector(data[0]);
  if(motor == nullptr)
  {
    return;
  }

  switch(data[1])
  {
    case 'P':   // <sel>P<ddd> - set percent
    {
      uint32_t percent;

      if((dataLen == MOTOR_CMD_SET_PERCENT_LEN) &&
         readUintField(&data[2], 3U, &percent))
      {
        /* A percent means open loop, so the controller lets go first.
           Order matters: disable() before setPercent() is what guarantees
           a controller pass already in flight cannot overwrite this value
           - see Threading in Controller.hpp. */
        Controller* controller = controllerForSelector(data[0]);
        if(controller != nullptr)
        {
          controller->disable();
        }

        // No range check needed: setPercent() clamps above 100 itself,
        // and three digits cannot exceed 999.
        motor->setPercent((uint8_t)percent);
      }
      break;
    }

    case 'S':   // <sel>S<dddd> - set speed (RPM), closed loop
    {
      uint32_t rpm;
      Controller* controller = controllerForSelector(data[0]);

      if((controller != nullptr) &&
         (dataLen == MOTOR_CMD_SET_SPEED_LEN) &&
         readUintField(&data[2], MOTOR_RPM_FIELD_WIDTH, &rpm))
      {
        /* Demand first, so the first pass after enabling already works
           towards it. Four digits cannot exceed uint16_t. */
        controller->setRequiredRpm((uint16_t)rpm);
        controller->enable();
      }
      break;
    }

    case 'D':   // <sel>D<F|R|D> - set direction
    {
      TC78H611FNG::direction direction;

      if((dataLen == MOTOR_CMD_DIRECTION_LEN) &&
         motorDirectionFromChar(data[2], &direction))
      {
        motor->setDirection(direction);
      }
      break;
    }

    case 'G':   // reads are answered on the ack, in motorSockInstantHandler()
    default:
      break;
  }
}

// The reads. Runs in reader0Task, before the ack is sent, so what it
// writes here rides back on that ack.
//
// The handler is registered per protocol, so it sees the sets too.
// Returning false for those leaves the ack as a plain ACK_OK, which is
// exactly what they want.
//
// This only calls getters while motorSockReceiveHandler() does all the
// writing from another task. No lock is needed: each getter reads a
// single byte or word, which the M4 loads atomically, so a read can be
// stale by one command but never torn.
bool motorSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if((rxFrame.dataLen != MOTOR_CMD_DIRECTION_LEN) || (rxFrame.data[1] != 'G'))
  {
    return false;   // not a read - leave the ack alone
  }

  TC78H611FNG* motor = motorForSelector(rxFrame.data[0]);
  if(motor == nullptr)
  {
    return false;
  }

  switch(rxFrame.data[2])
  {
    case 'P':   // percent, 3 digits - same format the set takes
      writeUintField(motor->getPercent(), 3U, data);
      *dataLen = 3U;
      return true;

    case 'F':   // frequency in Hz, 4 digits - pwmFreqValues spans 100..2000
      writeUintField((uint32_t)motor->getFrequency(), 4U, data);
      *dataLen = 4U;
      return true;

    case 'D':   // direction, one of F/R/D
      data[0] = motorDirectionToChar(motor->getDirection());
      *dataLen = 1U;
      return true;

    case 'S':   // required speed in RPM, 4 digits - same format the set takes
    {
      Controller* controller = controllerForSelector(rxFrame.data[0]);
      if(controller == nullptr)
      {
        return false;
      }
      writeUintField(controller->getRequiredRpm(), MOTOR_RPM_FIELD_WIDTH, data);
      *dataLen = MOTOR_RPM_FIELD_WIDTH;
      return true;
    }

    default:
      return false;
  }
}

//--------------------------------------------------------------
// CTRL0 socket - speed controller commands over SerLink0 (uart2).
//
// Frame data is <controller><command><args>:
//
//   Sets. Handled by controlSockReceiveHandler(), acked with a plain
//   ACK_OK:
//
//     CTRL0T516006BR0120   required speed = 120 RPM (always 4 digits,
//                          zero padded) - also enables the controller,
//                          so the motor goes closed loop
//     CTRL0T523003BDF      direction = forward
//     CTRL0T523003BDR      direction = reverse
//     CTRL0T523003BDD      direction = disabled (idle - the motor coasts)
//     CTRL0T516008BI002000 integral gain = 0.002 - six digits, in
//                          millionths (000000..999999, so 0..0.999999).
//                          Takes effect on the next pass, enabled or not;
//                          the output carries on from where it is. Lasts
//                          until reset - CONTROLB_INTEGRAL_GAIN is the
//                          boot value
//
//   Reads. Handled by controlSockInstantHandler(), answered on the ack:
//
//     CTRL0T529003BGR  ->  CTRL0A5290040120   required speed, 4 digits
//     CTRL0T529003BGD  ->  CTRL0A529001F      direction, one of F/R/D
//     CTRL0T529003BGF  ->  CTRL0A5290011      tacho fault, 1 or 0 - 1 once
//                          the controller has stopped the motor because
//                          tachoB read zero for CONTROLB_TACHO_TIMEOUT_S
//                          with the duty at or above
//                          CONTROLB_TACHO_CHECK_MIN_PERCENT.
//                          Cleared by the next speed set (BR<dddd>).
//     CTRL0T529003BGI  ->  CTRL0A529006002000  integral gain, millionths -
//                          same format the set takes
//     CTRL0T529003BGA  ->  CTRL0A529016002000.0150.0148
//                          all at once: <gain>.<required>.<measured> -
//                          integral gain (6 digits, millionths), required
//                          RPM (4) and measured RPM (4, clamped at 9999).
//                          One read, so the three are from the same moment
//
//   Status. Sent unsolicited by controlBTask every
//   TACHO_PUBLISH_PERIOD_MS while liftB is moving, as 'U' (no ack
//   expected):
//
//     CTRL0U001008030.0350   motorB duty 30%, tachoB 350 RPM
//
//   Duty is 3 digits, RPM 4 (clamped at 9999). It carries no controller
//   letter - it is always controllerB, the only one. Add one when
//   controllerA exists.
//
// The controller only drives the motor while it has a direction - with
// it idle the controller holds - so set a direction as well as a speed.
// Either order works. The direction set does not enable the controller:
// on its own it just starts the motor at whatever duty cycle it has.
// Open loop (and so disabling the controller) is still the MOTOR
// socket's percent set.
//
// The direction read reports the motor, so a direction set through the
// MOTOR socket shows here too - see Direction in Controller.hpp.
//
// <controller> is resolved by controllerForSelector(), so it follows the
// MOTOR socket's selectors: only controllerB exists, and 'A' reaches it
// too until channel A is built.

#define CONTROL_CMD_MIN_LEN        2U   // <ctl><cmd>
#define CONTROL_CMD_SET_RPM_LEN    6U   // <ctl>R<dddd>
#define CONTROL_CMD_DIRECTION_LEN  3U   // <ctl>D<F|R|D>
#define CONTROL_CMD_SET_GAIN_LEN   8U   // <ctl>I<dddddd>
#define CONTROL_CMD_GET_LEN        3U   // <ctl>G<R|D|F|I|A>
#define CONTROL_RPM_FIELD_WIDTH    4U
#define CONTROL_PWM_FIELD_WIDTH    3U   // status frame duty cycle, 0..100

// Integral gain on the wire: an integer number of millionths, so the
// socket never has to parse or print a float. 1e-6 is far finer than any
// useful step (the boot value is 0.002 = 002000).
#define CONTROL_GAIN_FIELD_WIDTH   6U
#define CONTROL_GAIN_FIELD_MAX     999999U
#define CONTROL_GAIN_SCALE         1000000.0f

// Same letters as the MOTOR socket's direction commands, D for disabled
// meaning idle.
static bool controlDirectionFromChar(char value, ControllerDirection* direction)
{
  switch(value)
  {
    case 'F': *direction = ControllerDirection::forward; return true;
    case 'R': *direction = ControllerDirection::reverse; return true;
    case 'D': *direction = ControllerDirection::idle;    return true;
    default:  return false;
  }
}

static char controlDirectionToChar(ControllerDirection direction)
{
  switch(direction)
  {
    case ControllerDirection::forward: return 'F';
    case ControllerDirection::reverse: return 'R';
    default:                           return 'D';
  }
}

// The integral gain as CONTROL_GAIN_FIELD_WIDTH digits of millionths, no
// NUL. Rounded, so 0.002 set as 002000 reads back as 002000 despite the
// float in between. Clamped: a gain given in the config can be up to
// Controller::MAX_INTEGRAL_GAIN (1.0), one digit wider than the field, and
// writeUintField() would otherwise keep only its low digits.
static void writeGainField(float gain, char* dst)
{
  uint32_t micro = (uint32_t)((gain * CONTROL_GAIN_SCALE) + 0.5f);
  if(micro > CONTROL_GAIN_FIELD_MAX) { micro = CONTROL_GAIN_FIELD_MAX; }

  writeUintField(micro, CONTROL_GAIN_FIELD_WIDTH, dst);
}

// The sets. Runs in serLink0Task after the ack has gone out, so a
// malformed command is dropped silently - read it back to confirm.
void controlSockReceiveHandler(const char* data, uint16_t dataLen)
{
  if(dataLen < CONTROL_CMD_MIN_LEN)
  {
    return;
  }

  Controller* controller = controllerForSelector(data[0]);
  if(controller == nullptr)
  {
    return;
  }

  switch(data[1])
  {
    case 'R':   // <ctl>R<dddd> - set required speed, closed loop
    {
      uint32_t rpm;

      if((dataLen == CONTROL_CMD_SET_RPM_LEN) &&
         readUintField(&data[2], CONTROL_RPM_FIELD_WIDTH, &rpm))
      {
        /* Demand first, so the first pass after enabling already works
           towards it. Four digits cannot exceed uint16_t. */
        controller->setRequiredRpm((uint16_t)rpm);
        controller->enable();
      }
      break;
    }

    case 'D':   // <ctl>D<F|R|D> - set direction
    {
      ControllerDirection direction;

      if((dataLen == CONTROL_CMD_DIRECTION_LEN) &&
         controlDirectionFromChar(data[2], &direction))
      {
        controller->setDirection(direction);
      }
      break;
    }

    case 'I':   // <ctl>I<dddddd> - set integral gain, in millionths
    {
      uint32_t micro;

      /* setIntegralGain() range checks too, but six digits cannot leave
         0..0.999999, which is inside its range - so a refusal here can only
         be a malformed field, dropped like any other. */
      if((dataLen == CONTROL_CMD_SET_GAIN_LEN) &&
         readUintField(&data[2], CONTROL_GAIN_FIELD_WIDTH, &micro))
      {
        (void)controller->setIntegralGain((float)micro / CONTROL_GAIN_SCALE);
      }
      break;
    }

    case 'G':   // reads are answered on the ack, in controlSockInstantHandler()
    default:
      break;
  }
}

// The read. Runs in reader0Task, before the ack is sent. Returns false
// for the set, leaving its ack a plain ACK_OK. Getter only, so no lock -
// same reasoning as motorSockInstantHandler().
bool controlSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if((rxFrame.dataLen != CONTROL_CMD_GET_LEN) || (rxFrame.data[1] != 'G'))
  {
    return false;   // not a read - leave the ack alone
  }

  Controller* controller = controllerForSelector(rxFrame.data[0]);
  if(controller == nullptr)
  {
    return false;
  }

  switch(rxFrame.data[2])
  {
    case 'R':   // required speed in RPM, 4 digits - same format the set takes
      writeUintField(controller->getRequiredRpm(), CONTROL_RPM_FIELD_WIDTH, data);
      *dataLen = CONTROL_RPM_FIELD_WIDTH;
      return true;

    case 'D':   // direction, one of F/R/D - same letters the set takes
      data[0] = controlDirectionToChar(controller->getDirection());
      *dataLen = 1U;
      return true;

    case 'F':   // tacho fault, 1 or 0 - see Unresponsive tachometer in Controller.hpp
      data[0] = controller->isTachoFault() ? '1' : '0';
      *dataLen = 1U;
      return true;

    case 'I':   // integral gain in millionths, 6 digits - same format the set takes
      writeGainField(controller->getIntegralGain(), data);
      *dataLen = CONTROL_GAIN_FIELD_WIDTH;
      return true;

    case 'A':   // <gain>.<required>.<measured> - see the CTRL0 notes
    {
      /* writeUintField() keeps only the low digits, so clamp the measured
         speed to the field rather than report an unrelated number - a
         faulty tacho can read up to 65535. The required speed was set
         through a 4 digit field, so it always fits. */
      uint32_t rpm = controller->getRpm();
      if(rpm > 9999U) { rpm = 9999U; }

      uint16_t len = 0U;
      writeGainField(controller->getIntegralGain(), &data[len]);
      len += CONTROL_GAIN_FIELD_WIDTH;
      data[len++] = '.';
      writeUintField(controller->getRequiredRpm(), CONTROL_RPM_FIELD_WIDTH, &data[len]);
      len += CONTROL_RPM_FIELD_WIDTH;
      data[len++] = '.';
      writeUintField(rpm, CONTROL_RPM_FIELD_WIDTH, &data[len]);
      len += CONTROL_RPM_FIELD_WIDTH;

      *dataLen = len;
      return true;
    }

    default:
      return false;
  }
}

//--------------------------------------------------------------
// LIFT0 socket - lift commands over SerLink2 (MQTT).
//
// Frame data is <lift><command><args>:
//
//   Commands. Handled by liftSockReceiveHandler(), which runs in
//   serLink2Task and only parses: the command goes on liftCmdQueue for
//   controlBTask, which owns liftB. Send as 'U' or 'T' - a 'T' ack says
//   the frame arrived, not that the lift moved:
//
//     LIFT0U645006BSF234   start forward, 234 edges (1..6 digits)
//     LIFT0U645006BSR234   start reverse, 234 edges
//     LIFT0U645006BG2000   down to the ground: reverse until the ground
//                          sensor closes, or 2000 edges, whichever is
//                          first (1..6 digits). Already on the ground,
//                          it does not move, but still sends the done
//                          message (travelled 0).
//     LIFT0U645002BX       stop - the motor coasts
//
//   A start (S or G) while the lift is moving is ignored - stop it first.
//   So is a distance of zero, and so is any start while controllerB's
//   required speed is zero (it boots at CONTROLB_BOOT_RPM): the speed is
//   set through CTRL0 (BR<dddd>), not here - see Speed in Lift.hpp.
//
//   Read. Handled by liftSockInstantHandler(), answered on the ack, so it
//   must be sent as 'T':
//
//     LIFT0T645002BT  ->  LIFT0A645014M000120.000234
//
//   <M|I|G>      moving, idle, or idle with the last move ended by the
//                ground sensor (a G move that ran out of edges is I)
//   <dddddd>     edges travelled since the last start - still counting
//                after a move ends, so the coast overrun shows
//   <dddddd>     that start's target
//
//   Both clamped at 999999.
//
//   Done. Sent unsolicited by controlBTask on the pass a move ends -
//   distance reached, ground reached, stop command or tacho fault - with
//   the lift's letter in front,
//   since nothing asked:
//
//     LIFT0U<rrr>015BI000234.000234   liftB idle, 234 of 234 edges
//
//   The travelled count is taken at that pass, so it does not include
//   the coast that follows; a status read a moment later does. A start
//   and a stop landing in the same pass never show the lift moving, so
//   send nothing. 'U' unless LIFT_DONE_ACK - see there.
//
// <lift> is resolved by liftForSelector(): only liftB exists.

#define LIFT_CMD_MIN_LEN         2U   // <lift><cmd>
#define LIFT_CMD_START_MIN_LEN   4U   // <lift>S<F|R><d>
#define LIFT_DISTANCE_MAX_DIGITS 6U
#define LIFT_CMD_START_MAX_LEN   (3U + LIFT_DISTANCE_MAX_DIGITS)
#define LIFT_CMD_GROUND_MIN_LEN  3U   // <lift>G<d>
#define LIFT_CMD_GROUND_MAX_LEN  (2U + LIFT_DISTANCE_MAX_DIGITS)
#define LIFT_CMD_STATUS_LEN      2U   // <lift>T
#define LIFT_FIELD_WIDTH         LIFT_DISTANCE_MAX_DIGITS
#define LIFT_FIELD_MAX           999999U
#define LIFT_STATUS_LEN          (2U + (2U * LIFT_FIELD_WIDTH))   // <M|I|G><d6>.<d6>

// The done message's frame type. 'U' (false) matches the CTRL0 status
// frame: fire and forget, and over MQTT - TCP underneath - it is lost
// only if the broker or the PC is not there. true sends it as 'T', and
// writer2 then retries until the PC acks it; only worth it if the PC
// side sends acks, or every done message costs writer2 its full retry
// cycle and holds up the frames queued behind it.
#define LIFT_DONE_ACK            false

// <M|I|G><travelled>.<target> into dst, LIFT_STATUS_LEN chars, no NUL -
// the status read's answer and the done message share it. Getters only,
// so safe from any task.
static void writeLiftStatus(const Lift& lift, char* dst)
{
  /* writeUintField() keeps only the low digits, so clamp rather than
     report an unrelated number. */
  uint32_t travelled = lift.getTravelled();
  uint32_t target = lift.getTarget();
  if(travelled > LIFT_FIELD_MAX) { travelled = LIFT_FIELD_MAX; }
  if(target > LIFT_FIELD_MAX)    { target = LIFT_FIELD_MAX; }

  if(lift.getStatus() == Lift::status::moving)           { dst[0] = 'M'; }
  else if(lift.getEndReason() == Lift::endReason::ground) { dst[0] = 'G'; }
  else                                                    { dst[0] = 'I'; }
  writeUintField(travelled, LIFT_FIELD_WIDTH, &dst[1]);
  dst[1U + LIFT_FIELD_WIDTH] = '.';
  writeUintField(target, LIFT_FIELD_WIDTH, &dst[2U + LIFT_FIELD_WIDTH]);
}

static Lift* liftForSelector(char selector)
{
  switch(selector)
  {
    case 'B':
      return &liftB;

    default:
      return nullptr;
  }
}

// Runs in serLink2Task. Parses and posts; liftB itself is only touched
// by controlBTask. Never blocks - a full queue drops the command.
void liftSockReceiveHandler(const char* data, uint16_t dataLen)
{
  if(dataLen < LIFT_CMD_MIN_LEN)
  {
    return;
  }

  LiftCmd cmd = {};
  cmd.lift = liftForSelector(data[0]);
  if(cmd.lift == nullptr)
  {
    return;
  }

  switch(data[1])
  {
    case 'S':   // <lift>S<F|R><d..d> - start
    {
      if((dataLen < LIFT_CMD_START_MIN_LEN) || (dataLen > LIFT_CMD_START_MAX_LEN))
      {
        return;
      }

      if(data[2] == 'F')      { cmd.dir = Lift::direction::forward; }
      else if(data[2] == 'R') { cmd.dir = Lift::direction::reverse; }
      else                    { return; }

      if(!readUintField(&data[3], (uint8_t)(dataLen - 3U), &cmd.distance))
      {
        return;
      }

      cmd.op = LiftCmd::start;
      break;
    }

    case 'G':   // <lift>G<d..d> - down to the ground, d..d edges at most
    {
      if((dataLen < LIFT_CMD_GROUND_MIN_LEN) || (dataLen > LIFT_CMD_GROUND_MAX_LEN))
      {
        return;
      }

      if(!readUintField(&data[2], (uint8_t)(dataLen - 2U), &cmd.distance))
      {
        return;
      }

      cmd.op = LiftCmd::ground;
      break;
    }

    case 'X':   // <lift>X - stop
      if(dataLen != LIFT_CMD_MIN_LEN)
      {
        return;
      }
      cmd.op = LiftCmd::stop;
      break;

    case 'T':   // status - answered on the ack, in liftSockInstantHandler()
    default:
      return;
  }

  (void)xQueueSend(liftCmdQueue, &cmd, 0U);
}

// The status read. Runs in reader2Task, before the ack is sent. Getters
// only - see Threading in Lift.hpp.
bool liftSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if((rxFrame.dataLen != LIFT_CMD_STATUS_LEN) || (rxFrame.data[1] != 'T'))
  {
    return false;   // a command - leave the ack a plain ACK_OK
  }

  Lift* lift = liftForSelector(rxFrame.data[0]);
  if(lift == nullptr)
  {
    return false;
  }

  writeLiftStatus(*lift, data);
  *dataLen = LIFT_STATUS_LEN;
  return true;
}

//--------------------------------------------------------------
// Analog input supervision.
//
// Nothing here is in the sample path: TIM2 triggers the scans, the DMA
// fills the buffer and the DMA interrupt averages each half and publishes
// it. This task only calls run(), which blocks internally and restarts the
// sampling if it ever stops - see the comment on Adc::run().
void startAdcTask(void *argument)
{
  for(;;)
  {
    adc1.run();
  }
}

//--------------------------------------------------------------
// controlB task - motorB closed loop, at CONTROLB_PERIOD_MS.
//
// For now it only services the tachometer. tachoB.update() drains the
// timestamps the EXTI4 ISR has queued since the last pass and turns the
// complete revolutions among them into an RPM; it does not block, so the
// period is set here with vTaskDelayUntil rather than inside the driver.
//
// This is the one task allowed to call update(). The getters are safe
// from anywhere - see the threading note in Tachometer.hpp.
//
// liftB runs between the update and the delay - and runs controllerB -
// so both always see the reading taken this pass. controllerB does
// nothing until a lift start (or the MOTOR/CTRL0 socket) enables it.
void startControlBTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROLB_PERIOD_MS);

  /* A whole number of loop passes - the static_assert above says so. */
  const uint32_t publishEvery = TACHO_PUBLISH_PERIOD_MS / CONTROLB_PERIOD_MS;
  uint32_t passes = 0U;

  char speedData[1U + TACHO_RPM_FIELD_WIDTH + 1U + TACHO_GLITCH_FIELD_WIDTH];
  char statusData[CONTROL_PWM_FIELD_WIDTH + 1U + CONTROL_RPM_FIELD_WIDTH];

  /* liftB's status as of the end of the previous pass, so a move ending
     shows as moving -> idle across liftB.run(). */
  Lift::status liftBLastStatus = liftB.getStatus();
  char liftDoneData[1U + LIFT_STATUS_LEN];   // <lift><status>

  for(;;)
  {
    tachoB.update();

    /* Lift commands from the LIFT0 socket. After the update, so a start
       measures from this pass's edge count; drained completely, so a
       start and a stop sent together are both seen, in order. */
    LiftCmd cmd;
    bool liftBGroundAlready = false;
    while(xQueueReceive(liftCmdQueue, &cmd, 0U) == pdTRUE)
    {
      if(cmd.op == LiftCmd::start)
      {
        (void)cmd.lift->start(cmd.dir, cmd.distance);   // refused if moving
      }
      else if(cmd.op == LiftCmd::ground)
      {
        /* Accepted but still idle: already on the ground, so it never
           moved. The PC still waits for a done message - send one below. */
        if(cmd.lift->toGroundLevel(cmd.distance) &&
           (cmd.lift->getStatus() == Lift::status::idle))
        {
          liftBGroundAlready = true;
        }
      }
      else
      {
        cmd.lift->stop();
      }
    }

    /* Ends the move if its distance is up, then runs controllerB - or
       holds it while the direction is idle. See Lift::run(). */
    liftB.run();

    /* A move has ended since the last pass - distance or ground reached
       in run(), or a stop command above - or a ground move found the
       lift already there. Tell the PC over the socket the start came in
       on. Non-blocking: sendData() only queues the frame for writer2,
       and it is dropped if mqtt2 is not connected. */
    Lift::status liftBStatus = liftB.getStatus();
    if((((liftBLastStatus == Lift::status::moving) &&
         (liftBStatus == Lift::status::idle)) || liftBGroundAlready) &&
       (liftMqttSocket != nullptr))
    {
      liftDoneData[0] = 'B';
      writeLiftStatus(liftB, &liftDoneData[1]);
      liftMqttSocket->sendData(liftDoneData, (uint16_t)sizeof(liftDoneData),
        LIFT_DONE_ACK);
    }
    liftBLastStatus = liftBStatus;

    if(++passes >= publishEvery)
    {
      passes = 0U;

      /* writeUintField() writes the low digits of whatever it is given,
         so a value wider than the field would be silently mangled -
         getRpm() saturates at 65535, which is five digits. Clamp to the
         field instead, so an implausible reading shows as 9999 rather
         than as some unrelated number. Both fields below are 4 wide. */
      static_assert(TACHO_RPM_FIELD_WIDTH == CONTROL_RPM_FIELD_WIDTH,
        "rpmField is clamped for both sockets");
      uint32_t rpmField = tachoB.getRpm();
      if(rpmField > 9999U)
      {
        rpmField = 9999U;
      }

      /* <ppp>.<rrrr> - see the CTRL0 status frame. The duty cycle is the
         motor's actual one, so it is right in open loop as well.
         setPercent() clamps at 100, so three digits always fit. */
      writeUintField(controllerB.getPwmPercent(), CONTROL_PWM_FIELD_WIDTH,
        &statusData[0]);
      statusData[CONTROL_PWM_FIELD_WIDTH] = '.';
      writeUintField(rpmField, CONTROL_RPM_FIELD_WIDTH,
        &statusData[CONTROL_PWM_FIELD_WIDTH + 1U]);

      /* 'U': fire and forget, same reasoning as TACHO. Sent on both
         links, uart2 and MQTT - over MQTT it is simply dropped while
         mqtt2 is not connected.

         Only while liftB is moving, so an idle board keeps the console
         quiet. liftBStatus is this pass's, taken after liftB.run(), so
         the pass a move ends sends the LIFT0 done frame above instead.
         A closed loop run started from CTRL0 or MOTOR, with no lift
         move, is not reported - read it with CTRL0T529003BGR. */
      if(liftBStatus == Lift::status::moving)
      {
        if(controlSocket != nullptr)
        {
          controlSocket->sendData(statusData, (uint16_t)sizeof(statusData), false);
        }
        if(controlMqttSocket != nullptr)
        {
          controlMqttSocket->sendData(statusData, (uint16_t)sizeof(statusData), false);
        }
      }

      if(tachoSocket != nullptr)
      {
        speedData[0] = 'R';
        writeUintField(rpmField, TACHO_RPM_FIELD_WIDTH, &speedData[1]);

        /* Edges rejected by the ISR's filters since init() - see
           Tachometer::onEdge(). Climbing with duty cycle means PWM noise
           is reaching PF4. */
        speedData[1U + TACHO_RPM_FIELD_WIDTH] = '.';
        writeUintField(tachoB.getGlitchCount(), TACHO_GLITCH_FIELD_WIDTH,
          &speedData[2U + TACHO_RPM_FIELD_WIDTH]);

        /* Non-blocking, and fire and forget. */
        //tachoSocket->sendData(speedData, (uint16_t)sizeof(speedData), false);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//--------------------------------------------------------------
// ADC00 socket - analog readings over SerLink0 (uart2).
//
// Reads only, so every command is handled by adcSockInstantHandler() and
// the answer rides back on the ack. Frame data is <command><selector>, on
// top of SerLink's usual 12 character header:
//
//   ADC00T529002G1  ->  ADC00A5290042047    raw count, 4 digits (0..4095)
//   ADC00T529002V1  ->  ADC00A5290041650    millivolts, 4 digits
//   ADC00T529002GA  ->  ADC00A529016<16>    all channels raw, 4 digits each
//   ADC00T529001E   ->  ADC00A5290040000    overrun count, 4 digits
//
// <selector> is the channel index, 0 based, in the rank order the .ioc
// builds - so '1' is rank 2, ADC1_IN3, the motorB current sense. 'A' reads
// them all in one frame, which is the useful one from a terminal.
//
// Every value is an average of the last 32 scans, refreshed at 10 Hz. A
// read returns the most recent set; it never waits for the next one.
//
// 'E' is the overrun count. It should stay at zero - see Adc::run() for
// what a non-zero value means.

// Command + selector, or a bare command for the status reads.
#define ADC_CMD_READ_LEN     2U
#define ADC_CMD_STATUS_LEN   1U
#define ADC_FIELD_WIDTH      4U

// Runs in reader0Task, before the ack goes out. Only calls getters, which
// read one half-word each - atomic on this core, so a value can be one
// publish stale but never torn. Same reasoning as motorSockInstantHandler().
bool adcSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if((rxFrame.dataLen == ADC_CMD_STATUS_LEN) && (rxFrame.data[0] == 'E'))
  {
    writeUintField(adc1.getOverrunCount(), ADC_FIELD_WIDTH, data);
    *dataLen = ADC_FIELD_WIDTH;
    return true;
  }

  if(rxFrame.dataLen != ADC_CMD_READ_LEN)
  {
    return false;   // not a read - leave the ack alone
  }

  const char command  = rxFrame.data[0];
  const char selector = rxFrame.data[1];

  if((command != 'G') && (command != 'V'))
  {
    return false;
  }

  if(selector == 'A')   // every channel, raw counts, in rank order
  {
    if(command != 'G')
    {
      return false;   // no millivolts variant - one field type per frame
    }

    const uint8_t numChannels = adc1.getNumChannels();

    for(uint8_t ch = 0U; ch < numChannels; ch++)
    {
      writeUintField(adc1.getCount(ch), ADC_FIELD_WIDTH, &data[ch * ADC_FIELD_WIDTH]);
    }

    *dataLen = (uint16_t)(numChannels * ADC_FIELD_WIDTH);
    return true;
  }

  if((selector < '0') || (selector > '9'))
  {
    return false;
  }

  const uint8_t channel = (uint8_t)(selector - '0');
  if(channel >= adc1.getNumChannels())
  {
    return false;
  }

  writeUintField((command == 'G') ? adc1.getCount(channel)
                                  : adc1.getMillivolts(channel),
    ADC_FIELD_WIDTH, data);

  *dataLen = ADC_FIELD_WIDTH;
  return true;
}

//--------------------------------------------------------------
// MQTT publish
//
// Every MQTT_PUBLISH_PERIOD_MS publishes "msg stm32: <n>" to MQTT_TOPIC, n
// counting up from 0. While not connected it retries the connection once per
// period instead, so the broker can start after the board, or restart.
void startMqttTask(void *argument)
{
  uint32_t count = 0;
  char     msg[32];

  /* initTasks() runs before StartDefaultTask() calls MX_LWIP_Init(), which
     creates the tcpip core lock every MqttPubSub method (bar receive()) takes.
     The netif is only brought up after that, so it is the signal that lwIP
     is ready. Waiting for the link as well saves a connect that could only
     time out. */
  while(!netif_is_up(&gnetif) || !netif_is_link_up(&gnetif))
  {
    osDelay(500);
  }

  // Only recorded here. Sent once the broker accepts the connection, and
  // again after every reconnect.
  mqtt.subscribe(MQTT_SUB_TOPIC);

  for(;;)
  {
    if(mqtt.isConnected())
    {
      int msgLen = snprintf(msg, sizeof(msg), "msg stm32: %lu", (unsigned long)count);

      mqtt.publish(MQTT_TOPIC, msg, (uint16_t)msgLen);
      count++;
    }
    else
    {
      // Returns false, harmlessly, while an earlier attempt is still pending
      mqtt.connect();
    }

    osDelay(MQTT_PUBLISH_PERIOD_MS);
  }
}

//--------------------------------------------------------------
// Owns mqtt2: the only caller of its receive path, and the task that keeps
// the connection up. See SerLinkMqttAdapter.hpp for why the transmit side
// is not here - write() publishes directly from the caller's task.
void startMqtt2Task(void *argument)
{
  /* Same wait as startMqttTask. initTasks() ran before MX_LWIP_Init(),
     which is what creates the tcpip core lock that start() reaches
     through, and the netif coming up is the signal that it exists. */
  while(!netif_is_up(&gnetif) || !netif_is_link_up(&gnetif))
  {
    osDelay(500);
  }

  mqtt2.start();

  for(;;)
  {
    mqtt2.run();
  }
}

//--------------------------------------------------------------
// SerLink2: the same stack as SerLink0, carried by mqtt2 instead of
// uart2. Everything was set up in initTasks(), sockets included.
void startWriter2Task(void *argument)
{
  for(;;)
  {
    writer2.run();
  }
}

void startReader2Task(void *argument)
{
  for(;;)
  {
    reader2.run();
  }
}

void startSerLink2Task(void *argument)
{
  for(;;)
  {
    transport2.run();
  }
}

//--------------------------------------------------------------
// MQTT receive
//
// Handles each message arriving on MQTT_SUB_TOPIC. For now it just echoes
// the payload to MQTT_TOPIC as "stm32 rx: <payload>", so a message published
// to test/stm32/cmd from another machine shows up on the existing test/hello
// subscriber. Replace with real command handling.
void startMqttRxTask(void *argument)
{
  MqttPubSub::Message rxMsg;
  char reply[16 + MqttPubSub::MAX_PAYLOAD_LEN];

  mqttSocket = transport0.acquireSocket("MQTT0", nullptr, nullptr);  // for debugging

  for(;;)
  {
    /* Blocks until a message arrives. Safe before lwIP is up: rxQueue exists
       from mqtt.init(), and nothing can be posted to it until then anyway -
       which also means lwIP is up by the time publish() below runs. */
    if(mqtt.receive(rxMsg))
    {
      // int replyLen = snprintf(reply, sizeof(reply), "stm32 rx: %s", rxMsg.payload);

      // mqtt.publish(MQTT_TOPIC, reply, (uint16_t)replyLen);

      if(mqttSocket != nullptr)
      {
        mqttSocket->sendData(rxMsg.payload, rxMsg.payloadLen, false);
      }
    }
  }
}


//--------------------------------------------------------------
// Radio receive
//
// Receives from the nRF24L01 and forwards each packet to the "RAD00"
// SerLink socket, hex encoded. On a terminal a 32-byte packet arrives as:
//
//   RAD00U001064<64 hex chars>
//   \____/|\_/\_/
//     |   | |   `- dataLen, 64
//     |   | `----- rollcode
//     |   `------- type U (unidirectional - no ack requested)
//     `----------- protocol
//
// RADIO_MODE selects how the task learns a packet has landed: blocking on
// the nINT interrupt (HAL_GPIO_EXTI_Callback() below), or polling every
// RADIO_POLL_MS. Either way each wake drains the whole RX FIFO.
void startRadioRxTask(void *argument)
{
  /* Must be byte-for-byte the array the transmitter passes to
     RF24::openWritingPipe(). Both libraries clock address[0] out first, so
     matching the array order is what matters - there is no reversal to
     undo at this end.

     This is the Arduino's  const byte RADIO_ADDRESS[6] = "00001";  minus
     the string literal's trailing NUL, which RF24 ignores: the address is
     five bytes wide, and the [6] is only there to hold the terminator. */
  static const uint8_t radioRxAddress[nRF24L01::ADDRESS_LEN] =
    { '0', '0', '0', '0', '1' };

  uint8_t payload[RADIO_PAYLOAD_LEN];
  char    hex[SerLink::Frame::MAX_DATALEN];

  /* Retry rather than give up: init() only fails when the device is not
     answering on SPI, which on a plug-in module is usually a wiring or
     power fault that can be fixed without resetting the board. */
  while(!radio.init(RADIO_MODE))
  {
    osDelay(1000);
  }

  radio.setChannel(RADIO_CHANNEL);
  radio.setPayloadLen(RADIO_PAYLOAD_LEN);
  radio.openReadingPipe(1, radioRxAddress);
  radio.startListening();

  for(;;)
  {
    /* Interrupt mode wakes on an nINT falling edge, or after
       RADIO_IRQ_TIMEOUT_MS as a backstop in case an edge is ever missed.
       waitForData() returns immediately in polled mode, so that path needs
       its own delay or this loop would never yield. */
    if(RADIO_MODE == nRF24L01::Mode::Interrupt)
    {
      radio.waitForData(RADIO_IRQ_TIMEOUT_MS);
    }
    else
    {
      osDelay(RADIO_POLL_MS);
    }

    /* Drain the FIFO, don't read just one. nINT is edge-triggered and read()
       clears RX_DR, so a packet that lands while RX_DR is already set gets no
       edge of its own: read one per wake and it sits in the FIFO until the
       next packet arrives, leaving the task permanently behind. Each read()
       checks RX_EMPTY after the previous iteration cleared RX_DR, so anything
       arriving mid-loop is picked up here. Bounded in practice by the FIFO
       depth of three. */
    uint8_t len;
    while((len = radio.read(payload, RADIO_PAYLOAD_LEN)) > 0)   // 0 = FIFO empty
    {
      if(radioSocket != nullptr)
      {
        /* Hex, not raw. SerLink frames are newline-terminated text and
           Frame::setData() copies with strncpy(), so a 0x00 anywhere in the
           payload would truncate the frame and a '\n' would split it.
           32 bytes -> 64 chars, exactly Frame::MAX_DATALEN. */
        //uint16_t hexLen = bytesToHex(payload, len, hex);

        uint8_t textLen = (uint8_t)strnlen((const char*)payload, len);

        radioSocket->sendData((char*) payload, textLen, false);

        /* ack=false: a radio packet is a notification, and blocking the
           drain loop on a round trip would drop the next packet. */
        //radioSocket->sendData(hex, hexLen, false);
      }
    }
  }
}

//--------------------------------------------------------------
// Radio transmit
//
// Every RADIO_TX_PERIOD_MS sends "stm cnt: <n>", n counting up from 0. The
// count advances on every attempt, acked or not, so gaps in the sequence at
// the far end show lost packets.
//
// The far end must be listening on radioTxAddress: for an Arduino RF24
// sketch,  radio.openReadingPipe(1, "00002")  then  radio.startListening().
// Auto-ack is on, so a write() that nobody acks runs to MAX_RT (~28 ms) and
// returns false.
void startRadioTxTask(void *argument)
{
  /* Same byte order as radioRxAddress: address[0] goes out first, matching
     the array the Arduino passes to openReadingPipe(). */
  static const uint8_t radioTxAddress[nRF24L01::ADDRESS_LEN] =
    { '0', '0', '0', '0', '1' };

  uint32_t count = 0;
  char     msg[nRF24L01::MAX_PAYLOAD_LEN];

  /* Polled whatever RADIO_MODE says: write() learns how each transmission
     ended by polling STATUS, so nINT would have nothing to wake. */
  while(!radio.init(nRF24L01::Mode::Polled))
  {
    osDelay(1000);
  }

  radio.setChannel(RADIO_CHANNEL);
  radio.setPayloadLen(RADIO_PAYLOAD_LEN);   // before openWritingPipe(), which sizes pipe 0 from it

  /* Once is enough. init() leaves the radio in TX standby and nothing here
     calls startListening(), which is what would close pipe 0 and stop
     write() hearing the auto-ack. */
  radio.openWritingPipe(radioTxAddress);

  for(;;)
  {
    // write() zero-pads to the payload width, so the NUL goes out too.
    int msgLen = snprintf(msg, sizeof(msg), "stm cnt: %lu", (unsigned long)count);

    radio.write((const uint8_t*)msg, (uint8_t)msgLen);
    count++;

    osDelay(RADIO_TX_PERIOD_MS);
  }
}

static uint16_t bytesToHex(const uint8_t* src, uint8_t srcLen, char* dst)
{
  static const char digits[] = "0123456789ABCDEF";
  uint16_t written = 0;

  for(uint8_t i = 0; i < srcLen; i++)
  {
    dst[written++] = digits[(src[i] >> 4) & 0x0F];
    dst[written++] = digits[ src[i]       & 0x0F];
  }

  return written;   // not NUL terminated - callers pass the length explicitly
}

/* EXTI9_5 fires for the nRF24L01 nINT line (PF5, falling edge). CubeMX
   generates the vector and enables it at priority 7, which is numerically
   at or below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, so the
   FromISR call inside onIrq() is legal.

   EXTI4 fires for the motorB tachometer (PF4, both edges - the rising
   one re-arms the input, see Tachometer.hpp). That pin is
   not in the .ioc, so its vector is hand written in stm32f4xx_it.c and
   Tachometer::init() sets the same priority 7, for the same reason.

   extern "C" is mandatory: without it this compiles to a mangled symbol,
   HAL's __weak definition stays live, and the callback silently never
   fires. See the worked example at the bottom of app_main.cpp.

   Both owners are called. Each onIrq() returns immediately unless its own
   object has been initialised, and RADIO_SERLINK makes sure only one ever
   is. The test driver's onIrq() also does nothing in polled mode, where
   nINT is masked off anyway. */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == nRF24L01_nINT_Pin)
  {
    radio.onIrq();
    radio1.onIrq();
  }
  else if(GPIO_Pin == TACHOB_PIN)
  {
    /* Reads TIM5->CNT and queues it. Nothing else - the arithmetic is
       controlBTask's job. */
    tachoB.onEdge();
  }
}

//--------------------------------------------------------------
// Stack overflow - enabled by configCHECK_FOR_STACK_OVERFLOW (method 2),
// set in the .ioc. CubeMX also generates an empty stub of this hook in
// freertos.c's USER CODE 4 block; that stub is removed, and must stay
// removed or the link fails on a duplicate definition.
//
// Called by the kernel from the context switch (PendSV), with the
// offending task's stack already corrupt, so nothing here may use the
// RTOS, the HAL or much stack. It records which task it was, turns LD3
// (red, PB14) on solid and stops.
//
// Solid, not flashing, to tell it apart from Error_Handler() in main.c,
// which flashes the same LED. In the debugger, stackOverflowTaskName
// names the task; raise that task's .stack_size and check the others
// with the DBG00 stack query.
//
// extern "C" for the same reason as HAL_GPIO_EXTI_Callback: FreeRTOS
// calls it by its C name.
volatile char stackOverflowTaskName[configMAX_TASK_NAME_LEN];

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
  (void)xTask;

  taskDISABLE_INTERRUPTS();

  /* The name lives in the TCB, not on the stack, so it is still intact.
     Copied by hand: this is no place to call into the C library. */
  for(uint8_t i = 0U; i < configMAX_TASK_NAME_LEN; i++)
  {
    stackOverflowTaskName[i] = pcTaskName[i];
    if(pcTaskName[i] == '\0')
    {
      break;
    }
  }

  /* By register, like Error_Handler(). MX_GPIO_Init() has configured PB14
     as an output long before any task runs. */
  LD3_GPIO_Port->BSRR = LD3_Pin;

  /* And name the task on the serial console, so no debugger is needed.
     Straight to the USART2 registers, polling: with interrupts off and
     the scheduler stuck, neither the uart2 driver nor HAL_UART can run.
     USART2 is already configured by MX_USART2_UART_Init(). Any frame the
     driver was part way through sending is cut short - hence the leading
     CR LF, to start on a clean line. */
  static const char prefix[] = "\r\nSTACK OVERFLOW: ";
  for(const char* p = prefix; *p != '\0'; p++)
  {
    while((USART2->SR & USART_SR_TXE) == 0U) { }
    USART2->DR = (uint8_t)*p;
  }
  for(uint8_t i = 0U; (i < configMAX_TASK_NAME_LEN) && (stackOverflowTaskName[i] != '\0'); i++)
  {
    while((USART2->SR & USART_SR_TXE) == 0U) { }
    USART2->DR = (uint8_t)stackOverflowTaskName[i];
  }
  for(const char* p = "\r\n"; *p != '\0'; p++)
  {
    while((USART2->SR & USART_SR_TXE) == 0U) { }
    USART2->DR = (uint8_t)*p;
  }

  for(;;)
  {
  }
}

//--------------------------------------------------------------
// DBG00 socket - reads, answered on the ack:
//
//   DBG00T349001R    ->  DBG00A349002OK              link check
//   DBG00T349003S00  ->  DBG00A349016reader0Task:0412  task 00: name and
//                                                  its stack's all-time
//                                                  lowest free space, in
//                                                  bytes (4 digits)
//   DBG00T349003S99  ->  DBG00A349003END           index past the last task
//   DBG00T349002SL   ->  DBG00A349015mqttRxTask:0096   the task with the
//                                                  least free, same format
//   DBG00T349001M    ->  DBG00A349021C.0012.0340.0000.0000
//                        SerLink2's MQTT link (mqtt2): C connected / D not,
//                        then frames received, frames sent, sends dropped,
//                        and messages on a topic other than the down one.
//                        4 digits each, clamped at 9999. Ask over uart2 -
//                        it is the way to see why MQTT is silent.
//
// Step S00, S01, ... until END to list every task, the kernel's own
// (IDLE, Tmr Svc, tcpip_thread, ...) included. Tasks are numbered in
// creation order, so an index names the same task on every call.
//
// "Lowest free" is uxTaskGetStackHighWaterMark(): the least the stack has
// ever had spare since the task started - a record, not a snapshot. Run
// the board through everything (MQTT, radio, closed loop, serial traffic)
// before trusting it. Under ~100 bytes is too tight: an interrupt landing
// in a task that has used the FPU stacks 100+ bytes of FPU context on top.

// Bigger than the number of tasks this build creates (about 23). If it is
// ever exceeded, uxTaskGetSystemState() returns 0 and the query answers
// ERR rather than a wrong list.
#define DEBUG_MAX_TASKS 32U

#define DEBUG_STACK_FIELD_WIDTH 4U   // bytes; the biggest stack is 2048

// File scope, not on reader0Task's stack - at ~40 bytes a task it would
// take most of it. Shared by reader0Task and reader1Task (both run this
// handler), which is why stackQuery() fills and reads it with the
// scheduler suspended.
static TaskStatus_t debugTaskStatus[DEBUG_MAX_TASKS];

// Writes "<name>:<dddd>" for one task into data, returns its length.
// Name is at most configMAX_TASK_NAME_LEN - 1 = 15 characters, so this
// is at most 20, well inside Frame::MAX_DATALEN.
static uint16_t formatTaskStack(const TaskStatus_t* status, char* data)
{
  uint16_t len = 0U;

  for(const char* p = status->pcTaskName;
      (*p != '\0') && (len < (configMAX_TASK_NAME_LEN - 1U)); p++)
  {
    data[len++] = *p;
  }
  data[len++] = ':';

  /* usStackHighWaterMark is in StackType_t words. */
  uint32_t freeBytes = (uint32_t)status->usStackHighWaterMark * sizeof(StackType_t);
  if(freeBytes > 9999U)
  {
    freeBytes = 9999U;
  }
  writeUintField(freeBytes, DEBUG_STACK_FIELD_WIDTH, &data[len]);

  return (uint16_t)(len + DEBUG_STACK_FIELD_WIDTH);
}

static uint16_t copyReply(const char* text, char* data)
{
  uint16_t len = (uint16_t)strlen(text);
  memcpy(data, text, len);
  return len;
}

// command is "L" (least free) or two digits (task index).
static uint16_t stackQuery(const char* command, uint16_t commandLen, char* data)
{
  uint32_t index = 0U;
  bool least = (commandLen == 1U) && (command[0] == 'L');

  if(!least && !((commandLen == 2U) && readUintField(command, 2U, &index)))
  {
    return copyReply("ERR", data);
  }

  uint16_t len;

  /* Suspended, not in a critical section: uxTaskGetSystemState() scans
     every task's stack for its high water mark, which is too long to run
     with interrupts off - the tacho edges and uart2 must keep coming. */
  vTaskSuspendAll();
  {
    UBaseType_t count = uxTaskGetSystemState(debugTaskStatus, DEBUG_MAX_TASKS, nullptr);

    if(count == 0U)
    {
      len = copyReply("ERR", data);
    }
    else if(least)
    {
      UBaseType_t lowest = 0U;
      for(UBaseType_t i = 1U; i < count; i++)
      {
        if(debugTaskStatus[i].usStackHighWaterMark < debugTaskStatus[lowest].usStackHighWaterMark)
        {
          lowest = i;
        }
      }
      len = formatTaskStack(&debugTaskStatus[lowest], data);
    }
    else
    {
      /* The kernel lists tasks by state, so the array order changes as
         they block and wake. Order by creation number instead, which is
         fixed: select the index-th smallest xTaskNumber. Selection, not a
         sort, because only one entry is wanted and count is small. */
      const TaskStatus_t* found = nullptr;
      uint32_t previous = 0U;
      bool first = true;

      for(uint32_t n = 0U; n <= index; n++)
      {
        found = nullptr;
        for(UBaseType_t i = 0U; i < count; i++)
        {
          uint32_t number = debugTaskStatus[i].xTaskNumber;
          if((first || (number > previous)) &&
             ((found == nullptr) || (number < found->xTaskNumber)))
          {
            found = &debugTaskStatus[i];
          }
        }
        if(found == nullptr)
        {
          break;   // fewer tasks than index + 1
        }
        previous = found->xTaskNumber;
        first = false;
      }

      len = (found != nullptr) ? formatTaskStack(found, data)
                               : copyReply("END", data);
    }
  }
  (void)xTaskResumeAll();

  return len;
}

bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if(rxFrame.dataLen < 1)
  {
    return false;
  }

  switch(rxFrame.data[0])
  {
    case 'R':
      *dataLen = copyReply("OK", data);
      return true;

    case 'S':
      *dataLen = stackQuery(&rxFrame.data[1], (uint16_t)(rxFrame.dataLen - 1), data);
      return true;

    case 'M':   // mqtt2 link state - see the DBG00 notes above
    {
      /* Getters only; each counter is one aligned word, so reading them
         from reader0Task alongside mqtt2Task is safe - at worst one is a
         message behind another. */
      const uint32_t counts[] =
      {
        mqtt2.getRxFrames(), mqtt2.getTxFrames(),
        mqtt2.getTxDropped(), mqtt2.getOtherTopic()
      };

      data[0] = mqtt2.isConnected() ? 'C' : 'D';
      uint16_t len = 1U;
      for(uint32_t count : counts)
      {
        data[len++] = '.';
        writeUintField((count > 9999U) ? 9999U : count, 4U, &data[len]);
        len += 4U;
      }
      *dataLen = len;
      return true;
    }

    default:
      return false; // not handled
  }
}