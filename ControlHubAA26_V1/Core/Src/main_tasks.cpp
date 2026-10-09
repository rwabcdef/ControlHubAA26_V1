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

 # event echo - sent by the board, unasked: MQTT and radio events, see logEvent()
 DBG00U123008BTN01 1P        # remote button 1 pressed
 DBG00U124012CTRL0 BR0120    # the PC set the target speed

 # ping - any socket, any link ('S' = system frame, see Socket.hpp). Answered by
 # SerLink itself; never reaches the socket's handlers.
 MOTORS045004PING    # -> MOTORA045008PINGBACK  socket exists on this link
 XXXXXS045004PING    # -> XXXXXA045900          no such socket: plain ACK_OK
 LIFT0S045004PING    # -> LIFT0A045008PINGBACK over MQTT, but LIFT0A045900
                     #    on serial - LIFT0 is on MQTT only

 # Modes - hubApp (HubApp.hpp) keeps the hub Idle, in Control (motorB held at a
 # speed until stopped - from the PC or the remote hub) or in Lift (a liftB
 # move - from the PC only). Every command that starts, stops or steers the
 # motor goes through it: a start is ignored unless Idle, and so is a
 # direction change. CTRL0T529003BGO reads the mode.

# Motor socket - the TC78H611FNG dual H-bridge on TIM8, channel B (IN1B/IN2B on J10 pins 10 and 8)
# A bring-up / debug path: P and D drive the bridge directly, Idle only.
MOTORT516005BP050   # open loop: 50% duty (Idle only)
MOTORT523003BDF     # open loop: direction forward - starts it (Idle only)
MOTORT523003BDD     # open loop: stop (direction disabled)
MOTORT516006BS0300  # = CTRL0 BR0300 + BS: a Control run at 300 RPM
MOTORT529003BGS     # read the target RPM back, 4 digits

# Control socket - controllerB and Control runs. On serial AND on MQTT
# (SerLink2) - two sockets sharing the same handlers, both posting to hubApp.
#
# Safety limits (see Unresponsive tachometer in Controller.hpp): the duty is
# capped at the max duty (BM, boot CONTROLB_OUTPUT_MAX_PERCENT 50%), and if
# the tacho reads 0 for CONTROLB_TACHO_TIMEOUT_S (10 s) with a speed demanded
# and the duty at or above CONTROLB_TACHO_CHECK_MIN_PERCENT (20%), the
# controller stops the motor (0%, direction D), latches a tacho fault and the
# hub goes Idle. The next start clears it.

# a Control run
CTRL0T523003BDF     # selected direction forward (Idle only)
CTRL0T516006BR0120  # target 120 RPM - does not start anything
CTRL0T523002BS      # start
CTRL0T523002BX      # stop (a lift move too)

# sets - plain ACK_OK; a malformed set is dropped silently, so read it back
CTRL0T516006BR0120  # target speed 120 RPM - live in Control, refused in Lift
CTRL0T523003BDR     # selected direction reverse (F forward) - Idle only
CTRL0T516005BM050   # max duty 50% (CONTROLB_TACHO_CHECK_MIN_PERCENT..100) - live
CTRL0T516008BI002000  # integral gain = 0.002 (6 digits, millionths: 000000..999999).
                      # Next pass, bumpless; lasts until reset (boot value is
                      # CONTROLB_INTEGRAL_GAIN)

# reads - answered on the ack
CTRL0T529003BGR     # target RPM                -> CTRL0A5290040120
CTRL0T529003BGD     # selected direction, F/R   -> CTRL0A529001R
CTRL0T529003BGM     # max duty                  -> CTRL0A529003050
CTRL0T529003BGO     # mode I/C/L + source P/R/- -> CTRL0A529002CP
CTRL0T529003BGF     # tacho fault, 1/0          -> CTRL0A5290011
CTRL0T529003BGI     # integral gain, millionths -> CTRL0A529006002000
CTRL0T529003BGA     # all: gain.target.measured RPM
                    #                           -> CTRL0A529016002000.0150.0148

# Sent by the board on MQTT only, every STATUS_PUBLISH_PERIOD_MS while running
# and once when the run ends ('U', so no ack):
CTRL0U001015CF030.0350.1234  # Control, forward, duty 30%, 350 RPM, 1234 mA

# Lift socket - liftB, over MQTT only (SerLink2: publish to hub/aa26/serlink/down).
# Direction and distance only - distance in tachoB edges (2 per rev). The speed
# is the target speed: set it first with CTRL0 BR<dddd> (a BR during the move
# is refused); it boots at CONTROLB_BOOT_RPM (100). A start with the target at 0,
# or while not Idle, is ignored silently, so nothing moves.
CTRL0T516006BR0020  # speed for the moves that follow: 20 RPM
LIFT0U645006BSF234  # start liftB forward for 234 edges (1..6 digits)
LIFT0U645006BSR234  # start liftB reverse for 234 edges
LIFT0U645006BG2000  # liftB down to the ground sensor (PB9), 2000 edges max
LIFT0U645002BX      # stop liftB now (coasts) - a Control run too
LIFT0T645002BT      # liftB status -> LIFT0A645014M000120.000234
                    #   M moving / I idle / G idle on the ground sensor,
                    #   edges travelled, target
# Sent by the board when a move ends (arrived, stopped, ground, or tacho fault):
LIFT0U001015BI000234.000234   # liftB idle, 234 of 234 edges travelled
LIFT0U001015BG000180.002000   # liftB ground move found the ground after 180
LIFT0U001015BI000000.000234   # ended short - with CTRL0 BGF reading 1, the
                              # tacho timeout stopped it

# Remote hub - an Arduino UNO R4 over the radio (SerLink1), so nothing to
# type here. It sends BTN01 (1P start/stop, 2P direction), POT01 (P050 - the
# speed of a run it started) and HBT01 (H every 500 ms - a run it started stops
# if they cease for 2 s). The hub sends it LED01: A0/A1/AF0002020 (Idle /
# Control / Lift) and B0/B1 (forward / reverse selected).

# Adc socket - ADC1, ranks IN0/IN3/IN4/IN5 (PA0/PA3/PA4/PA5). Channel 1 is the motorB current sense.
ADC00T529002G1      # raw count, 4 digits
ADC00T529002V1      # millivolts, 4 digits
ADC00T529002GA      # all four channels, raw

# SD card socket - SDC00, over SerLink0 (uart2). See the SD card block
# below initTasks()'s globals for the wiring, and SdCard.hpp. One file
# open at a time, for reading or for writing.
#
# Status <S><P|A><M|U><R|W|C>.<rr>: card Present/Absent, volume Mounted/
# Unmounted, a file open for Reading / Writing or Closed, and the last
# FatFs result (FRESULT, 2 digits: 00 ok, 03 not ready, 04 no such file,
# 07 denied - wrong open mode, or card full, 08 already exists, 13 not
# FAT). The board sends it by itself when a card is inserted (once
# mounted, or not) or removed, and the read answers it on the ack:
SDC00T560001S              # -> SDC00A560007SPMC.00   present, mounted, no file
SDC00U001007SPMC.00        # sent by the board: card inserted and mounted
SDC00U002007SAUC.03        # sent by the board: card removed
SDC00T564001E              # diagnostics -> SDC00A564015E00000008.00041
                           #   HAL SD error code (hex HAL_SD_ERROR_* bits, of
                           #   the most recent transfer) . last mount in ms
                           #   See writeSdDiagnostics() for the common bits.

# Commands - plain ACK_OK, then the answer arrives as a frame of its own,
# because the SD work happens in sdCardTask, not on the link's task.
SDC00T561009OREAD.TXT      # open an existing file, read only. Mounts first
                           # if needed. -> O<rr>.<size, 10 digits>
                           #    SDC00U003014O00.0000001234
                           #    SDC00U004014O04.0000000000   (no such file)
SDC00T562007R0.0100        # read 100 bytes from offset 0 of the open file.
                           # R<offset 1..9 digits>.<length 1..4 digits>, at
                           # most SDCARD_READ_MAX (512) bytes. The data comes
                           # back as D frames, then R<rr>.<bytes read>:
                           #    SDC00U005064DHello world\0D\0A...
                           #    SDC00U006008R00.0100
                           # Printable ASCII as is, '\' as '\\', any other
                           # byte as '\' + 2 hex digits (\0D\0A is CR LF), so
                           # binary survives and text stays readable. Past the
                           # end reads short (fewer bytes, rr still 00).
SDC00T563001C              # close the file -> C<rr>, e.g. SDC00U007003C00
                           # For a write file this is what flushes it -
                           # always close before pulling the card.

# Writing a text file - open, lines, close. W<mode><path>, mode:
#   O overwrite - create it, or empty an existing one
#   A append    - create it, or add to the end of an existing one
#   N new       - create it; W08 (FR_EXIST) if it is already there
# -> W<rr>.<size>: the size it was opened at (append: the existing size).
SDC00T565009WALOG.TXT      # -> SDC00U008014W00.0000000034
SDC00T566012LHello world   # write a line: the text, then SDCARD_LINE_END
                           # (CR LF) -> L<rr>.<new file size>
                           #    SDC00U009014L00.0000000047
SDC00T566001L              # L alone writes a blank line
SDC00T567010PPart one     # P<text>: text only, no line end - for a line
SDC00T566011L, part two    # longer than one frame (63 characters), send P
                           # pieces, then L with the last piece. -> P<rr>.<size>
                           # Here the line is "Part one, part two".
SDC00T563001C              # close -> C00
#
# Each L/P is synced to the card before it is answered (SDCARD_SYNC_EACH_WRITE),
# so an answered line survives the card being pulled. The text is written
# exactly as sent, no escapes - so mind the length field (L + text). Send
# the next line once the last one is answered: with several in flight,
# a command that finds sdCardTask's queue full is answered
#   SDC00U...002QL             # Q + the command - dropped, send it again
# R on a write file, or L/P on a read file, is refused with rr 07.

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
#include "SdCard.hpp"
#include "Radio.hpp"
#include "MqttPubSub.hpp"
#include "SerLinkMqttAdapter.hpp"
#include "HubApp.hpp"
#include "lwip/netif.h"

//--------------------------------------------------------------
// Task stacks are 256 words (1 KB) except where noted. The exceptions:
//
// MQTT_TASK_STACK_SIZE - every task that calls into lwIP's MQTT client:
// mqtt2Task, writer2Task and reader2Task (the last two through
// SerLinkMqttAdapter::write()). MqttPubSub takes the tcpip
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
// Runs hubApp every CONTROLB_PERIOD_MS: the tachometer update, the mode
// logic, liftB and controllerB, and the status and LED frames.
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

/* Definitions for sdCardTask */
// Owns sdCard, so every FatFs call happens here. 2 KB: FatFs puts its
// 512 byte long file name buffer on the caller's stack, on top of its own
// frames and the SD disk layer's - see Threading in SdCard.hpp.
osThreadId_t sdCardTaskHandle;
const osThreadAttr_t sdCardTask_attributes = {
  .name = "sdCardTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//--------------------------------------------------------------
void startWriter0Task(void *argument);
void startReader0Task(void *argument);
void startSerLink0Task(void *argument);
void StartLedTask(void *argument);
void startWriter1Task(void *argument);
void startReader1Task(void *argument);
void startSerLink1Task(void *argument);
void startRadio1Task(void *argument);
void startMotorTask(void *argument);
void startAdcTask(void *argument);
void startControlBTask(void *argument);
void startMqtt2Task(void *argument);
void startWriter2Task(void *argument);
void startReader2Task(void *argument);
void startSerLink2Task(void *argument);
void startSdCardTask(void *argument);

bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// MOTOR socket handlers - the command set is documented above their
// implementations, below startMotorTask().
void motorSockReceiveHandler(const char* data, uint16_t dataLen);
bool motorSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// CTRL0 socket handlers - speed controller commands, documented above
// their implementations, below the MOTOR socket's.
void controlSockReceiveHandler(const char* data, uint16_t dataLen);
void controlMqttSockReceiveHandler(const char* data, uint16_t dataLen);
bool controlSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// LIFT0 socket handlers - lift commands, documented above their
// implementations, below the CTRL0 socket's.
void liftSockReceiveHandler(const char* data, uint16_t dataLen);
bool liftSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// ADC00 socket handler - reads only, documented above its implementation.
bool adcSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// SDC00 socket handlers - SD card commands, documented at the top of the
// file and above their implementations.
void sdSockReceiveHandler(const char* data, uint16_t dataLen);
bool sdSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// The remote hub's sockets on SerLink1 (radio) - BTN01, POT01, HBT01,
// documented above their implementations, below the LIFT0 socket's.
void buttonSockReceiveHandler(const char* data, uint16_t dataLen);
void potSockReceiveHandler(const char* data, uint16_t dataLen);
void heartbeatSockReceiveHandler(const char* data, uint16_t dataLen);

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
// The remote hub (Arduino UNO R4, sketches/remote_hub in the
// Arduino_uno_r4_gp repo) - all on transport1, the radio:
//
//   BTN01  remote -> hub, 'T'   button events: 1P start/stop, 2P direction
//   POT01  remote -> hub, 'U'   pot percent, P050; the speed of a remote run
//   HBT01  remote -> hub, 'U'   H, every 500 ms
//   LED01  hub -> remote, 'U'   the remote's LEDs - hubApp sends them
//
// Every frame from the remote is a sign of life to hubApp, which stops a
// run the remote started once none has come for REMOTE_HEARTBEAT_TIMEOUT_MS.
// The remote's frames carry 9 data characters at most (its UART_BUFF_LEN).
#define REMOTE_HEARTBEAT_TIMEOUT_MS 2000U   // 4 missed heartbeats
#define REMOTE_LED_REFRESH_MS       1000U   // the remote waits 3 s before
                                            // showing the link as lost
#define REMOTE_POT_RPM_MAX          300U    // the speed at pot 100%
#define REMOTE_LED_RUN_ID           'A'     // off Idle, on Control, flash Lift
#define REMOTE_LED_DIRECTION_ID     'B'     // on = reverse selected

SerLink::Socket* ledRadioSocket = nullptr;
SerLink::Socket* buttonRadioSocket = nullptr;
SerLink::Socket* potRadioSocket = nullptr;
SerLink::Socket* heartbeatRadioSocket = nullptr;

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

// Direction output for the current measurement hardware, so it knows
// which way motorB's current flows - see "Direction output" in
// TC78H611FNG.hpp. PA15 (JTDI, free because the .ioc debug mode is
// Serial Wire) is claimed in the .ioc as GPIO_Output MOTORB_DIR, so
// MX_GPIO_Init() sets it up low; the driver then re-applies it.
// MOTORB_DIRECTION_FORWARD_HIGH true drives it high for forward, low
// for reverse.
#define MOTORB_DIRECTION_PORT          MOTORB_DIR_GPIO_Port
#define MOTORB_DIRECTION_PIN           MOTORB_DIR_Pin
#define MOTORB_DIRECTION_FORWARD_HIGH  true

TC78H611FNG_Standby motorStandby(GPIOB, GPIO_PIN_8);

TC78H611FNG motorB(GPIOC, GPIO_PIN_6,   // IN1B, TIM8_CH1
                   GPIOC, GPIO_PIN_7,   // IN2B, TIM8_CH2
                   MOTORB_PWM_FREQ,
                   MOTORB_DIRECTION_PORT, MOTORB_DIRECTION_PIN,
                   MOTORB_DIRECTION_FORWARD_HIGH);

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

// The CTRL0 status frame (speed, duty, current - see the CTRL0 notes)
// goes out unsolicited while the motor runs, rather than being polled: the
// PC's dashboard just listens. Sent as 'U' (no ack) because a telemetry
// frame that went missing is better dropped than retried - the next one is
// only STATUS_PUBLISH_PERIOD_MS away, and waiting on an ack would stall
// the control loop.
#define STATUS_PUBLISH_PERIOD_MS 250

// hubApp publishes on a whole number of controlB passes rather than
// keeping a second timebase, so the periods have to divide.
static_assert((STATUS_PUBLISH_PERIOD_MS % CONTROLB_PERIOD_MS) == 0,
  "STATUS_PUBLISH_PERIOD_MS must be a whole number of controlB periods");
static_assert((REMOTE_LED_REFRESH_MS % CONTROLB_PERIOD_MS) == 0,
  "REMOTE_LED_REFRESH_MS must be a whole number of controlB periods");

//--------------------------------------------------------------
// motorB speed controller, run by controlBTask - see Controller.hpp.
//
// Idle until hubApp starts a Control run or a lift move enables it, and
// disabled again when the run ends. While it is enabled it owns motorB's
// duty cycle, and a percent written by anything else is overwritten on
// the next pass.
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
// driven until the timeout below stops it. It is the boot value: CTRL0
// BM<ddd> changes it at run time (Controller::setOutputMaxPercent()).
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

// The target speed hubApp boots with. Setting it does not enable the
// controller, so nothing moves at boot - it is the speed a Control run or
// a lift move runs at until CTRL0 BR<dddd> sets another.
#define CONTROLB_BOOT_RPM           100U

// motorB current: the PA3 sense voltage (adc1 channel 1, mV) times this
// gives mA, for the CTRL0 status frame.
// TODO: the real value, from the TC78H611FNG current sense / the board's
// sense resistor. 1 reports the raw millivolts as mA until then.
#define MOTORB_CURRENT_MA_PER_MV    1.0f

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
// liftB owns controllerB: hubApp.run() calls liftB.run(), which runs the
// controller. Lift runs it whether or not a move is in progress, so a
// Control run (no lift move) is driven through it too.
//
// liftB is direction and distance only - the speed is controllerB's,
// hubApp's target speed (CTRL0 BR<dddd>) as it was at the start: hubApp
// refuses BR during a move and does not re-apply the target. A start
// with a zero demand is refused. hubApp refuses direction changes while
// anything runs too, so a move cannot be stalled from outside. See Speed
// in Lift.hpp.
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

// Acquired on transport2 (SerLink2, MQTT) only.
SerLink::Socket* liftMqttSocket = nullptr;

// CTRL0 - the controller and Control runs. Two sockets, one per link,
// sharing the same handlers (see the CTRL0 section):
//
//   controlSocket      transport0 - SerLink0, serial (uart2)
//   controlMqttSocket  transport2 - SerLink2, MQTT (mqtt2)
//
// Both post to the same hubApp, so the serial console counts as the PC.
// The status frame goes out on controlMqttSocket only.
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

#define ADC1_MOTORB_CURRENT_CHANNEL 1U   // PA3, see the table above

// Acquired on transport0 (uart2), alongside the motor socket.
SerLink::Socket* adcSocket = nullptr;

//--------------------------------------------------------------
// The event echo: user generated and low rate frames on MQTT and the radio
// are repeated to the serial console, out of transport0's DBG00 socket, as
// a 'U' frame whose data is "<protocol> <data>":
//
//   DBG00U<rrr>008BTN01 1P      remote button 1 pressed
//   DBG00U<rrr>009POT01 P050    remote pot moved to 50% (on a change only)
//   DBG00U<rrr>012CTRL0 BR0120  CTRL0 set or command from the PC (MQTT)
//   DBG00U<rrr>013LIFT0 BSF234  LIFT0 command from the PC
//   DBG00U<rrr>021LIFT0 BI000234.000234  LIFT0 done frame, hub -> PC
//   DBG00U<rrr>008LED01 A1      LED01 to the remote, on a change only
//
// Not echoed: anything periodic or high rate - the CTRL0 status frame,
// reads (CTRL0 BG*, LIFT0 BT, DBG00), HBT01, the LED01 refresh - and the
// serial console's own commands.

// transport0's DBG00, acquired in initTasks().
SerLink::Socket* debugSocket = nullptr;

// Any task: sendData() only queues the frame for transport0, without
// blocking, and drops it if the queue is full.
static void logEvent(const char* protocol, const char* data, uint16_t dataLen)
{
  if(debugSocket == nullptr)
  {
    return;
  }

  char buf[SerLink::Frame::MAX_DATALEN];
  const uint16_t protoLen = (uint16_t)strlen(protocol);
  const uint16_t room = (uint16_t)(sizeof(buf) - protoLen - 1U);
  if(dataLen > room) { dataLen = room; }

  memcpy(buf, protocol, protoLen);
  buf[protoLen] = ' ';
  memcpy(&buf[protoLen + 1U], data, dataLen);

  (void)debugSocket->sendData(buf, (uint16_t)(protoLen + 1U + dataLen), false);
}

//--------------------------------------------------------------
// hubApp - the hub's mode (Idle / Control / Lift) and the arbiter of every
// command that starts, stops or steers motorB. See HubApp.hpp. Owned by
// controlBTask; the socket handlers only parse and post to it.
static void sendLiftDone();   // the LIFT0 done frame, in the LIFT0 section

static const HubAppConfig hubAppConfig =
{
  &controllerB,
  &liftB,
  []() -> uint16_t                                        // getCurrentMa
  {
    float ma = (float)adc1.getMillivolts(ADC1_MOTORB_CURRENT_CHANNEL) *
               MOTORB_CURRENT_MA_PER_MV;
    return (ma > 65535.0f) ? 65535U : (uint16_t)(ma + 0.5f);
  },
  sendLiftDone,
  CONTROLB_BOOT_RPM,
  REMOTE_POT_RPM_MAX,
  CONTROLB_PERIOD_MS,
  STATUS_PUBLISH_PERIOD_MS,
  REMOTE_LED_REFRESH_MS,
  REMOTE_HEARTBEAT_TIMEOUT_MS,
  REMOTE_LED_RUN_ID,
  REMOTE_LED_DIRECTION_ID,
  logEvent
};

HubApp hubApp(hubAppConfig);

//--------------------------------------------------------------
// SD card - SDIO, 4 bit, through FatFs. See SdCard.hpp.
//
// Wired to CN8 (UM1974 Table 19, the SDMMC/I2S_A group), which carries
// the SDIO pins in a row. Card header pin -> CN8 pin:
//
//   1  DAT3 (~CS)   -> CN8 8    PC11  SDIO_D3
//   2  CMD  (MOSI)  -> CN8 12   PD2   SDIO_CMD
//   3  DAT0 (MISO)  -> CN8 2    PC8   SDIO_D0
//   4  SCK          -> CN8 10   PC12  SDIO_CK
//   7  DAT1         -> CN8 4    PC9   SDIO_D1
//   8  DAT2         -> CN8 6    PC10  SDIO_D2
//   9  CD           -> CN8 14   PG2   GPIO input (card detect)
//   5/11 GND        -> CN8 11 or 13
//   6/12 VCC 3.3 V  -> CN8 7    +3.3 V   - NOT CN8 9, which is +5 V
//
// The SPI names on the card header are the same contacts in SPI mode;
// SDIO uses them as DAT3/CMD/DAT0.
//
// CD: PG2 is CN8 14, next to the SDIO group, and otherwise unused. Like
// PB8 and PF4 it is not claimed in the .ioc - SdCard::init() configures
// it - so nothing stops a future CubeMX edit handing it to a peripheral;
// claim it there if this becomes permanent.
//
// SDCARD_DETECT_ACTIVE is the level CD reads with a card in. Most sockets
// switch CD to GND on insertion, hence GPIO_PIN_RESET, with SdCard::init()
// pulling the pin up. If the status reads A with a card in and P without,
// flip it to GPIO_PIN_SET. If CD is not wired at all, set
// SDCARD_DETECT_PORT to nullptr: the card then always reads present.
//
// The bus extras CubeMX does not do - the pull-ups, the SDIO interrupt,
// the 8 MHz clock - are in USER CODE blocks; see SdCard.hpp for where.
#define SDCARD_DETECT_PORT    GPIOG
#define SDCARD_DETECT_PIN     GPIO_PIN_2
#define SDCARD_DETECT_ACTIVE  GPIO_PIN_RESET

// sdCardTask wakes at least this often to poll the card detect pin (the
// debounce itself is SdCard::DETECT_SETTLE_MS).
#define SDCARD_POLL_MS        50U

// The most one SDC00 read command returns. The data goes out as D frames,
// up to 63 characters each, and an unprintable byte takes 3 - so 512
// bytes is 9 frames of text, or 25 of binary.
#define SDCARD_READ_MAX       512U

// The gap after each frame sdCardTask sends. A read reply is a burst of
// frames, but transport0's queue and writer0's are 5 deep each, and
// writer0 drops a frame silently when its queue is full - so pace the
// burst instead. A full 78 character frame takes ~7 ms at 115200 baud,
// so this leaves room for everyone else's frames too.
#define SDCARD_FRAME_GAP_MS   20U
#define SDCARD_SEND_TRIES     5U

// main.c's SDIO handle; fatfs.c's volume (SDFatFS) and drive path
// (SDPath, "0:/"), which MX_FATFS_Init() links to the SD driver before
// initTasks() runs.
extern SD_HandleTypeDef hsd;

SdCard sdCard(&hsd, &SDFatFS, SDPath,
              SDCARD_DETECT_PORT, SDCARD_DETECT_PIN, SDCARD_DETECT_ACTIVE);

// An SDC00 command, parsed by sdSockReceiveHandler() in serLink0Task and
// carried to sdCardTask, which owns sdCard - the same arrangement as
// hubApp's AppCmd. The path or text travels in the item, so it cannot be
// overwritten by the next command before it is used.
struct SdCmd
{
  enum op_t : uint8_t { open, openWrite, read, writeLine, writeText, close };

  op_t              op;
  SdCard::WriteMode mode;                          // openWrite
  uint32_t          offset;                        // read
  uint32_t          len;                           // read: bytes; write*: text length
  char              text[SerLink::Frame::MAX_DATALEN];  // open*: path, NUL
                                                   // terminated; write*: the text
};

// Lines can arrive faster than sdCardTask writes them (each line is
// synced - several card writes), so this is a few deep. Full, a command
// is not lost silently: the receive handler answers Q<cmd> instead, and
// the sender should resend it. Pacing a writer on each L/P reply never
// needs more than one slot.
#define SDCARD_CMD_QUEUE_LENGTH 4

// What L<text> ends each line with. CR LF, as the serial console uses;
// "\n" for Unix-style files.
#define SDCARD_LINE_END       "\r\n"

// Sync after every L/P write, so a line that has been answered is on the
// card even if it is pulled out (or the power goes) before C. Costs a
// FAT and a directory sector write per line; 0 leaves it all to C.
#define SDCARD_SYNC_EACH_WRITE 1
StaticQueue_t sdCmdStaticQueue;
uint8_t sdCmdQueueStorageArea[SDCARD_CMD_QUEUE_LENGTH * sizeof(SdCmd)];
QueueHandle_t sdCmdQueue;

// Acquired on transport0 (uart2).
SerLink::Socket* sdSocket = nullptr;

//--------------------------------------------------------------
// nRF24L01 radio (radio1, above) on SPI5. The driver owns CE (PF6) and CSN
// (PF10); spi5 itself handles only SCK/MISO/MOSI. Channel 76, 1 Mbps and
// 32 byte payloads are fixed in Components/Radio and must match the remote
// hub - the Arduino RF24 library's own defaults. A mismatch on any of them
// means silence, not corruption.
#define RADIO1_DETECT_TIMEOUT_MS 500 // radio1Task: how long the boot-time detect
                                     // check waits for the module (Tpor is 100 ms)

//--------------------------------------------------------------
// MQTT over lwIP - the broker SerLink2 connects to.
#define MQTT_BROKER_IP          "192.168.0.196"
#define MQTT_BROKER_PORT        1883

extern struct netif gnetif;   // lwip.c

//--------------------------------------------------------------
// SerLink2 link layer - SerLink over MQTT, for the PC.
//
// mqtt2Client is the hub's connection to the broker, reserved for SerLink
// and read by nothing but mqtt2. MqttPubSub has one rxQueue for all of its
// subscriptions and receive() does not filter by topic, so if another
// connection is ever wanted (for other topics) it must be a separate
// MqttPubSub instance - two tasks receiving on one client would steal each
// other's messages. See the ownership note in SerLinkMqttAdapter.hpp.
//
// MQTT2_CLIENT_ID must be unique on the broker: a second connection with
// the same id kicks the first one off, and the two would sit there
// disconnecting each other in a loop.
//
// The topics are a pair, not one topic. A broker delivers to every
// subscriber including the publisher, so a single topic would feed every
// frame and every ack straight back into our own Reader.
//
//   down   PC -> controlHub    (subscribed to here)
//   up     controlHub -> PC    (published here)
//
// The payload is the serialised frame exactly as it would appear on
// uart2, so the strings at the top of this file can be pasted straight
// into mosquitto_pub, and mosquitto_sub reads the link like a terminal.
#define MQTT2_CLIENT_ID    "stm32-serlink"          // unique on the broker
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
// Sockets: DBG00 (link check, stack query), CTRL0 (controller and
// Control runs) and LIFT0, DBG00 and CTRL0 sharing their handlers with
// SerLink0 - a socket belongs to one transport, a handler does not. So
// CTRL0 over MQTT and CTRL0 over uart2 post to the same hubApp.
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

  /* Only validates the config - the controller stays disabled until
     hubApp starts a run. Before controlBTask exists, like tachoB. */
  controllerB.init();

  /* Only validates - the lift is idle until a start command. hubApp, which
     owns both, is initialised at the end of initTasks(), once the sockets
     it sends on exist - still before the scheduler starts, so before
     controlBTask can run it or any handler can post to it. */
  liftB.init();

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

     transport0 holds five of the SERLINK_CONFIG__MAX_SOCKETS slots -
     MOTOR, CTRL0, DBG00, ADC00 and SDC00, all here.
     An acquire past the limit returns a silent nullptr, which is why
     every socket pointer is checked before use. */
  motorSocket = transport0.acquireSocket("MOTOR", motorSockReceiveHandler,
    motorSockInstantHandler);

  /* Same split as MOTOR: sets in serLink0Task, reads on the ack. */
  controlSocket = transport0.acquireSocket("CTRL0", controlSockReceiveHandler,
    controlSockInstantHandler);

  /* Reads on the ack, and the event echo's way out (logEvent()). */
  debugSocket = transport0.acquireSocket("DBG00", nullptr, debugSockInstantHandler);

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

  /* SD card. init() only sets up the card detect pin - the card itself
     cannot be touched until the scheduler runs (see SdCard.hpp), so
     sdCardTask mounts it on its first polls. The command queue comes
     before the socket that posts to it, and both before the task that
     drains it. Commands in serLink0Task, the status read on the ack. */
  sdCard.init();
  sdCmdQueue = xQueueCreateStatic(SDCARD_CMD_QUEUE_LENGTH, sizeof(SdCmd),
    sdCmdQueueStorageArea, &sdCmdStaticQueue);
  sdSocket = transport0.acquireSocket("SDC00", sdSockReceiveHandler,
    sdSockInstantHandler);

  sdCardTaskHandle = osThreadNew(startSdCardTask, NULL, &sdCardTask_attributes);

  writer0TaskHandle = osThreadNew(startWriter0Task, NULL, &writer0Task_attributes);

  reader0TaskHandle = osThreadNew(startReader0Task, NULL, &reader0Task_attributes);

  serLink0TaskHandle = osThreadNew(startSerLink0Task, NULL, &serLink0Task_attributes);

   /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

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
  /* The same handlers as uart2's CTRL0, but the sets and commands are
     echoed to the serial console first. */
  controlMqttSocket = transport2.acquireSocket("CTRL0", controlMqttSockReceiveHandler,
    controlSockInstantHandler);

  /* Commands in serLink2Task, posted on to hubApp; the status read on the
     ack, from reader2Task. */
  liftMqttSocket = transport2.acquireSocket("LIFT0", liftSockReceiveHandler,
    liftSockInstantHandler);

  writer2.init([](char* buffer) -> uint8_t { return mqtt2.write(buffer); });
  reader2.init(mqtt2.rxDataQueue, &writer2, transport2.queue);
  reader2.setAckWriteFunc([](char* buffer) -> uint8_t { return mqtt2.write(buffer); });

  writer2TaskHandle = osThreadNew(startWriter2Task, NULL, &writer2Task_attributes);
  reader2TaskHandle = osThreadNew(startReader2Task, NULL, &reader2Task_attributes);
  serLink2TaskHandle = osThreadNew(startSerLink2Task, NULL, &serLink2Task_attributes);

  /* SerLink1, over radio1, to the remote hub. radio1Task is the nRF24L01's
     only owner - the driver is not thread-safe.

     The remote's sockets: its button, pot and heartbeat frames are parsed
     in serLink1Task and passed on to hubApp; LED01 is send-only, from
     hubApp in controlBTask. DBG00 is acquired later, from serLink1Task. */
  transport1Queue = xQueueCreateStatic(TRANSPORT1_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport1QueueStorageArea, &transport1StaticQueue);
  transport1.init(transport1Queue);

  ledRadioSocket = transport1.acquireSocket("LED01");
  buttonRadioSocket = transport1.acquireSocket("BTN01", buttonSockReceiveHandler);
  potRadioSocket = transport1.acquireSocket("POT01", potSockReceiveHandler);
  heartbeatRadioSocket = transport1.acquireSocket("HBT01", heartbeatSockReceiveHandler);

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

  /* Last, once every socket it sends on exists. Creates the queue the
     socket handlers post to - before the scheduler starts, so before any
     of them can run. Status on MQTT, LEDs on the radio. */
  (void)hubApp.init(controlMqttSocket, ledRadioSocket);
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
  for(;;)
  {
    transport0.run();
  }
  /* USER CODE END startSerLink0Task */
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

// Owns the nRF24L01: all SPI to it happens here.
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
//                          - open loop. Idle only
//     MOTORT516006AS0300   speed = 300 RPM (always 4 digits, zero padded)
//                          - the same as CTRL0 BR0300 then BS: hubApp's
//                          target speed, and a Control run from the PC
//     MOTORT523003ADF      direction = forward    - Idle only
//     MOTORT523003ADR      direction = reverse    - Idle only
//     MOTORT523003ADD      direction = disabled   - Idle only
//
//   P and D drive the bridge directly, under hubApp's nose - a bring-up
//   and debug path, so they are refused unless hubApp is Idle. hubApp
//   does not see an open loop run they start: stop it with ADD (or
//   AP000), not CTRL0 BX.
//
//   Reads. Handled by motorSockInstantHandler(), which piggybacks the
//   answer onto the ack instead of sending a frame of its own:
//
//     MOTORT529003AGP  ->  MOTORA529003030    percent,   3 digits
//     MOTORT529003AGF  ->  MOTORA5290041000   frequency, 4 digits (Hz)
//     MOTORT529003AGD  ->  MOTORA529001F      direction, one of F/R/D
//     MOTORT529003AGS  ->  MOTORA5290040300   required speed, 4 digits (RPM)
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

  /* The mode check for P and D. Read here, in serLink0Task, while hubApp
     changes it in controlBTask - so a start landing in the same moment can
     slip past. Acceptable on a debug path; everything that matters goes
     through hubApp's queue. */
  const bool idle = (hubApp.getMode() == HubApp::mode::idle);

  switch(data[1])
  {
    case 'P':   // <sel>P<ddd> - set percent, Idle only
    {
      uint32_t percent;

      if(idle &&
         (dataLen == MOTOR_CMD_SET_PERCENT_LEN) &&
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

    case 'S':   // <sel>S<dddd> - set speed (RPM): CTRL0 BR<dddd> + BS
    {
      uint32_t rpm;

      if((dataLen == MOTOR_CMD_SET_SPEED_LEN) &&
         readUintField(&data[2], MOTOR_RPM_FIELD_WIDTH, &rpm))
      {
        /* Target first, so the start already runs at it. Four digits
           cannot exceed uint16_t. The start is ignored unless Idle, and
           the target is refused during a lift move. */
        (void)hubApp.setTargetRpm((uint16_t)rpm);

        AppCmd cmd = {};
        cmd.op = AppCmd::start;
        (void)hubApp.post(cmd);
      }
      break;
    }

    case 'D':   // <sel>D<F|R|D> - set direction, Idle only
    {
      TC78H611FNG::direction direction;

      if(idle &&
         (dataLen == MOTOR_CMD_DIRECTION_LEN) &&
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

    case 'S':   // hubApp's target speed in RPM, 4 digits - same format the set takes
      writeUintField(hubApp.getTargetRpm(), MOTOR_RPM_FIELD_WIDTH, data);
      *dataLen = MOTOR_RPM_FIELD_WIDTH;
      return true;

    default:
      return false;
  }
}

//--------------------------------------------------------------
// CTRL0 socket - the controller and Control runs, over SerLink2 (MQTT)
// and SerLink0 (uart2). Both post to hubApp - see HubApp.hpp for the
// modes and what each command may do in them.
//
// Frame data is <controller><command><args>:
//
//   Sets and commands. Handled by controlSockReceiveHandler(), acked with
//   a plain ACK_OK - the ack says the frame arrived, not that it was
//   accepted; read back to confirm:
//
//     CTRL0T516006BR0120   target speed = 120 RPM (always 4 digits, zero
//                          padded, 0000 allowed). Does NOT start anything:
//                          it is the speed of the next Control run or lift
//                          move, and live in a Control run in progress (bar
//                          a remote run, whose speed is the remote's pot).
//                          Ignored during a lift move - read BGR to check
//     CTRL0T523003BDF      selected direction = forward - Idle only
//     CTRL0T523003BDR      selected direction = reverse - Idle only
//     CTRL0T516005BM050    max duty = 50% (always 3 digits). From
//                          CONTROLB_TACHO_CHECK_MIN_PERCENT to 100, else
//                          ignored; applies at once, running or not.
//                          CONTROLB_OUTPUT_MAX_PERCENT is the boot value
//     CTRL0T516008BI002000 integral gain = 0.002 - six digits, in
//                          millionths (000000..999999, so 0..0.999999).
//                          Takes effect on the next pass, running or not;
//                          the output carries on from where it is. Lasts
//                          until reset - CONTROLB_INTEGRAL_GAIN is the
//                          boot value
//     CTRL0T523002BS       start a Control run (from the PC) in the
//                          selected direction at the target speed. Idle only
//     CTRL0T523002BX       stop - a Control run or a lift move, whoever
//                          started it
//
//   Reads. Handled by controlSockInstantHandler(), answered on the ack:
//
//     CTRL0T529003BGR  ->  CTRL0A5290040120   target speed, 4 digits
//     CTRL0T529003BGD  ->  CTRL0A529001F      selected direction, F or R
//     CTRL0T529003BGM  ->  CTRL0A529003050    max duty, 3 digits
//     CTRL0T529003BGO  ->  CTRL0A529002CP     mode I/C/L, then who started
//                          the run: P PC, R remote, - Idle
//     CTRL0T529003BGF  ->  CTRL0A5290011      tacho fault, 1 or 0 - 1 once
//                          the controller has stopped the motor because
//                          tachoB read zero for CONTROLB_TACHO_TIMEOUT_S
//                          with the duty at or above
//                          CONTROLB_TACHO_CHECK_MIN_PERCENT.
//                          Cleared by the next start
//     CTRL0T529003BGI  ->  CTRL0A529006002000  integral gain, millionths -
//                          same format the set takes
//     CTRL0T529003BGA  ->  CTRL0A529016002000.0150.0148
//                          all at once: <gain>.<target>.<measured> -
//                          integral gain (6 digits, millionths), target
//                          RPM (4) and measured RPM (4, clamped at 9999).
//                          One read, so the three are from the same moment
//
//   Status. Sent unsolicited by hubApp on MQTT only, as 'U' (no ack
//   expected), every STATUS_PUBLISH_PERIOD_MS while a Control run or a
//   lift move is in progress, and once more when it ends:
//
//     CTRL0U001015CF030.0350.1234
//
//     <I|C|L>   mode: Idle, Control, Lift
//     <F|R>     direction - the run's, or the selected one while Idle
//     <ddd>     motorB duty, percent
//     <dddd>    tachoB speed, RPM, clamped at 9999
//     <dddd>    motorB current, mA (MOTORB_CURRENT_MA_PER_MV), clamped
//
// <controller> is resolved by controllerForSelector(), so it follows the
// MOTOR socket's selectors: only controllerB exists, and 'A' reaches it
// too until channel A is built.

#define CONTROL_CMD_MIN_LEN        2U   // <ctl><cmd>
#define CONTROL_CMD_SET_RPM_LEN    6U   // <ctl>R<dddd>
#define CONTROL_CMD_DIRECTION_LEN  3U   // <ctl>D<F|R>
#define CONTROL_CMD_SET_GAIN_LEN   8U   // <ctl>I<dddddd>
#define CONTROL_CMD_SET_DUTY_LEN   5U   // <ctl>M<ddd>
#define CONTROL_CMD_RUN_LEN        2U   // <ctl>S and <ctl>X
#define CONTROL_CMD_GET_LEN        3U   // <ctl>G<R|D|M|O|F|I|A>
#define CONTROL_RPM_FIELD_WIDTH    4U
#define CONTROL_DUTY_FIELD_WIDTH   3U   // max duty, 0..100

// Integral gain on the wire: an integer number of millionths, so the
// socket never has to parse or print a float. 1e-6 is far finer than any
// useful step (the boot value is 0.002 = 002000).
#define CONTROL_GAIN_FIELD_WIDTH   6U
#define CONTROL_GAIN_FIELD_MAX     999999U
#define CONTROL_GAIN_SCALE         1000000.0f

// The selected direction: F or R. There is no D here any more - a run
// is ended with BX, and hubApp idles the direction itself.
static bool controlDirectionFromChar(char value, ControllerDirection* direction)
{
  switch(value)
  {
    case 'F': *direction = ControllerDirection::forward; return true;
    case 'R': *direction = ControllerDirection::reverse; return true;
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

// The sets and commands. Runs in serLink0Task or serLink2Task, whichever
// link the frame came in on, after the ack has gone out - so a malformed
// command is dropped silently; read it back to confirm.
//
// The target speed, the gain and the max duty are written straight away
// (each is safe from any task), so a read straight after the set sees
// it. Anything that starts, stops or steers goes to hubApp's queue.
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

  AppCmd cmd = {};

  switch(data[1])
  {
    case 'R':   // <ctl>R<dddd> - target speed
    {
      uint32_t rpm;

      /* Four digits cannot exceed uint16_t. Refused during a lift move;
         the ack has gone, so the PC reads BGR back to tell. */
      if((dataLen == CONTROL_CMD_SET_RPM_LEN) &&
         readUintField(&data[2], CONTROL_RPM_FIELD_WIDTH, &rpm))
      {
        (void)hubApp.setTargetRpm((uint16_t)rpm);
      }
      return;
    }

    case 'D':   // <ctl>D<F|R> - selected direction
      if((dataLen != CONTROL_CMD_DIRECTION_LEN) ||
         !controlDirectionFromChar(data[2], &cmd.dir))
      {
        return;
      }
      cmd.op = AppCmd::direction;
      break;

    case 'M':   // <ctl>M<ddd> - max duty
    {
      uint32_t percent;

      /* setOutputMaxPercent() refuses anything out of range, which here
         can only be a value below the tacho check or above 100. */
      if((dataLen == CONTROL_CMD_SET_DUTY_LEN) &&
         readUintField(&data[2], CONTROL_DUTY_FIELD_WIDTH, &percent) &&
         (percent <= 100U))
      {
        (void)controller->setOutputMaxPercent((uint8_t)percent);
      }
      return;
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
      return;
    }

    case 'S':   // <ctl>S - start a Control run
      if(dataLen != CONTROL_CMD_RUN_LEN) { return; }
      cmd.op = AppCmd::start;
      break;

    case 'X':   // <ctl>X - stop
      if(dataLen != CONTROL_CMD_RUN_LEN) { return; }
      cmd.op = AppCmd::stop;
      break;

    case 'G':   // reads are answered on the ack, in controlSockInstantHandler()
    default:
      return;
  }

  /* Never blocks - a full queue drops the command; read the mode back. */
  (void)hubApp.post(cmd);
}

// CTRL0 over MQTT: echo, then the same as uart2's. Reads never get here -
// they are answered on the ack - so only sets and commands are echoed.
void controlMqttSockReceiveHandler(const char* data, uint16_t dataLen)
{
  logEvent("CTRL0", data, dataLen);
  controlSockReceiveHandler(data, dataLen);
}

// The reads. Runs in reader0Task or reader2Task, before the ack is sent.
// Returns false for the sets, leaving their ack a plain ACK_OK. Getters
// only, so no lock - same reasoning as motorSockInstantHandler().
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
    case 'R':   // target speed in RPM, 4 digits - same format the set takes
      writeUintField(hubApp.getTargetRpm(), CONTROL_RPM_FIELD_WIDTH, data);
      *dataLen = CONTROL_RPM_FIELD_WIDTH;
      return true;

    case 'D':   // selected direction, F or R - same letters the set takes
      data[0] = controlDirectionToChar(hubApp.getSelectedDirection());
      *dataLen = 1U;
      return true;

    case 'M':   // max duty, 3 digits - same format the set takes
      writeUintField(controller->getOutputMaxPercent(), CONTROL_DUTY_FIELD_WIDTH, data);
      *dataLen = CONTROL_DUTY_FIELD_WIDTH;
      return true;

    case 'O':   // mode and run source, e.g. CP
      data[0] = HubApp::modeToChar(hubApp.getMode());
      data[1] = HubApp::sourceToChar(hubApp.getSource());
      *dataLen = 2U;
      return true;

    case 'F':   // tacho fault, 1 or 0 - see Unresponsive tachometer in Controller.hpp
      data[0] = controller->isTachoFault() ? '1' : '0';
      *dataLen = 1U;
      return true;

    case 'I':   // integral gain in millionths, 6 digits - same format the set takes
      writeGainField(controller->getIntegralGain(), data);
      *dataLen = CONTROL_GAIN_FIELD_WIDTH;
      return true;

    case 'A':   // <gain>.<target>.<measured> - see the CTRL0 notes
    {
      /* writeUintField() keeps only the low digits, so clamp the measured
         speed to the field rather than report an unrelated number - a
         faulty tacho can read up to 65535. The target was set through a
         4 digit field, so it always fits. */
      uint32_t rpm = controller->getRpm();
      if(rpm > 9999U) { rpm = 9999U; }

      uint16_t len = 0U;
      writeGainField(controller->getIntegralGain(), &data[len]);
      len += CONTROL_GAIN_FIELD_WIDTH;
      data[len++] = '.';
      writeUintField(hubApp.getTargetRpm(), CONTROL_RPM_FIELD_WIDTH, &data[len]);
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
//   serLink2Task and only parses: the command is posted to hubApp, which
//   owns liftB. Send as 'U' or 'T' - a 'T' ack says the frame arrived,
//   not that the lift moved:
//
//     LIFT0U645006BSF234   start forward, 234 edges (1..6 digits)
//     LIFT0U645006BSR234   start reverse, 234 edges
//     LIFT0U645006BG2000   down to the ground: reverse until the ground
//                          sensor closes, or 2000 edges, whichever is
//                          first (1..6 digits). Already on the ground,
//                          it does not move, but still sends the done
//                          message (travelled 0).
//     LIFT0U645002BX       stop - the motor coasts. Stops a Control run
//                          too, like CTRL0 BX
//
//   A start (S or G) is ignored unless hubApp is Idle - stop the lift or
//   the Control run first. So is a distance of zero, and so is any start
//   while the target speed is zero (it boots at CONTROLB_BOOT_RPM): the
//   speed is set through CTRL0 (BR<dddd>), not here - see Speed in
//   Lift.hpp.
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
//   Done. Sent unsolicited by hubApp (sendLiftDone()) on the pass a move ends -
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
// writer2 then waits up to its ack timeout for the PC's ack (it does not
// resend); only worth it if the PC side sends acks, or every done message
// holds up the frames queued behind it for the whole timeout.
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

// The done frame: <lift><M|I|G><travelled>.<target>, on the LIFT0 socket
// the commands come in on. Called by hubApp, in controlBTask, when a move
// ends. Non-blocking: sendData() only queues the frame for writer2, and
// it is dropped if mqtt2 is not connected.
static void sendLiftDone()
{
  if(liftMqttSocket == nullptr)
  {
    return;
  }

  char data[1U + LIFT_STATUS_LEN];
  data[0] = 'B';   // liftB, the only lift
  writeLiftStatus(liftB, &data[1]);
  liftMqttSocket->sendData(data, (uint16_t)sizeof(data), LIFT_DONE_ACK);
  logEvent("LIFT0", data, (uint16_t)sizeof(data));
}

// Runs in serLink2Task. Parses and posts; liftB itself is only touched
// by hubApp, in controlBTask. Never blocks - a full queue drops the
// command.
void liftSockReceiveHandler(const char* data, uint16_t dataLen)
{
  /* The BT read is answered on the ack and never gets here. */
  logEvent("LIFT0", data, dataLen);

  if(dataLen < LIFT_CMD_MIN_LEN)
  {
    return;
  }

  /* Only liftB exists, and hubApp drives it - the selector is checked,
     not passed on. */
  if(liftForSelector(data[0]) == nullptr)
  {
    return;
  }

  AppCmd cmd = {};

  switch(data[1])
  {
    case 'S':   // <lift>S<F|R><d..d> - start
    {
      if((dataLen < LIFT_CMD_START_MIN_LEN) || (dataLen > LIFT_CMD_START_MAX_LEN))
      {
        return;
      }

      if(data[2] == 'F')      { cmd.liftDir = Lift::direction::forward; }
      else if(data[2] == 'R') { cmd.liftDir = Lift::direction::reverse; }
      else                    { return; }

      if(!readUintField(&data[3], (uint8_t)(dataLen - 3U), &cmd.distance))
      {
        return;
      }

      cmd.op = AppCmd::liftStart;
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

      cmd.op = AppCmd::liftGround;
      break;
    }

    case 'X':   // <lift>X - stop
      if(dataLen != LIFT_CMD_MIN_LEN)
      {
        return;
      }
      cmd.op = AppCmd::stop;
      break;

    case 'T':   // status - answered on the ack, in liftSockInstantHandler()
    default:
      return;
  }

  (void)hubApp.post(cmd);
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
// The remote hub's sockets - BTN01, POT01, HBT01 - over SerLink1 (radio).
// See the remote hub block above initTasks() for what each carries, and
// The remote in HubApp.hpp for what hubApp does with it.
//
// The frame data is the remote's own HardMod event format (Button.hpp,
// pot.hpp in the Arduino repo):
//
//   BTN01T<rrr>0021P    button 1 pressed - start/stop
//   BTN01T<rrr>0022P    button 2 pressed - toggle the direction (Idle only)
//                       (L long press, R<ddd> release: ignored)
//   POT01U<rrr>004P050  pot at 50% - the speed of a remote run
//   HBT01U<rrr>001H     heartbeat
//
// All three run in serLink1Task. Every frame, of any of them, is a sign of
// life; only button presses go through hubApp's queue - the pot and the
// heartbeat are single values, so they are just written.

#define REMOTE_BUTTON_LEN     2U   // <id><event>
#define REMOTE_POT_LEN        4U   // <id><ddd>
#define REMOTE_POT_WIDTH      3U

void buttonSockReceiveHandler(const char* data, uint16_t dataLen)
{
  hubApp.remoteAlive();
  logEvent("BTN01", data, dataLen);

  if((dataLen != REMOTE_BUTTON_LEN) || (data[1] != 'P'))
  {
    return;   // only presses mean anything
  }

  AppCmd cmd = {};
  switch(data[0])
  {
    case '1': cmd.op = AppCmd::remoteStartStop; break;
    case '2': cmd.op = AppCmd::remoteDirection; break;
    default:  return;
  }

  (void)hubApp.post(cmd);
}

void potSockReceiveHandler(const char* data, uint16_t dataLen)
{
  hubApp.remoteAlive();

  /* The remote re-sends the pot every second, so only a change is echoed.
     serLink1Task only, so a plain static will do. */
  static uint32_t lastLogged = 0xFFFFFFFFU;

  uint32_t percent;
  if((dataLen == REMOTE_POT_LEN) &&
     readUintField(&data[1], REMOTE_POT_WIDTH, &percent))
  {
    hubApp.setPotPercent((percent > 100U) ? 100U : (uint8_t)percent);

    if(percent != lastLogged)
    {
      lastLogged = percent;
      logEvent("POT01", data, dataLen);
    }
  }
}

void heartbeatSockReceiveHandler(const char* data, uint16_t dataLen)
{
  (void)data;
  (void)dataLen;
  hubApp.remoteAlive();
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
// controlB task - motorB, at CONTROLB_PERIOD_MS.
//
// tachoB.update() drains the timestamps the EXTI4 ISR has queued since
// the last pass and turns the complete revolutions among them into an
// RPM; it does not block, so the period is set here with vTaskDelayUntil
// rather than inside the driver. This is the one task allowed to call
// update(). The getters are safe from anywhere - see the threading note
// in Tachometer.hpp.
//
// hubApp.run() comes after the update, so the commands it applies, liftB
// and controllerB (which it runs) and the status frame all see this
// pass's reading. This is hubApp's owning task, and so liftB's and
// controllerB's - see Threading in HubApp.hpp.
void startControlBTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROLB_PERIOD_MS);

  for(;;)
  {
    tachoB.update();

    hubApp.run();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//--------------------------------------------------------------
// SDC00 socket - the SD card over SerLink0 (uart2). The command set and
// the answers are at the top of the file.
//
// The status read is answered on the ack (sdSockInstantHandler, in
// reader0Task) from SdCard's getters. Everything that touches the card -
// open, read, write, close - is posted to sdCardTask instead, because
// FatFs blocks on the SD DMA and only sdCardTask may call it (Threading
// in SdCard.hpp). Those answer with 'U' frames of their own once done; a
// malformed command is dropped silently after its ack, as elsewhere, and
// one that finds the queue full is answered Q<cmd>.

#define SDCARD_STATUS_LEN       7U    // S<P|A><M|U><R|W|C>.<rr>
#define SDCARD_RESULT_WIDTH     2U    // FRESULT, 0..19
#define SDCARD_SIZE_WIDTH       10U   // a uint32_t file size
#define SDCARD_OFFSET_MAX_WIDTH 9U    // so a typed offset cannot overflow
#define SDCARD_LEN_MAX_WIDTH    4U    // SDCARD_READ_MAX fits in 4 digits

static_assert(SDCARD_READ_MAX <= 9999U,
  "SDC00 read lengths are 4 digits on the wire");

// Read data lands here, not on sdCardTask's stack. Word aligned so the
// SD DMA can fill it directly - sd_diskio.c would cope with an unaligned
// buffer (ENABLE_SCRATCH_BUFFER), but only a sector at a time.
alignas(4) static uint8_t sdReadBuffer[SDCARD_READ_MAX];

// BSP_SD_Init() asks this before it touches the card. The generated one in
// bsp_driver_sd.c is __weak and always answers present; this replaces it.
// extern "C" or it would not: a mangled name leaves the weak one live,
// and the link still succeeds (see app_main.cpp).
extern "C" uint8_t BSP_SD_IsDetected(void)
{
  return sdCard.isCardPresent() ? SD_PRESENT : SD_NOT_PRESENT;
}

// S<P|A><M|U><R|W|C>.<rr> - the status read and the board's own status
// frames: file open for Reading, for Writing, or Closed. Getters only, so
// it is safe from reader0Task.
static void writeSdStatus(char* dst)
{
  dst[0] = 'S';
  dst[1] = sdCard.isCardPresent() ? 'P' : 'A';
  dst[2] = sdCard.isMounted()     ? 'M' : 'U';
  dst[3] = !sdCard.isFileOpen()   ? 'C' :
           (sdCard.isFileWritable() ? 'W' : 'R');
  dst[4] = '.';
  writeUintField((uint32_t)sdCard.getLastResult(), SDCARD_RESULT_WIDTH, &dst[5]);
}

// sdCardTask only. Sends one 'U' frame, then waits SDCARD_FRAME_GAP_MS -
// see there for why. sendData() fails only with transport0's queue full,
// so that is retried a few times before the frame is given up.
static void sdSend(char* data, uint16_t dataLen)
{
  if(sdSocket == nullptr)
  {
    return;
  }

  for(uint8_t tries = 0U; tries < SDCARD_SEND_TRIES; tries++)
  {
    if(sdSocket->sendData(data, dataLen, false))
    {
      break;
    }
    osDelay(SDCARD_FRAME_GAP_MS);
  }

  osDelay(SDCARD_FRAME_GAP_MS);
}

// One byte of read data, escaped for a frame: printable ASCII as is, '\'
// doubled, anything else as '\' and two hex digits. CR and LF have to go
// that way in any case - a raw newline would end the frame on the wire.
// Returns the characters written to dst, 1 to 3.
static uint8_t escapeSdByte(uint8_t value, char* dst)
{
  static const char hexDigits[] = "0123456789ABCDEF";

  if(value == (uint8_t)'\\')
  {
    dst[0] = '\\';
    dst[1] = '\\';
    return 2U;
  }

  if((value >= 0x20U) && (value <= 0x7EU))
  {
    dst[0] = (char)value;
    return 1U;
  }

  dst[0] = '\\';
  dst[1] = hexDigits[value >> 4];
  dst[2] = hexDigits[value & 0x0FU];
  return 3U;
}

// sdCardTask only. The read data as D frames, as full as they will go
// without splitting an escape across two.
static void sendSdData(const uint8_t* buffer, uint32_t len)
{
  char frameData[SerLink::Frame::MAX_DATALEN];
  uint16_t frameLen = 1U;
  frameData[0] = 'D';

  for(uint32_t i = 0U; i < len; i++)
  {
    char escaped[3];
    uint8_t escapedLen = escapeSdByte(buffer[i], escaped);

    if((frameLen + escapedLen) > (uint16_t)SerLink::Frame::MAX_DATALEN)
    {
      sdSend(frameData, frameLen);
      frameLen = 1U;
    }

    memcpy(&frameData[frameLen], escaped, escapedLen);
    frameLen = (uint16_t)(frameLen + escapedLen);
  }

  if(frameLen > 1U)
  {
    sdSend(frameData, frameLen);
  }
}

// sdCardTask only. <letter><rr>.<file size, 10 digits> - the answer to
// O, W, L and P.
static void sendSdResultAndSize(char letter, FRESULT result)
{
  char reply[1U + SDCARD_RESULT_WIDTH + 1U + SDCARD_SIZE_WIDTH];
  reply[0] = letter;
  writeUintField((uint32_t)result, SDCARD_RESULT_WIDTH, &reply[1]);
  reply[1U + SDCARD_RESULT_WIDTH] = '.';
  writeUintField(sdCard.getFileSize(), SDCARD_SIZE_WIDTH,
    &reply[2U + SDCARD_RESULT_WIDTH]);
  sdSend(reply, (uint16_t)sizeof(reply));
}

// sdCardTask only - carries out one command and sends its answer.
static void runSdCmd(const SdCmd& cmd)
{
  switch(cmd.op)
  {
    case SdCmd::open:       // -> O<rr>.<size>
      sendSdResultAndSize('O', sdCard.open(cmd.text));
      break;

    case SdCmd::openWrite:  // -> W<rr>.<size>
      sendSdResultAndSize('W', sdCard.openWrite(cmd.text, cmd.mode));
      break;

    case SdCmd::writeLine:  // -> L<rr>.<size>
    case SdCmd::writeText:  // -> P<rr>.<size>
    {
      /* One f_write() for the text and its line end, so the sync below
         never lands between the two. */
      static const char lineEnd[] = SDCARD_LINE_END;
      char buffer[SerLink::Frame::MAX_DATALEN + sizeof(lineEnd) - 1U];

      uint32_t len = cmd.len;
      memcpy(buffer, cmd.text, len);
      if(cmd.op == SdCmd::writeLine)
      {
        memcpy(&buffer[len], lineEnd, sizeof(lineEnd) - 1U);
        len += (uint32_t)(sizeof(lineEnd) - 1U);
      }

      uint32_t written = 0U;
      FRESULT result = sdCard.write((const uint8_t*)buffer, len,
        (SDCARD_SYNC_EACH_WRITE != 0), &written);
      sendSdResultAndSize((cmd.op == SdCmd::writeLine) ? 'L' : 'P', result);
      break;
    }

    case SdCmd::read:   // -> D frames, then R<rr>.<bytes read>
    {
      FRESULT result;
      uint32_t bytesRead = 0U;

      /* Checked here rather than in the receive handler, so a bad length
         still gets an answer: R19 (FR_INVALID_PARAMETER). */
      if((cmd.len == 0U) || (cmd.len > SDCARD_READ_MAX))
      {
        result = FR_INVALID_PARAMETER;
      }
      else
      {
        result = sdCard.read(cmd.offset, sdReadBuffer, cmd.len, &bytesRead);
      }

      /* Whatever arrived, even if the read then failed part way. */
      sendSdData(sdReadBuffer, bytesRead);

      char reply[1U + SDCARD_RESULT_WIDTH + 1U + SDCARD_LEN_MAX_WIDTH];
      reply[0] = 'R';
      writeUintField((uint32_t)result, SDCARD_RESULT_WIDTH, &reply[1]);
      reply[1U + SDCARD_RESULT_WIDTH] = '.';
      writeUintField(bytesRead, SDCARD_LEN_MAX_WIDTH,
        &reply[2U + SDCARD_RESULT_WIDTH]);
      sdSend(reply, (uint16_t)sizeof(reply));
      break;
    }

    case SdCmd::close:  // -> C<rr>
    {
      FRESULT result = sdCard.close();

      char reply[1U + SDCARD_RESULT_WIDTH];
      reply[0] = 'C';
      writeUintField((uint32_t)result, SDCARD_RESULT_WIDTH, &reply[1]);
      sdSend(reply, (uint16_t)sizeof(reply));
      break;
    }

    default:
      break;
  }
}

// Owns sdCard. Waits for a command for up to SDCARD_POLL_MS, and polls
// the card detect pin every time round - a mount or an unmount there is
// reported with a status frame, so a terminal sees the card come and go.
//
// sdCard.poll() mounts in this task too, so a card inserted mid-command
// is only picked up once that command is done - and one pulled out
// mid-command makes the command fail first.
void startSdCardTask(void *argument)
{
  for(;;)
  {
    SdCmd cmd;
    if(xQueueReceive(sdCmdQueue, &cmd, pdMS_TO_TICKS(SDCARD_POLL_MS)) == pdTRUE)
    {
      runSdCmd(cmd);
    }

    if(sdCard.poll())
    {
      char status[SDCARD_STATUS_LEN];
      writeSdStatus(status);
      sdSend(status, (uint16_t)sizeof(status));
    }
  }
}

// Copies a path into cmd.text, NUL terminated for FatFs. False if it is
// empty or too long to terminate.
static bool copySdPath(const char* src, uint16_t len, SdCmd* cmd)
{
  if((len == 0U) || (len >= sizeof(cmd->text)))
  {
    return false;
  }
  memcpy(cmd->text, src, len);
  cmd->text[len] = '\0';
  return true;
}

// The commands. Runs in serLink0Task - parses and posts, and never
// blocks: with the queue full the command is answered Q<cmd> and dropped.
void sdSockReceiveHandler(const char* data, uint16_t dataLen)
{
  if(dataLen < 1U)
  {
    return;
  }

  SdCmd cmd = {};

  switch(data[0])
  {
    case 'O':   // O<path> - open for reading
      if(!copySdPath(&data[1], (uint16_t)(dataLen - 1U), &cmd))
      {
        return;
      }
      cmd.op = SdCmd::open;
      break;

    case 'W':   // W<O|A|N><path> - open for writing: overwrite/append/new
      if(dataLen < 3U)
      {
        return;
      }
      switch(data[1])
      {
        case 'O': cmd.mode = SdCard::WriteMode::overwrite; break;
        case 'A': cmd.mode = SdCard::WriteMode::append;    break;
        case 'N': cmd.mode = SdCard::WriteMode::createNew; break;
        default:  return;
      }
      if(!copySdPath(&data[2], (uint16_t)(dataLen - 2U), &cmd))
      {
        return;
      }
      cmd.op = SdCmd::openWrite;
      break;

    case 'L':   // L<text> - write text and a line end; L alone, a blank line
    case 'P':   // P<text> - write text only, for lines longer than a frame
      cmd.len = (uint32_t)(dataLen - 1U);
      if((data[0] == 'P') && (cmd.len == 0U))
      {
        return;
      }
      memcpy(cmd.text, &data[1], cmd.len);   /* <= MAX_DATALEN - 1, fits */
      cmd.op = (data[0] == 'L') ? SdCmd::writeLine : SdCmd::writeText;
      break;

    case 'R':   // R<offset>.<len>
    {
      /* Find the '.', then both fields have to be all digits and within
         their widths. */
      uint16_t dot = 1U;
      while((dot < dataLen) && (data[dot] != '.'))
      {
        dot++;
      }

      uint16_t offsetWidth = (uint16_t)(dot - 1U);
      uint16_t lenWidth = (uint16_t)(dataLen - dot - 1U);
      if((dot >= dataLen) ||
         (offsetWidth == 0U) || (offsetWidth > SDCARD_OFFSET_MAX_WIDTH) ||
         (lenWidth == 0U) || (lenWidth > SDCARD_LEN_MAX_WIDTH))
      {
        return;
      }

      if(!readUintField(&data[1], (uint8_t)offsetWidth, &cmd.offset) ||
         !readUintField(&data[dot + 1U], (uint8_t)lenWidth, &cmd.len))
      {
        return;
      }

      cmd.op = SdCmd::read;
      break;
    }

    case 'C':   // C
      if(dataLen != 1U)
      {
        return;
      }
      cmd.op = SdCmd::close;
      break;

    case 'S':   // status      - answered on the ack, in sdSockInstantHandler()
    case 'E':   // diagnostics - likewise
    default:
      return;
  }

  /* A write line lost here would leave a hole in the file with nothing
     to show for it, so say so: Q and the command letter. sendData() is
     non-blocking, so this is safe from serLink0Task. */
  if((xQueueSend(sdCmdQueue, &cmd, 0U) != pdTRUE) && (sdSocket != nullptr))
  {
    char busy[2] = { 'Q', data[0] };
    (void)sdSocket->sendData(busy, (uint16_t)sizeof(busy), false);
  }
}

// E<hhhhhhhh>.<mmmmm> - the HAL's SD error code (hsd.ErrorCode, the
// HAL_SD_ERROR_* bits, hex) and how long the last mount took in ms
// (clamped to 99999). For bring-up: a status of 01 (FR_DISK_ERR) says a
// transfer failed, this says how. The HAL clears ErrorCode at the start of
// each operation, so it describes the most recent one. Common bits:
//   00000002 data CRC fail      00000008 data timeout
//   00000004 command timeout    00000020 RX overrun
//   00000001 command CRC fail   40000000 DMA transfer error
#define SDCARD_DIAG_LEN  (1U + 8U + 1U + 5U)

static void writeSdDiagnostics(char* dst)
{
  static const char hexDigits[] = "0123456789ABCDEF";

  /* One aligned word, written by the SDIO/DMA interrupts - reading it
     here is safe, if possibly a transfer behind. */
  uint32_t errorCode = hsd.ErrorCode;

  dst[0] = 'E';
  for(uint8_t i = 0U; i < 8U; i++)
  {
    dst[1U + i] = hexDigits[(errorCode >> (28U - (4U * i))) & 0x0FU];
  }
  dst[9] = '.';

  uint32_t mountMs = sdCard.getLastMountMs();
  writeUintField((mountMs > 99999U) ? 99999U : mountMs, 5U, &dst[10]);
}

// The reads - status and diagnostics. Run in reader0Task, before the ack
// is sent - getters only.
bool sdSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  if(rxFrame.dataLen != 1U)
  {
    return false;   // a command - leave the ack a plain ACK_OK
  }

  switch(rxFrame.data[0])
  {
    case 'S':
      writeSdStatus(data);
      *dataLen = SDCARD_STATUS_LEN;
      return true;

    case 'E':
      writeSdDiagnostics(data);
      *dataLen = SDCARD_DIAG_LEN;
      return true;

    default:
      return false;   // C (close) is a command
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
// Owns mqtt2: the only caller of its receive path, and the task that keeps
// the connection up. See SerLinkMqttAdapter.hpp for why the transmit side
// is not here - write() publishes directly from the caller's task.
void startMqtt2Task(void *argument)
{
  /* initTasks() ran before MX_LWIP_Init(), which is what creates the
     tcpip core lock that start() reaches through, and the netif coming up
     is the signal that it exists. Waiting for the link as well saves a
     connect that could only time out. */
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
   fires. See the worked example at the bottom of app_main.cpp. */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == nRF24L01_nINT_Pin)
  {
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
//   DBG00T349002SL   ->  DBG00A349014mqtt2Task:0096    the task with the
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