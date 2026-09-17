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

 # led socket - relayed to the radio, and ultimatelty to the remote hub (arduino uno r4)
LED01U492002A1
LED01U492002A0

LED01T492002A1
LED01T492002A0

# Motor socket - the TC78H611FNG dual H-bridge on TIM8, channel B (IN1B/IN2B on J10 pins 10 and 8)
MOTORT516005BP050
MOTORT516005BP010

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
#include "nRF24L01.hpp"
#include "spi5.h"
#include "Radio.hpp"
#include "MqttPubSub.hpp"
#include "lwip/netif.h"

//--------------------------------------------------------------
/* Definitions for writer0Task */
osThreadId_t writer0TaskHandle;
const osThreadAttr_t writer0Task_attributes = {
  .name = "writer0Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for reader0Task */
osThreadId_t reader0TaskHandle;
const osThreadAttr_t reader0Task_attributes = {
  .name = "reader0Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for serLink0Task */
osThreadId_t serLink0TaskHandle;
const osThreadAttr_t serLink0Task_attributes = {
  .name = "serLink0Task",
  .stack_size = 512 * 4,
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for radioTxTask */
osThreadId_t radioTxTaskHandle;
const osThreadAttr_t radioTxTask_attributes = {
  .name = "radioTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for writer1Task */
osThreadId_t writer1TaskHandle;
const osThreadAttr_t writer1Task_attributes = {
  .name = "writer1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for reader1Task */
osThreadId_t reader1TaskHandle;
const osThreadAttr_t reader1Task_attributes = {
  .name = "reader1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for serLink1Task */
osThreadId_t serLink1TaskHandle;
const osThreadAttr_t serLink1Task_attributes = {
  .name = "serLink1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for radio1Task */
osThreadId_t radio1TaskHandle;
const osThreadAttr_t radio1Task_attributes = {
  .name = "radio1Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mqttTask */
osThreadId_t mqttTaskHandle;
const osThreadAttr_t mqttTask_attributes = {
  .name = "mqttTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for mqttRxTask */
osThreadId_t mqttRxTaskHandle;
const osThreadAttr_t mqttRxTask_attributes = {
  .name = "mqttRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for relayTask */
osThreadId_t relayTaskHandle;
const osThreadAttr_t relayTask_attributes = {
  .name = "relayTask",
  .stack_size = 512 * 4,
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

bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

// MOTOR socket handlers - the command set is documented above their
// implementations, below startMotorTask().
void motorSockReceiveHandler(const char* data, uint16_t dataLen);
bool motorSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

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
  motorB.setPercent(MOTORB_START_PERCENT);
  motorStandby.enable();

  motorTaskHandle = osThreadNew(startMotorTask, NULL, &motorTask_attributes);

  writer0.init(uart2_writeBlocking);
  reader0.init(uart2Queue, &writer0, transport0.queue);

  /* Sets are handled by motorSockReceiveHandler (in serLink0Task), reads
     by motorSockInstantHandler (in reader0Task, so the answer rides back
     on the ack).

     Order against reader0.init() no longer matters - Reader sets its
     instant handler table up in its constructor - but this stays next to
     the reader/writer setup it depends on. Still before the scheduler
     starts, so no task can see the socket half-registered.

     Note this is the fifth and last socket transport0 can hold -
     SERLINK_CONFIG__MAX_SOCKETS is 5, and DBG00 and MQTT0 are acquired
     later by their own tasks. A sixth would get a silent nullptr. */
  motorSocket = transport0.acquireSocket("MOTOR", motorSockReceiveHandler,
    motorSockInstantHandler);

  writer0TaskHandle = osThreadNew(startWriter0Task, NULL, &writer0Task_attributes);

  reader0TaskHandle = osThreadNew(startReader0Task, NULL, &reader0Task_attributes);

  serLink0TaskHandle = osThreadNew(startSerLink0Task, NULL, &serLink0Task_attributes);

   /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of mqttTask and mqttRxTask */
  // Creates mqtt.rxQueue, before mqttRxTask can block on it. Subscribing has
  // to wait for lwIP, so that happens in mqttTask.
  mqtt.init();

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
void startRadio1Task(void *argument)
{
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

  motorB.setPercent(MOTORB_START_PERCENT);
  motorB.setDirection(TC78H611FNG::reverse);

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
static void motorWriteUint(uint32_t value, uint8_t width, char* dst)
{
  for(uint8_t i = width; i > 0U; i--)
  {
    dst[i - 1U] = (char)('0' + (value % 10U));
    value /= 10U;
  }
}

static bool motorReadUint(const char* src, uint8_t width, uint32_t* value)
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
         motorReadUint(&data[2], 3U, &percent))
      {
        // No range check needed: setPercent() clamps above 100 itself,
        // and three digits cannot exceed 999.
        motor->setPercent((uint8_t)percent);
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
      motorWriteUint(motor->getPercent(), 3U, data);
      *dataLen = 3U;
      return true;

    case 'F':   // frequency in Hz, 4 digits - pwmFreqValues spans 100..2000
      motorWriteUint((uint32_t)motor->getFrequency(), 4U, data);
      *dataLen = 4U;
      return true;

    case 'D':   // direction, one of F/R/D
      data[0] = motorDirectionToChar(motor->getDirection());
      *dataLen = 1U;
      return true;

    default:
      return false;
  }
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
}

//--------------------------------------------------------------
bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data)
{
  uint8_t index = 0;
  if(rxFrame.data[index++] == 'R')
  {
    memset(data, 0, 10); // clear outgoing buffer
    strncpy(data, "OK", 2);
    *dataLen = 2;
    return true;
  }
  return false; // not handled
}