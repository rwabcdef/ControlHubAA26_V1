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

 */

#include <cstdio>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
#include "main_tasks.h"
#include "queue.h"
#include "Reader.hpp"
#include "Transport.hpp"
#include "uart2.h"
#include "Button.hpp"
#include "Led.hpp"
#include "PWM.hpp"
#include "TC78H611FNG.hpp"
#include "nRF24L01.hpp"
#include "spi5.h"
#include "Radio.hpp"

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

bool debugSockInstantHandler(SerLink::Frame &rxFrame, uint16_t* dataLen, char* data);

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

//--------------------------------------------------------------
// Board LEDs (GPIOB)
Led ledBoardGreen(GPIOB, GPIO_PIN_0);

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

  writer0.init();
  reader0.init(uart2Queue, &writer0, transport0.queue);

  writer0TaskHandle = osThreadNew(startWriter0Task, NULL, &writer0Task_attributes);

  reader0TaskHandle = osThreadNew(startReader0Task, NULL, &reader0Task_attributes);

  serLink0TaskHandle = osThreadNew(startSerLink0Task, NULL, &serLink0Task_attributes);

   /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  // The nRF24L01 has one owner at a time - SerLink1 through radio1, or one of
  // the raw test tasks - since the driver is not thread-safe. Select with
  // RADIO_SERLINK and RADIO_TEST_TX.
#if RADIO_SERLINK
  transport1Queue = xQueueCreateStatic(TRANSPORT1_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport1QueueStorageArea, &transport1StaticQueue);
  transport1.init(transport1Queue);

  // Before writer1/reader1: init() creates the queues they are handed.
  radio1.init();

  // Queued now, applied once radio1Task has brought the device up. SerLink
  // needs the radio listening between transmissions or no ack ever arrives.
  radio1.startListening();

  writer1.init(radio1.eventQueue);
  reader1.init(radio1.rxDataQueue, &writer1, transport1.queue, radio1.eventQueue);

  writer1TaskHandle = osThreadNew(startWriter1Task, NULL, &writer1Task_attributes);

  reader1TaskHandle = osThreadNew(startReader1Task, NULL, &reader1Task_attributes);

  serLink1TaskHandle = osThreadNew(startSerLink1Task, NULL, &serLink1Task_attributes);

  radio1TaskHandle = osThreadNew(startRadio1Task, NULL, &radio1Task_attributes);
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