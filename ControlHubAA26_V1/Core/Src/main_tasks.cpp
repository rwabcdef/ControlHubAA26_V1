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

//--------------------------------------------------------------
void startWriter0Task(void *argument);
void startReader0Task(void *argument);
void startSerLink0Task(void *argument);
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

void initTasks()
{
  // Created synchronously here (rather than inside startSerLink0Task) so
  // transport0.queue is guaranteed valid before any task - including
  // startReader0Task, which passes it to reader0.init() - can run.
  transport0Queue = xQueueCreateStatic(TRANSPORT0_QUEUE_LENGTH, sizeof(SerLink::FrameMsg),
    transport0QueueStorageArea, &transport0StaticQueue);
  transport0.init(transport0Queue, transport0ReceiveCallback, transport0AckCallback);

  writer0.init();
  reader0.init(uart2Queue, &writer0, transport0.queue);

  writer0TaskHandle = osThreadNew(startWriter0Task, NULL, &writer0Task_attributes);

  reader0TaskHandle = osThreadNew(startReader0Task, NULL, &reader0Task_attributes);

  serLink0TaskHandle = osThreadNew(startSerLink0Task, NULL, &serLink0Task_attributes);
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