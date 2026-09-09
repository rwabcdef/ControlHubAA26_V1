

#include "Reader.hpp"
#include "uart2.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define IDLE 0
#define ACKDELAY 1
#define TXACK 2

#define ACK_DELAY_MS 50

using namespace SerLink;

Reader::Reader(uint8_t id): id(id)
{
  this->currentState = IDLE;
}

void Reader::init(QueueHandle_t uartRxQueue, Writer* writer, QueueHandle_t consumerQueue)
{
  this->uartRxQueue = uartRxQueue;
  this->writer = writer;
  this->consumerQueue = consumerQueue;
  this->numInstantHandlers = 0;
}

void Reader::run()
{
  switch(this->currentState)
  {
    case IDLE: { this->currentState = this->idle(); break; }
    case ACKDELAY: { this->currentState = this->ackDelay(); break; }
    case TXACK: { this->currentState = this->txAck(); break; }
  }
}

//-----------------------------------------------------------------
// Start of state methods

uint8_t Reader::idle()
{
  if(this->checkUartFrameRx())
  {
    // Convert received message from uart layer to Frame.
    Frame::fromString(this->rxMsg.data, &this->rxFrame);

    if(this->rxFrame.type == Frame::TYPE_TRANSMISSION)
    {

      this->rxFrame.copy(&this->ackFrame);
			this->ackFrame.type = Frame::TYPE_ACK;
			this->ackFrame.dataLen = Frame::ACK_OK;
			memset(this->ackFrame.data, 0, Frame::MAX_DATALEN);

      readHandler instantHandler = this->getInstantHandler(this->rxFrame.protocol);
			if(instantHandler == nullptr)
			{
			  // No instant (i.e. piggyback) handler has been found - so do nothing.
			}
      else
			{
			  // An instant (i.e. piggyback) handler has been found for this protocol,
			  // so call it now.
			  // The instant handler callback sets the ackFrame's data & dataLen.
			  bool useReturn = instantHandler(this->rxFrame, &this->ackFrame.dataLen, this->ackFrame.data);

        if(!useReturn)
			  {
			    // do not use data length and data in ack frame that was set by the instantHandler
			    this->ackFrame.type = Frame::TYPE_ACK;
				  this->ackFrame.dataLen = Frame::ACK_OK;
			  }
			  else
			  {
			    // do nothing - use data length and data in ack frame that was set by the instantHandler
			  }
      }

      // pass the received frame to the consumer queue if it exists
      if(this->consumerQueue != nullptr)
      {
        // copy the received frame into the rxFrameMsg and send it to the consumer queue
        this->rxFrame.copy(&this->rxFrameMsg.frame);
        this->rxFrameMsg.type = FrameMsg::TYPE_RX;

        xQueueSend(this->consumerQueue, &this->rxFrameMsg, 0);
      }

      // capture the exact point of transition - ackDelay() waits an
      // absolute ACK_DELAY_MS measured from here, not from whenever it
      // next gets a run() call
      this->ackDelayStartTick = xTaskGetTickCount();
      return ACKDELAY;
    }
    else if(this->rxFrame.type == Frame::TYPE_UNIDIRECTION)
    {
      // pass the received frame to the consumer queue if it exists
      if(this->consumerQueue != nullptr)
      {
        this->rxFrameMsg.frame.copy(&this->rxFrame);
        this->rxFrameMsg.type = FrameMsg::TYPE_RX;

        xQueueSend(this->consumerQueue, &this->rxFrameMsg, 0);
      }
      return IDLE;
    }
    else if(this->rxFrame.type == Frame::TYPE_ACK){

      // pass the received ack frame to the writer if it exists
      if(this->writer != nullptr)
      {
        this->writer->setAckFrame(&this->rxFrame);
      }

      // also send the ack frame to the consumer queue if it exists - it can contain data (piggy-backing)
      if(this->consumerQueue != nullptr)
      {
        this->rxFrameMsg.frame.copy(&this->rxFrame);
        this->rxFrameMsg.type = FrameMsg::TYPE_ACK; 
        xQueueSend(this->consumerQueue, &this->rxFrameMsg, 0);
      }
      return IDLE;
    }
  }

  return IDLE;
}

uint8_t Reader::ackDelay()
{
  vTaskDelayUntil(&this->ackDelayStartTick, pdMS_TO_TICKS(ACK_DELAY_MS));
  return TXACK;
}

uint8_t Reader::txAck()
{
  // send the ack frame
  this->ackFrame.toString(this->ackBuffer, nullptr);
  this->uartWrite(this->ackBuffer);

  return IDLE;
}

// end of state methods
//-------------------------------------------------------------

uint8_t Reader::uartWrite(char* buffer)
{
#ifdef READER_CONFIG__READER0

  if(this->id == READER_CONFIG__READER0_ID)
  {
    return uart2_writeBlocking(buffer);
  }

#endif
  return 1;
}

// Checks uart layer (below) to see if a frame has been received.
bool Reader::checkUartFrameRx()
{

  if (xQueueReceive(this->uartRxQueue, &this->rxMsg, portMAX_DELAY) == pdTRUE)
  {
    if((this->rxMsg.type == UART_MSG_TYPE__FRAME_RX) && (this->rxMsg.len > 1))
    {
      return true;
    }
    
  }

  return false;
}

bool Reader::registerInstantCallback(char* protocol, readHandler handler)
{
  if(this->numInstantHandlers < (READER_CONFIG__MAX_NUM_INSTANT_HANDLERS - 1))
  {
    strncpy(this->handlerRegistrations[this->numInstantHandlers].protocol, protocol, Frame::LEN_PROTOCOL);
    this->handlerRegistrations[this->numInstantHandlers].handler = handler;
    this->numInstantHandlers++;
    return true;
  }
  else
  {
    // No more registrations are available
    return false;
  }
}

readHandler Reader::getInstantHandler(char* protocol)
{
  for(uint8_t i=0; i<READER_CONFIG__MAX_NUM_INSTANT_HANDLERS; i++)
  {
    if(0 == strncmp(this->handlerRegistrations[i].protocol, protocol, Frame::LEN_PROTOCOL))
    {
      return this->handlerRegistrations[i].handler;
    }
  }
  return nullptr;
}
