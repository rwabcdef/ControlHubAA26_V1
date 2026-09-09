/*
 * Writer.cpp
 *
 *  Created on: 31 Oct 2023
 *      Author: rw123
 */

#include "Writer.hpp"
#include "uart2.h"
#include <string.h>
#include <stdio.h>

#define IDLE 0
#define ACKWAIT 1

#define WRITER_ACK_TIMEOUT_MS 1000

using namespace SerLink;

Writer::Writer(uint8_t id): id(id)
{
}

void Writer::init()
{
  this->id = id;
  this->currentState = IDLE;
  this->status = Writer::STATUS_IDLE;

  this->txQueue = xQueueCreateStatic(Writer::TX_QUEUE_LENGTH, sizeof(Frame), this->txQueueStorageArea, &this->txStaticQueue);
  this->ackQueue = xQueueCreateStatic(Writer::ACK_QUEUE_LENGTH, sizeof(Frame), this->ackQueueStorageArea, &this->ackStaticQueue);
}

void Writer::run()
{
  switch(this->currentState)
    {
      case IDLE: { this->currentState = this->idle(); break;}
      case ACKWAIT: { this->currentState = this->ackWait(); break;}
    }
}

// Used to send frame (called from app layer above). Non-blocking: rejects
// rather than blocking the caller if txQueue is already full.
uint8_t Writer::sendFrame(Frame* frame)
{
  if(xQueueSend(this->txQueue, frame, 0) != pdTRUE)
  {
    return 1;
  }

  this->status = Writer::STATUS_BUSY;
  return 0;
}

// Used to check tx status (called from app layer above)
uint8_t Writer::getStatus()
{
  uint8_t status = this->status;
  if(Writer::STATUS_BUSY == this->status)
  {
    // writer is still busy - do nothing, i.e. don't clear status
  }
  else
  {
    this->status = Writer::STATUS_IDLE; // clear status
  }
  return status;
}

uint8_t Writer::getStatusProtocol(char* protocol)
{
  uint8_t status = this->status;
  if(Writer::STATUS_BUSY == this->status)
  {
    // writer is still busy - do nothing, i.e. don't clear status
  }
  else
  {
    // writer is not busy
    if(strncmp(this->txFrame.protocol, protocol, SerLink::Frame::LEN_PROTOCOL) == 0)
    {
      // the last frame that was sent has the same protocol as the specified one
      this->status = Writer::STATUS_IDLE; // clear status
    }
    else
    {
      // protocol mis-match - so do nothing
    }
  }
  return status;
}

void Writer::getStatusStr(char* str)
{
  switch(this->status)
  {
    case STATUS_IDLE: sprintf(str, "idle\0", 0); break;
    case STATUS_BUSY: sprintf(str, "busy\0", 0); break;
    case STATUS_OK: sprintf(str, "ok\0", 0); break;
    case STATUS_TIMEOUT: sprintf(str, "timeout\0", 0); break;
    case STATUS_PROTOCOL_ERROR: sprintf(str, "protocol error\0", 0); break;
  }
}

// Called by a Reader to pass an ack frame to the Writer. Non-blocking: the
// ack is silently dropped rather than blocking the caller if ackQueue is full.
void Writer::setAckFrame(Frame* frame)
{
  xQueueSend(this->ackQueue, frame, 0);
}

//----------------------------------------------------------------
// start of state methods
uint8_t Writer::idle()
{
  Frame frame;

  // block forever - there's nothing to do until the app layer asks us to send
  if(xQueueReceive(this->txQueue, &frame, portMAX_DELAY) == pdTRUE)
  {
    frame.copy(&this->txFrame); // remember what we're sending, for the ack check in ackWait()

    uint8_t ret;
    this->txFrame.toString((char*)this->txBuffer, &ret);

    this->uartWrite((char*)this->txBuffer); // blocking - done by the time this returns

    if(this->txFrame.type == Frame::TYPE_UNIDIRECTION)
    {
      this->status = Writer::STATUS_IDLE;
      return IDLE;
    }

    return ACKWAIT;
  }

  return IDLE;
}

uint8_t Writer::ackWait()
{
  Frame ackFrame;

  // block until an ack arrives, or up to the ack timeout
  if(xQueueReceive(this->ackQueue, &ackFrame, pdMS_TO_TICKS(WRITER_ACK_TIMEOUT_MS)) == pdTRUE)
  {
    if(0 == strncmp(ackFrame.protocol, this->txFrame.protocol, Frame::LEN_PROTOCOL))
    {
      this->status = Writer::STATUS_OK;
    }
    else
    {
      this->status = Writer::STATUS_PROTOCOL_ERROR;
    }
  }
  else
  {
    this->status = Writer::STATUS_TIMEOUT;
  }

  return IDLE;
}
// end of state methods
//----------------------------------------------------------------

uint8_t Writer::uartWrite(char* buffer)
{
#ifdef WRITER_CONFIG__WRITER0

  if(this->id == WRITER_CONFIG__WRITER0_ID)
  {
    return uart2_writeBlocking(buffer);
  }

#endif
  return 1;
}
