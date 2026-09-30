

#include "Reader.hpp"
#include "Transport.hpp"
#include "Socket.hpp"
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

  /* No instant handler table here any more: each Socket keeps its own,
     and the Reader finds the socket through Transport (setTransport()).
     transport is left alone - its default member initialiser has already
     set it, and Transport::init() may set it before init() below. */
}

void Reader::init(QueueHandle_t uartRxQueue, Writer* writer, QueueHandle_t consumerQueue,
  QueueHandle_t ackTxQueue)
{
  this->uartRxQueue = uartRxQueue;
  this->ackTxQueue = ackTxQueue;
  this->writer = writer;
  this->consumerQueue = consumerQueue;

  // transport is deliberately NOT touched here - Transport::init() sets it,
  // and is usually called first.
}

void Reader::setTransport(Transport* transport)
{
  this->transport = transport;
}

void Reader::setAckWriteFunc(WriteDataFunc ackWrite)
{
  this->ackWriteFunc = ackWrite;
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

    if((this->rxFrame.type == Frame::TYPE_TRANSMISSION) ||
       (this->rxFrame.type == Frame::TYPE_SYSTEM))
    {
      // Both are acked the same way. The difference is what happens after:
      // a 'T' frame goes on to the consumer, an 'S' frame does not - see
      // txAck().
      this->rxFrame.copy(&this->ackFrame);
      this->ackFrame.type = Frame::TYPE_ACK;
      this->ackFrame.dataLen = Frame::ACK_OK;
      memset(this->ackFrame.data, 0, Frame::MAX_DATALEN);

      // The socket decides what, if anything, rides back on the ack - its
      // instant handler for a 'T', its system commands (PING) for an 'S'.
      // No socket for this protocol, or no Transport: a plain ACK_OK, which
      // is also how a PING tells "no such socket" from "link down".
      Socket* socket = (this->transport != nullptr)
                       ? this->transport->findSocket(this->rxFrame.protocol)
                       : nullptr;

      if((socket != nullptr) &&
         !socket->onInstant(this->rxFrame, &this->ackFrame.dataLen, this->ackFrame.data))
      {
        // Nothing to add - discard anything the handler may have written.
        this->ackFrame.dataLen = Frame::ACK_OK;
      }

      // A received 'T' frame is passed to the consumer queue by txAck(),
      // once the ack has been sent - see txAck().

      // capture the exact point of transition - ackDelay() waits an
      // absolute ACK_DELAY_MS measured from here, not from whenever it
      // next gets a run() call
      this->ackDelayStartTick = xTaskGetTickCount();
      return ACKDELAY;
    }
    else if((this->rxFrame.type == Frame::TYPE_UNIDIRECTION) ||
            (this->rxFrame.type == Frame::TYPE_RELAY_ACK))
    {
      // No ack is sent for a unidirectional frame (or a relay ack)
      // pass the received frame to the consumer queue if it exists
      if(this->consumerQueue != nullptr)
      {
        this->rxFrame.copy(&this->rxFrameMsg.frame);
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
        this->rxFrame.copy(&this->rxFrameMsg.frame);
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

  // Only now pass the received 'T' frame to the consumer queue (as the Arduino
  // Reader does), so the ack always precedes anything the consumer sends in
  // response - e.g. a SerlinkRelay relaying the frame, whose relay ack ('B')
  // would otherwise beat this ack if the far end acks within ACK_DELAY_MS.
  // rxFrame is unchanged since idle(): no frame is read in ACKDELAY / TXACK.
  //
  // An 'S' frame stops here: it was for the SerLink layer, answered on the
  // ack, and carries nothing for the socket's owner or its relay.
  if((this->consumerQueue != nullptr) && (this->rxFrame.type == Frame::TYPE_TRANSMISSION))
  {
    this->rxFrame.copy(&this->rxFrameMsg.frame);
    this->rxFrameMsg.type = FrameMsg::TYPE_RX;

    xQueueSend(this->consumerQueue, &this->rxFrameMsg, 0);
  }

  return IDLE;
}

// end of state methods
//-------------------------------------------------------------

uint8_t Reader::uartWrite(char* buffer)
{
  // Checked before the id mapping, so an instance with an external link can
  // never also write to a uart.
  if(this->ackWriteFunc != nullptr)
  {
    return this->ackWriteFunc(buffer);
  }

  if(this->ackTxQueue != nullptr)
  {
    return RadioMsg::queueTxData(this->ackTxQueue, buffer);
  }

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

