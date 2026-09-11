/*
 * Radio.cpp
 */

#include "Radio.hpp"
#include "uart2.h"
#include "task.h"
#include <string.h>

#define INIT 0
#define IDLE 1
#define RX   2
#define TX   3

const uint8_t Radio::DEFAULT_ADDRESS[nRF24L01::ADDRESS_LEN] = { 'S', 'L', 'N', 'K', '1' };

Radio::Radio(GPIO_TypeDef* cePort, uint16_t cePin,
             GPIO_TypeDef* csnPort, uint16_t csnPin)
  : eventQueue(nullptr), rxDataQueue(nullptr),
    nrf(cePort, cePin, csnPort, csnPin)
{
  this->currentState  = INIT;
  this->txReturnState = IDLE;
  this->irqPending    = false;
  this->rxSynced      = false;
  memset(this->address, 0, nRF24L01::ADDRESS_LEN);
}

void Radio::init(const uint8_t* address)
{
  memcpy(this->address, address, nRF24L01::ADDRESS_LEN);

  this->currentState  = INIT;
  this->txReturnState = IDLE;
  this->irqPending    = false;

  this->irqMsg.msgType = RadioMsg::RX_IRQ;
  memset(this->irqMsg.buffer, 0, RADIOMSG__FRAME_LEN_MAX);

  this->resetRxFrame();
  this->rxSynced = false;

  this->rxDataQueue = xQueueCreateStatic(RADIO__RXDATA_QUEUE_LENGTH, sizeof(UartMessage_t),
    this->rxDataQueueStorageArea, &this->rxDataStaticQueue);

  /* Last: onIrq() treats a null eventQueue as "not initialised yet". */
  this->eventQueue = xQueueCreateStatic(RADIO__EVENT_QUEUE_LENGTH, sizeof(RadioMsg),
    this->eventQueueStorageArea, &this->eventStaticQueue);
}

void Radio::run()
{
  switch(this->currentState)
  {
    case INIT: { this->currentState = this->initDevice(); break; }
    case IDLE: { this->currentState = this->idle(); break; }
    case RX:   { this->currentState = this->rx(); break; }
    case TX:   { this->currentState = this->tx(); break; }
  }
}

uint8_t Radio::startListening()
{
  return this->postCommand(RadioMsg::START_LISTENING);
}

uint8_t Radio::stopListening()
{
  return this->postCommand(RadioMsg::STOP_LISTENING);
}

uint8_t Radio::write(char* buffer)
{
  return RadioMsg::queueTxData(this->eventQueue, buffer);
}

void Radio::onIrq()
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if((this->eventQueue == nullptr) || this->irqPending)
  {
    return;   // not initialised yet, or an RX_IRQ is already waiting
  }

  /* To the front: the RX FIFO is only three deep, so draining it should not
     wait behind queued transmissions. If the queue is full the edge is lost
     with irqPending still clear, so the next edge tries again -- and tx()
     drains after every transmission and rx() on its backstop timeout, so a
     lost edge only delays a read. */
  if(xQueueSendToFrontFromISR(this->eventQueue, &this->irqMsg, &higherPriorityTaskWoken) == pdTRUE)
  {
    this->irqPending = true;
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

//----------------------------------------------------------------
// start of state methods

uint8_t Radio::initDevice()
{
  /* Interrupt mode unmasks nINT. The driver also creates the semaphore its
     own waitForData() uses; that goes unused here, since onIrq() posts to
     eventQueue instead. */
  if(!this->nrf.init(nRF24L01::Mode::Interrupt))
  {
    /* Not answering on SPI - usually wiring or power on a plug-in module,
       which can be fixed without a reset. */
    vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_MS));
    return INIT;
  }

  this->nrf.setChannel(CHANNEL);
  this->nrf.setPayloadLen(PACKET_LEN);   // before the pipes, which are sized from it
  this->nrf.openReadingPipe(1, this->address);

  return IDLE;
}

// Powered up in standby, not receiving. Blocks until there is a frame to
// send or a command to start listening.
uint8_t Radio::idle()
{
  if(xQueueReceive(this->eventQueue, &this->eventMsg, portMAX_DELAY) != pdTRUE)
  {
    return IDLE;
  }

  switch(this->eventMsg.msgType)
  {
    case RadioMsg::TX_DATA:
      this->txReturnState = IDLE;
      return TX;

    case RadioMsg::START_LISTENING:
      this->nrf.startListening();
      return RX;

    case RadioMsg::RX_IRQ:
      /* Not listening, so nothing to read - but clear the flag, or onIrq()
         would never post another. */
      this->irqPending = false;
      return IDLE;

    default:
      return IDLE;   // STOP_LISTENING: already idle
  }
}

// Listening. Blocks until there is a frame to send, a command, or nINT.
uint8_t Radio::rx()
{
  if(xQueueReceive(this->eventQueue, &this->eventMsg, pdMS_TO_TICKS(RX_BACKSTOP_MS)) == pdTRUE)
  {
    switch(this->eventMsg.msgType)
    {
      case RadioMsg::TX_DATA:
        this->txReturnState = RX;
        return TX;

      case RadioMsg::STOP_LISTENING:
        this->nrf.stopListening();
        this->drainRxFifo();   // deliver whatever arrived before listening stopped
        return IDLE;

      default:
        break;   // RX_IRQ. START_LISTENING is a no-op when already listening.
    }
  }

  /* RX_IRQ, or RX_BACKSTOP_MS with no event at all. */
  this->drainRxFifo();
  return RX;
}

// Sends eventMsg.buffer split into packets, then goes back to txReturnState.
uint8_t Radio::tx()
{
  uint8_t  packet[PACKET_LEN];
  uint16_t frameLen = (uint16_t)strnlen(this->eventMsg.buffer, RADIOMSG__FRAME_LEN_MAX);
  uint16_t offset   = 0;

  if(this->txReturnState == RX)
  {
    this->nrf.stopListening();
  }

  /* Every time, not once: startListening() closes pipe 0, and write() needs
     pipe 0 open on the TX address to hear the auto-ack. */
  this->nrf.openWritingPipe(this->address);

  while(offset < frameLen)
  {
    uint16_t chunkLen = frameLen - offset;
    if(chunkLen > PACKET_DATA_LEN)
    {
      chunkLen = PACKET_DATA_LEN;
    }

    packet[0] = (offset == 0) ? PACKET_FLAG_START : 0;
    memcpy(&packet[PACKET_HEADER_LEN], &this->eventMsg.buffer[offset], chunkLen);

    // write() zero-pads the rest of the packet
    if(!this->nrf.write(packet, (uint8_t)(PACKET_HEADER_LEN + chunkLen)))
    {
      break;   // not acked: the rest of the frame is useless without this part
    }

    offset += chunkLen;
  }

  if(this->txReturnState == RX)
  {
    this->nrf.startListening();

    /* Picks up anything whose RX_IRQ never made it onto a full eventQueue.
       Without this, a steady run of TX_DATA would keep rx() from ever
       reaching its own drain. */
    this->drainRxFifo();
  }

  return this->txReturnState;
}

// end of state methods
//----------------------------------------------------------------

uint8_t Radio::postCommand(RadioMsg::MsgType command)
{
  RadioMsg msg;

  if(this->eventQueue == nullptr)
  {
    return 1;
  }

  msg.msgType = command;
  memset(msg.buffer, 0, RADIOMSG__FRAME_LEN_MAX);

  return (xQueueSend(this->eventQueue, &msg, pdMS_TO_TICKS(RADIOMSG__TX_QUEUE_TIMEOUT_MS)) == pdTRUE) ? 0 : 1;
}

void Radio::drainRxFifo()
{
  uint8_t packet[PACKET_LEN];

  /* Cleared before reading, never after: an edge that lands mid-drain then
     posts a fresh RX_IRQ instead of being swallowed. Each read() checks
     RX_EMPTY after the previous one cleared RX_DR, so the loop also picks up
     anything arriving while it runs. */
  this->irqPending = false;

  while(this->nrf.read(packet, PACKET_LEN) > 0)
  {
    this->reassemble(packet);
  }
}

void Radio::reassemble(const uint8_t* packet)
{
  if((packet[0] & PACKET_FLAG_START) != 0)
  {
    /* Anything already collected belongs to a frame the sender gave up on
       part way through. */
    this->resetRxFrame();
    this->rxSynced = true;
  }

  if(!this->rxSynced)
  {
    return;   // the middle of a frame whose start was never seen
  }

  for(uint8_t i = PACKET_HEADER_LEN; i < PACKET_LEN; i++)
  {
    char c = (char)packet[i];

    if(c == '\0')
    {
      break;   // zero padding: the rest of this packet is empty
    }

    if(this->rxFrame.len >= (UART2__BUFFER_LEN - 1))
    {
      /* No '\n' within a buffer's worth: drop it, as uart2 does, and ignore
         everything up to the next START. */
      this->resetRxFrame();
      this->rxSynced = false;
      return;
    }

    this->rxFrame.data[this->rxFrame.len++] = c;

    if(c == '\n')
    {
      // Dropped if the Reader has fallen behind, as uart2 does.
      xQueueSend(this->rxDataQueue, &this->rxFrame, 0);

      this->resetRxFrame();
      this->rxSynced = false;   // one frame per START
      return;
    }
  }
}

// Zero-filled, not just len = 0: the Reader parses data as a string.
void Radio::resetRxFrame()
{
  memset(this->rxFrame.data, 0, UART2__BUFFER_LEN);
  this->rxFrame.len  = 0;
  this->rxFrame.type = UART_MSG_TYPE__FRAME_RX;
}
