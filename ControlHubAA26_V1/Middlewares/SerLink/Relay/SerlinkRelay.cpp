#include "SerlinkRelay.hpp"
#include "Socket.hpp"
#include "task.h"

using namespace SerLink;

SerlinkRelayPair::SerlinkRelayPair()
: socketA(nullptr), socketB(nullptr), ackWait(false), ackSource(nullptr),
  ackDest(nullptr), ackRollCode(0), startTick(0)
{
}

SerlinkRelay::SerlinkRelay()
: numPairs(0), queue(nullptr)
{
}

void SerlinkRelay::init()
{
  this->queue = xQueueCreateStatic(SERLINK_RELAY__QUEUE_LENGTH, sizeof(SerlinkRelayMsg),
    this->queueStorageArea, &this->staticQueue);
}

bool SerlinkRelay::registerPair(Socket* socketA, Socket* socketB)
{
  if((socketA == nullptr) || (socketB == nullptr))
  {
    return false;
  }

  if(this->numPairs >= SERLINK_RELAY__MAX_NUM_PAIRS)
  {
    // No more pairs are available
    return false;
  }

  SerlinkRelayPair* pair = &this->pairs[this->numPairs];
  pair->socketA = socketA;
  pair->socketB = socketB;
  pair->ackWait = false;
  this->numPairs++;

  socketA->setRelay(this);
  socketB->setRelay(this);
  return true;
}

bool SerlinkRelay::relayFrame(Socket* socket, Frame* frame, uint8_t type)
{
  if(this->queue == nullptr)
  {
    return false; // init() not called
  }

  SerlinkRelayMsg relayMsg;
  relayMsg.frame = *frame;
  relayMsg.socket = socket;
  relayMsg.type = type;

  return (xQueueSend(this->queue, &relayMsg, 0) == pdTRUE);
}

void SerlinkRelay::run()
{
  if(xQueueReceive(this->queue, &this->msg, portMAX_DELAY) != pdTRUE)
  {
    return;
  }

  SerlinkRelayPair* pair = this->findPair(this->msg.socket);
  if(pair == nullptr)
  {
    return; // not a relayed socket
  }

  if(this->msg.type == SerlinkRelayMsg::TYPE_ACK)
  {
    this->checkAck(pair);
  }
  else
  {
    this->relay(pair);
  }
}

SerlinkRelayPair* SerlinkRelay::findPair(Socket* socket)
{
  for(uint8_t i = 0; i < this->numPairs; i++)
  {
    if((this->pairs[i].socketA == socket) || (this->pairs[i].socketB == socket))
    {
      return &this->pairs[i];
    }
  }
  return nullptr;
}

void SerlinkRelay::relay(SerlinkRelayPair* pair)
{
  Socket* source = this->msg.socket;
  Socket* dest = (source == pair->socketA) ? pair->socketB : pair->socketA;
  Frame* frame = &this->msg.frame;

  if(frame->type == Frame::TYPE_UNIDIRECTION)
  {
    dest->sendFrame(frame);
  }
  else if(frame->type == Frame::TYPE_TRANSMISSION)
  {
    if(dest->sendFrame(frame))
    {
      pair->ackWait = true;
      pair->ackSource = source;
      pair->ackDest = dest;
      pair->ackRollCode = frame->rollCode;
      pair->startTick = xTaskGetTickCount();
    }
  }
  else
  {
    // Not a relayed frame type (e.g. 'B') - so do nothing
  }
}

void SerlinkRelay::checkAck(SerlinkRelayPair* pair)
{
  Frame* ack = &this->msg.frame;

  if(!pair->ackWait || (this->msg.socket != pair->ackDest) || (ack->rollCode != pair->ackRollCode))
  {
    return; // not the ack being waited for
  }

  pair->ackWait = false;

  if((xTaskGetTickCount() - pair->startTick) > pdMS_TO_TICKS(SERLINK_RELAY__ACK_TIMEOUT_MS))
  {
    return; // too late - no relay ack is sent
  }

  // Send the relay ack back to the source. The ack's roll code, data length
  // (e.g. ACK_OK) & data are kept; sendFrame() sets the source's protocol.
  ack->type = Frame::TYPE_RELAY_ACK;
  pair->ackSource->sendFrame(ack);
}
