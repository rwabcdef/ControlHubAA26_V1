#include "Socket.hpp"
#include "Transport.hpp"
#include <string.h>

using namespace SerLink;

Socket::Socket()
{
  this->protocol[0] = '\0'; // sentinel: not yet acquired (init() not called)
  this->transport = nullptr;
  this->txRollCode = 0;
  this->receiveCallback = nullptr;
  this->instantHandler = nullptr;
  this->relay = nullptr;
}

void Socket::init(char* protocol, Transport* transport, onReceiveCallback receiveCallback,
  readHandler instantHandler)
{
  this->transport = transport;
  this->txRollCode = 0;
  this->receiveCallback = receiveCallback;
  this->instantHandler = instantHandler;

  /* The protocol last: a non-empty protocol is what makes the socket
     visible to Transport::findSocket(), and the Reader task may be
     searching while a socket is acquired from another task (DBG00 and
     MQTT0 still are). Filled in first, the handler is already there by
     the time the socket can be found. */
  strncpy(this->protocol, protocol, Frame::LEN_PROTOCOL);

  this->rxQueue = xQueueCreateStatic(SERLINK_CONFIG__SOCKET_RX_MAX_MSGS, sizeof(SocketMsg),
    (uint8_t*)this->staticRxQueueStorageArea, &this->staticRxQueue);

  this->txQueue = xQueueCreateStatic(SERLINK_CONFIG__SOCKET_TX_MAX_MSGS, sizeof(SocketMsg),
    (uint8_t*)this->staticTxQueueStorageArea, &this->staticTxQueue);
}

bool Socket::onInstant(Frame& rxFrame, uint16_t* dataLen, char* data)
{
  if(rxFrame.type == Frame::TYPE_SYSTEM)
  {
    /* Exact match on length and bytes: frame data is not NUL terminated. */
    const uint16_t pingLen = (uint16_t)strlen(SYS_PING);
    if((rxFrame.dataLen == pingLen) && (0 == memcmp(rxFrame.data, SYS_PING, pingLen)))
    {
      const uint16_t pingbackLen = (uint16_t)strlen(SYS_PINGBACK);
      memcpy(data, SYS_PINGBACK, pingbackLen);
      *dataLen = pingbackLen;
      return true;
    }

    return false;   // unknown system command - plain ACK_OK
  }

  if((rxFrame.type == Frame::TYPE_TRANSMISSION) && (this->instantHandler != nullptr))
  {
    return this->instantHandler(rxFrame, dataLen, data);
  }

  return false;
}

bool Socket::isAcquired()
{
  return this->protocol[0] != '\0';
}

bool Socket::matchesProtocol(char* protocol)
{
  return (strncmp(this->protocol, protocol, Frame::LEN_PROTOCOL) == 0);
}

// Non-blocking: hands the frame to transport->sendData() (see
// Transport::run()'s TYPE_TX handling) and returns whether it was
// accepted, rather than waiting here for the ack.
bool Socket::sendData(char* data, uint16_t dataLen, bool ack)
{
  char type = ack ? Frame::TYPE_TRANSMISSION : Frame::TYPE_UNIDIRECTION;
  
  Frame frame(this->protocol, type, this->txRollCode, dataLen, data);
  Frame::incRollCode(&this->txRollCode);

  return this->transport->sendFrame(&frame);
}

bool Socket::sendFrame(Frame* frame)
{
  if(this->transport == nullptr)
  {
    return false; // not acquired
  }

  Frame txFrame = *frame;
  txFrame.setProtocol(this->protocol);

  return this->transport->sendFrame(&txFrame);
}

void Socket::setRelay(SerlinkRelay* relay)
{
  this->relay = relay;
}

SerlinkRelay* Socket::getRelay()
{
  return this->relay;
}

bool Socket::receiveData(uint16_t* dataLen, char* data, uint16_t timeoutMs)
{
  SocketMsg msg;

  if(xQueueReceive(this->rxQueue, &msg, pdMS_TO_TICKS(timeoutMs)) == pdTRUE)
  {
    *dataLen = msg.dataLen;
    memcpy(data, msg.data, msg.dataLen);
    return true;
  }

  return false;
}

void Socket::deliverReceivedData(char* data, uint16_t dataLen)
{
  if(this->receiveCallback != nullptr)
  {
    this->receiveCallback(data, dataLen);
  }
  else
  {
    SocketMsg msg;
    msg.dataLen = (dataLen < Frame::MAX_DATALEN) ? dataLen : Frame::MAX_DATALEN;
    memcpy(msg.data, data, msg.dataLen);
    msg.type = SocketMsg::TYPE_RX;

    xQueueSend(this->rxQueue, &msg, 0); // non-blocking - dropped if rxQueue is full
  }
}
