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
}

void Socket::init(char* protocol, Transport* transport, onReceiveCallback receiveCallback)
{
  strncpy(this->protocol, protocol, Frame::LEN_PROTOCOL);
  this->transport = transport;
  this->txRollCode = 0;
  this->receiveCallback = receiveCallback;

  this->rxQueue = xQueueCreateStatic(SERLINK_CONFIG__SOCKET_RX_MAX_MSGS, sizeof(SocketMsg),
    (uint8_t*)this->staticRxQueueStorageArea, &this->staticRxQueue);

  this->txQueue = xQueueCreateStatic(SERLINK_CONFIG__SOCKET_TX_MAX_MSGS, sizeof(SocketMsg),
    (uint8_t*)this->staticTxQueueStorageArea, &this->staticTxQueue);
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
