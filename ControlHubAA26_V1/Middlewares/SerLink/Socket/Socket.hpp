
#ifndef SOCKET_HPP_
#define SOCKET_HPP_

#include "SerLink_config.hpp"
#include "SerLink_Msgs.hpp"
#include "FreeRTOS.h"
#include "queue.h"

namespace SerLink {

  // Only used as a pointer here - forward declared (rather than #included)
  // to avoid a Transport.hpp <-> Socket.hpp circular include.
  class Transport;

  class Socket {
    protected:
      char protocol[Frame::LEN_PROTOCOL];
      uint16_t txRollCode;

      // sendData() calls transport->sendData() rather than
      // writer->sendFrame() directly, so all sends (like all receives) go
      // via Transport::run().
      Transport* transport;

      // Receive queue
      StaticQueue_t staticRxQueue;
      char staticRxQueueStorageArea[SERLINK_CONFIG__SOCKET_RX_MAX_MSGS * sizeof(SocketMsg)];
      QueueHandle_t rxQueue;

      // Transmit queue (contains messages to be sent, and tack messages)
      StaticQueue_t staticTxQueue;
      char staticTxQueueStorageArea[SERLINK_CONFIG__SOCKET_RX_MAX_MSGS * sizeof(SocketMsg)];
      QueueHandle_t txQueue;

      onReceiveCallback receiveCallback;

    public:
      Socket();
      void init(char* protocol, Transport* transport, onReceiveCallback = nullptr);
      bool sendData(char* data, uint16_t dataLen, bool ack);
      bool receiveData(uint16_t* dataLen, char* data, uint16_t timeoutMs);

      // Called (by Transport, once it identifies this socket's protocol as
      // the recipient) when data has been received for this socket. If
      // receiveCallback is set it's invoked directly, in the caller's
      // context; otherwise the data is queued (non-blocking - silently
      // dropped if rxQueue is full) for later retrieval via receiveData().
      void deliverReceivedData(char* data, uint16_t dataLen);

      // True once init() has assigned this socket a protocol.
      bool isAcquired();

      // Used by Transport to find which acquired socket a received frame
      // belongs to.
      bool matchesProtocol(char* protocol);
  };

}

#endif /* SOCKET_HPP_ */