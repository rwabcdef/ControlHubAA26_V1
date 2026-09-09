
#ifndef SERLINK_MSGS_HPP_
#define SERLINK_MSGS_HPP_

#include "Frame.hpp"

namespace SerLink {

  // Instant read handler callback - populates the ack frame's dataLen and data,
  // if it wants to piggyback data on the ack.
  typedef bool (*readHandler)(Frame &rxFrame, uint16_t* dataLen, char* data);

  // Socket onReceive callback function type definition
  typedef void (*onReceiveCallback)(const char* data, uint16_t dataLen);

  class FrameMsg {
    public:
      static const uint8_t TYPE_RX = 1;
      static const uint8_t TYPE_TX = 2;
      static const uint8_t TYPE_ACK = 3;

      Frame frame;
      uint8_t type;
  };

  class SocketMsg {
    public:
      static const uint8_t TYPE_RX = 1;
      static const uint8_t TYPE_TX = 2;
      static const uint8_t TYPE_ACK_OK = 3;
      static const uint8_t TYPE_TIMEOUT = 4;

      uint16_t dataLen;
      char data[Frame::MAX_DATALEN];
      uint8_t type;
  };  
}

#endif /* SERLINK_MSGS_HPP_ */