
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
  class SerlinkRelay;

  /*
   * Instant handling - what rides back on the ack
   * ---------------------------------------------
   * A received 'T' or 'S' frame is acked by the Reader, in the Reader's
   * task, before Transport or the socket's receive callback ever see the
   * frame. What goes INSIDE that ack is decided here: the Reader finds
   * this socket through Transport::findSocket() and calls onInstant().
   *
   *   'T'  onInstant() calls the instantHandler given to acquireSocket(),
   *        if there is one - a read, answered on the ack. Afterwards the
   *        frame goes on to the receive callback as usual (in the
   *        Transport task).
   *   'S'  a system command, answered by onInstant() itself. The frame
   *        goes no further: the socket's owner and its relay never see
   *        it. Commands:
   *
   *          PING  ->  PINGBACK    PROTOS<rrr>004PING -> PROTOA<rrr>008PINGBACK
   *
   *        Anything else gets a plain ACK_OK.
   *
   * So a PING proves the link, the Reader task and that this protocol has
   * a socket on this transport. A PING to a protocol with no socket gets
   * a plain ACK_OK from the Reader; no answer at all means the link (or
   * the node) is down, or the node predates 'S' - an old Reader drops
   * frame types it does not know. A relayed socket answers PING itself,
   * so pinging LED01 says nothing about the far end of the relay.
   *
   * Threading: onInstant() runs in the Reader's task. instantHandler must
   * therefore be safe there - getters, typically.
   */
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

      // Answers 'T' reads on the ack - see Instant handling, above.
      readHandler instantHandler;

      // Set by SerlinkRelay::registerPair() (see setRelay()).
      SerlinkRelay* relay;

    public:
      // System commands ('S' frames) and their answers. Not NUL terminated
      // on the wire - compared and copied by length.
      static constexpr const char* SYS_PING = "PING";
      static constexpr const char* SYS_PINGBACK = "PINGBACK";

      Socket();
      void init(char* protocol, Transport* transport, onReceiveCallback = nullptr,
        readHandler instantHandler = nullptr);

      // Called by the Reader, in its task, for a received 'T' or 'S' frame
      // for this socket, before the ack goes out. Returns true if it has
      // set dataLen and data for the ack; false leaves the ack a plain
      // ACK_OK. See Instant handling, above.
      bool onInstant(Frame& rxFrame, uint16_t* dataLen, char* data);
      bool sendData(char* data, uint16_t dataLen, bool ack);

      // Sends a copy of frame: its type, roll code, data length & data are
      // kept, the protocol is set to this socket's. Non-blocking, as sendData().
      bool sendFrame(Frame* frame);

      // While a relay is set, Transport hands this socket's received frames
      // and acks to the relay (see SerlinkRelay) instead of delivering them.
      void setRelay(SerlinkRelay* relay);
      SerlinkRelay* getRelay();
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