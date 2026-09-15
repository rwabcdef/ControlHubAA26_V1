/*
 * SerlinkRelay.hpp
 *
 * Relays SerLink traffic between pairs of Sockets, e.g. a transport0 (uart2)
 * socket and a transport1 (radio1) socket. Each registered pair is relayed in
 * both directions:
 *
 *   'U' frame received for one socket -> sent by the other socket.
 *
 *   'T' frame received for one socket -> sent (as 'T') by the other socket.
 *       When the destination's ack ('A') is received, a relay ack frame
 *       ('B', Frame::TYPE_RELAY_ACK) is sent back by the source socket,
 *       containing the ack's data length (e.g. ACK_OK) & data. If the ack
 *       arrives more than SERLINK_RELAY__ACK_TIMEOUT_MS after the 'T' frame
 *       was relayed, no 'B' frame is sent.
 *
 * Relayed frames keep the roll code of the received frame, so the source
 * can match the 'B' frame to the 'T' frame it sent, e.g.
 *   uart2 -> LED01T492002A1   (reader0 acks it with 'A', as for any 'T')
 *   radio -> LED01T492002A1
 *   radio <- LED01A492900     (far end ack)
 *   uart2 <- LED01B492900     (relay ack)
 *
 * Threading: the Transport tasks hand frames and acks for a relayed socket to
 * relayFrame(), which only posts them to this relay's queue. run() - called
 * repeatedly from the one task that owns the relay - does all the relaying,
 * so the pair state has a single owner. Frames and acks for a relayed socket
 * are not delivered to its receive callback / receiveData().
 *
 * Only one 'T' frame per pair is waited on at a time: a new 'T' frame
 * received for either socket of the pair replaces the one being waited on.
 * 'B' frames received for a relayed socket are not relayed.
 */

#ifndef SERLINK_RELAY_HPP_
#define SERLINK_RELAY_HPP_

#include <stdint.h>
#include "Frame.hpp"
#include "FreeRTOS.h"
#include "queue.h"

#define SERLINK_RELAY__MAX_NUM_PAIRS 5
#define SERLINK_RELAY__QUEUE_LENGTH 5
#define SERLINK_RELAY__ACK_TIMEOUT_MS 1500

namespace SerLink {

  // Only used as a pointer here - forward declared (rather than #included)
  // to avoid a Socket.hpp <-> SerlinkRelay.hpp circular include.
  class Socket;

  // Item type for SerlinkRelay's queue.
  class SerlinkRelayMsg {
    public:
      static const uint8_t TYPE_RX = 1;   // frame received for socket
      static const uint8_t TYPE_ACK = 2;  // ack frame received for socket

      Frame frame;
      Socket* socket;
      uint8_t type;
  };

  class SerlinkRelayPair {
    public:
      Socket* socketA;
      Socket* socketB;

      // 'T' frame waiting for the destination's ack
      bool ackWait;
      Socket* ackSource; // socket the 'T' frame was received for ('B' frame is sent by this)
      Socket* ackDest;   // socket the 'T' frame was sent by (ack is received for this)
      uint16_t ackRollCode;
      TickType_t startTick;

      SerlinkRelayPair();
  };

  class SerlinkRelay {
    public:
      SerlinkRelay();

      // Creates the queue. Call once, before registerPair() and run(). Safe
      // before the scheduler starts.
      void init();

      // Registers a pair of (already acquired) sockets to relay between. Call
      // before the Transport tasks run. Returns false if all
      // SERLINK_RELAY__MAX_NUM_PAIRS pairs are in use.
      bool registerPair(Socket* socketA, Socket* socketB);

      // Called by a Transport task when a frame (TYPE_RX) or an ack
      // (TYPE_ACK) is received for a relayed socket. Non-blocking: returns
      // false (frame dropped) if the queue is full.
      bool relayFrame(Socket* socket, Frame* frame, uint8_t type);

      // Services one message. Call repeatedly from the task that owns this
      // relay, after init(). Blocks until a message arrives.
      void run();

    protected:
      SerlinkRelayPair pairs[SERLINK_RELAY__MAX_NUM_PAIRS];
      uint8_t numPairs;

      StaticQueue_t staticQueue;
      uint8_t queueStorageArea[SERLINK_RELAY__QUEUE_LENGTH * sizeof(SerlinkRelayMsg)];
      QueueHandle_t queue;

      SerlinkRelayMsg msg; // last message taken from queue

      SerlinkRelayPair* findPair(Socket* socket);

      // Relays the 'U' or 'T' frame in msg to the other socket of pair.
      void relay(SerlinkRelayPair* pair);

      // If the ack in msg is the one pair is waiting for, sends the 'B' frame.
      void checkAck(SerlinkRelayPair* pair);
  };

} // namespace SerLink

#endif /* SERLINK_RELAY_HPP_ */
