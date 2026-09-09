/*
 * Writer.hpp
 *
 *  Created on: 12 Jul 2023
 *      Author: rw123
 */

 #ifndef WRITER_WRITER_HPP_
 #define WRITER_WRITER_HPP_

 #include <stdint.h>
 #include "Writer_config.hpp"
 #include "StateMachine.hpp"
 #include "Frame.hpp"
 #include "FreeRTOS.h"
 #include "queue.h"


 namespace SerLink
 {
 class Writer : public StateMachine
 {
 private:
   static const uint8_t TX_QUEUE_LENGTH = 5;
   static const uint8_t ACK_QUEUE_LENGTH = 2;

   // Big enough for the longest string Frame::toString() can ever produce.
   static const uint16_t TX_BUFFER_LEN = Frame::LEN_HEADER + Frame::MAX_DATALEN + 2;

   uint8_t id;
   uint8_t status;

   // Frame currently in flight (copied in from txQueue by idle(), read back
   // by ackWait() to check the ack's protocol matches).
   Frame txFrame;

   char txBuffer[Frame::MAX_FRAME_LEN];

   StaticQueue_t txStaticQueue;
   uint8_t txQueueStorageArea[TX_QUEUE_LENGTH * sizeof(Frame)];
   QueueHandle_t txQueue;

   StaticQueue_t ackStaticQueue;
   uint8_t ackQueueStorageArea[ACK_QUEUE_LENGTH * sizeof(Frame)];
   QueueHandle_t ackQueue;

   uint8_t idle();
   uint8_t ackWait();

 public:

   static const uint8_t STATUS_IDLE = 20;
   static const uint8_t STATUS_BUSY = 21;
   static const uint8_t STATUS_OK = 50;
   static const uint8_t STATUS_TIMEOUT = 51;
   static const uint8_t STATUS_PROTOCOL_ERROR = 52;

   Writer(uint8_t id);
   void init();
   void run();

   // Used to send frame. Non-blocking: returns 1 (rather than blocking the
   // caller) if txQueue is full.
   uint8_t sendFrame(Frame* frame);

   uint8_t getStatus();

   // Returns send status with respect to the specified protocol
   uint8_t getStatusProtocol(char* protocol);

   void getStatusStr(char* str);

   // Called by a Reader to pass an ack frame to the Writer. Non-blocking:
   // the ack is silently dropped if ackQueue is full.
   void setAckFrame(Frame* frame);

   uint8_t uartWrite(char* buffer);
 };

 } // end namespace SerLink

 #endif /* WRITER_WRITER_HPP_ */
