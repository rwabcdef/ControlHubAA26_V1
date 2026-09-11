/*
 * RadioMsg.hpp
 *
 * Item type for Radio::eventQueue, and so also for the extTxOutQueue a
 * SerLink Writer or Reader posts serialised frames onto.
 */

#ifndef RADIOMSG_HPP_
#define RADIOMSG_HPP_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

// Longest NUL-terminated serialised SerLink frame a TX_DATA message carries.
// Frame::toString() writes up to Frame::MAX_FRAME_LEN (78) bytes including
// the NUL; Writer.cpp static_asserts that this is big enough.
#define RADIOMSG__FRAME_LEN_MAX 80

// How long queueTxData() waits for space on a full queue.
#define RADIOMSG__TX_QUEUE_TIMEOUT_MS 200

class RadioMsg
{
  public:
    enum MsgType : uint8_t
    {
      RX_IRQ          = 1,  // nINT asserted. Carries no data.
      TX_DATA         = 2,  // buffer holds a serialised SerLink frame to transmit
      START_LISTENING = 3,  // command: IDLE -> RX
      STOP_LISTENING  = 4   // command: RX -> IDLE
    };

    MsgType msgType;
    char    buffer[RADIOMSG__FRAME_LEN_MAX];   // TX_DATA only: NUL-terminated

    // Copies frame into a TX_DATA message and posts it to queue, waiting up
    // to RADIOMSG__TX_QUEUE_TIMEOUT_MS for space. Task context only. Returns
    // 0 if queued; 1 if queue is null, the frame does not fit, or the queue
    // stayed full.
    static uint8_t queueTxData(QueueHandle_t queue, const char* frame);
};

#endif /* RADIOMSG_HPP_ */
