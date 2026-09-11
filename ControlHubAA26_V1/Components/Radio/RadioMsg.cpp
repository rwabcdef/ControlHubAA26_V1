/*
 * RadioMsg.cpp
 */

#include "RadioMsg.hpp"
#include <string.h>

uint8_t RadioMsg::queueTxData(QueueHandle_t queue, const char* frame)
{
  RadioMsg msg;
  size_t   len;

  if((queue == nullptr) || (frame == nullptr))
  {
    return 1;
  }

  /* Reject rather than truncate: a frame cut short loses its '\n' and can
     never be parsed at the far end. */
  len = strnlen(frame, RADIOMSG__FRAME_LEN_MAX);
  if(len >= RADIOMSG__FRAME_LEN_MAX)
  {
    return 1;
  }

  msg.msgType = RadioMsg::TX_DATA;
  memset(msg.buffer, 0, RADIOMSG__FRAME_LEN_MAX);
  memcpy(msg.buffer, frame, len);

  return (xQueueSend(queue, &msg, pdMS_TO_TICKS(RADIOMSG__TX_QUEUE_TIMEOUT_MS)) == pdTRUE) ? 0 : 1;
}
