#include "SerLinkRadioAdapter.hpp"
#include "uart2.h" // UART_MSG_TYPE__FRAME_RX
#include <string.h>

SerLinkRadioAdapter::SerLinkRadioAdapter(Radio* radio)
: radio(radio), pRxFramebuffer(nullptr), rxFrameBufferLen(0), rxLen(0)
{
}

bool SerLinkRadioAdapter::init(char* pRxBuffer, uint8_t rxBufferLen)
{
  this->pRxFramebuffer = pRxBuffer;
  this->rxFrameBufferLen = rxBufferLen;
  this->rxLen = 0;

  return (this->radio != nullptr) && (this->radio->rxDataQueue != nullptr);
}

bool SerLinkRadioAdapter::checkFrameRx()
{
  if((this->radio == nullptr) || (this->radio->rxDataQueue == nullptr) ||
    (this->pRxFramebuffer == nullptr))
  {
    return false;
  }

  if(xQueueReceive(this->radio->rxDataQueue, &this->rxMsg, 0) != pdTRUE)
  {
    return false; // nothing waiting
  }

  // Same check as Reader::checkUartFrameRx()
  if((this->rxMsg.type != UART_MSG_TYPE__FRAME_RX) || (this->rxMsg.len <= 1))
  {
    return false;
  }

  if(this->rxMsg.len > this->rxFrameBufferLen)
  {
    return false; // too long for the external buffer - dropped
  }

  memset(this->pRxFramebuffer, 0, this->rxFrameBufferLen);
  memcpy(this->pRxFramebuffer, this->rxMsg.data, this->rxMsg.len);
  this->rxLen = (uint8_t)this->rxMsg.len;
  return true;
}

uint8_t SerLinkRadioAdapter::getRxLenAndReset()
{
  return this->rxLen;
}

uint8_t SerLinkRadioAdapter::write(const char* buffer)
{
  if((this->radio == nullptr) || (this->radio->eventQueue == nullptr))
  {
    return WRITE_STATUS_ERROR;
  }

  // 1 = the queue stayed full (Frame::toString() output always fits)
  return (0 == RadioMsg::queueTxData(this->radio->eventQueue, buffer)) ? WRITE_STATUS_OK : WRITE_STATUS_BUSY;
}

bool SerLinkRadioAdapter::getTxBusy()
{
  return false;
}
