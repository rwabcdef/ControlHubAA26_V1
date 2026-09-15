#include "SerLinkUartAdapter.hpp"
#include "uart2.h"
#include <string.h>

SerLinkUartAdapter::SerLinkUartAdapter()
: pRxFramebuffer(nullptr), rxFrameBufferLen(0), rxLen(0)
{
}

bool SerLinkUartAdapter::init(char* pRxBuffer, uint8_t rxBufferLen)
{
  this->pRxFramebuffer = pRxBuffer;
  this->rxFrameBufferLen = rxBufferLen;
  this->rxLen = 0;

  return uart2Queue != nullptr;
}

bool SerLinkUartAdapter::checkFrameRx()
{
  if((uart2Queue == nullptr) || (this->pRxFramebuffer == nullptr))
  {
    return false;
  }

  if(xQueueReceive(uart2Queue, &this->rxMsg, 0) != pdTRUE)
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

uint8_t SerLinkUartAdapter::getRxLenAndReset()
{
  return this->rxLen;
}

uint8_t SerLinkUartAdapter::write(const char* buffer)
{
  // uart2_writeBlocking() only reads buffer
  return (0 == uart2_writeBlocking((char*)buffer)) ? WRITE_STATUS_OK : WRITE_STATUS_ERROR;
}

bool SerLinkUartAdapter::getTxBusy()
{
  return false;
}
