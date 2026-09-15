/*
 * SerLinkUartAdapter.hpp
 *
 * Implements SerLink::LinkInterface over uart2 (see uart2.h):
 *
 *   checkFrameRx() <-- uart2Queue (non-blocking) <-- uart2 ISR
 *   write()        --> uart2_writeBlocking()
 *
 * uart2Queue has a single consumer: while a Reader takes frames from it
 * directly (Reader::init(uart2Queue, ...)), do not also use this adapter.
 * Not used by ControlHubAA26's Reader & Writer yet (see LinkInterface.hpp).
 */

#ifndef SERLINK_UART_ADAPTER_HPP_
#define SERLINK_UART_ADAPTER_HPP_

#include "LinkInterface.hpp"
#include "main.h" // UartMessage_t

class SerLinkUartAdapter : public SerLink::LinkInterface
{
  public:
    // Touches no hardware, so fine for a static instance.
    SerLinkUartAdapter();

    // uart2_init() must already have been called (it creates uart2Queue).
    bool init(char* pRxBuffer, uint8_t rxBufferLen) override;

    // Non-blocking: takes one frame off uart2Queue, and copies it into the
    // external rx frame buffer. A frame too long for the buffer is dropped.
    bool checkFrameRx() override;

    uint8_t getRxLenAndReset() override;

    // Blocking: returns once the whole frame has been transmitted.
    uint8_t write(const char* buffer) override;

    // Always false: write() is blocking.
    bool getTxBusy() override;

  protected:
    char* pRxFramebuffer;     // external buffer
    uint8_t rxFrameBufferLen; // external buffer length
    uint8_t rxLen;
    UartMessage_t rxMsg;      // last message taken from uart2Queue
};

#endif /* SERLINK_UART_ADAPTER_HPP_ */
