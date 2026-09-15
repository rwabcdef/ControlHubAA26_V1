/*
 * LinkInterface.hpp
 *
 * Interface to the (newline framed) transport below a SerLink Reader &
 * Writer, e.g. uart2 or radio1, so a Reader & Writer need not know which
 * driver they are using:
 *
 *   SerLinkUartAdapter  uart2Adapter;                // uart2
 *   SerLinkRadioAdapter radio1Adapter(&radio1);      // nRF24L01 (Radio)
 *
 * Note: ControlHubAA26's Reader & Writer do not use this interface yet - they
 * take their queues (uart2Queue, Radio::rxDataQueue / eventQueue) directly.
 */

#ifndef LINK_INTERFACE_HPP_
#define LINK_INTERFACE_HPP_

#include <stdint.h>

namespace SerLink
{

class LinkInterface
{
  public:
    // write() return codes
    static const uint8_t WRITE_STATUS_OK = 1;
    static const uint8_t WRITE_STATUS_BUSY = 2;
    static const uint8_t WRITE_STATUS_ERROR = 3;

    // Initialises the link, and sets the external rx frame buffer.
    // Returns false if the link could not be opened.
    virtual bool init(char* pRxBuffer, uint8_t rxBufferLen) = 0;
    // Returns true if a frame has been received (and copied into the
    // external rx frame buffer).
    virtual bool checkFrameRx() = 0;
    // Gets received frame length (and resets rx flag).
    virtual uint8_t getRxLenAndReset() = 0;
    // Writes a NUL terminated, '\n' terminated frame. Returns WRITE_STATUS_*.
    virtual uint8_t write(const char* buffer) = 0;
    // Returns whether or not link tx is busy.
    virtual bool getTxBusy() = 0;

  protected:
    // Not virtual: implementations are never deleted through this interface,
    // and a virtual destructor would pull in operator delete.
    ~LinkInterface() = default;
};

} // end namespace SerLink

#endif /* LINK_INTERFACE_HPP_ */
