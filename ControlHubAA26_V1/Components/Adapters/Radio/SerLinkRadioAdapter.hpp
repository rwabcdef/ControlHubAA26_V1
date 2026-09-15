/*
 * SerLinkRadioAdapter.hpp
 *
 * Implements SerLink::LinkInterface over the nRF24L01 (see Radio.hpp):
 *
 *   checkFrameRx() <-- Radio::rxDataQueue (non-blocking) <-- Radio::run()
 *   write()        --> Radio::eventQueue (TX_DATA)      --> Radio::run()
 *
 * Radio::run() is still called by the radio's own task. Radio::rxDataQueue
 * has a single consumer: while a Reader takes frames from it directly
 * (Reader::init(radio1.rxDataQueue, ...)), do not also use this adapter.
 * Not used by ControlHubAA26's Reader & Writer yet (see LinkInterface.hpp).
 */

#ifndef SERLINK_RADIO_ADAPTER_HPP_
#define SERLINK_RADIO_ADAPTER_HPP_

#include "LinkInterface.hpp"
#include "Radio.hpp"

class SerLinkRadioAdapter : public SerLink::LinkInterface
{
  public:
    // Touches no hardware, so fine for a static instance.
    SerLinkRadioAdapter(Radio* radio);

    // radio->init() must already have been called (it creates the queues).
    bool init(char* pRxBuffer, uint8_t rxBufferLen) override;

    // Non-blocking: takes one frame off Radio::rxDataQueue, and copies it into
    // the external rx frame buffer. A frame too long for the buffer is dropped.
    bool checkFrameRx() override;

    uint8_t getRxLenAndReset() override;

    // Queues the frame for Radio::run() to transmit, waiting up to
    // RADIOMSG__TX_QUEUE_TIMEOUT_MS for space on Radio::eventQueue.
    uint8_t write(const char* buffer) override;

    // Always false: frames are queued to the radio, which gives no tx busy
    // indication.
    bool getTxBusy() override;

  protected:
    Radio* radio;
    char* pRxFramebuffer;     // external buffer
    uint8_t rxFrameBufferLen; // external buffer length
    uint8_t rxLen;
    UartMessage_t rxMsg;      // last message taken from Radio::rxDataQueue
};

#endif /* SERLINK_RADIO_ADAPTER_HPP_ */
