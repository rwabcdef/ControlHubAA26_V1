/*
 * Radio.hpp
 *
 * SerLink over the nRF24L01. A state machine that owns one nRF24L01 and
 * carries serialised SerLink frames in both directions, so a Writer/Reader
 * pair can use the radio in place of a uart:
 *
 *   Writer, Reader --TX_DATA--> eventQueue --> run() --> nRF24L01
 *   nINT (onIrq)   --RX_IRQ---> eventQueue --> run() <-- nRF24L01
 *   Reader <------------------ rxDataQueue <-- run()
 *
 * All SPI to the device happens in the one task that calls run(). Everything
 * else -- write(), startListening(), stopListening(), onIrq() -- only posts to
 * eventQueue, so the driver keeps its single owner.
 *
 * Over the air
 * ------------
 * A serialised frame (up to 77 chars, '\n' terminated) does not fit in one
 * 32-byte packet, so it is split across as many as it needs:
 *
 *   byte 0       flags: PACKET_FLAG_START on the first packet of a frame
 *   bytes 1..31  frame text; the last packet is zero-padded
 *
 * The receiver collects text from a START packet up to '\n', then posts the
 * frame to rxDataQueue -- the same newline framing uart2 uses. Auto-ack makes
 * each packet reliable and in order. If one still fails (MAX_RT) the sender
 * abandons the rest of the frame, and the far end discards the partial when
 * the next START arrives. SerLink's own ack timeout reports the loss.
 *
 * Both ends use the same address in both directions, so this is a two-node
 * link. Two ends transmitting at the same moment are each out of RX while the
 * other sends, and both frames fail.
 */

#ifndef RADIO_HPP_
#define RADIO_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "StateMachine.hpp"
#include "nRF24L01.hpp"
#include "RadioMsg.hpp"

#define RADIO__EVENT_QUEUE_LENGTH  5
#define RADIO__RXDATA_QUEUE_LENGTH 5

class Radio : public StateMachine
{
  public:
    // Link address used when init() is given none. Must match the far end.
    static const uint8_t DEFAULT_ADDRESS[nRF24L01::ADDRESS_LEN];

    // RX_IRQ from onIrq(); TX_DATA, START_LISTENING and STOP_LISTENING from
    // the public methods below, or from a Writer or Reader posting TX_DATA
    // directly. Item type RadioMsg. Valid after init().
    QueueHandle_t eventQueue;

    // Frames received over the air, one per item, as the same UartMessage_t
    // that uart2Queue carries -- so a Reader consumes it unchanged. Valid
    // after init().
    QueueHandle_t rxDataQueue;

    // Pins as for nRF24L01. Touches no hardware, so fine for a static
    // instance constructed before HAL_Init().
    Radio(GPIO_TypeDef* cePort, uint16_t cePin,
          GPIO_TypeDef* csnPort, uint16_t csnPin);

    // Creates the queues and enters INIT. Safe before the scheduler starts.
    // The device itself is brought up by run(), in the owning task, retrying
    // until it answers on SPI; the state machine then goes to IDLE.
    void init(const uint8_t* address = DEFAULT_ADDRESS);

    // Services one state. Call repeatedly from the one task that owns this
    // radio, after init(). Blocks until there is something to do.
    void run();

    // Queue a command for run(): IDLE -> RX, or RX -> IDLE. Applied in order
    // with any queued TX_DATA. Task context only. 0 if queued, 1 otherwise.
    uint8_t startListening();
    uint8_t stopListening();

    // Queues buffer, a NUL-terminated serialised frame, as TX_DATA -- the
    // same as a Writer posting to eventQueue itself. Task context only.
    // 0 if queued, 1 otherwise.
    uint8_t write(char* buffer);

    // ISR-safe. Call from HAL_GPIO_EXTI_Callback() for nRF24L01_nINT_Pin.
    void onIrq();

  private:
    static const uint8_t  PACKET_LEN        = nRF24L01::MAX_PAYLOAD_LEN;
    static const uint8_t  PACKET_HEADER_LEN = 1;
    static const uint8_t  PACKET_DATA_LEN   = PACKET_LEN - PACKET_HEADER_LEN;
    static const uint8_t  PACKET_FLAG_START = 0x01;
    static const uint8_t  CHANNEL           = 76;
    static const uint32_t INIT_RETRY_MS     = 1000;

    // rx() drains the RX FIFO after this long with no event at all, in case
    // an nINT edge was ever missed.
    static const uint32_t RX_BACKSTOP_MS    = 1000;

    nRF24L01 nrf;
    uint8_t  address[nRF24L01::ADDRESS_LEN];
    uint8_t  txReturnState;   // IDLE or RX: where tx() goes back to

    // Set by onIrq() once it has posted an RX_IRQ that run() has not acted on
    // yet; cleared before each FIFO drain. Collapses a burst of nINT edges --
    // TX_DS on every packet of a frame, say -- into one queued event, so they
    // cannot crowd TX_DATA out of eventQueue.
    volatile bool irqPending;

    RadioMsg      eventMsg;   // last message taken from eventQueue
    RadioMsg      irqMsg;     // pre-built RX_IRQ for onIrq() to post

    UartMessage_t rxFrame;    // frame being reassembled from packets
    bool          rxSynced;   // a START has been seen for the frame in rxFrame

    StaticQueue_t eventStaticQueue;
    uint8_t       eventQueueStorageArea[RADIO__EVENT_QUEUE_LENGTH * sizeof(RadioMsg)];

    StaticQueue_t rxDataStaticQueue;
    uint8_t       rxDataQueueStorageArea[RADIO__RXDATA_QUEUE_LENGTH * sizeof(UartMessage_t)];

    uint8_t initDevice();
    uint8_t idle();
    uint8_t rx();
    uint8_t tx();

    void    drainRxFifo();
    void    reassemble(const uint8_t* packet);
    void    resetRxFrame();
    uint8_t postCommand(RadioMsg::MsgType command);
};

#endif /* RADIO_HPP_ */
