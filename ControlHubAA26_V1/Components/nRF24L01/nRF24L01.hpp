/*
 * nRF24L01.hpp
 *
 * Driver for the Nordic nRF24L01(+) 2.4 GHz transceiver, sitting on the
 * blocking spi5 layer (HAL/spi5).
 *
 * Wiring on this board (NUCLEO-F439ZI, all pins fixed by CubeMX):
 *   PF7  SPI5_SCK     PF10 CSN  (nRF24L01_SS_*)
 *   PF8  SPI5_MISO    PF6  CE   (nRF24L01_CE_*)
 *   PF9  SPI5_MOSI    PF5  nINT (nRF24L01_nINT_*, EXTI9_5, falling edge)
 *
 * All three device pins are passed to the constructor and driven here. The
 * spi5 layer handles only SCK/MISO/MOSI: it is a shared bus that may pick up
 * further slaves, so chip select belongs to the device driver, which is also
 * the only thing that knows how long CSN has to stay asserted (here, across
 * a command byte plus its payload).
 *
 * Polled vs interrupt
 * -------------------
 * Both modes use the same read/write API. The difference is only in how a
 * task learns that a packet has landed:
 *
 *   Mode::Polled     -- call available() as often as you like. The three IRQ
 *                       sources are masked in CONFIG, so nINT stays idle.
 *
 *   Mode::Interrupt  -- call waitForData(timeoutMs), which blocks on a
 *                       semaphore given by onIrq(). Wire it up by calling
 *                       onIrq() from HAL_GPIO_EXTI_Callback() when GPIO_Pin
 *                       is nRF24L01_nINT_Pin.
 *
 * onIrq() touches no SPI -- it only gives the semaphore. Reading STATUS from
 * an ISR would mean a blocking bus transfer at interrupt time, and the RX
 * FIFO holds three packets, so there is nothing to lose by deferring the
 * whole transaction to the task.
 *
 * Everything else (available(), read(), write(), all the setters) performs
 * blocking SPI and must be called from a task, never from an ISR.
 *
 * Not thread-safe: one owning task per instance.
 *
 * Fixed-width payloads only -- setPayloadLen() applies to both ends. Dynamic
 * payload length (DPL) is not enabled.
 */

#ifndef NRF24L01_HPP_
#define NRF24L01_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

class nRF24L01
{
  public:
    static const uint8_t MAX_PAYLOAD_LEN = 32;
    static const uint8_t ADDRESS_LEN     = 5;
    static const uint8_t PIPE_COUNT      = 6;

    // STATUS.RX_P_NO reads 0x07 when the RX FIFO is empty.
    static const uint8_t PIPE_NONE = 0x07;

    // Default write() timeout. With the default setRetries(5, 15) a fully
    // failed transmission gives up after 16 transmissions and 15 retry
    // gaps: 16 * ~330us of air time plus 15 * 1500us, so roughly 28ms at
    // 1 Mbps with a 32-byte payload. Longer at 250kbps.
    static const uint32_t DEFAULT_TX_TIMEOUT_MS = 100;

    enum class Mode : uint8_t
    {
      Polled,     // nINT masked off; poll available()
      Interrupt   // nINT active; block in waitForData()
    };

    enum class DataRate : uint8_t
    {
      Rate1Mbps,
      Rate2Mbps,
      Rate250kbps   // nRF24L01+ only
    };

    enum class TxPower : uint8_t
    {
      Min18dBm,
      Low12dBm,
      High6dBm,
      Max0dBm
    };

    // ce* addresses the CE line (nRF24L01_CE_GPIO_Port / nRF24L01_CE_Pin),
    // csn* the active-low chip select (nRF24L01_SS_GPIO_Port /
    // nRF24L01_SS_Pin). The constructor only stores them: for a static
    // instance it runs before HAL_Init(), so it must not touch the HAL.
    nRF24L01(GPIO_TypeDef* cePort, uint16_t cePin,
             GPIO_TypeDef* csnPort, uint16_t csnPin);

    // Resets the device to a known configuration: 2-byte CRC, 5-byte
    // addresses, auto-ack on, 1 Mbps, 0 dBm, channel 76, 32-byte payloads,
    // 15 retries at 1500us. Powers the radio up and leaves it in TX standby
    // (CE low) -- call startListening() to receive.
    //
    // Must be called from a task, after spi5_init(). In Mode::Interrupt it
    // also creates the semaphore that waitForData() blocks on, so it must
    // run before any nINT edge can arrive.
    //
    // Returns false if the device does not answer on SPI (see isPresent()).
    bool init(Mode mode = Mode::Polled);

    // Round-trips a value through RF_CH. False means nothing is responding
    // -- wiring, power, or the bus is misconfigured. This cannot tell an
    // nRF24L01 from an nRF24L01+.
    bool isPresent();

    /* ---- configuration: call after init(), before startListening() ---- */

    void setChannel(uint8_t channel);              // 0..125
    void setDataRate(DataRate rate);
    void setTxPower(TxPower power);
    void setPayloadLen(uint8_t len);               // 1..32, both ends
    void setAutoAck(bool enabled);

    // delay250us: retransmit gap, in units of 250us minus one (0 = 250us,
    // 15 = 4000us). count: 0..15 retransmits, 0 disables.
    void setRetries(uint8_t delay250us, uint8_t count);

    // address is ADDRESS_LEN bytes, LSByte first, as the datasheet has it.
    // openWritingPipe() also points pipe 0 at the same address and enables
    // it, which is what auto-ack needs in order to hear the ack come back.
    //
    // That overwrites whatever RX address pipe 0 was using. startListening()
    // puts it back (see openReadingPipe), so a bidirectional state machine
    // does not have to save and restore it around every transmission.
    void openWritingPipe(const uint8_t* address);

    // pipe 0..5, enabled as a side effect. init() leaves every RX pipe
    // disabled, so only the pipes opened here are live. Pipes 2..5 share
    // pipe 1's upper four bytes, so for those only address[0] is used --
    // exactly as the datasheet describes.
    //
    // A pipe 0 address is also cached, because openWritingPipe() has to
    // clobber the real register to receive acks; startListening() restores
    // it from the cache. Pipe 0 is closed entirely by startListening() if
    // it was never opened for reading, so a transmitter's own TX address
    // cannot go on quietly matching received packets.
    void openReadingPipe(uint8_t pipe, const uint8_t* address);

    /* ---------------------------- operation ---------------------------- */

    // Enters RX mode and starts listening (CE high). Leaves the RX FIFO
    // alone, so packets received before a stopListening() / write()
    // excursion are still there to read afterwards.
    void startListening();

    // Returns to TX standby (CE low). Required before write().
    void stopListening();

    // True if a packet is waiting in the RX FIFO (FIFO_STATUS.RX_EMPTY clear).
    // Used in both modes: poll it directly in Mode::Polled, or call it after
    // waitForData() wakes in Mode::Interrupt. If pipe is non-null it receives
    // the originating pipe number (STATUS.RX_P_NO), or PIPE_NONE when there
    // is no packet. Non-blocking, but performs one SPI transfer, so task
    // context only.
    bool available(uint8_t* pipe = nullptr);

    // Interrupt mode: blocks until onIrq() reports an nINT edge, or the
    // timeout expires. Returns true if woken by an edge -- the caller still
    // calls available()/read() to find out what actually arrived, since
    // nINT is also raised by TX_DS and MAX_RT. Returns false immediately in
    // Mode::Polled.
    bool waitForData(uint32_t timeoutMs);

    // Copies one packet out of the RX FIFO. Returns the number of bytes
    // written into data (the payload length, clamped to maxLen), or 0 if
    // the FIFO was empty. The FIFO slot is always fully drained regardless
    // of maxLen, so a short buffer cannot desynchronise the device.
    uint8_t read(uint8_t* data, uint8_t maxLen);

    // Blocking send. Pads to the configured payload length if len is
    // shorter. Returns true once the packet is acknowledged (or simply
    // sent, if auto-ack is off), false on MAX_RT or timeout. Requires TX
    // standby -- call stopListening() first.
    bool write(const uint8_t* data, uint8_t len,
               uint32_t timeoutMs = DEFAULT_TX_TIMEOUT_MS);

    // ISR-safe. Call from HAL_GPIO_EXTI_Callback() for nRF24L01_nINT_Pin.
    // Performs no SPI: it only gives the semaphore waitForData() blocks on.
    void onIrq();

    /* --------------------------- diagnostics --------------------------- */

    uint8_t getStatus();
    uint8_t readRegister(uint8_t reg, uint8_t* status = nullptr);
    void    writeRegister(uint8_t reg, uint8_t value);
    void    flushRx();
    void    flushTx();

    bool    isListening() const { return this->listening; }
    uint8_t getPayloadLen() const { return this->payloadLen; }

    /* ------------------------ registers and bits ----------------------- */

    static const uint8_t REG_CONFIG      = 0x00;
    static const uint8_t REG_EN_AA       = 0x01;
    static const uint8_t REG_EN_RXADDR   = 0x02;
    static const uint8_t REG_SETUP_AW    = 0x03;
    static const uint8_t REG_SETUP_RETR  = 0x04;
    static const uint8_t REG_RF_CH       = 0x05;
    static const uint8_t REG_RF_SETUP    = 0x06;
    static const uint8_t REG_STATUS      = 0x07;
    static const uint8_t REG_OBSERVE_TX  = 0x08;
    static const uint8_t REG_RPD         = 0x09;
    static const uint8_t REG_RX_ADDR_P0  = 0x0A;   // .. P5 at 0x0F
    static const uint8_t REG_TX_ADDR     = 0x10;
    static const uint8_t REG_RX_PW_P0    = 0x11;   // .. P5 at 0x16
    static const uint8_t REG_FIFO_STATUS = 0x17;
    static const uint8_t REG_DYNPD       = 0x1C;
    static const uint8_t REG_FEATURE     = 0x1D;

    static const uint8_t CONFIG_MASK_RX_DR  = (1 << 6);
    static const uint8_t CONFIG_MASK_TX_DS  = (1 << 5);
    static const uint8_t CONFIG_MASK_MAX_RT = (1 << 4);
    static const uint8_t CONFIG_EN_CRC      = (1 << 3);
    static const uint8_t CONFIG_CRCO        = (1 << 2);   // 1 = 2-byte CRC
    static const uint8_t CONFIG_PWR_UP      = (1 << 1);
    static const uint8_t CONFIG_PRIM_RX     = (1 << 0);

    static const uint8_t STATUS_RX_DR   = (1 << 6);
    static const uint8_t STATUS_TX_DS   = (1 << 5);
    static const uint8_t STATUS_MAX_RT  = (1 << 4);
    static const uint8_t STATUS_TX_FULL = (1 << 0);

    static const uint8_t FIFO_STATUS_RX_EMPTY = (1 << 0);

  private:
    /* SPI commands */
    static const uint8_t CMD_R_REGISTER   = 0x00;   // | reg
    static const uint8_t CMD_W_REGISTER   = 0x20;   // | reg
    static const uint8_t CMD_R_RX_PAYLOAD = 0x61;
    static const uint8_t CMD_W_TX_PAYLOAD = 0xA0;
    static const uint8_t CMD_FLUSH_TX     = 0xE1;
    static const uint8_t CMD_FLUSH_RX     = 0xE2;
    static const uint8_t CMD_NOP          = 0xFF;

    static const uint8_t REGISTER_MASK = 0x1F;

    /* Datasheet timings (us) */
    static const uint32_t T_PD2STBY_US  = 5000;  // power down -> standby
    static const uint32_t T_STBY2A_US   = 130;   // standby -> RX/TX active
    static const uint32_t T_CE_PULSE_US = 15;    // CE high to start a TX (min 10)

    GPIO_TypeDef* cePort;
    uint16_t      cePin;
    GPIO_TypeDef* csnPort;
    uint16_t      csnPin;

    Mode     mode;
    uint8_t  payloadLen;
    uint8_t  config;        // shadow of REG_CONFIG, so PWR_UP/PRIM_RX edits
                            // do not disturb the CRC and mask bits
    bool     listening;

    // read() always drains a full payload from the FIFO, even when the
    // caller's buffer is shorter, so it needs somewhere to put the rest.
    uint8_t  rxBuffer[MAX_PAYLOAD_LEN];

    // Pipe 0's RX address as the application set it, kept because
    // openWritingPipe() has to overwrite the hardware register with the TX
    // address for auto-ack. pipe0RxValid is false until openReadingPipe(0,
    // ...) is called, which is how startListening() tells "restore it" from
    // "this pipe is not for receiving at all".
    uint8_t  pipe0RxAddress[ADDRESS_LEN];
    bool     pipe0RxValid;

    SemaphoreHandle_t irqSemaphore;
    StaticSemaphore_t irqSemaphoreBuffer;

    void ceHigh();
    void ceLow();

    // Chip select, active low. Asserted for a whole transaction, not per
    // byte -- the device latches a command on the falling edge and treats
    // everything up to the rising edge as that command's payload.
    void csnLow();
    void csnHigh();

    void sendCommand(uint8_t command);
    void writeRegisterBuf(uint8_t reg, const uint8_t* data, uint8_t len);
    void readRegisterBuf(uint8_t reg, uint8_t* data, uint8_t len);
    void writeConfig();

    // Busy-wait on the Cortex-M4 cycle counter. Used for the sub-millisecond
    // state transitions above, which are far shorter than a scheduler tick.
    static void enableCycleCounter();
    static void delayUs(uint32_t us);
};

#endif /* NRF24L01_HPP_ */
