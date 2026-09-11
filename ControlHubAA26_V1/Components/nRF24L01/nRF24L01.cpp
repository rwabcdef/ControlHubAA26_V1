/*
 * nRF24L01.cpp
 */

#include "nRF24L01.hpp"
#include "spi5.h"
#include "task.h"
#include <string.h>

namespace
{
  /* Set by nRF24L01::enableCycleCounter() once it has proved the DWT cycle
     counter actually runs. It normally does on a Cortex-M4 whether or not a
     debugger is attached, but some tools leave the trace block powered down
     -- and a delayUs() spinning on a counter that never advances would hang
     the task forever, so the fallback below is not optional. */
  bool cycleCounterOk = false;

  /* Millisecond sleep. Yields if the scheduler is running so a 100 ms wait
     does not stall every other task; busy-waits on the HAL tick otherwise,
     which keeps init() usable before osKernelStart(). */
  void delayMs(uint32_t ms)
  {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
      vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
      HAL_Delay(ms);
    }
  }
}

nRF24L01::nRF24L01(GPIO_TypeDef* cePort, uint16_t cePin,
                   GPIO_TypeDef* csnPort, uint16_t csnPin)
{
  this->cePort       = cePort;
  this->cePin        = cePin;
  this->csnPort      = csnPort;
  this->csnPin       = csnPin;
  this->mode         = Mode::Polled;
  this->payloadLen   = MAX_PAYLOAD_LEN;
  this->config       = CONFIG_EN_CRC | CONFIG_CRCO;
  this->listening    = false;
  this->irqSemaphore = nullptr;
  this->pipe0RxValid = false;

  memset(this->rxBuffer, 0, MAX_PAYLOAD_LEN);
  memset(this->pipe0RxAddress, 0, ADDRESS_LEN);
}

/* -------------------------------------------------------------------------- */
/* Timing                                                                     */

void nRF24L01::enableCycleCounter()
{
  uint32_t before;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  before = DWT->CYCCNT;
  __NOP();
  __NOP();
  __NOP();
  cycleCounterOk = (DWT->CYCCNT != before);
}

void nRF24L01::delayUs(uint32_t us)
{
  const uint32_t cyclesPerUs = SystemCoreClock / 1000000U;

  if (cycleCounterOk)
  {
    /* Unsigned subtraction, so a CYCCNT wrap mid-wait is harmless. */
    uint32_t start  = DWT->CYCCNT;
    uint32_t target = us * cyclesPerUs;

    while ((DWT->CYCCNT - start) < target)
    {
      /* spin */
    }
  }
  else
  {
    /* Roughly three cycles per iteration. Only ever over-delays, which for
       every use here (settling times with a minimum, not a maximum) is
       harmless. */
    volatile uint32_t loops = (us * cyclesPerUs) / 3U;

    while (loops > 0U)
    {
      loops--;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* Device pins                                                                */

void nRF24L01::ceHigh()
{
  HAL_GPIO_WritePin(this->cePort, this->cePin, GPIO_PIN_SET);
}

void nRF24L01::ceLow()
{
  HAL_GPIO_WritePin(this->cePort, this->cePin, GPIO_PIN_RESET);
}

/* CSN is this driver's own chip select, not the spi5 layer's: SPI5 is a
   shared bus, and only the device driver knows that a transaction runs from
   the command byte through to the end of its payload. */
void nRF24L01::csnLow()
{
  HAL_GPIO_WritePin(this->csnPort, this->csnPin, GPIO_PIN_RESET);
}

void nRF24L01::csnHigh()
{
  HAL_GPIO_WritePin(this->csnPort, this->csnPin, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* Raw SPI access                                                             */

void nRF24L01::sendCommand(uint8_t command)
{
  char cmd = (char)command;
  char status;

  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  this->csnHigh();
}

uint8_t nRF24L01::getStatus()
{
  char cmd = (char)CMD_NOP;
  char status = 0;

  /* Every command returns STATUS on MISO while the command byte goes out,
     so a NOP is the cheapest way to sample it. */
  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  this->csnHigh();

  return (uint8_t)status;
}

uint8_t nRF24L01::readRegister(uint8_t reg, uint8_t* status)
{
  char tx[2];
  char rx[2] = { 0, 0 };

  tx[0] = (char)(CMD_R_REGISTER | (reg & REGISTER_MASK));
  tx[1] = (char)CMD_NOP;

  this->csnLow();
  spi5_transfer(tx, rx, 2);
  this->csnHigh();

  /* STATUS comes back on MISO alongside the command byte, so callers that
     need both get it without a second transaction. */
  if (status != nullptr)
  {
    *status = (uint8_t)rx[0];
  }

  return (uint8_t)rx[1];
}

void nRF24L01::writeRegister(uint8_t reg, uint8_t value)
{
  char tx[2];

  tx[0] = (char)(CMD_W_REGISTER | (reg & REGISTER_MASK));
  tx[1] = (char)value;

  this->csnLow();
  spi5_write(tx, 2);
  this->csnHigh();
}

void nRF24L01::writeRegisterBuf(uint8_t reg, const uint8_t* data, uint8_t len)
{
  char cmd = (char)(CMD_W_REGISTER | (reg & REGISTER_MASK));
  char status;

  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  spi5_write((char*)const_cast<uint8_t*>(data), len);
  this->csnHigh();
}

void nRF24L01::readRegisterBuf(uint8_t reg, uint8_t* data, uint8_t len)
{
  char cmd = (char)(CMD_R_REGISTER | (reg & REGISTER_MASK));
  char status;

  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  spi5_read((char*)data, len);
  this->csnHigh();
}

void nRF24L01::writeConfig()
{
  this->writeRegister(REG_CONFIG, this->config);
}

void nRF24L01::flushRx()
{
  this->sendCommand(CMD_FLUSH_RX);
}

void nRF24L01::flushTx()
{
  this->sendCommand(CMD_FLUSH_TX);
}

/* -------------------------------------------------------------------------- */
/* Bring-up                                                                   */

bool nRF24L01::isPresent()
{
  /* RF_CH is a plain read/write register with no side effects, so a
     round-trip through it is a safe liveness probe. Restored afterwards. */
  uint8_t original = this->readRegister(REG_RF_CH);
  bool    present;

  this->writeRegister(REG_RF_CH, 0x5A);
  present = (this->readRegister(REG_RF_CH) == 0x5A);

  this->writeRegister(REG_RF_CH, original);

  return present;
}

bool nRF24L01::init(Mode mode)
{
  enableCycleCounter();

  this->mode      = mode;
  this->listening = false;

  /* Park the device lines before anything else touches the bus: CE low
     leaves the radio in standby rather than listening or transmitting, CSN
     high leaves it deselected so another slave could use SPI5. */
  this->ceLow();
  this->csnHigh();

  if (mode == Mode::Interrupt)
  {
    /* Created before the radio is powered up, so the first nINT edge can
       never reach a semaphore that does not exist yet. */
    this->irqSemaphore = xSemaphoreCreateBinaryStatic(&this->irqSemaphoreBuffer);
    configASSERT(this->irqSemaphore != nullptr);
  }

  /* Tpor: the device ignores SPI for up to 100 ms after VDD comes up. In
     practice the RTOS has been running for longer than that by the time a
     task calls init(), but the wait costs nothing once. */
  delayMs(100);

  /* 2-byte CRC, still powered down. In polled mode all three interrupt
     sources are masked so nINT never asserts. */
  this->config = CONFIG_EN_CRC | CONFIG_CRCO;
  if (mode == Mode::Polled)
  {
    this->config |= (CONFIG_MASK_RX_DR | CONFIG_MASK_TX_DS | CONFIG_MASK_MAX_RT);
  }
  this->writeConfig();
  delayUs(T_PD2STBY_US);

  if (!this->isPresent())
  {
    return false;   // nothing on the bus - do not pretend to be configured
  }

  this->writeRegister(REG_SETUP_AW, 0x03);   // 5-byte addresses

  this->setRetries(5, 15);                   // 1500us gap, 15 retransmits
  this->setDataRate(DataRate::Rate1Mbps);
  this->setTxPower(TxPower::Max0dBm);
  this->setChannel(76);
  this->setPayloadLen(MAX_PAYLOAD_LEN);
  this->setAutoAck(true);

  /* Fixed-width payloads: no dynamic payload length, no ack payloads. */
  this->writeRegister(REG_DYNPD, 0x00);
  this->writeRegister(REG_FEATURE, 0x00);

  /* Disable every RX pipe; openReadingPipe() and openWritingPipe() enable
     the ones actually in use. The reset default leaves pipes 0 and 1 live
     on address E7E7E7E7E7, and pipe 0 wins a tie -- so without this, a
     caller who opens pipe 1 on the default address sees traffic arrive on
     pipe 0 instead. */
  this->writeRegister(REG_EN_RXADDR, 0x00);

  this->flushRx();
  this->flushTx();
  this->writeRegister(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);

  /* Power up into TX standby (CE is low), ready for write() or for
     startListening() to flip PRIM_RX. */
  this->config |= CONFIG_PWR_UP;
  this->writeConfig();
  delayUs(T_PD2STBY_US);

  return true;
}

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */

void nRF24L01::setChannel(uint8_t channel)
{
  if (channel > 125)
  {
    channel = 125;
  }

  this->writeRegister(REG_RF_CH, channel);
}

void nRF24L01::setDataRate(DataRate rate)
{
  /* RF_DR_LOW (bit 5) and RF_DR_HIGH (bit 3) select the rate between them;
     everything else in RF_SETUP (notably RF_PWR) is preserved. */
  uint8_t rf = this->readRegister(REG_RF_SETUP);

  rf &= (uint8_t)~((1 << 5) | (1 << 3));

  switch (rate)
  {
    case DataRate::Rate2Mbps:
      rf |= (1 << 3);
      break;

    case DataRate::Rate250kbps:
      rf |= (1 << 5);
      break;

    case DataRate::Rate1Mbps:
    default:
      /* both bits clear */
      break;
  }

  this->writeRegister(REG_RF_SETUP, rf);
}

void nRF24L01::setTxPower(TxPower power)
{
  uint8_t rf = this->readRegister(REG_RF_SETUP);

  rf &= (uint8_t)~0x06;   // clear RF_PWR (bits 2:1)

  switch (power)
  {
    case TxPower::Min18dBm:  rf |= 0x00; break;
    case TxPower::Low12dBm:  rf |= 0x02; break;
    case TxPower::High6dBm:  rf |= 0x04; break;
    case TxPower::Max0dBm:
    default:                 rf |= 0x06; break;
  }

  this->writeRegister(REG_RF_SETUP, rf);
}

void nRF24L01::setPayloadLen(uint8_t len)
{
  uint8_t pipe;

  if (len == 0)
  {
    len = 1;
  }
  else if (len > MAX_PAYLOAD_LEN)
  {
    len = MAX_PAYLOAD_LEN;
  }

  this->payloadLen = len;

  for (pipe = 0; pipe < PIPE_COUNT; pipe++)
  {
    this->writeRegister((uint8_t)(REG_RX_PW_P0 + pipe), len);
  }
}

void nRF24L01::setAutoAck(bool enabled)
{
  this->writeRegister(REG_EN_AA, enabled ? 0x3F : 0x00);
}

void nRF24L01::setRetries(uint8_t delay250us, uint8_t count)
{
  if (delay250us > 15)
  {
    delay250us = 15;
  }
  if (count > 15)
  {
    count = 15;
  }

  this->writeRegister(REG_SETUP_RETR, (uint8_t)((delay250us << 4) | count));
}

void nRF24L01::openWritingPipe(const uint8_t* address)
{
  if (address == nullptr)
  {
    return;
  }

  this->writeRegisterBuf(REG_TX_ADDR, address, ADDRESS_LEN);

  /* Pipe 0 has to carry the same address for auto-ack: that is the pipe the
     ack comes back on, and it has to be enabled in EN_RXADDR to hear it. */
  this->writeRegisterBuf(REG_RX_ADDR_P0, address, ADDRESS_LEN);
  this->writeRegister(REG_RX_PW_P0, this->payloadLen);

  this->writeRegister(REG_EN_RXADDR,
                      (uint8_t)(this->readRegister(REG_EN_RXADDR) | 0x01));
}

void nRF24L01::openReadingPipe(uint8_t pipe, const uint8_t* address)
{
  uint8_t enabled;

  if ((pipe >= PIPE_COUNT) || (address == nullptr))
  {
    return;
  }

  if (pipe < 2)
  {
    this->writeRegisterBuf((uint8_t)(REG_RX_ADDR_P0 + pipe), address, ADDRESS_LEN);

    if (pipe == 0)
    {
      /* Kept so startListening() can undo openWritingPipe()'s overwrite.
         The register itself is not a reliable place to read it back from:
         between here and the next startListening() it may be holding the
         TX address instead. */
      memcpy(this->pipe0RxAddress, address, ADDRESS_LEN);
      this->pipe0RxValid = true;
    }
  }
  else
  {
    /* Pipes 2..5 are one byte wide; they inherit pipe 1's top four bytes. */
    this->writeRegister((uint8_t)(REG_RX_ADDR_P0 + pipe), address[0]);
  }

  this->writeRegister((uint8_t)(REG_RX_PW_P0 + pipe), this->payloadLen);

  enabled = this->readRegister(REG_EN_RXADDR);
  enabled |= (uint8_t)(1 << pipe);
  this->writeRegister(REG_EN_RXADDR, enabled);
}

/* -------------------------------------------------------------------------- */
/* Operation                                                                  */

void nRF24L01::startListening()
{
  uint8_t enabled = this->readRegister(REG_EN_RXADDR);

  /* Undo openWritingPipe(). It had to point pipe 0 at the TX address so the
     auto-ack could be heard, which destroys whatever RX address pipe 0 was
     using -- the classic "transmitting once silently kills receiving"
     failure. Restore it here, or close pipe 0 outright if it was never
     opened for reading, so a transmitter's own TX address does not sit
     there matching incoming packets. */
  if (this->pipe0RxValid)
  {
    this->writeRegisterBuf(REG_RX_ADDR_P0, this->pipe0RxAddress, ADDRESS_LEN);
    enabled |= 0x01;
  }
  else
  {
    enabled &= (uint8_t)~0x01;
  }
  this->writeRegister(REG_EN_RXADDR, enabled);

  this->config |= (CONFIG_PWR_UP | CONFIG_PRIM_RX);
  this->writeConfig();

  /* No flushRx() here. Anything already in the RX FIFO has been acked, so
     the transmitter will not resend it: flushing on every return to RX would
     silently discard whatever arrived just before a transmission. init()
     flushes once at bring-up. */
  this->writeRegister(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
  this->flushTx();

  this->ceHigh();
  delayUs(T_STBY2A_US);

  this->listening = true;
}

void nRF24L01::stopListening()
{
  this->ceLow();
  delayUs(T_STBY2A_US);

  this->config &= (uint8_t)~CONFIG_PRIM_RX;
  this->config |= CONFIG_PWR_UP;
  this->writeConfig();
  delayUs(T_STBY2A_US);

  this->listening = false;
}

bool nRF24L01::available(uint8_t* pipe)
{
  /* FIFO_STATUS.RX_EMPTY rather than STATUS.RX_DR: RX_DR is a latched
     interrupt flag, so it can be clear while packets are still queued (the
     FIFO holds three). RX_EMPTY always reflects what is actually there. */
  uint8_t status = 0;
  bool hasData = (this->readRegister(REG_FIFO_STATUS, &status) & FIFO_STATUS_RX_EMPTY) == 0;

  if (pipe != nullptr)
  {
    *pipe = hasData ? (uint8_t)((status >> 1) & 0x07) : PIPE_NONE;
  }

  return hasData;
}

bool nRF24L01::waitForData(uint32_t timeoutMs)
{
  if ((this->mode != Mode::Interrupt) || (this->irqSemaphore == nullptr))
  {
    return false;
  }

  return (xSemaphoreTake(this->irqSemaphore, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

uint8_t nRF24L01::read(uint8_t* data, uint8_t maxLen)
{
  char cmd = (char)CMD_R_RX_PAYLOAD;
  char status;
  uint8_t copyLen;

  if ((data == nullptr) || (maxLen == 0))
  {
    return 0;
  }

  if (!this->available(nullptr))
  {
    return 0;
  }

  /* Always clock out the full configured width, whatever the caller asked
     for: the device advances the FIFO by one slot per R_RX_PAYLOAD and a
     short read would leave the rest of the packet behind. */
  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  spi5_read((char*)this->rxBuffer, this->payloadLen);
  this->csnHigh();

  // Clear STATUS.RX_DR so the next packet can assert nINT.
  // At this point it is possible that the Rx FIFO is NOT empty, e.g.,
  // a second packet could have arrived immediately after the first, and before
  // the first was read.
  this->writeRegister(REG_STATUS, STATUS_RX_DR);

  copyLen = (maxLen < this->payloadLen) ? maxLen : this->payloadLen;
  memcpy(data, this->rxBuffer, copyLen);

  return copyLen;
}

bool nRF24L01::write(const uint8_t* data, uint8_t len, uint32_t timeoutMs)
{
  uint8_t txBuffer[MAX_PAYLOAD_LEN];
  char    cmd = (char)CMD_W_TX_PAYLOAD;
  char    status;
  uint32_t deadline;

  if ((data == nullptr) || (len == 0) || this->listening)
  {
    return false;   // write() needs TX standby - call stopListening() first
  }

  if (len > this->payloadLen)
  {
    len = this->payloadLen;
  }

  /* Fixed-width payloads: the receiver expects exactly payloadLen bytes, so
     a short message is zero-padded rather than sent short. */
  memset(txBuffer, 0, MAX_PAYLOAD_LEN);
  memcpy(txBuffer, data, len);

  this->writeRegister(REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);
  this->flushTx();

  this->csnLow();
  spi5_transfer(&cmd, &status, 1);
  spi5_write((char*)txBuffer, this->payloadLen);
  this->csnHigh();

  /* A CE pulse of at least 10us starts one transmission and returns the
     device to standby when it finishes. */
  this->ceHigh();
  delayUs(T_CE_PULSE_US);
  this->ceLow();

  deadline = HAL_GetTick() + timeoutMs;

  for (;;)
  {
    uint8_t s = this->getStatus();

    if ((s & STATUS_TX_DS) != 0)
    {
      this->writeRegister(REG_STATUS, STATUS_TX_DS);
      return true;
    }

    if ((s & STATUS_MAX_RT) != 0)
    {
      /* Retries exhausted. The payload stays in the TX FIFO until flushed,
         and MAX_RT blocks all further transmission until it is cleared. */
      this->writeRegister(REG_STATUS, STATUS_MAX_RT);
      this->flushTx();
      return false;
    }

    /* Signed comparison, so the 49-day HAL tick wrap cannot strand us. */
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      break;
    }
  }

  this->flushTx();

  return false;
}

void nRF24L01::onIrq()
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (this->irqSemaphore == nullptr)
  {
    return;   // polled mode, or init() has not run yet
  }

  /* No SPI here: this runs in the EXTI9_5 ISR. The task that wakes on the
     semaphore does the register reads. */
  xSemaphoreGiveFromISR(this->irqSemaphore, &higherPriorityTaskWoken);

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
