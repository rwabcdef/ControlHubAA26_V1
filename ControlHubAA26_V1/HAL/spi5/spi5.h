/*
 * spi5.h
 *
 * Basic CPU-driven (blocking) SPI service for SPI5.
 *
 * Every call here busy-waits inside the HAL until the transfer completes --
 * no interrupts, no DMA. Transfers are short (an nRF24L01 command is at most
 * 33 bytes, ~63us at 5.25 MHz) so this is cheap enough to do from a task.
 *
 * Slave select
 * ------------
 * Deliberately not handled here. NSS is SPI_NSS_SOFT, and each slave's chip
 * select is that device driver's business -- SPI5 may end up carrying more
 * than one. A driver owns its own CS pin and brackets a whole transaction
 * with it, because a device typically needs CS held across a command byte
 * plus its payload:
 *
 *     csnLow();                          // in the device driver
 *     spi5_transfer(&cmd, &status, 1);
 *     spi5_read(payload, 32);
 *     csnHigh();
 *
 * Bus speed
 * ---------
 * CubeMX leaves SPI5 at SPI_BAUDRATEPRESCALER_2, which on this clock tree
 * (SYSCLK 168 MHz, APB2 = HCLK/2 = 84 MHz) is 42 MHz. spi5_init() re-applies
 * the prescaler as SPI5_BAUDRATE_PRESCALER (/16 = 5.25 MHz) and re-inits the
 * peripheral, so the setting survives a CubeMX regeneration of main.c.
 * Change it there, not in main.c.
 *
 * One speed serves the whole bus, so it has to suit the slowest device on
 * it. Today that is the nRF24L01, whose ceiling is 10 MHz; adding a slower
 * part means lowering SPI5_BAUDRATE_PRESCALER to match.
 *
 * Mode 0 (CPOL low, CPHA 1-edge, MSB first, 8-bit) is what CubeMX generates
 * and what the nRF24L01 wants -- left alone. A device needing a different
 * mode would have to reconfigure the peripheral around its own transfers.
 *
 * Not thread-safe: there is no bus mutex. If more than one task ever drives
 * SPI5, serialise them outside this layer.
 */

#ifndef SPI5_H_
#define SPI5_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Return codes, matching uart2_writeBlocking()'s 0 == success convention. */
#define SPI5_OK     0
#define SPI5_ERROR  1

/* Per-transfer HAL timeout. Generous: a 33-byte transfer is ~63us. */
#define SPI5_TIMEOUT_MS 100

/* /16 of APB2 (84 MHz) = 5.25 MHz. The nRF24L01 tolerates up to 10 MHz;
   /8 would be 10.5 MHz, marginally over, so /16 is the next safe step. */
#define SPI5_BAUDRATE_PRESCALER SPI_BAUDRATEPRESCALER_16

/* Held on MOSI while clocking data in. 0xFF is a harmless NOP to most
   SPI devices, the nRF24L01 included. */
#define SPI5_DUMMY_BYTE 0xFF

#ifdef __cplusplus
extern "C" {
#endif

/* Defined by CubeMX in main.c and configured by MX_SPI5_Init(). */
extern SPI_HandleTypeDef hspi5;

/* Applies the post-CubeMX setup: drops the bus to SPI5_BAUDRATE_PRESCALER.
   Call once, after MX_SPI5_Init(); safe to call before the scheduler
   starts. Returns SPI5_OK, or SPI5_ERROR if HAL_SPI_Init() rejected the
   new prescaler.

   Touches no chip selects -- each device driver parks its own. */
uint8_t spi5_init(void);

/* Blocking write - returns once len bytes have been clocked out. The bytes
   clocked back in on MISO are discarded. */
uint8_t spi5_write(char* data, uint16_t len);

/* Blocking read - clocks len bytes in, holding MOSI at SPI5_DUMMY_BYTE. */
uint8_t spi5_read(char* data, uint16_t len);

/* Blocking full-duplex exchange of len bytes. txData and rxData may not be
   NULL and may not overlap; use spi5_write()/spi5_read() for half-duplex. */
uint8_t spi5_transfer(char* txData, char* rxData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SPI5_H_ */
