/*
 * spi5.c
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "spi5.h"

/* hspi5 itself lives in main.c -- CubeMX generates the definition and
   MX_SPI5_Init() configures it (master, mode 0, 8-bit, MSB first, soft NSS).
   Defining it here too would be a duplicate symbol, and CubeMX would restore
   its copy on the next regeneration anyway. */

/* Clocked out on MOSI by spi5_read(). 32 bytes is a compromise: big enough
   that a whole nRF24L01 RX payload reads in one HAL call, small enough not
   to matter. Longer reads simply loop. Not const: HAL_SPI_TransmitReceive()
   takes a non-const pTxData, and casting the qualifier away would be worse
   than 32 bytes of RAM. */
#define SPI5_READ_CHUNK 32
static uint8_t spi5DummyTx[SPI5_READ_CHUNK];

uint8_t spi5_init(void)
{
    uint16_t i;

    /* The peripheral, its clock and PF7/PF8/PF9 are already set up by
       MX_SPI5_Init() and HAL_SPI_MspInit(). Do not re-init them here.
       Chip selects are not touched at all -- each device driver owns and
       parks its own. */

    for (i = 0; i < SPI5_READ_CHUNK; i++)
    {
        spi5DummyTx[i] = SPI5_DUMMY_BYTE;
    }

    /* Drop 42 MHz -> 5.25 MHz. HAL_SPI_Init() disables the peripheral,
       rewrites CR1 and re-enables it, so this is a complete re-init rather
       than a poke at the BR field. */
    hspi5.Init.BaudRatePrescaler = SPI5_BAUDRATE_PRESCALER;
    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        return SPI5_ERROR;
    }

    return SPI5_OK;
}

uint8_t spi5_write(char* data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
    {
        return SPI5_ERROR;
    }

    if (HAL_SPI_Transmit(&hspi5, (uint8_t*)data, len, SPI5_TIMEOUT_MS) != HAL_OK)
    {
        return SPI5_ERROR;
    }

    return SPI5_OK;
}

uint8_t spi5_read(char* data, uint16_t len)
{
    uint16_t offset = 0;

    if ((data == NULL) || (len == 0))
    {
        return SPI5_ERROR;
    }

    /* HAL_SPI_Receive() would do this in one call, but in full-duplex
       master mode it leaves MOSI carrying whatever happens to be in the
       data register. Driving it explicitly keeps the line at a known NOP
       for the whole read. */
    while (offset < len)
    {
        uint16_t chunk = len - offset;
        if (chunk > SPI5_READ_CHUNK)
        {
            chunk = SPI5_READ_CHUNK;
        }

        if (HAL_SPI_TransmitReceive(&hspi5, spi5DummyTx, (uint8_t*)&data[offset],
                                    chunk, SPI5_TIMEOUT_MS) != HAL_OK)
        {
            return SPI5_ERROR;
        }

        offset += chunk;
    }

    return SPI5_OK;
}

uint8_t spi5_transfer(char* txData, char* rxData, uint16_t len)
{
    if ((txData == NULL) || (rxData == NULL) || (len == 0))
    {
        return SPI5_ERROR;
    }

    if (HAL_SPI_TransmitReceive(&hspi5, (uint8_t*)txData, (uint8_t*)rxData,
                                len, SPI5_TIMEOUT_MS) != HAL_OK)
    {
        return SPI5_ERROR;
    }

    return SPI5_OK;
}
