/*
 * uart2.c
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "uart2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>

/* huart2 itself lives in main.c -- CubeMX generates the definition and
   MX_USART2_UART_Init() configures it (115200 8N1, TX+RX). Defining it here
   too would be a duplicate symbol, and CubeMX would restore its copy on the
   next regeneration anyway. */

QueueHandle_t uart2Queue = NULL;

static StaticQueue_t uart2StaticQueue;
static uint8_t       uart2QueueStorageArea[UART2_QUEUE_LENGTH * sizeof(UartMessage_t)];

static volatile char     rx_byte;
static volatile char     rxBuffer[UART2__BUFFER_LEN];
static volatile uint16_t rxLen;

/* Staged here rather than on the stack: the ISR runs on the main stack, which
   the linker script sizes at only 0x400, and UartMessage_t is ~132 bytes. The
   ISR is the only writer, and the copy into the queue completes before it
   returns. */
static UartMessage_t txToQueueMsg;

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

// called by HAL_UART_IRQHandler, i.e., called in the interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART2)
    {
        if (rxLen < (UART2__BUFFER_LEN - 1))
        {
            rxBuffer[rxLen++] = rx_byte;

            if (rx_byte == '\n')
            {
                // end of frame
                txToQueueMsg.len  = rxLen;
                txToQueueMsg.type = UART_MSG_TYPE__FRAME_RX;
                memset(txToQueueMsg.data, 0, UART2__BUFFER_LEN);
                memcpy(txToQueueMsg.data, (const char*)rxBuffer, rxLen);

                // If the queue is full the frame is dropped.
                xQueueSendFromISR(uart2Queue, &txToQueueMsg, &xHigherPriorityTaskWoken);

                rxLen = 0;
            }
        }
        else
        {
            // no terminator within a buffer's worth - discard the partial frame
            rxLen = 0;
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1); // re-arm interrupt

        // If xHigherPriorityTaskWoken was set to true, we should yield.
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void uart2_init(void)
{
    /* The peripheral, its clock and the PD5/PD6 pins are already set up by
       MX_USART2_UART_Init() and HAL_UART_MspInit(). Do not re-init them here:
       PA2/PA3 (the usual USART2 pins) carry RMII_MDIO/ADC on this board. */

    memset((void*)rxBuffer, 0, UART2__BUFFER_LEN);
    rxLen = 0;

    /* Built before the interrupt is enabled, so the ISR can never reach a
       queue that does not exist yet. */
    uart2Queue = xQueueCreateStatic(UART2_QUEUE_LENGTH, sizeof(UartMessage_t),
                                    uart2QueueStorageArea, &uart2StaticQueue);
    configASSERT(uart2Queue != NULL);

    /* Priority 5 == configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, the highest
       priority from which FreeRTOS FromISR calls are legal. Matches ETH and
       the SDIO DMA streams. */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);  // start RX interrupt
}

bool uart2_frameRx(char* data, uint16_t* dataLen)
{
    UartMessage_t rxMsg;

    if (uart2Queue == NULL)
    {
        return false;   // uart2_init() has not run yet
    }

    if (xQueueReceive(uart2Queue, &rxMsg, 0) != pdTRUE)
    {
        return false;   // nothing waiting - non-blocking
    }

    memcpy(data, rxMsg.data, rxMsg.len);
    data[rxMsg.len] = '\0';     // frames end in '\n', not '\0'
    *dataLen = rxMsg.len;

    return true;
}

uint8_t uart2_writeBlocking(char* buffer)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)strlen(buffer), HAL_MAX_DELAY) != HAL_OK)
    {
        return 1;
    }
    return 0;
}
