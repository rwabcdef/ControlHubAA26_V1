/*
 * uart2.h
 *
 * Basic interrupt-driven UART service for USART2.
 *
 * RX runs byte-by-byte under the USART2 interrupt, appending into a linear
 * buffer until a '\n' arrives. The completed frame is posted to uart2Queue as
 * a UartMessage_t. TX is blocking.
 *
 * Consumers take frames off uart2Queue directly -- SerLink's Reader does this,
 * via Reader::init(uart2Queue, ...) -- or call uart2_frameRx() for a simple
 * non-blocking read. Both drain the same queue, so use one or the other.
 *
 * UartMessage_t, UART2__BUFFER_LEN and UART2_QUEUE_LENGTH are declared in
 * main.h, because SerLink's Reader.hpp needs the type and includes main.h.
 *
 * On this board USART2 is on PD5 (TX) / PD6 (RX) -- the ST Zio connector,
 * NOT the ST-LINK virtual COM port. The VCP is USART3 on PD8/PD9.
 */

#ifndef UART2_H_
#define UART2_H_

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

#define UART_MSG_TYPE__FRAME_RX 1

#ifdef __cplusplus
extern "C" {
#endif

/* Defined by CubeMX in main.c and initialised by MX_USART2_UART_Init(). */
extern UART_HandleTypeDef huart2;

/* Receive queue, created by uart2_init(). Carries UartMessage_t. */
extern QueueHandle_t uart2Queue;

/* Creates the receive queue and starts the RX interrupt stream. Call once,
   after MX_USART2_UART_Init(): the peripheral, its clock and PD5/PD6 are
   already configured by CubeMX, so this only builds the queue, sets up the
   NVIC and arms the first receive. Safe to call before the scheduler starts. */
void uart2_init(void);

// Non-blocking bare metal read.
bool uart2_frameRx(char* data, uint16_t* dataLen);

// Blocking write - returns once the whole buffer has been transmitted.
uint8_t uart2_writeBlocking(char* buffer);

#ifdef __cplusplus
}
#endif

#endif /* UART2_H_ */
