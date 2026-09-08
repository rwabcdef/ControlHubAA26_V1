/**
  ******************************************************************************
  * @file    app_main.h
  * @brief   C entry point into the C++ application layer.
  *
  * This is the single boundary between the CubeMX-generated C code and the C++
  * application. main.c knows only about App_Main(); everything above it can be
  * C++. The extern "C" block below gives App_Main C linkage in both languages,
  * so main.c can call it without the name being mangled.
  ******************************************************************************
  */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Entry point of the C++ application layer.
  * @note   Called from StartDefaultTask, after MX_LWIP_Init(). This runs in an
  *         RTOS task (not from main()), which is what the lwIP raw API and any
  *         blocking driver call require. Do not call it before the scheduler
  *         has started.
  */
void App_Main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
