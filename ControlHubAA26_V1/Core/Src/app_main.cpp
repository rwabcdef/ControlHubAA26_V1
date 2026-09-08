/**
  ******************************************************************************
  * @file    app_main.cpp
  * @brief   C++ application layer.
  *
  * Everything CubeMX generates stays C; application code lives here in C++.
  * Keeping the split this way means CubeMX can regenerate main.c at any time
  * without touching a line of application code.
  *
  * Calling C from C++ works as long as the C header wraps its declarations in
  * extern "C" -- every CubeMX header (main.h, the HAL headers) already does,
  * and so do httpd_app.h and app_main.h.
  *
  * Going the other way needs care. C++ mangles FUNCTION names (plain global
  * VARIABLES are left alone), so anything C code must link against has to be
  * declared extern "C". That matters most for HAL's __weak callbacks: define
  * one here without extern "C" and it silently fails to override the weak
  * default -- it links cleanly and simply never gets called. See the worked
  * example at the bottom of this file.
  ******************************************************************************
  */

#include "app_main.h"

#include "main.h"
#include "httpd_app.h"

/* Objects with static storage duration are constructed by __libc_init_array,
   which the startup code runs before main(). Constructors must therefore not
   touch the HAL or the RTOS: at that point HAL_Init() has not run and the
   scheduler is not started. Do that work in App_Main() instead. */

namespace
{

/**
  * @brief  Application root object. Owns the C++ side of the system.
  */
class Application
{
public:
  /** Called once, after the RTOS is running and lwIP is up. */
  void start()
  {
    HTTPD_App_Init();
  }
};

Application g_app;

} /* anonymous namespace */

/* -------------------------------------------------------------------------- */

void App_Main(void)
{
  g_app.start();
}

/* -------------------------------------------------------------------------- */
/* Overriding a HAL __weak callback from C++.
 *
 * The extern "C" is mandatory. Without it this compiles to a mangled symbol,
 * HAL's __weak definition stays live, and the callback never fires.
 *
 * Note that HAL_TIM_PeriodElapsedCallback is already implemented in main.c for
 * the TIM6 HAL timebase -- do not also define it here, or you will get a
 * duplicate symbol. This is left as a commented pattern for other callbacks.
 */
#if 0
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  (void)GPIO_Pin;
}
#endif
