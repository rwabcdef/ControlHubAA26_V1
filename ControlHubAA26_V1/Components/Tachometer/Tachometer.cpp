/*
 * Tachometer.cpp
 *
 * See Tachometer.hpp for the measurement model and the threading contract.
 */

#include "Tachometer.hpp"

/* EXTI line N is driven by pin N of exactly one port, selected by
   SYSCFG_EXTICR[N/4] - so it is the pin NUMBER, not the port, that picks
   the line and therefore the vector. Lines 0..4 have a vector each; 5..9
   share EXTI9_5_IRQn and 10..15 share EXTI15_10_IRQn.

   Only the dedicated lines are mapped here on purpose. A shared vector
   would have to demux, and its handler in stm32f4xx_it.c would have to
   call HAL_GPIO_EXTI_IRQHandler() for *every* line in the group that can
   fire - PF5 (the radio nINT) on EXTI9_5, PC13 (USER_Btn) on EXTI15_10.
   Miss one and its pending bit is never cleared, which is an interrupt
   storm rather than a missed edge. Returning NonMaskableInt_IRQn makes
   init() fail loudly instead of quietly mis-wiring that. */
static IRQn_Type irqnForPin(uint16_t pin)
{
  switch(pin)
  {
    case GPIO_PIN_0: return EXTI0_IRQn;
    case GPIO_PIN_1: return EXTI1_IRQn;
    case GPIO_PIN_2: return EXTI2_IRQn;
    case GPIO_PIN_3: return EXTI3_IRQn;
    case GPIO_PIN_4: return EXTI4_IRQn;
    default:         return NonMaskableInt_IRQn;
  }
}

static bool enableGpioClock(GPIO_TypeDef* port)
{
  if(port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if(port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else if(port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
  else                   { return false; }

  return true;
}

/* TIM2..TIM7 and TIM12..TIM14 hang off APB1. When the APB1 prescaler is
   anything but 1 the timer clock is twice PCLK1, which is the case here
   (168 MHz SYSCLK, PCLK1 42 MHz, timer clock 84 MHz). Read rather than
   hard coded, so a change to the clock tree retunes the prescaler instead
   of silently changing the tick rate. PWM::timerClockHz() does the same
   thing for its own timers. */
static uint32_t apb1TimerClockHz(void)
{
  RCC_ClkInitTypeDef clockConfig = {};
  uint32_t flashLatency = 0U;
  HAL_RCC_GetClockConfig(&clockConfig, &flashLatency);

  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  if(clockConfig.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    pclk *= 2U;
  }

  return pclk;
}

Tachometer::Tachometer(TIM_TypeDef* timebase, GPIO_TypeDef* port, uint16_t pin)
  : timebase(timebase),
    port(port),
    pin(pin),
    timerHandle(),
    queue(nullptr),
    staticQueue(),
    queueStorageArea(),
    lastEdgeTick(0U),
    hasLastEdge(false),
    glitchCount(0U),
    droppedEdges(0U),
    rpm(0U),
    stalled(true),
    revolutions(0U),
    revStartTick(0U),
    hasRevStart(false),
    edgesSinceRevStart(0U),
    lastSeenTick(0U),
    hasSeenEdge(false)
{
  /* Only stores its arguments - no HAL, no RTOS - so a file scope
     Tachometer is safe. This runs from __libc_init_array, before
     HAL_Init() and before the scheduler. The work is all in init(). */
}

bool Tachometer::init()
{
  if((timebase == nullptr) || (port == nullptr))
  {
    return false;
  }

  if(irqnForPin(pin) == NonMaskableInt_IRQn)
  {
    return false;
  }

  /* Queue first. From the moment startPin() enables the NVIC line an edge
     can arrive - interrupts are already on when initTasks() runs - and
     onEdge() needs somewhere to put it. */
  queue = xQueueCreateStatic(TACHO__QUEUE_LENGTH, sizeof(uint32_t),
    queueStorageArea, &staticQueue);
  if(queue == nullptr)
  {
    return false;
  }

  /* Timebase second, so CNT is already running when the first edge reads
     it. A stopped counter would timestamp every edge identically. */
  if(!startTimebase())
  {
    return false;
  }

  return startPin();
}

bool Tachometer::startTimebase()
{
  /* TIM5 is not in the .ioc, so HAL_TIM_Base_MspInit() - which is where
     CubeMX puts the clock enable for the timers it owns - does nothing
     for it. Without this the writes below land on a dead peripheral and
     CNT never moves. */
  if(timebase == TIM5)      { __HAL_RCC_TIM5_CLK_ENABLE(); }
  else if(timebase == TIM2) { __HAL_RCC_TIM2_CLK_ENABLE(); }
  else                      { return false; }

  uint32_t timerClock = apb1TimerClockHz();
  if(timerClock < TICK_HZ)
  {
    return false;
  }

  timerHandle.Instance = timebase;
  timerHandle.Init.Prescaler = (timerClock / TICK_HZ) - 1U;
  timerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

  /* Full 32 bit range. Every interval in this class is computed as an
     unsigned difference, so the wrap - 2^32 ticks, about 119 hours at
     10 kHz - needs no handling at all. */
  timerHandle.Init.Period = 0xFFFFFFFFU;
  timerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if(HAL_TIM_Base_Init(&timerHandle) != HAL_OK)
  {
    return false;
  }

  /* Deliberately not HAL_TIM_Base_Start_IT(). The counter free runs with
     no interrupt and no vector of its own; onEdge() simply reads CNT. */
  return (HAL_TIM_Base_Start(&timerHandle) == HAL_OK);
}

bool Tachometer::startPin()
{
  if(!enableGpioClock(port))
  {
    return false;
  }

  /* SYSCFG, which HAL_GPIO_Init() writes to route the pin to its EXTI
     line, is already clocked by HAL_MspInit() in stm32f4xx_hal_msp.c. */
  GPIO_InitTypeDef gpioInit = {};
  gpioInit.Pin = pin;
  gpioInit.Mode = GPIO_MODE_IT_FALLING;

  /* The magnet pulls the 3144 open collector output low, so the line
     idles high and the falling edge is the magnet arriving. The pull-up
     costs nothing against the level shifter push-pull drive and keeps the
     input defined if the sensor is ever unplugged - without it a floating
     pin would generate edges of its own. */
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(port, &gpioInit);

  IRQn_Type irqn = irqnForPin(pin);

  /* Configuring the pin can itself latch a pending bit, and a real edge
     may have arrived while the port clock came up. Clear both before
     enabling, or init() is followed immediately by a spurious edge that
     would start a revolution from a meaningless timestamp. */
  __HAL_GPIO_EXTI_CLEAR_IT(pin);
  HAL_NVIC_ClearPendingIRQ(irqn);

  HAL_NVIC_SetPriority(irqn, TACHO__EXTI_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(irqn);

  return true;
}

void Tachometer::onEdge()
{
  /* First statement in the ISR path: the sooner CNT is read, the less of
     whatever latency got us here ends up in the timestamp. */
  uint32_t now = timebase->CNT;

  /* Unsigned difference, so this is correct across the counter wrap. */
  if(hasLastEdge && ((uint32_t)(now - lastEdgeTick) < TACHO__MIN_EDGE_TICKS))
  {
    glitchCount++;
    return;
  }

  lastEdgeTick = now;
  hasLastEdge = true;

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if(xQueueSendFromISR(queue, &now, &higherPriorityTaskWoken) != pdTRUE)
  {
    /* The control task has fallen behind. Dropping the newest edge
       undercounts revolutions, so the reading goes low rather than
       wild - and getDroppedEdges() says why. */
    droppedEdges++;
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void Tachometer::update()
{
  uint32_t revTicksTotal = 0U;
  uint16_t revsThisPass = 0U;

  uint32_t tick = 0U;
  while(xQueueReceive(queue, &tick, 0U) == pdTRUE)
  {
    lastSeenTick = tick;
    hasSeenEdge = true;

    if(!hasRevStart)
    {
      /* First edge after init() or after a stall: it opens a revolution
         but cannot close one, because nothing is known about how long
         ago the previous magnet went past. */
      revStartTick = tick;
      hasRevStart = true;
      edgesSinceRevStart = 0U;
      continue;
    }

    edgesSinceRevStart++;
    if(edgesSinceRevStart >= PULSES_PER_REV)
    {
      /* A whole turn of the wheel, so the uneven magnet spacing has
         cancelled - see the note in Tachometer.hpp. */
      revTicksTotal += (uint32_t)(tick - revStartTick);
      revsThisPass++;
      revolutions++;

      revStartTick = tick;
      edgesSinceRevStart = 0U;
    }
  }

  if((revsThisPass > 0U) && (revTicksTotal > 0U))
  {
    /* revsThisPass revolutions took revTicksTotal ticks. Averaging them
       together rather than keeping only the last one is both what a
       control loop wants and free resolution: the quantisation is one
       tick over the whole span, not one tick per revolution.

       Worst case here is QUEUE_LENGTH/PULSES_PER_REV revolutions, so the
       numerator stays far inside 32 bits. */
    uint32_t computed = ((uint32_t)revsThisPass * TICK_HZ * 60U) / revTicksTotal;

    rpm = (computed > 0xFFFFU) ? 0xFFFFU : (uint16_t)computed;
    stalled = false;
    return;
  }

  /* No complete revolution this pass. That is normal at low speed with a
     short update period, so the last reading stands until the sensor has
     been quiet long enough to mean stopped. */
  const uint32_t stallTicks = ((uint32_t)TACHO__STALL_TIMEOUT_MS * TICK_HZ) / 1000U;

  if(!hasSeenEdge || ((uint32_t)(timebase->CNT - lastSeenTick) > stallTicks))
  {
    rpm = 0U;
    stalled = true;

    /* Drop the half-timed revolution. Keeping it would time the restart
       from before the stop and report an absurdly low first speed. */
    hasRevStart = false;
    edgesSinceRevStart = 0U;
  }
}

uint16_t Tachometer::getRpm() const
{
  return rpm;
}

bool Tachometer::isStalled() const
{
  return stalled;
}

uint32_t Tachometer::getRevolutions() const
{
  return revolutions;
}

uint16_t Tachometer::getGlitchCount() const
{
  return glitchCount;
}

uint16_t Tachometer::getDroppedEdges() const
{
  return droppedEdges;
}
