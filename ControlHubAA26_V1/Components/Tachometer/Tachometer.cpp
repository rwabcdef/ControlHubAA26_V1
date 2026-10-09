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
    armed(false),
    glitchCount(0U),
    droppedEdges(0U),
    rpm(0U),
    stalled(true),
    revolutions(0U),
    edges(0U),
    revStartTick(0U),
    hasRevStart(false),
    edgesSinceRevStart(0U),
    avgTicks(),
    avgIndex(0U),
    avgCount(0U),
    avgTicksSum(0U),
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
  gpioInit.Mode = GPIO_MODE_IT_RISING_FALLING;

  /* The magnet pulls the 3144 open collector output low, so the line
     idles high and the falling edge is the magnet arriving. Both edges
     interrupt: the rising one, the magnet leaving, is what re-arms the
     input - see armed in Tachometer.hpp.

     No internal pull-up. The level shifter is a BC182L emitter follower
     (collector at 12 V) into a 33k/12k divider, with PF4 on the tap. It
     drives high, but low is only the divider itself - 33k || 12k = 8.8k
     to ground once the transistor is off. The ~40k pull-up against that
     held the low at ~0.8 V, under 0.2 V from VIL, and the fuzz on it
     produced phantom falling edges: readings of 2x and 3x the true
     speed, and 2/3x when an edge was missed. The 12k already keeps the
     pin defined while the shifter is connected. */
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &gpioInit);

  IRQn_Type irqn = irqnForPin(pin);

  /* Configuring the pin can itself latch a pending bit, and a real edge
     may have arrived while the port clock came up. Clear both before
     enabling, or init() is followed immediately by a spurious edge that
     would start a revolution from a meaningless timestamp. */
  __HAL_GPIO_EXTI_CLEAR_IT(pin);
  HAL_NVIC_ClearPendingIRQ(irqn);

  /* Start armed only if no magnet is over the sensor now. If one is, the
     line is already low and its falling edge was missed; waiting for it
     to go high first stops noise on that low from counting as an edge. */
  armed = ((port->IDR & pin) != 0U);

  HAL_NVIC_SetPriority(irqn, TACHO__EXTI_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(irqn);

  return true;
}

void Tachometer::onEdge()
{
  /* First statement in the ISR path: the sooner CNT is read, the less of
     whatever latency got us here ends up in the timestamp. */
  uint32_t now = timebase->CNT;

  /* The EXTI4 NVIC line is deliberately left disabled in the .ioc, so
     only init() enables it, after creating the queue. If it is ever
     ticked there, MX_GPIO_Init() enables it first, and an edge in that
     window - a wheel still coasting after a reset - must be ignored,
     not queued to a null handle. */
  if(queue == nullptr)
  {
    return;
  }

  /* A real magnet holds the line low for milliseconds; a spike coupled in
     from the motor PWM is gone within microseconds. By the time the ISR
     gets here - a microsecond or two after the edge - a real edge is
     still low and a spike has already let go. The divider feeding PF4 is
     ~8.8k in both states, so the line picks such spikes up readily, and
     EXTI latches ones far too short for a scope at 100 ms/div to show.
     Read IDR directly: this is the ISR hot path.

     Both edges land here and EXTI does not say which one fired, so the
     level is the only thing to go on. High means the magnet has gone, or
     a spike on a high line has already recovered - either way the line
     is idle, and the next falling edge may count. Not a glitch: every
     real rising edge takes this path. */
  if((port->IDR & pin) != 0U)
  {
    armed = true;
    return;
  }

  /* Low, but the line has not been seen high since the last accepted
     edge. The low state is only the ~8.8k divider holding the pin down,
     so motor PWM spikes lift it briefly and each one falls back as a new
     falling edge while the pin still reads low - which the level check
     above cannot catch. At 30% duty these came through several ms after
     the real edge, past TACHO__MIN_EDGE_TICKS, and read as extra
     revolutions. A magnet cannot arrive twice without leaving between. */
  if(!armed)
  {
    glitchCount++;
    return;
  }

  /* Unsigned difference, so this is correct across the counter wrap. */
  if(hasLastEdge && ((uint32_t)(now - lastEdgeTick) < TACHO__MIN_EDGE_TICKS))
  {
    glitchCount++;
    return;
  }

  lastEdgeTick = now;
  hasLastEdge = true;
  armed = false;

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
  bool revCompleted = false;

  uint32_t tick = 0U;
  while(xQueueReceive(queue, &tick, 0U) == pdTRUE)
  {
    lastSeenTick = tick;
    hasSeenEdge = true;

    /* Before the revolution logic, so no edge is skipped - see getEdges(). */
    edges++;

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
      pushRevTicks((uint32_t)(tick - revStartTick));
      revCompleted = true;
      revolutions++;

      revStartTick = tick;
      edgesSinceRevStart = 0U;
    }
  }

  if(revCompleted && (avgTicksSum > 0U))
  {
    /* The last avgCount revolutions took avgTicksSum ticks - see
       TACHO__AVG_REVS. Dividing total revolutions by total time, rather
       than averaging per-revolution RPMs, is the correct mean for a rate
       and keeps the quantisation at one tick over the whole window.

       avgCount * TICK_HZ * 60 is at most 16 * 600,000, far inside 32
       bits whatever TACHO__AVG_REVS is set to within its limit. */
    uint32_t computed = ((uint32_t)avgCount * TICK_HZ * 60U) / avgTicksSum;

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

    /* And the averaging window, for the same reason: revolutions from
       before a stop say nothing about the speed after it. */
    avgCount = 0U;
    avgIndex = 0U;
    avgTicksSum = 0U;
  }
}

void Tachometer::pushRevTicks(uint32_t ticks)
{
  /* Running sum over a ring: subtract the revolution falling out of the
     window, add the new one. Until the ring has filled, the average is
     over however many revolutions there are, so the first reading after
     a start comes from one revolution rather than waiting for four. */
  if(avgCount == TACHO__AVG_REVS)
  {
    avgTicksSum -= avgTicks[avgIndex];
  }
  else
  {
    avgCount++;
  }

  avgTicks[avgIndex] = ticks;
  avgTicksSum += ticks;

  avgIndex++;
  if(avgIndex >= TACHO__AVG_REVS)
  {
    avgIndex = 0U;
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

uint32_t Tachometer::getEdges() const
{
  return edges;
}

uint16_t Tachometer::getGlitchCount() const
{
  return glitchCount;
}

uint16_t Tachometer::getDroppedEdges() const
{
  return droppedEdges;
}
