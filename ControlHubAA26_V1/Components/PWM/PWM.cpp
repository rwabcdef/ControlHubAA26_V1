#include "PWM.hpp"

PWM::TimerState PWM::timerStates[PWM::MAX_TIMERS] = {};

// Pins PWM currently knows how to drive. Add an entry here (and extend
// enableGpioClock()/enableTimerClock() if it needs a new port/timer) to
// support another pin.
static const PwmPinMapping pwmPinMappings[] =
{
  { GPIOD, GPIO_PIN_12, TIM4, TIM_CHANNEL_1, GPIO_AF2_TIM4 },
  { GPIOD, GPIO_PIN_13, TIM4, TIM_CHANNEL_2, GPIO_AF2_TIM4 },
  { GPIOD, GPIO_PIN_14, TIM4, TIM_CHANNEL_3, GPIO_AF2_TIM4 },
  { GPIOD, GPIO_PIN_15, TIM4, TIM_CHANNEL_4, GPIO_AF2_TIM4 },
  { GPIOC, GPIO_PIN_6,  TIM8, TIM_CHANNEL_1, GPIO_AF3_TIM8 },
  { GPIOC, GPIO_PIN_8,  TIM8, TIM_CHANNEL_3, GPIO_AF3_TIM8 },
  { GPIOC, GPIO_PIN_9,  TIM8, TIM_CHANNEL_4, GPIO_AF3_TIM8 },
  // TIM8_CH2 would be PC7, but this board uses PC7 as I2S3_MCK for the
  // CS43L22 audio codec (MX_I2S3_Init). Adding it here would let a PWM
  // instance silently reconfigure that pin to AF3 and kill the codec
  // clock, so CH2 is deliberately left unmapped.
};

const PwmPinMapping* PWM::findMapping(GPIO_TypeDef* port, uint16_t pin)
{
  for(const PwmPinMapping& candidate : pwmPinMappings)
  {
    if(candidate.port == port && candidate.pin == pin)
    {
      return &candidate;
    }
  }
  return nullptr;
}

void PWM::enableGpioClock(GPIO_TypeDef* port)
{
  if(port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
}

void PWM::enableTimerClock(TIM_TypeDef* instance)
{
  if(instance == TIM4)      { __HAL_RCC_TIM4_CLK_ENABLE(); }
  else if(instance == TIM8) { __HAL_RCC_TIM8_CLK_ENABLE(); }
}

uint32_t PWM::timerClockHz(TIM_TypeDef* instance)
{
  RCC_ClkInitTypeDef clockConfig = {};
  uint32_t flashLatency = 0U;
  HAL_RCC_GetClockConfig(&clockConfig, &flashLatency);

  // TIM8 (like TIM1 and TIM9-11) is clocked from APB2; the other general
  // purpose timers used here are clocked from APB1.
  if(instance == TIM8)
  {
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    if(clockConfig.APB2CLKDivider != RCC_HCLK_DIV1)
    {
      pclk *= 2U;
    }
    return pclk;
  }

  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  if(clockConfig.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    pclk *= 2U;
  }
  return pclk;
}

PWM::TimerState* PWM::timerStateFor(TIM_TypeDef* instance)
{
  // Reuse the slot already tracking this timer, if there is one.
  for(TimerState& state : timerStates)
  {
    if(state.instance == instance)
    {
      return &state;
    }
  }

  // Otherwise claim a fresh slot for it.
  for(TimerState& state : timerStates)
  {
    if(state.instance == nullptr)
    {
      state.instance = instance;
      return &state;
    }
  }

  // Out of timer slots - increase MAX_TIMERS in PWM.hpp if this happens.
  return nullptr;
}

PWM::PWM(GPIO_TypeDef* port, uint16_t pin, pwmFreqValues frequency)
: port(port), pin(pin), mapping(findMapping(port, pin)), frequency(frequency)
, percent(0U), channelStarted(false)
{
}

void PWM::init()
{
  if(mapping == nullptr || channelStarted)
  {
    return;
  }

  TimerState* state = timerStateFor(mapping->timerInstance);
  if(state == nullptr)
  {
    return;
  }

  enableGpioClock(mapping->port);

  if(!state->baseInitialized)
  {
    enableTimerClock(mapping->timerInstance);

    state->handle.Instance = mapping->timerInstance;
    state->handle.Init.Prescaler = (timerClockHz(mapping->timerInstance) / 1000000U) - 1U;
    state->handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    state->handle.Init.Period = (1000000U / static_cast<uint32_t>(frequency)) - 1U;
    state->handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    state->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&state->handle);

    state->baseInitialized = true;
  }
  // Note: every channel on a given timer shares that timer's single
  // prescaler/period, so only the first PWM instance to reach a timer
  // controls its frequency - later instances on the same timer run at
  // that frequency too, regardless of what they were constructed with.

  GPIO_InitTypeDef gpioConfig = {};
  gpioConfig.Pin = mapping->pin;
  gpioConfig.Mode = GPIO_MODE_AF_PP;
  gpioConfig.Pull = GPIO_NOPULL;
  gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
  gpioConfig.Alternate = mapping->alternateFunction;
  HAL_GPIO_Init(mapping->port, &gpioConfig);

  TIM_OC_InitTypeDef outputConfig = {};
  outputConfig.OCMode = TIM_OCMODE_PWM1;
  outputConfig.Pulse = 0U;
  outputConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  outputConfig.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&state->handle, &outputConfig, mapping->channel);
  HAL_TIM_PWM_Start(&state->handle, mapping->channel);

  channelStarted = true;
}

void PWM::setPercent(uint8_t value)
{
  if(mapping == nullptr)
  {
    return;
  }

  if(!channelStarted)
  {
    init();
  }

  TimerState* state = timerStateFor(mapping->timerInstance);
  if(state == nullptr)
  {
    return;
  }

  if(value > 100U)
  {
    value = 100U;
  }

  percent = value;
  uint32_t period = state->handle.Init.Period;
  uint32_t pulse = (period + 1U) * percent / 100U;
  if(pulse > period)
  {
    pulse = period;
  }
  __HAL_TIM_SET_COMPARE(&state->handle, mapping->channel, pulse);
}

uint8_t PWM::getPercent() const
{
  return percent;
}

void PWM::setFrequency(pwmFreqValues value)
{
  frequency = value;

  if(mapping == nullptr)
  {
    return;
  }

  TimerState* state = timerStateFor(mapping->timerInstance);
  if(state == nullptr || !state->baseInitialized)
  {
    // Timer isn't running yet - init() will pick the new frequency up.
    return;
  }

  // The period belongs to the timer, not the channel, so this retunes
  // every channel on that timer. Other PWM instances sharing it keep
  // their old compare value and so end up at a different duty cycle -
  // call setPercent() on those instances afterwards to restore theirs.
  uint32_t period = (1000000U / static_cast<uint32_t>(frequency)) - 1U;
  __HAL_TIM_SET_AUTORELOAD(&state->handle, period);

  // If the counter is already past the new (smaller) period it would
  // otherwise run all the way to 0xFFFF before reloading. Restarting it
  // bounds the disturbance to a single period.
  if(__HAL_TIM_GET_COUNTER(&state->handle) > period)
  {
    __HAL_TIM_SET_COUNTER(&state->handle, 0U);
  }

  setPercent(percent);
}

pwmFreqValues PWM::getFrequency() const
{
  return frequency;
}

// pwmFreqValues in enum order - index N is the Nth selectable frequency.
// Keep this in step with the enum in PWM.hpp.
static const pwmFreqValues pwmFrequencyList[] =
{
  PWM_FREQ_100HZ,
  PWM_FREQ_500HZ,
  PWM_FREQ_1KHZ,
  PWM_FREQ_2KHZ
};

uint8_t PWM::frequencyCount()
{
  return static_cast<uint8_t>(sizeof(pwmFrequencyList) / sizeof(pwmFrequencyList[0]));
}

bool PWM::frequencyFromIndex(uint8_t index, pwmFreqValues& value)
{
  if(index >= frequencyCount())
  {
    return false;
  }
  value = pwmFrequencyList[index];
  return true;
}
