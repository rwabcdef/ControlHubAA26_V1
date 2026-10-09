#include "TC78H611FNG.hpp"

TC78H611FNG::TC78H611FNG(GPIO_TypeDef* outAPort, uint16_t outAPin,
                         GPIO_TypeDef* outBPort, uint16_t outBPin,
                         pwmFreqValues frequency,
                         GPIO_TypeDef* directionPort, uint16_t directionPin,
                         bool directionForwardHigh)
: outA(outAPort, outAPin, frequency), outB(outBPort, outBPin, frequency)
, currentDirection(idle), percent(0U)
, directionPort(directionPort), directionPin(directionPin)
, directionForwardHigh(directionForwardHigh), directionInitialized(false)
{
  // Deliberately no hardware access here - PWM configures its timer and
  // GPIO lazily, so an instance can live at file scope and still be set
  // up after HAL_Init()/clock configuration has run.
}

void TC78H611FNG::enableGpioClock(GPIO_TypeDef* port)
{
  if(port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if(port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else if(port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
}

void TC78H611FNG::applyDirectionOutput()
{
  if(this->directionPort == nullptr)
  {
    return;
  }

  GPIO_PinState forwardLevel =
    this->directionForwardHigh ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState reverseLevel =
    this->directionForwardHigh ? GPIO_PIN_RESET : GPIO_PIN_SET;

  if(!this->directionInitialized)
  {
    enableGpioClock(this->directionPort);

    // Set the output level before switching the pin to push-pull, so it
    // comes up at the forward level rather than glitching.
    HAL_GPIO_WritePin(this->directionPort, this->directionPin, forwardLevel);

    GPIO_InitTypeDef gpioConfig = {};
    gpioConfig.Pin = this->directionPin;
    gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
    gpioConfig.Pull = GPIO_NOPULL;
    gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(this->directionPort, &gpioConfig);

    this->directionInitialized = true;
  }

  if(this->currentDirection == forward)
  {
    HAL_GPIO_WritePin(this->directionPort, this->directionPin, forwardLevel);
  }
  else if(this->currentDirection == reverse)
  {
    HAL_GPIO_WritePin(this->directionPort, this->directionPin, reverseLevel);
  }
}

void TC78H611FNG::applyOutputs()
{
  // Direction output first - see "Direction output" in the header.
  this->applyDirectionOutput();

  uint8_t aPercent = (this->currentDirection == reverse) ? this->percent : 0U;
  uint8_t bPercent = (this->currentDirection == forward) ? this->percent : 0U;

  // Drop the output that is being de-asserted before raising the other,
  // so the two inputs are never both high mid-change. Per the datasheet's
  // Input/Output functions table that combination is Short brake, which
  // is not what a direction change is asking for.
  if(aPercent == 0U)
  {
    this->outA.setPercent(0U);
    this->outB.setPercent(bPercent);
  }
  else
  {
    this->outB.setPercent(0U);
    this->outA.setPercent(aPercent);
  }
}

void TC78H611FNG::setPercent(uint8_t value)
{
  if(value > 100U)
  {
    value = 100U;
  }

  this->percent = value;
  this->applyOutputs();
}

uint8_t TC78H611FNG::getPercent() const
{
  return this->percent;
}

void TC78H611FNG::setDirection(direction value)
{
  this->currentDirection = value;
  this->applyOutputs();
}

TC78H611FNG::direction TC78H611FNG::getDirection() const
{
  return this->currentDirection;
}

void TC78H611FNG::setFrequency(pwmFreqValues value)
{
  // Both channels are retuned even when they share a timer: the first
  // call moves the shared period, which leaves the second channel's
  // compare value stale, and the second call puts it right.
  this->outA.setFrequency(value);
  this->outB.setFrequency(value);

  this->applyOutputs();
}

pwmFreqValues TC78H611FNG::getFrequency() const
{
  return this->outA.getFrequency();
}
