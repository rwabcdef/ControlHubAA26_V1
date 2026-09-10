#include "TC78H611FNG.hpp"

TC78H611FNG::TC78H611FNG(GPIO_TypeDef* outAPort, uint16_t outAPin,
                         GPIO_TypeDef* outBPort, uint16_t outBPin,
                         pwmFreqValues frequency)
: outA(outAPort, outAPin, frequency), outB(outBPort, outBPin, frequency)
, currentDirection(idle), percent(0U)
{
  // Deliberately no hardware access here - PWM configures its timer and
  // GPIO lazily, so an instance can live at file scope and still be set
  // up after HAL_Init()/clock configuration has run.
}

void TC78H611FNG::applyOutputs()
{
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
