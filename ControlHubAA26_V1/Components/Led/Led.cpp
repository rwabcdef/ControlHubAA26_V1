#include "Led.hpp"

#define STATE_OFF 0
#define STATE_ON 1
#define STATE_FLASH_ON 2
#define STATE_FLASH_OFF 3

Led::Led(GPIO_TypeDef* port, uint16_t pin, bool flashEndEventEnabled)
: port(port), pin(pin), flashEndEventEnabled(flashEndEventEnabled)
, numFlashes(0), onPeriods(0), offPeriods(0)
{
  this->type = None;
  this->currentState = STATE_OFF;
  this->flashEndFlag = false;
  this->flashCount = 0;
  this->periodCount = 0;
  this->finalFlashOff = false;
  HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
}

void Led::on()
{
  this->type = On;
}

void Led::off()
{
  this->type = Off;
}

void Led::flash(uint8_t numFlashes, uint8_t onPeriods, uint8_t offPeriods, bool finalFlashOff)
{
  this->type = Flash;
  this->numFlashes = numFlashes;
  this->onPeriods = onPeriods;
  this->offPeriods = offPeriods;
  this->finalFlashOff = finalFlashOff;
}

void Led::setFlashEndEventEnabled(bool enabled)
{
  this->flashEndEventEnabled = enabled;
}

bool Led::getFlashEnd()
{
  if(this->flashEndFlag)
  {
    this->flashEndFlag = false;
    return true;
  }
  return false;
}

void Led::run()
{
  this->common();
  switch(this->currentState)
  {
    case STATE_OFF: { this->currentState = this->offState(); break; }
    case STATE_ON: { this->currentState = this->onState(); break; }
    case STATE_FLASH_ON: { this->currentState = this->flashOnState(); break; }
    case STATE_FLASH_OFF: { this->currentState = this->flashOffState(); break; }
  }
}

void Led::common()
{
  if(this->type == On)
  {
    this->currentState = STATE_ON;
    HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
  }
  else if(this->type == Off)
  {
    this->currentState = STATE_OFF;
    HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
  }
  else if(this->type == Flash)
  {
    this->currentState = STATE_FLASH_ON;
    this->flashCount = 0;
    this->periodCount = 0;
    HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
  }
  this->type = None; // clear event type
}

uint8_t Led::onState(){
  return STATE_ON;
}

uint8_t Led::offState(){
  return STATE_OFF;
}

uint8_t Led::flashOnState(){
  if(this->periodCount >= this->onPeriods)
  {
    this->periodCount = 0;

    if(this->finalFlashOff == true)
    {
      // The sequence of flashes will end with the LED off, so we need to
      // check if we are at the end of the flash sequence before turning
      // off the LED

      uint8_t count = this->flashCount;
      if(++count >= this->numFlashes && this->numFlashes != 0)
      {
        // Final flash has just ended - so end of flash sequence
        HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);

        if(this->flashEndEventEnabled)
        {
          this->flashEndFlag = true;
        }

        return STATE_OFF;
      }
    }

    HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
    return STATE_FLASH_OFF;
  }
  else
  {
    this->periodCount++;
    return STATE_FLASH_ON;
  }
}

uint8_t Led::flashOffState(){
  if(this->periodCount >= this->offPeriods)
  {
    this->periodCount = 0;
    this->flashCount++;
    if(this->numFlashes == 0)
    {
      // special case - infinite flashes
      HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
      return STATE_FLASH_ON;
    }
    if(this->flashCount >= this->numFlashes)
    {
      // End of flash sequence
      HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);

      if(this->flashEndEventEnabled)
      {
        this->flashEndFlag = true;
      }

      return STATE_OFF;
    }
    else
    {
      HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
      return STATE_FLASH_ON;
    }
  }
  else
  {
    this->periodCount++;
    return STATE_FLASH_OFF;
  }
}
