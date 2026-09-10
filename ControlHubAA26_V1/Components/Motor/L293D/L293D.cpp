#include "L293D.hpp"

L293D::L293D(GPIO_TypeDef* enPort, uint16_t enPin,
             GPIO_TypeDef* in1Port, uint16_t in1Pin,
             GPIO_TypeDef* in2Port, uint16_t in2Pin,
             pwmFreqValues frequency)
: enable(enPort, enPin, frequency)
, in1Port(in1Port), in1Pin(in1Pin), in2Port(in2Port), in2Pin(in2Pin)
, currentDirection(idle), percent(0U), gpioInitialized(false)
{
  // Deliberately no hardware access here - see initGpio().
}

void L293D::enableGpioClock(GPIO_TypeDef* port)
{
  if(port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
}

void L293D::initGpio()
{
  if(this->gpioInitialized)
  {
    return;
  }

  enableGpioClock(this->in1Port);
  enableGpioClock(this->in2Port);

  // Park both direction pins low before switching them to push-pull, so
  // the driver starts from the idle combination rather than a glitch.
  HAL_GPIO_WritePin(this->in1Port, this->in1Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(this->in2Port, this->in2Pin, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpioConfig = {};
  gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
  gpioConfig.Pull = GPIO_NOPULL;
  gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;

  gpioConfig.Pin = this->in1Pin;
  HAL_GPIO_Init(this->in1Port, &gpioConfig);

  gpioConfig.Pin = this->in2Pin;
  HAL_GPIO_Init(this->in2Port, &gpioConfig);

  this->gpioInitialized = true;
}

void L293D::applyOutputs()
{
  this->initGpio();

  GPIO_PinState in1State = GPIO_PIN_RESET;
  GPIO_PinState in2State = GPIO_PIN_RESET;
  uint8_t enPercent = 0U;

  switch(this->currentDirection)
  {
    case forward:
      in1State = GPIO_PIN_SET;
      enPercent = this->percent;
      break;

    case reverse:
      in2State = GPIO_PIN_SET;
      enPercent = this->percent;
      break;

    case idle:
    default:
      // Both direction pins low and the enable at 0% - outputs off.
      break;
  }

  // Direction pins first, then enable. A GPIO write lands immediately
  // whereas the compare value is preloaded and takes effect at the next
  // timer update, so settling direction before enable is the order that
  // keeps the two consistent.
  HAL_GPIO_WritePin(this->in1Port, this->in1Pin, in1State);
  HAL_GPIO_WritePin(this->in2Port, this->in2Pin, in2State);
  this->enable.setPercent(enPercent);
}

void L293D::setPercent(uint8_t value)
{
  if(value > 100U)
  {
    value = 100U;
  }

  this->percent = value;
  this->applyOutputs();
}

uint8_t L293D::getPercent() const
{
  return this->percent;
}

void L293D::setDirection(direction value)
{
  if(value == this->currentDirection)
  {
    return;
  }

  this->currentDirection = value;
  this->applyOutputs();
}

L293D::direction L293D::getDirection() const
{
  return this->currentDirection;
}

void L293D::setFrequency(pwmFreqValues value)
{
  this->enable.setFrequency(value);
  this->applyOutputs();
}

pwmFreqValues L293D::getFrequency() const
{
  return this->enable.getFrequency();
}
