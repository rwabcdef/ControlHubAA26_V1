#include "TC78H611FNG_Standby.hpp"

TC78H611FNG_Standby::TC78H611FNG_Standby(GPIO_TypeDef* port, uint16_t pin)
: port(port), pin(pin), initialized(false)
{
}

void TC78H611FNG_Standby::enableGpioClock(GPIO_TypeDef* port)
{
  if(port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
}

void TC78H611FNG_Standby::init()
{
  if(this->initialized)
  {
    return;
  }

  enableGpioClock(this->port);

  // Park the output low before switching the pin to push-pull, so the
  // first thing the driver sees on /STBY is standby rather than a glitch.
  HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpioConfig = {};
  gpioConfig.Pin = this->pin;
  gpioConfig.Mode = GPIO_MODE_OUTPUT_PP;
  gpioConfig.Pull = GPIO_NOPULL;
  gpioConfig.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(this->port, &gpioConfig);

  this->initialized = true;
}

void TC78H611FNG_Standby::enable()
{
  this->init();
  HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_SET);
}

void TC78H611FNG_Standby::disable()
{
  this->init();
  HAL_GPIO_WritePin(this->port, this->pin, GPIO_PIN_RESET);
}
