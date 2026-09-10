#include "Button.hpp"

#define RELEASED 0
#define PRESSED 1

Button::Button(GPIO_TypeDef* port, uint16_t pin, bool pressedPinState,
  bool releaseActive, uint8_t longPressThreshold)
: port(port), pin(pin), longPressThreshold(longPressThreshold)
, releaseActive(releaseActive), pressedPinState(pressedPinState)
{
  this->currentState = RELEASED;
  this->eventPinState = this->pressedPinState;
  this->stable = true;
  this->previousPinState = !this->pressedPinState;
  this->eventType = None;
  this->activeCount = 0;
  this->inActiveCount = 0;
  this->pressedCount = 0;
}

void Button::enableRelease(bool value){ this->releaseActive = value; }

void Button::setLongPressThreshold(uint8_t value){ this->longPressThreshold = value; }

void Button::run()
{
  switch(this->currentState)
  {
    case RELEASED: { this->currentState = this->released(); break; }
    case PRESSED: { this->currentState = this->pressed(); break; }
  }
}

Button::eventTypes Button::getEvent(uint8_t* pressDuration)
{
  eventTypes ret = this->eventType;
  if((ret == Released) && (pressDuration != nullptr))
  {
    *pressDuration = this->pressedCount;
  }
  this->eventType = None; // clear event (flag interface)
  return ret;
}

//---------------------------------------------------
Button::internalEventTypes Button::eventCheck()
{
  Button::internalEventTypes ret = NoEvent;
  bool currentPinState = (HAL_GPIO_ReadPin(this->port, this->pin) == GPIO_PIN_SET);

  if(this->stable)
  {
    if((currentPinState == this->eventPinState) &&
      (this->previousPinState != this->eventPinState))
    {
      // edge detected
      ret = Edge;
      this->activeCount = 0;
      this->inActiveCount = 0;
      this->stable = false;
    }
    // long press check
    else if((currentPinState != this->eventPinState) &&
      (this->eventPinState != this->pressedPinState))
    {
      this->inActiveCount = 0;
      this->activeCount++;
      if((this->longPressThreshold > 0) && (this->activeCount >= this->longPressThreshold))
      {
        // long press event
        this->activeCount = 0;
        ret = StableActiveLong;
      }
    }
  }
  else
  {
    // currently not stable
    if(currentPinState == this->eventPinState)
    {
      this->inActiveCount = 0;
      this->activeCount++;
      if(this->activeCount >= STABLE_ACTIVE_THRESHOLD)
      {
        // button is stable & active
        this->stable = true;
        ret = StableActive;
      }
    }
    else
    {
      this->activeCount = 0;
      this->inActiveCount++;
      if(this->inActiveCount >= STABLE_INACTIVE_THRESHOLD)
      {
        // button is stable & inactive
        this->stable = true;
        ret = StableInactive;
      }
    }
  }
  this->previousPinState = currentPinState;
  return ret;
}
//---------------------------------------------------

// state methods
uint8_t Button::released()
{
  Button::internalEventTypes event = this->eventCheck();
  if(event == Edge)
  {
    this->eventType = Pressed; // set event code (flag interface)
    this->pressedCount = 0;
    return PRESSED;
  }
  return RELEASED;
}

uint8_t Button::pressed()
{
  Button::internalEventTypes event = this->eventCheck();
  if(event == NoEvent)
  {
    this->pressedCount++;
    return PRESSED;
  }
  else if(event == StableActive)
  {
    // change the polarity of the edge event state
    this->eventPinState = (!this->eventPinState);
    return PRESSED;
  }
  else if(event == StableInactive)
  {
    // button is inactive - so return to released state.
    // This situation will occur with a very quick button press.
    this->eventPinState = this->pressedPinState;
    return RELEASED;
  }
  else if(event == Edge)
  {
    // button released
    if(this->releaseActive == true)
    {
      this->eventType = Released; // set event code (flag interface)
    }

    this->eventPinState = this->pressedPinState;
    return RELEASED;
  }
  else if(event == StableActiveLong)
  {
    // long press
    this->eventType = LongPressed; // set event code (flag interface)
    return PRESSED;
  }
  return PRESSED;
}
