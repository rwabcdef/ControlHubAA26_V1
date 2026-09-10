/*
 * Button.hpp
 *
 * Ported from Arduino1's Button (HardMod::Std::Button), stripped of the
 * ButtonEvent/ButtonConfigEvent/IdChar serialisation framework (not present
 * in this project) and adapted for STM32 HAL GPIO.
 *
 * Carries no timing of its own - the owning task is responsible for
 * calling run() at a steady period (e.g. via vTaskDelayUntil), matching
 * POLL_PERIOD_MS which the debounce thresholds are tuned against.
 */

#ifndef BUTTON_HPP_
#define BUTTON_HPP_

#include <stdint.h>
#include "main.h"
#include "StateMachine.hpp"

class Button : public StateMachine
{
  public:
    // Expected caller poll period (ms) that the debounce thresholds below
    // are tuned against.
    static const uint8_t POLL_PERIOD_MS = 20;

    enum eventTypes{
      None = 0,
      Pressed,
      LongPressed,
      Released,
      Stuck
    };

    Button(GPIO_TypeDef* port, uint16_t pin, bool pressedPinState,
      bool releaseActive = false, uint8_t longPressThreshold = 0);

    // Samples and debounces the pin and updates state. Must be called
    // once per POLL_PERIOD_MS by the owning task; run() itself does not
    // block or delay.
    void run();

    void enableRelease(bool value);
    void setLongPressThreshold(uint8_t value);

    // Returns the latest event and clears it (flag interface). If the
    // event is Released and pressDuration is non-null, it's set to the
    // press length in poll periods.
    eventTypes getEvent(uint8_t* pressDuration = nullptr);

  private:
    static const uint8_t STABLE_ACTIVE_THRESHOLD = 5;
    static const uint8_t STABLE_INACTIVE_THRESHOLD = 5;

    enum internalEventTypes{
      NoEvent = 0,
      Edge,
      StableActive,
      StableInactive,
      StableActiveLong
    };

    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t longPressThreshold;
    bool releaseActive;
    bool pressedPinState;
    bool eventPinState;
    bool previousPinState;
    bool stable;
    uint8_t activeCount;
    uint8_t inActiveCount;
    uint8_t pressedCount;
    eventTypes eventType;

    // state methods
    uint8_t released();
    uint8_t pressed();

    internalEventTypes eventCheck();
};

#endif /* BUTTON_HPP_ */
