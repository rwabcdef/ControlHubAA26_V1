/*
 * Led.hpp
 *
 * Ported from Arduino1's Led (HardMod::Std::Led), stripped of the
 * LedEvent/LedUtils/IdChar serialisation framework (not present in this
 * project) and adapted for STM32 HAL GPIO.
 *
 * Carries no timing of its own - the owning task is responsible for
 * calling run() at a steady period (e.g. via vTaskDelayUntil), matching
 * PERIOD_MS which onPeriods/offPeriods (passed to flash()) are counted in.
 */

#ifndef LED_HPP_
#define LED_HPP_

#include <stdint.h>
#include "main.h"
#include "StateMachine.hpp"

class Led : public StateMachine
{
  public:
    // Expected caller poll period (ms) that onPeriods/offPeriods are
    // counted in units of.
    static const uint16_t PERIOD_MS = 250;

    Led(GPIO_TypeDef* port, uint16_t pin, bool flashEndEventEnabled = false);

    void on();
    void off();

    // numFlashes == 0 means flash indefinitely, until on()/off() is
    // called. finalFlashOff selects whether the last flash in a finite
    // sequence ends with the LED off (true) or on (false).
    void flash(uint8_t numFlashes, uint8_t onPeriods, uint8_t offPeriods, bool finalFlashOff);

    // Samples any pending on()/off()/flash() request and steps the state
    // machine by one period. Must be called once per PERIOD_MS by the
    // owning task; run() itself does not block or delay.
    void run();

    void setFlashEndEventEnabled(bool enabled);

    // Returns true (and clears the flag) once, when a flash sequence
    // ends. Only set if flashEndEventEnabled.
    bool getFlashEnd();

  private:
    enum eventType{
      None = 0,
      On,
      Off,
      Flash
    };

    GPIO_TypeDef* port;
    uint16_t pin;
    eventType type;
    uint8_t numFlashes;
    uint8_t flashCount;
    int8_t periodCount;
    uint8_t onPeriods;
    uint8_t offPeriods;
    bool finalFlashOff;
    bool flashEndFlag;
    bool flashEndEventEnabled;

    // state methods
    uint8_t onState();
    uint8_t offState();
    uint8_t flashOnState();
    uint8_t flashOffState();
    void common();
};

#endif /* LED_HPP_ */
