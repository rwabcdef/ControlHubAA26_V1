/*
 * L293D.hpp
 *
 * Controls ONE motor channel of an L293D quadruple half-H driver, with
 * PWM speed control.
 *
 * Ported from the Arduino1 project's HardMod::Std::Motor, dropped of the
 * MotorEvent serialisation framework (not present in this project) and
 * adapted to STM32 HAL plus this project's PWM class.
 *
 * Pin roles - note these differ from TC78H611FNG. The L293D separates
 * enable from direction, so the PWM sits on a fixed enable pin and the
 * two direction pins are plain GPIO:
 *
 *   en  : EN1 (or EN2) - PWM, sets speed
 *   in1 : IN1 (or IN3) - plain GPIO
 *   in2 : IN2 (or IN4) - plain GPIO
 *
 *   idle    : EN = 0%,  IN1 = Low,  IN2 = Low   -> outputs high impedance
 *   forward : EN = PWM, IN1 = High, IN2 = Low
 *   reverse : EN = PWM, IN1 = Low,  IN2 = High
 *
 * One L293D package drives two motors: EN1 with IN1/IN2, and EN2 with
 * IN3/IN4. That is two L293D instances, one per channel. There is no
 * device-wide standby pin, so unlike TC78H611FNG there is no separate
 * standby class - EN is per-channel and this class owns it.
 *
 * The enable pin must be PWM-capable and present in pwmPinMappings[] in
 * PWM.cpp. The two direction pins can be any GPIO.
 *
 * DIRECTION REVERSAL - the L293D has no internal dead time (the
 * TC78H611FNG has ~300 ns). Going straight from forward to reverse at a
 * non-zero percent applies full reverse voltage while the motor is still
 * turning, which is hard on the driver and the supply. setDirection()
 * does not refuse this (the Arduino original returned false when percent
 * was non-zero); sequencing is left to the caller. To reverse gently:
 *
 *   motor.setDirection(L293D::idle);   // coast
 *   ...let the motor slow down...
 *   motor.setDirection(L293D::reverse);
 */

#ifndef L293D_HPP_
#define L293D_HPP_

#include <stdint.h>
#include "main.h"
#include "PWM.hpp"

class L293D
{
  public:
    enum direction : uint8_t
    {
      idle = 0,
      forward,
      reverse
    };

    // frequency uses PWM's pwmFreqValues enum. enPort/enPin must be a
    // PWM-capable pin; the in1/in2 pins are plain GPIO outputs.
    L293D(GPIO_TypeDef* enPort, uint16_t enPin,
          GPIO_TypeDef* in1Port, uint16_t in1Pin,
          GPIO_TypeDef* in2Port, uint16_t in2Pin,
          pwmFreqValues frequency);

    // Duty cycle on the enable pin. Values above 100 are clamped to 100.
    // Setting a percent while idle stores it; it takes effect on the next
    // setDirection().
    void setPercent(uint8_t percent);
    uint8_t getPercent() const;

    // See the direction reversal note above - this does not enforce a
    // coast between opposing directions.
    void setDirection(direction value);
    direction getDirection() const;

    // Retunes the PWM. Because a period is a property of the timer, this
    // also retunes any other PWM channel sharing the same timer - see the
    // comment on PWM::setFrequency().
    void setFrequency(pwmFreqValues value);
    pwmFreqValues getFrequency() const;

  private:
    PWM enable;              // EN1 / EN2
    GPIO_TypeDef* in1Port;   // IN1 / IN3
    uint16_t in1Pin;
    GPIO_TypeDef* in2Port;   // IN2 / IN4
    uint16_t in2Pin;
    direction currentDirection;
    uint8_t percent;
    bool gpioInitialized;

    // The direction GPIOs are configured on first use rather than in the
    // constructor, so an L293D can live at file scope and still be set up
    // after HAL_Init()/clock configuration has run. PWM does the same for
    // the enable pin.
    void initGpio();

    // Pushes currentDirection/percent out to the enable and direction pins.
    void applyOutputs();

    static void enableGpioClock(GPIO_TypeDef* port);
};

#endif /* L293D_HPP_ */
