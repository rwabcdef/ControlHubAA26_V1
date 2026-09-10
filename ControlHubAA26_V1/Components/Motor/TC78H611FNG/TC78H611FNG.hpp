/*
 * TC78H611FNG.hpp
 *
 * Controls ONE H-bridge (channel A or channel B) of a Toshiba
 * TC78H611FNG dual H-bridge driver, with PWM speed control.
 *
 * outA is wired to the bridge's IN1x pin, outB to its IN2x pin
 * (IN1A/IN2A for channel A, IN1B/IN2B for channel B). Drive two motors
 * from one IC with two TC78H611FNG instances, one per pin pair, both
 * sharing a single TC78H611FNG_Standby.
 *
 * Direction, per the datasheet's Input/Output functions table
 * (/STBY high throughout - held by the TC78H611FNG_Standby object):
 *
 *   idle    : IN1x = Low, IN2x = Low   -> outputs high impedance (Stop)
 *   forward : IN1x = Low, IN2x = PWM
 *   reverse : IN1x = PWM, IN2x = Low
 *
 * Both pins are driven by a PWM channel rather than one PWM channel and
 * one plain GPIO. A PWM channel at 0% sits constantly low, which is
 * exactly the "Low" the table asks for, and it means swapping the PWM
 * between outA and outB on a direction change is just two setPercent()
 * calls - no GPIO reconfiguration and no pin-mode glitch. The cost is
 * that BOTH pins must be PWM-capable and present in pwmPinMappings[] in
 * PWM.cpp.
 *
 * Note that the IC's inputs are rated to 500 kHz max and its outputs
 * carry a ~300 ns internal dead time, so the pwmFreqValues range is
 * comfortably within spec.
 */

#ifndef TC78H611FNG_HPP_
#define TC78H611FNG_HPP_

#include <stdint.h>
#include "main.h"
#include "PWM.hpp"

class TC78H611FNG
{
  public:
    enum direction : uint8_t
    {
      idle = 0,
      forward,
      reverse
    };

    // frequency uses PWM's pwmFreqValues enum.
    TC78H611FNG(GPIO_TypeDef* outAPort, uint16_t outAPin,
                GPIO_TypeDef* outBPort, uint16_t outBPin,
                pwmFreqValues frequency);

    // Duty cycle applied to whichever output the current direction puts
    // the PWM on. Values above 100 are clamped to 100. Setting a percent
    // while idle stores it; it takes effect on the next setDirection().
    void setPercent(uint8_t percent);
    uint8_t getPercent() const;

    void setDirection(direction value);
    direction getDirection() const;

    // Retunes the PWM. Because a period is a property of the timer, this
    // also retunes any other PWM channel sharing the same timer - see the
    // comment on PWM::setFrequency().
    void setFrequency(pwmFreqValues value);
    pwmFreqValues getFrequency() const;

  private:
    PWM outA; // IN1A / IN1B
    PWM outB; // IN2A / IN2B
    direction currentDirection;
    uint8_t percent;

    // Pushes currentDirection/percent out to the two PWM channels.
    void applyOutputs();
};

#endif /* TC78H611FNG_HPP_ */
