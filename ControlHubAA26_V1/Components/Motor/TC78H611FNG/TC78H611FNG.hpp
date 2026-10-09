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
 *
 * Direction output (optional): a plain push-pull GPIO that mirrors the
 * direction, for external hardware that needs to know which way the
 * current flows (e.g. a current measurement circuit). It is not a
 * TC78H611FNG pin. directionForwardHigh picks the polarity: true drives
 * the pin high for forward and low for reverse, false the opposite.
 * idle leaves the pin at its last level - no current flows then, and
 * holding it saves a needless toggle on every stop. Until the first
 * forward/reverse it sits at the forward level. Pass a null
 * directionPort to leave the feature out.
 *
 * On a direction change the pin is updated BEFORE the PWM is applied,
 * so the measurement hardware is already switched by the time current
 * starts flowing the new way.
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
                pwmFreqValues frequency,
                GPIO_TypeDef* directionPort = nullptr,
                uint16_t directionPin = 0U,
                bool directionForwardHigh = true);

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

    GPIO_TypeDef* directionPort; // nullptr - no direction output
    uint16_t directionPin;
    bool directionForwardHigh;
    bool directionInitialized;

    // Pushes currentDirection/percent out to the two PWM channels.
    void applyOutputs();

    // Drives the direction output for currentDirection (idle leaves it
    // alone). Configures the GPIO on first use, like PWM and
    // TC78H611FNG_Standby, so the object can live at file scope.
    void applyDirectionOutput();

    static void enableGpioClock(GPIO_TypeDef* port);
};

#endif /* TC78H611FNG_HPP_ */
