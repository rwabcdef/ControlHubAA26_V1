/*
 * TC78H611FNG_Standby.hpp
 *
 * Drives the /STBY pin of a Toshiba TC78H611FNG dual H-bridge driver.
 *
 * /STBY is device-wide, not per-bridge: pulling it low puts both
 * H-bridges into standby (all four outputs high impedance, ~0 uA
 * consumption). One TC78H611FNG_Standby therefore serves both
 * TC78H611FNG instances sharing a single driver IC.
 *
 * Datasheet note: /STBY (or IN1/IN2) must be low across power supply
 * ON/OFF, otherwise unexpected current may flow in the output pins. The
 * pin is configured low here and only driven high by enable().
 */

#ifndef TC78H611FNG_STANDBY_HPP_
#define TC78H611FNG_STANDBY_HPP_

#include <stdint.h>
#include "main.h"

class TC78H611FNG_Standby
{
  public:
    TC78H611FNG_Standby(GPIO_TypeDef* port, uint16_t pin);

    // /STBY high - both H-bridges leave standby and follow their INx pins.
    void enable();

    // /STBY low - both H-bridges go high impedance regardless of INx.
    void disable();

  private:
    GPIO_TypeDef* port;
    uint16_t pin;
    bool initialized;

    // The GPIO is configured on first use rather than in the constructor,
    // so a TC78H611FNG_Standby can live at file scope and still be set up
    // after HAL_Init()/clock configuration has run.
    void init();

    static void enableGpioClock(GPIO_TypeDef* port);
};

#endif /* TC78H611FNG_STANDBY_HPP_ */
