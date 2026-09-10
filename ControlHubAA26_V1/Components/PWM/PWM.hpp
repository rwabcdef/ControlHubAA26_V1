#ifndef PWM_HPP_
#define PWM_HPP_

#include <stdint.h>
#include "main.h"

enum pwmFreqValues : uint32_t
{
  PWM_FREQ_100HZ = 100U,
  PWM_FREQ_500HZ = 500U,
  PWM_FREQ_1KHZ = 1000U,
  PWM_FREQ_2KHZ = 2000U
};

// One entry per pin PWM knows how to drive - the timer, channel and GPIO
// alternate function that pin needs to be configured for.
struct PwmPinMapping
{
  GPIO_TypeDef* port;
  uint16_t pin;
  TIM_TypeDef* timerInstance;
  uint32_t channel;
  uint8_t alternateFunction;
};

class PWM
{
  public:
    PWM(GPIO_TypeDef* port, uint16_t pin, pwmFreqValues frequency);

    void init();
    void setPercent(uint8_t percent);
    uint8_t getPercent() const;

    // Retunes the timer this channel lives on. See the note in
    // PWM.cpp - the period is shared, so this affects every channel on
    // that timer, not just this one.
    void setFrequency(pwmFreqValues frequency);
    pwmFreqValues getFrequency() const;

    // Maps a 0-based index onto pwmFreqValues in enum order, for command
    // interfaces that select a frequency by position. Returns false, and
    // leaves frequency untouched, if the index is out of range.
    static bool frequencyFromIndex(uint8_t index, pwmFreqValues& frequency);
    static uint8_t frequencyCount();

  private:
    // Per-timer state, shared by every PWM instance/channel that lives on
    // that timer. All channels of one timer share its prescaler/period,
    // so the base timer only ever gets initialized once - by whichever
    // PWM instance calls init() first for that timer.
    struct TimerState
    {
      TIM_TypeDef* instance;
      TIM_HandleTypeDef handle;
      bool baseInitialized;
    };

    // Number of distinct timers PWM can drive at once. Bump this if more
    // than this many different timers are used for PWM simultaneously.
    static constexpr uint8_t MAX_TIMERS = 4U;
    static TimerState timerStates[MAX_TIMERS];

    GPIO_TypeDef* port;
    uint16_t pin;
    const PwmPinMapping* mapping; // nullptr if this pin isn't supported
    pwmFreqValues frequency;
    uint8_t percent;
    bool channelStarted;

    static uint32_t timerClockHz(TIM_TypeDef* instance);
    static void enableGpioClock(GPIO_TypeDef* port);
    static void enableTimerClock(TIM_TypeDef* instance);
    static const PwmPinMapping* findMapping(GPIO_TypeDef* port, uint16_t pin);
    static TimerState* timerStateFor(TIM_TypeDef* instance);
};

#endif
