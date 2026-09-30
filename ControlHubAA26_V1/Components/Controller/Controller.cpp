/*
 * Controller.cpp
 *
 * See Controller.hpp for the control law, the threading contract and why
 * the output is a float.
 */

#include "Controller.hpp"
#include "FreeRTOS.h"
#include "task.h"

Controller::Controller(const ControllerConfig& config)
: config(config), requiredRpm(0U), enabled(false), rejectedCount(0U),
  tachoFault(false), driving(false), valid(false), output(0.0f),
  zeroRpmPasses(0U), zeroRpmLimitPasses(0U)
{
  /* Only stores the config: a file scope Controller is constructed
     before main(), when neither the HAL nor the RTOS exists. */
}

bool Controller::init()
{
  valid = (config.getRpm != nullptr) &&
          (config.getPwmPercent != nullptr) &&
          (config.setPwmPercent != nullptr) &&
          (config.setDirection != nullptr) &&
          (config.getDirection != nullptr) &&
          (config.outputMinPercent <= config.outputMaxPercent) &&
          (config.outputMaxPercent <= 100U) &&
          ((config.tachoUnresponsiveTimeout_S == 0U) || (config.periodMs > 0U));

  /* The timeout counted in run() passes, rounded up so it is never
     shorter than asked for. uint32_t: 65535 s in ms still fits. */
  zeroRpmLimitPasses = 0U;
  if(valid && (config.tachoUnresponsiveTimeout_S > 0U))
  {
    uint32_t timeoutMs = (uint32_t)config.tachoUnresponsiveTimeout_S * 1000U;
    zeroRpmLimitPasses = (timeoutMs + config.periodMs - 1U) / config.periodMs;
  }

  return valid;
}

void Controller::run()
{
  if(!valid || !enabled)
  {
    driving = false;
    return;
  }

  if(!driving)
  {
    /* Starting (or restarting after hold()): take over from wherever the
       motor is now, rather than from zero. */
    output = clamp((float)config.getPwmPercent());
    onReset();
    zeroRpmPasses = 0U;
    tachoFault = false;   // a fresh enable() is the reset
    driving = true;
  }

  uint16_t rpm = config.getRpm();
  if(rpm > config.maxPlausibleRpm)
  {
    /* A reading like this is a sensor fault, not a speed. Acting on it
       would drive the output hard the wrong way, so hold instead - the
       next good reading carries on from here. */
    rejectedCount++;
    return;
  }

  /* A zero reading against a non-zero demand, for too long: the
     tachometer is dead or unplugged, and the law would otherwise sit at
     outputMaxPercent indefinitely. Checked before the law runs, so the
     tripping pass writes nothing but the stop. See Unresponsive
     tachometer in Controller.hpp. */
  if((zeroRpmLimitPasses > 0U) && (rpm == 0U) && (requiredRpm > 0U))
  {
    if(++zeroRpmPasses >= zeroRpmLimitPasses)
    {
      stopForTachoFault();
      return;
    }
  }
  else
  {
    zeroRpmPasses = 0U;
  }

  /* Positive error: too slow, so the output goes up. int32_t, because the
     difference of two uint16_t can exceed int16_t either way. */
  int32_t errorRpm = (int32_t)requiredRpm - (int32_t)rpm;

  output = clamp(computeOutput(errorRpm, rpm, output));

  /* Rounded, not truncated, so the written duty cycle is the nearest one
     to the float - truncation would bias every output low by up to 1%.
     output is already clamped to 0..100, so this cannot overflow. */
  uint8_t percent = (uint8_t)(output + 0.5f);

  /* The enable flag is checked again with the scheduler suspended, in the
     same section as the write. A disable() that lands after this point
     cannot run until the write is done, and one that landed before it is
     seen here - so whatever the disabling task writes next always wins.
     See Threading in Controller.hpp. */
  vTaskSuspendAll();
  if(enabled)
  {
    config.setPwmPercent(percent);
  }
  (void)xTaskResumeAll();
}

void Controller::hold()
{
  /* driving = false also restarts the zero reading count on the next
     run(): a pass that could not drive the motor is no evidence against
     the tachometer. */
  driving = false;
}

void Controller::setRequiredRpm(uint16_t rpm)
{
  requiredRpm = rpm;
}

uint16_t Controller::getRequiredRpm() const
{
  return requiredRpm;
}

void Controller::enable()
{
  enabled = true;
}

void Controller::disable()
{
  enabled = false;
}

bool Controller::isEnabled() const
{
  return enabled;
}

void Controller::setDirection(ControllerDirection direction)
{
  /* Checked here rather than trusted: before init() has passed, the
     callback may be null. */
  if(valid)
  {
    config.setDirection(direction);
  }
}

ControllerDirection Controller::getDirection() const
{
  /* Unusable config: report the one direction that means "not driving". */
  return valid ? config.getDirection() : ControllerDirection::idle;
}

uint8_t Controller::getPwmPercent() const
{
  return valid ? config.getPwmPercent() : 0U;
}

uint8_t Controller::getOutputPercent() const
{
  return (uint8_t)(output + 0.5f);
}

float Controller::getOutput() const
{
  return output;
}

uint16_t Controller::getRejectedCount() const
{
  return rejectedCount;
}

bool Controller::isTachoFault() const
{
  return tachoFault;
}

void Controller::stopForTachoFault()
{
  /* Same guard as the write in run(): the enable flag is checked, and
     cleared, in the section that writes the stop. If another task has
     already called disable() it now owns the motor, and is left alone -
     the fault is still recorded. */
  bool stopped = false;

  vTaskSuspendAll();
  if(enabled)
  {
    enabled = false;
    config.setPwmPercent(0U);
    stopped = true;
  }
  (void)xTaskResumeAll();

  /* Idle as well as 0%, so nothing - a later percent set, say - can
     restart the motor until a direction is chosen again. Outside the
     suspended section: setDirection is not required to be safe there
     (see Threading). */
  if(stopped)
  {
    config.setDirection(ControllerDirection::idle);
  }

  output = 0.0f;
  driving = false;
  zeroRpmPasses = 0U;
  tachoFault = true;
}

float Controller::computeOutput(int32_t errorRpm, uint16_t rpm, float output)
{
  (void)rpm;
  return output + (config.integralGain * (float)errorRpm);
}

void Controller::onReset()
{
  /* The base law keeps no state beyond the output itself. */
}

float Controller::clamp(float value) const
{
  if(value < (float)config.outputMinPercent) { return (float)config.outputMinPercent; }
  if(value > (float)config.outputMaxPercent) { return (float)config.outputMaxPercent; }
  return value;
}
