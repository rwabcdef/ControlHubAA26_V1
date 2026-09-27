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
  driving(false), valid(false), output(0.0f)
{
  /* Only stores the config: a file scope Controller is constructed
     before main(), when neither the HAL nor the RTOS exists. */
}

bool Controller::init()
{
  valid = (config.getRpm != nullptr) &&
          (config.getPwmPercent != nullptr) &&
          (config.setPwmPercent != nullptr) &&
          (config.outputMinPercent <= config.outputMaxPercent) &&
          (config.outputMaxPercent <= 100U);

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
