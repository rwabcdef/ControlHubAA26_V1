/*
 * Lift.cpp
 *
 * See Lift.hpp for how directions map, what the distance is, and the
 * threading contract.
 */

#include "Lift.hpp"

Lift::Lift(Controller* controller, ControllerDirection forwardDirection,
           uint16_t speedRpm, uint32_t (*getDistance)())
: controller(controller), forwardDirection(forwardDirection),
  speedRpm(speedRpm), getDistance(getDistance),
  valid(false), started(false), startDistance(0U),
  state(status::idle), travelled(0U), target(0U)
{
}

bool Lift::init()
{
  valid = (controller != nullptr) &&
          (getDistance != nullptr) &&
          (forwardDirection != ControllerDirection::idle) &&
          (speedRpm > 0U);

  return valid;
}

bool Lift::start(direction d, uint32_t distance)
{
  if(!valid || (state == status::moving) || (distance == 0U))
  {
    return false;
  }

  startDistance = getDistance();
  started = true;
  travelled = 0U;
  target = distance;

  /* Speed, then direction, then enable, so the controller's first pass
     already has everything it needs. It seeds its output from the
     motor's present duty cycle (Bumpless start, Controller.hpp), which
     after a previous move is roughly the duty speedRpm needed then. */
  controller->setRequiredRpm(speedRpm);
  controller->setDirection(toControllerDirection(d));
  controller->enable();

  state = status::moving;
  return true;
}

void Lift::stop()
{
  if(!valid)
  {
    return;
  }

  /* Disable first. disable() guarantees no later controller pass writes
     the output (see Threading in Controller.hpp), so the idle below is
     the last word. */
  controller->disable();
  controller->setDirection(ControllerDirection::idle);

  state = status::idle;
}

void Lift::run()
{
  if(valid && started)
  {
    /* Unsigned difference, so a wrapped count is still right. Kept up
       after the move ends too, so the coast overrun shows. */
    travelled = getDistance() - startDistance;

    if((state == status::moving) && (travelled >= target))
    {
      stop();
    }
  }

  if(controller == nullptr)
  {
    return;
  }

  /* With the direction idle the bridge is stopped and the tacho reads
     zero whatever the duty cycle, so running the controller would only
     wind the output up to its limit for the motor to lurch at on the
     next direction change. hold() parks it instead, and it resumes
     from the motor's duty cycle once there is a direction again. */
  if(controller->getDirection() == ControllerDirection::idle)
  {
    controller->hold();
  }
  else
  {
    controller->run();
  }

  /* The controller has stopped the motor itself: the tacho read zero for
     too long (Unresponsive tachometer, Controller.hpp). No more edges
     will come, so the distance check above would never end the move -
     end it here, short. After the controller's pass rather than before,
     so it shows as idle in the same pass the motor stopped. The motor is
     already disabled and idle; there is nothing left for stop() to do. */
  if(valid && (state == status::moving) && controller->isTachoFault())
  {
    state = status::idle;
  }
}

Lift::status Lift::getStatus() const
{
  return state;
}

uint32_t Lift::getTravelled() const
{
  return travelled;
}

uint32_t Lift::getTarget() const
{
  return target;
}

uint16_t Lift::getSpeedRpm() const
{
  return speedRpm;
}

ControllerDirection Lift::toControllerDirection(direction d) const
{
  if(d == direction::forward)
  {
    return forwardDirection;
  }

  return (forwardDirection == ControllerDirection::forward)
         ? ControllerDirection::reverse
         : ControllerDirection::forward;
}
