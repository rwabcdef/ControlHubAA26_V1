/*
 * Lift.cpp
 *
 * See Lift.hpp for how directions map, what the distance is, the ground
 * sensor, and the threading contract.
 */

#include "Lift.hpp"

Lift::Lift(Controller* controller, ControllerDirection forwardDirection,
           uint32_t (*getDistance)(), bool (*atGround)())
: controller(controller), forwardDirection(forwardDirection),
  getDistance(getDistance), atGround(atGround),
  valid(false), started(false), groundMove(false), startDistance(0U),
  state(status::idle), travelled(0U), target(0U), ended(endReason::none)
{
}

bool Lift::init()
{
  valid = (controller != nullptr) &&
          (getDistance != nullptr) &&
          (forwardDirection != ControllerDirection::idle);

  return valid;
}

bool Lift::start(direction d, uint32_t distance)
{
  return beginMove(d, distance, false);
}

bool Lift::toGroundLevel(uint32_t maxDistance)
{
  if(!valid || (atGround == nullptr) || (state == status::moving) ||
     (maxDistance == 0U))
  {
    return false;
  }

  /* Checked before anything reaches the controller: setDirection() acts
     at once (Direction, Controller.hpp), so starting and letting run()
     notice would drive the lift into the ground for a moment. Recorded
     as a move that ended at once, so the status says where the lift is. */
  if(atGround())
  {
    mark(maxDistance);
    ended = endReason::ground;
    return true;
  }

  return beginMove(direction::reverse, maxDistance, true);
}

bool Lift::beginMove(direction d, uint32_t distance, bool toGround)
{
  /* A zero demand would enable a controller that holds the motor still,
     leaving a move that can never finish - see Speed in Lift.hpp. Only
     read here: the speed is the controller's, set by whoever set it. */
  if(!valid || (state == status::moving) || (distance == 0U) ||
     (controller->getRequiredRpm() == 0U))
  {
    return false;
  }

  mark(distance);
  groundMove = toGround;
  ended = endReason::none;

  /* Direction, then enable, so the controller's first pass already has
     everything it needs - the speed it already has. It seeds its output
     from the motor's present duty cycle (Bumpless start, Controller.hpp),
     which after a previous move at a similar speed is roughly right. */
  controller->setDirection(toControllerDirection(d));
  controller->enable();

  state = status::moving;
  return true;
}

void Lift::mark(uint32_t distance)
{
  startDistance = getDistance();
  started = true;
  travelled = 0U;
  target = distance;
}

void Lift::stop()
{
  end(endReason::stopped);
}

void Lift::end(endReason why)
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

  /* Only a move in progress has a reason to record - a stop() while idle
     leaves the last move's standing. */
  if(state == status::moving)
  {
    ended = why;
  }

  groundMove = false;
  state = status::idle;
}

void Lift::run()
{
  if(valid && started)
  {
    /* Unsigned difference, so a wrapped count is still right. Kept up
       after the move ends too, so the coast overrun shows. */
    travelled = getDistance() - startDistance;

    /* The ground first: on a pass where both are true the lift is on the
       ground, and that is the more useful thing to report. */
    if((state == status::moving) && groundMove && atGround())
    {
      end(endReason::ground);
    }
    else if((state == status::moving) && (travelled >= target))
    {
      end(endReason::arrived);
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
    ended = endReason::tachoFault;
    groundMove = false;
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

Lift::endReason Lift::getEndReason() const
{
  return ended;
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
