/*
 * Lift.hpp
 *
 * A lift, driven through one speed Controller: move so far in a given
 * direction, then stop. Direction and distance only - the speed is the
 * controller's business.
 *
 *   start(d, distance) --> controller: direction, enable
 *   run(), each pass   --> travelled = getDistance() - startDistance
 *                          travelled >= distance?  --> stop()
 *                      --> controller.run() (or hold() while idle)
 *
 * Lift owns its controller
 * ------------------------
 * run() is the controller's run() - the owning task calls lift.run()
 * and never controller.run() itself. That puts the idle -> hold() rule
 * here, next to the distance check, rather than in the task.
 *
 * The controller is run whether a move is in progress or not, so
 * anything else using it directly (the CTRL0 socket) keeps working while
 * the lift is idle. Mid-move, only its speed should be touched: a
 * direction or enable change from outside is not noticed, and the move
 * then only ends when the distance comes up (or never, if it stops the
 * motor - stop the lift instead).
 *
 * Direction
 * ---------
 * Lift::direction is the lift's own sense of forward and reverse, not
 * the motor's. Which motor direction moves the lift forward depends on
 * how the motor is mounted and wired, so it is given to the constructor
 * as forwardDirection; reverse is then the other one.
 *
 * Speed
 * -----
 * Not the lift's concern. A move runs at whatever speed the controller
 * has been given (Controller::setRequiredRpm(), e.g. through CTRL0), and
 * start() leaves it alone - so set the speed first, and it can be
 * changed mid-move too. start() does refuse a zero demand, though: the
 * controller would hold the motor still, and a move that can never
 * cover its distance would sit "moving" for ever (the tacho timeout
 * does not catch it - zero speed is what was asked for). For the same
 * reason, setting the demand to zero mid-move leaves the move stuck
 * until stop().
 *
 * Distance
 * --------
 * getDistance() is any monotonic count - for motorB, tachoB.getEdges().
 * Lift only ever takes differences of it, in unsigned arithmetic, so the
 * count wrapping is harmless. The tacho cannot sense direction, so this
 * is distance travelled, not position.
 *
 * stop() lets the motor coast, so a move overruns its target by however
 * far the lift coasts. getTravelled() keeps counting after a move ends,
 * until the next start(), so that overrun can be read back.
 *
 * If the controller stops the motor for a tacho fault (Unresponsive
 * tachometer, Controller.hpp), run() ends the move there and then, short
 * of its target: travelled < target in the status is how it shows, and
 * controller->isTachoFault() says why.
 *
 * Ground level
 * ------------
 * toGroundLevel(maxDistance) is a reverse move that also ends when the
 * ground sensor reads active - whichever comes first, the sensor or
 * maxDistance. maxDistance is the backstop for a sensor that never
 * closes (unplugged, or the lift jammed short of it), so make it a bit
 * more than the full travel.
 *
 * The sensor is atGround, a plain function pointer like getDistance,
 * returning true while the lift is on the ground. Lift knows nothing of
 * the pin behind it. It is sampled once per run() pass, before the
 * distance check, and a single active sample ends the move - no
 * debounce, because a bounce on closing still means the ground was
 * reached, and stopping early on a noise spike is the safe way to be
 * wrong. As with a distance stop the motor coasts, so it runs on past
 * the switch by the coast.
 *
 * Only ground moves watch the sensor; start(reverse, ...) drives past it.
 * Already on the ground, toGroundLevel() does not move at all: it
 * records a move that ended there at once (travelled 0, getEndReason()
 * ground) and returns true, without the direction or enable ever
 * reaching the controller.
 *
 * getEndReason() says why the last move ended, so a ground move that
 * ran out of distance can be told from one that found the ground.
 *
 * Threading
 * ---------
 * start(), toGroundLevel(), stop() and run() belong to one task - the
 * one that runs the controller, and which must also be the one calling
 * getDistance()'s source's update() (Tachometer::update()), before run()
 * each pass. atGround is only called from those, so from that task.
 *
 * The getters may be called from any task: each reads one aligned
 * scalar, stored atomically on this core. Read together they can be
 * from either side of a pass - never torn, but possibly one pass apart.
 */

#ifndef LIFT_HPP_
#define LIFT_HPP_

#include <stdint.h>
#include "Controller.hpp"

class Lift
{
  public:
    enum class direction : uint8_t
    {
      forward,
      reverse
    };

    enum class status : uint8_t
    {
      idle,     // stopped: never started, arrived, stop() called, or tacho fault
      moving
    };

    // Why the last move ended. none until one has.
    enum class endReason : uint8_t
    {
      none,
      arrived,      // distance reached
      ground,       // ground sensor active (toGroundLevel() only)
      stopped,      // stop() called
      tachoFault    // the controller stopped the motor
    };

    // Only stores the arguments: a file scope Lift is constructed before
    // main(), when neither the HAL nor the RTOS exists.
    //
    // forwardDirection is the controller direction that moves the lift
    // forward - forward or reverse, never idle. getDistance and atGround
    // are plain function pointers, as in ControllerConfig, so a
    // captureless lambda will do. atGround may be null, and then
    // toGroundLevel() is refused.
    Lift(Controller* controller, ControllerDirection forwardDirection,
         uint32_t (*getDistance)(), bool (*atGround)() = nullptr);
    virtual ~Lift() = default;

    // Validates the arguments. Returns false, and the lift stays idle
    // and refuses every start(), if controller or getDistance is null, or
    // forwardDirection is idle. run() still runs the controller if there
    // is one.
    bool init();

    // Starts a move of distance units (see Distance, above) in d.
    // Returns false, and changes nothing, if the lift is already moving,
    // distance is zero, the controller's required speed is zero (see
    // Speed, above), or init() did not pass.
    bool start(direction d, uint32_t distance);

    // Starts a reverse move that ends at the ground sensor or after
    // maxDistance, whichever is first - see Ground level, above. Refused
    // as start() is, and also if there is no atGround. Returns true
    // without moving if already on the ground.
    bool toGroundLevel(uint32_t maxDistance);

    // Ends the move: disables the controller and idles the motor, which
    // coasts. Harmless when already idle.
    void stop();

    // One pass: ends the move if it is on the ground (ground moves only)
    // or its distance is up, then runs the controller. Call at the
    // controller's periodMs, after the distance source has been updated
    // for this pass.
    void run();

    // Any task.
    status    getStatus() const;
    uint32_t  getTravelled() const;   // since the last start()
    uint32_t  getTarget() const;      // the last start()'s distance
    endReason getEndReason() const;   // of the last move; none while moving

  protected:
    Controller*         controller;
    ControllerDirection forwardDirection;
    uint32_t          (*getDistance)();
    bool              (*atGround)();

    // The controller direction that moves the lift in d.
    ControllerDirection toControllerDirection(direction d) const;

  private:
    bool valid;                // init() passed
    bool started;              // a start() has happened, so startDistance means something
    bool groundMove;           // the move in progress watches atGround
    uint32_t startDistance;    // getDistance() at the last start()

    // start() and toGroundLevel(), once their own checks have passed.
    bool beginMove(direction d, uint32_t distance, bool toGround);

    // Resets the travelled count and target for a new move.
    void mark(uint32_t distance);

    // stop()'s work, recording why.
    void end(endReason why);

    // Published to the getters.
    volatile status    state;
    volatile uint32_t  travelled;
    volatile uint32_t  target;
    volatile endReason ended;
};

#endif /* LIFT_HPP_ */
