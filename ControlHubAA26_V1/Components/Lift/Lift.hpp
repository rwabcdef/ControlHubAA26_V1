/*
 * Lift.hpp
 *
 * A lift, driven through one speed Controller: move so far in a given
 * direction, at a fixed speed, then stop.
 *
 *   start(d, distance) --> controller: speedRpm, direction, enable
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
 * anything else still using it directly (the CTRL0 socket, for now)
 * keeps working while the lift is idle. It will fight a move, though:
 * a direction or enable change from outside mid-move is not noticed,
 * and the move then only ends when the distance comes up. Lift should
 * be the controller's only user.
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
 * Every move runs at speedRpm, given at construction. start() sets it on
 * the controller each time, so a speed set on the controller some other
 * way lasts only until the next move.
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
 * Threading
 * ---------
 * start(), stop() and run() belong to one task - the one that runs the
 * controller, and which must also be the one calling getDistance()'s
 * source's update() (Tachometer::update()), before run() each pass.
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

    // Only stores the arguments: a file scope Lift is constructed before
    // main(), when neither the HAL nor the RTOS exists.
    //
    // forwardDirection is the controller direction that moves the lift
    // forward - forward or reverse, never idle. speedRpm is the speed of
    // every move. getDistance is a plain function pointer, as in
    // ControllerConfig, so a captureless lambda will do.
    Lift(Controller* controller, ControllerDirection forwardDirection,
         uint16_t speedRpm, uint32_t (*getDistance)());
    virtual ~Lift() = default;

    // Validates the arguments. Returns false, and the lift stays idle
    // and refuses every start(), if controller or getDistance is null,
    // forwardDirection is idle, or speedRpm is zero. run() still runs
    // the controller if there is one.
    bool init();

    // Starts a move of distance units (see Distance, above) in d.
    // Returns false, and changes nothing, if the lift is already moving,
    // distance is zero, or init() did not pass.
    bool start(direction d, uint32_t distance);

    // Ends the move: disables the controller and idles the motor, which
    // coasts. Harmless when already idle.
    void stop();

    // One pass: ends the move if its distance is up, then runs the
    // controller. Call at the controller's periodMs, after the distance
    // source has been updated for this pass.
    void run();

    // Any task.
    status   getStatus() const;
    uint32_t getTravelled() const;   // since the last start()
    uint32_t getTarget() const;      // the last start()'s distance
    uint16_t getSpeedRpm() const;

  protected:
    Controller*         controller;
    ControllerDirection forwardDirection;
    uint16_t            speedRpm;
    uint32_t          (*getDistance)();

    // The controller direction that moves the lift in d.
    ControllerDirection toControllerDirection(direction d) const;

  private:
    bool valid;                // init() passed
    bool started;              // a start() has happened, so startDistance means something
    uint32_t startDistance;    // getDistance() at the last start()

    // Published to the getters.
    volatile status   state;
    volatile uint32_t travelled;
    volatile uint32_t target;
};

#endif /* LIFT_HPP_ */
