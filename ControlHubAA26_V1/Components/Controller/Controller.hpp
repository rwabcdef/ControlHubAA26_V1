/*
 * Controller.hpp
 *
 * Closed loop speed control for one motor: a required RPM in, the
 * tachometer's measured RPM as feedback, a PWM duty cycle out.
 *
 *   requiredRpm --(+)--> error --> computeOutput() --> clamp --> setPwmPercent
 *                  ^(-)                                              |
 *                  |                                               motor
 *                  +---------------- getRpm <--- tachometer <--------+
 *
 * A base class for a family of controllers
 * ----------------------------------------
 * run() is fixed here and does the parts every controller shares: read
 * the feedback, reject an implausible reading, clamp the output, write it.
 * The control law itself is the one virtual, computeOutput(). A PID
 * subclass overrides that (and onReset(), if it keeps state of its own)
 * and inherits the rest, so every controller is driven identically -
 * the owning task never needs to know which one it has.
 *
 * This base class's own law is incremental ("velocity form"):
 *
 *   output += integralGain * error
 *
 * Each pass nudges the duty cycle by an amount proportional to the error.
 * That is integral action, not proportional: the adjustment accumulates,
 * so the output keeps moving until the error is zero. It therefore has
 * no steady state error, which a pure proportional controller
 * (output = Kp * error) always does - a motor needs a non-zero duty
 * cycle to hold any speed, and Kp * error can only supply that while an
 * error remains. The gain is named for what it does.
 *
 * Because the adjustment is per pass, the effective gain scales with the
 * run() rate: the same integralGain at 20 Hz is twice as aggressive as at
 * 10 Hz. periodMs is carried in the config so subclasses that work in
 * real time units (a PID's Ki and Kd) can convert.
 *
 * Output clamping is also the anti-windup: the running output is stored
 * clamped, so a motor that cannot reach its target leaves the output
 * pinned at outputMaxPercent rather than winding up beyond it. A subclass
 * with a separate integrator term has to limit that term itself.
 *
 * The output is held as a float and only rounded when written. With a
 * small gain the per pass adjustment is well under 1%, and an integer
 * accumulator would truncate it to zero every pass - the controller
 * would then never correct any error smaller than 1 / integralGain RPM.
 *
 * Bumpless start
 * --------------
 * When run() starts driving - after enable(), or after hold() - it seeds
 * its output from the motor's present duty cycle (getPwmPercent) rather
 * than from zero, so switching from open loop to closed loop does not
 * first drop the motor to a standstill.
 *
 * Feedback lag
 * ------------
 * Tachometer averages over whole revolutions (TACHO__AVG_REVS of them),
 * so its reading lags the real speed by up to about a second at low RPM,
 * and stays at zero until the first revolution completes. An integral
 * controller keeps accumulating through that lag, and overshoots if the
 * gain is too high for it. Tune integralGain low first and raise it.
 *
 * Unresponsive tachometer
 * -----------------------
 * A tachometer that has failed or come unplugged reads zero, and to the
 * law that looks like a motor far below its demand: the output climbs
 * to outputMaxPercent and stays there. outputMaxPercent bounds how hard
 * the motor is driven meanwhile, but not for how long. So if the reading
 * stays at zero for tachoUnresponsiveTimeout_S while run() is driving
 * hard enough that the motor should be turning, with a non-zero demand,
 * run() stops the motor itself: it disables the controller, writes 0%,
 * sets the direction idle and latches a fault (isTachoFault()).
 *
 * "Hard enough" is output >= tachoCheckMinPercent, and it is what keeps
 * the check independent of the gain. From rest the output ramps up from
 * 0% (Bumpless start, below) at integralGain * demand per pass - at a
 * low gain and a low demand that is well under 1%/s, so the ramp to the
 * motor's breakaway duty alone can take tens of seconds. A timer running
 * from the start would have to allow for the slowest ramp ever used, and
 * would then let a dead tacho drive the motor for just as long. Timed
 * from tachoCheckMinPercent instead, the ramp does not count however
 * slow it is, and the timeout only has to cover what follows: the rest
 * of the climb to breakaway, if tachoCheckMinPercent is set below it,
 * and the first revolution, which the tachometer needs before it reads
 * anything (Feedback lag, below). Set tachoCheckMinPercent at or just
 * above the breakaway duty, and at or below outputMaxPercent - above it
 * the check could never run, so init() refuses that.
 *
 * Only a sustained zero counts. A reading rejected as implausible does
 * not, and one non-zero reading restarts the count - as does the output
 * dropping below tachoCheckMinPercent. Nor does a zero demand, where zero
 * is the right answer, or a pass spent in hold().
 *
 * A demand the tachometer cannot measure looks exactly like a dead one:
 * if the motor is turning below the tachometer's lowest readable speed,
 * the reading is zero, the output climbs, and the check trips. Keep
 * demands above that floor (Tachometer.hpp, TACHO__STALL_TIMEOUT_MS).
 *
 * The fault stays set, and the controller disabled, until the next
 * enable(); the first run() after that clears it and starts again. A
 * sensor that is still dead trips it again one timeout later.
 *
 * Direction
 * ---------
 * The tachometer reports speed without sign, so the controller cannot
 * sense direction - it can only set it. setDirection() passes straight
 * through to the motor via config.setDirection, whether or not the
 * controller is enabled, and getDirection() reads the motor back rather
 * than a stored copy, so a direction set some other way (the MOTOR
 * socket) is still reported truthfully. With the direction idle the
 * motor cannot respond, and the owning task should call hold() instead
 * of run().
 *
 * ControllerDirection is the controller's own type rather than the motor
 * driver's, so this class does not depend on any one driver; the config
 * callbacks translate.
 *
 * Threading
 * ---------
 * run() and hold() belong to one task. setRequiredRpm(), setIntegralGain(),
 * enable() and disable() may be called from any other task: each writes
 * one aligned scalar (a float is a single 32 bit store on this core too),
 * which this core stores atomically. A gain change lands between passes -
 * it never tears a pass, and the output carries on from where it is, so
 * retuning a running motor is bumpless. They only record the
 * request - every change to the controller's running state happens in
 * run(), in the owning task, so there is nothing for them to race with.
 *
 * setDirection() is the exception: it acts at once, in the caller's
 * task, through config.setDirection. That callback must therefore be
 * safe to call alongside run()'s setPwmPercent from the owning task.
 *
 * run() writes the output with the scheduler suspended, having checked
 * the enable flag in the same suspended section. So a task that calls
 * disable() and then drives the motor itself can never have its own
 * setting overwritten by a run() that had already passed the check.
 * setPwmPercent therefore runs with the scheduler suspended: it must be
 * short and must not block. Interrupts stay on throughout.
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <stdint.h>

// idle means no drive at all - the motor coasts. See Direction, above.
enum class ControllerDirection : uint8_t
{
  forward,
  reverse,
  idle
};

class ControllerConfig
{
  public:
    // Feedback: the measured speed. For motorB, tachoB.getRpm().
    uint16_t (*getRpm)();

    // The motor's present duty cycle, read once whenever run() starts
    // driving, so the takeover is bumpless. For motorB, motorB.getPercent().
    uint8_t (*getPwmPercent)();

    // Output. For motorB, motorB.setPercent(). Called with the scheduler
    // suspended - see Threading, above.
    void (*setPwmPercent)(uint8_t percent);

    // Direction, passed through to the motor. For motorB, motorB's
    // setDirection() and getDirection(), translated. setDirection is
    // called from whichever task calls Controller::setDirection() - see
    // Threading, above.
    void (*setDirection)(ControllerDirection direction);
    ControllerDirection (*getDirection)();

    // Duty cycle change per pass, in percent per RPM of error. See the
    // class comment for why this is integral action. The starting value
    // only - setIntegralGain() changes it at run time, until reset.
    float integralGain;

    // Output limits, percent. outputMinPercent above zero keeps the motor
    // turning over rather than stalling at low demand.
    uint8_t outputMinPercent;
    uint8_t outputMaxPercent;

    // A reading above this is treated as a sensor fault: the pass is
    // skipped and the output held where it was. It should sit well above
    // anything the motor can really do.
    uint16_t maxPlausibleRpm;

    // Seconds the reading may stay at zero, while driving with a non-zero
    // demand, before run() stops the motor and latches a fault. 0 turns
    // the check off. See Unresponsive tachometer, above.
    uint16_t tachoUnresponsiveTimeout_S;

    // The output, percent, from which that timeout counts: the duty at
    // which the motor ought to be turning. Below it a zero reading is
    // expected - the output is still ramping up to breakaway - and not
    // counted. 0 counts from the start. Must be <= outputMaxPercent. See
    // Unresponsive tachometer, above.
    uint8_t tachoCheckMinPercent;

    // How often the owning task calls run(). Unused by this class's own
    // law; there for subclasses that need real time units.
    uint16_t periodMs;
};

class Controller
{
  public:
    // Plain function pointers rather than std::function: a captureless
    // lambda converts to one, which is all a file scope motor and
    // tachometer need, and it keeps std::function's possible heap
    // allocation and its exception path out of the application layer.
    explicit Controller(const ControllerConfig& config);
    virtual ~Controller() = default;

    // Validates the config. Returns false, and the controller stays
    // disabled, if it is unusable (a null callback, min > max, or a
    // tacho timeout with a zero periodMs).
    bool init();

    // One control pass. Call periodically, at config.periodMs, from the
    // owning task. Does nothing while disabled.
    void run();

    // Call instead of run() while the motor cannot respond - its
    // direction idle, say. Leaves the output alone and the controller
    // enabled, but forgets that it was driving, so the next run()
    // re-seeds from the motor's duty cycle instead of carrying on from
    // an error it could do nothing about.
    void hold();

    // Any task. The demand. Does not enable the controller by itself.
    void setRequiredRpm(uint16_t rpm);
    uint16_t getRequiredRpm() const;

    // Any task. The gain the control law uses from the next pass. Returns
    // false, and keeps the old gain, unless 0 <= gain <= MAX_INTEGRAL_GAIN
    // (NaN fails both tests, so is refused too). Zero is allowed: the
    // output then holds wherever it is. Starts at config.integralGain;
    // init() refuses a config whose gain is outside the same range.
    bool  setIntegralGain(float gain);
    float getIntegralGain() const;

    // 1% of duty per RPM of error, per pass - far past anything stable,
    // so only a typo gets near it.
    static constexpr float MAX_INTEGRAL_GAIN = 1.0f;

    // Any task. enable() hands the motor to the controller from the next
    // run(); disable() hands it back, and nothing more is written.
    // enable() is also how a tacho fault is cleared - see Unresponsive
    // tachometer, above.
    void enable();
    void disable();
    bool isEnabled() const;

    // Any task. Acts immediately, enabled or not - see Direction, above.
    // getDirection() reads the motor, not a stored copy.
    void setDirection(ControllerDirection direction);
    ControllerDirection getDirection() const;

    // Any task. The motor's duty cycle now, read through
    // config.getPwmPercent. Unlike getOutputPercent() this is right in
    // open loop too, whoever set it; while enabled the two agree.
    uint8_t getPwmPercent() const;

    // Any task. The measured speed now, read through config.getRpm - the
    // feedback, as the next run() would see it (so possibly a reading
    // run() will reject as implausible). 0 before init() has passed.
    uint16_t getRpm() const;

    // The last value written, rounded, and the unrounded one behind it.
    uint8_t getOutputPercent() const;
    float getOutput() const;

    // Diagnostics: readings rejected as implausible. Should stay at zero.
    uint16_t getRejectedCount() const;

    // Any task. True once run() has stopped the motor because the
    // tachometer read zero for tachoUnresponsiveTimeout_S with the output
    // at or above tachoCheckMinPercent; stays true
    // until the first run() after the next enable().
    bool isTachoFault() const;

  protected:
    // The control law. Given the error (required - measured; positive
    // means too slow), the measured speed and the present output, return
    // the new output in percent. It need not clamp - run() does.
    virtual float computeOutput(int32_t errorRpm, uint16_t rpm, float output);

    // Called when run() starts driving. A subclass with state of its own
    // (an integrator, a previous error) clears it here.
    virtual void onReset();

    ControllerConfig config;

  private:
    volatile uint16_t requiredRpm;
    volatile float    integralGain;   // the live gain; config.integralGain is its start
    volatile bool     enabled;
    volatile uint16_t rejectedCount;
    volatile bool     tachoFault;   // written by the owning task only

    // Owning task only.
    bool  driving;   // false until the first run() after enable() / hold()
    bool  valid;     // init() passed
    volatile float output;  // unrounded; read by getOutput() from any task
    uint32_t zeroRpmPasses;      // consecutive driving passes reading zero
    uint32_t zeroRpmLimitPasses; // the timeout in passes; 0 = check off

    float clamp(float value) const;
    void  stopForTachoFault();
};

#endif /* CONTROLLER_HPP_ */
