/*
 * Tachometer.hpp
 *
 * Speed sensing for one motor, from a magnet-and-Hall-sensor wheel.
 *
 *   magnet passes --> 3144 open collector pulls low --> level shifter (3 V)
 *                 --> GPIO EXTI falling edge --> read TIM5->CNT --> queue
 *                 --> drained by the control task, turned into RPM
 *
 * Edge filtering
 * --------------
 * onEdge() accepts a falling edge only if it passes three checks:
 *
 *   1. The pin still reads low in the ISR - rejects spikes on a high line
 *      that have gone by the time the ISR looks.
 *   2. The line has been seen high since the last accepted edge ("armed")
 *      - rejects spikes on a LOW line. The low state is held only by the
 *      level shifter's ~8.8k divider, so motor PWM spikes lift it briefly
 *      and each falls back as a fresh falling edge with the pin still low,
 *      which check 1 cannot see. At 30% duty these arrived several ms
 *      after the real edge and read as extra revolutions (348 RPM showing
 *      as 390, 464, 570). The EXTI interrupts on both edges so that the
 *      rising one, the magnet leaving, can re-arm.
 *   3. At least TACHO__MIN_EDGE_TICKS since the last accepted edge -
 *      rejects chatter on the real edge, where a spike can re-arm and
 *      then re-trigger within a few hundred us.
 *
 * Hardware filtering on PF4 (a 2200 pF cap to ground) reduces how much
 * of this reaches the pin. Much larger is worse, not better: the falling
 * edge discharges only through the ~8.8k divider, and 0.1 uF made it slow
 * enough for noise to cross the input threshold several times.
 *
 * Why a free running counter and not a 10 kHz tick ISR
 * ---------------------------------------------------
 * The timestamp comes from TIM5 running free at TICK_HZ with no interrupt
 * of its own: the edge ISR just reads CNT. A software tick counter would
 * cost TICK_HZ interrupts per second to maintain a number the timer
 * already holds. TIM5 is one of only two 32 bit timers on the F4 (the
 * other, TIM2, is the ADC's TRGO source), which is what lets every
 * interval be a plain uint32_t subtraction - the counter wraps every
 * 2^32 ticks and unsigned arithmetic carries straight through it, so
 * there is no wrap case to handle anywhere in this class.
 *
 * Why not input capture
 * ---------------------
 * Capture would latch CNT in hardware and so carry no interrupt latency
 * at all. At these rates that does not matter: a pulse interval is
 * milliseconds and ISR latency here is a few microseconds, well under one
 * tick. Capture would also cost a timer channel pin and, on every free
 * timer on this board, a 16 bit counter and its wrap handling. The EXTI
 * pin is the cheaper trade. Revisit only if the signal needs the hardware
 * input filter that capture brings and this class's glitch guard cannot
 * do the job.
 *
 * Why whole revolutions, not single intervals
 * -------------------------------------------
 * PULSES_PER_REV magnets glued to a wheel by hand are never exactly
 * evenly spaced, and their airgaps differ, so the sensor switches at
 * slightly different angles for each one. On this wheel the two intervals
 * measured about 290 and 240 units on a scope - a 19% alternation at
 * constant speed. Speed taken from any single interval would therefore
 * oscillate ~10% about the true value, which is roughly fifty times the
 * quantisation of a 10 kHz timebase.
 *
 * So update() only ever times a *whole* revolution: it counts edges and
 * uses the interval between every PULSES_PER_REV-th one. Whatever the
 * spacing error is, it cancels over a full turn. On top of that the
 * reading is a moving average over the last TACHO__AVG_REVS revolutions,
 * which damps what noise the edge filtering in onEdge() lets through and
 * is free extra resolution at speed.
 *
 * Stall
 * -----
 * An edge driven sensor cannot report "stopped" - it just goes quiet, and
 * the last computed speed would otherwise stand forever. update() watches
 * the time since the last edge and forces the reading to zero after
 * TACHO__STALL_TIMEOUT_MS, discarding the partial revolution so a restart
 * is not timed from before the stop.
 *
 * Threading
 * ---------
 * onEdge() runs in the EXTI ISR. Everything else - init(), update() and
 * the getters - belongs to one task, and init() must finish before the
 * scheduler starts. The ISR and that task share only the queue and a few
 * volatile scalars; each of those is a single aligned word or half-word,
 * which this core loads and stores atomically, so no lock is needed.
 *
 * update() is not a blocking run() like Transport's or Adc's - it drains
 * what is there and returns. The caller supplies the period, the same way
 * Led::run() is driven.
 */

#ifndef TACHOMETER_HPP_
#define TACHOMETER_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

// Timestamps buffered between update() calls. An edge arriving with the
// queue full is counted in getDroppedEdges() and lost, which shows up as
// a low reading rather than a wild one. 16 slots covers
//   16 edges / update period / PULSES_PER_REV  revolutions
// = 9600 RPM at a 50 ms period, well past this gearbox.
#define TACHO__QUEUE_LENGTH 16

// Edges closer together than this are treated as a glitch and dropped.
// 20 ticks at 10 kHz is 2 ms. The magnets split a turn roughly 45/55, so
// the shorter interval is ~0.45 rev and the ceiling is about 13,500 RPM -
// still far above this gearbox. Was 2 ticks (200 us), which let through
// noise landing a few hundred us after a real edge. The level check in
// onEdge() handles spikes; this handles chatter around a real edge.
#define TACHO__MIN_EDGE_TICKS 20

// Moving average: the reading is taken over the most recent this-many
// whole revolutions (total revolutions / total time), updated at every
// revolution rather than at every update(). More is smoother but slower
// to follow a change - the window is AVG_REVS revolutions long, so 2.4 s
// at 50 RPM with 2 (it was 4, and 4.8 s). A control loop sees that as
// lag, and at the low speeds a lift runs at it dominates. Whole
// revolutions, so it means the same whatever PULSES_PER_REV is. Limited
// to 16 to keep the arithmetic in update() inside 32 bits.
#define TACHO__AVG_REVS 2

static_assert((TACHO__AVG_REVS >= 1) && (TACHO__AVG_REVS <= 16),
  "TACHO__AVG_REVS must be 1..16");

// No edge for this long means stopped. Must be longer than the slowest
// pulse interval the motor can legitimately produce, or a slow crawl
// reads as a stall - the reading drops to zero between edges, and a
// controller holding that speed surges instead.
//
// The slowest readable speed is set by the LONGEST gap between magnets,
// not the average: with 2 magnets split ~45/55 that is 0.55 rev, so
//   floor ~= 0.55 * 60000 / TACHO__STALL_TIMEOUT_MS  RPM
// 500 ms gave ~66 RPM, which 50 RPM demands fell under. 2000 ms gives
// ~17 RPM (~9 RPM with 4 magnets at 90 degrees, longest gap ~0.3 rev).
//
// The cost: after a real stop, the last speed is reported for up to this
// long before it drops to zero.
#define TACHO__STALL_TIMEOUT_MS 2000

// Numerically at or below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
// which is what makes the xQueueSendFromISR() in onEdge() legal. Matches
// the EXTI9_5 line CubeMX configures for the radio's nINT.
#define TACHO__EXTI_PRIORITY 7

class Tachometer
{
  public:
    // TIM5 tick rate. The prescaler is derived from the live APB1 clock,
    // so this stays correct if the clock tree is retuned. 10 kHz gives
    // 100 us resolution: about 1% on a single revolution at 6000 RPM,
    // and better than that whenever update() sees more than one.
    static const uint32_t TICK_HZ = 10000U;

    // Magnets on the wheel, i.e. falling edges per revolution.
    static const uint8_t PULSES_PER_REV = 2U;

    // timebase must be a 32 bit timer (TIM2 or TIM5 on this part); the
    // wrap-free arithmetic in this class depends on it. port/pin is the
    // sensor input, and its pin NUMBER picks the EXTI line - see the
    // note on irqnForPin() in Tachometer.cpp.
    Tachometer(TIM_TypeDef* timebase, GPIO_TypeDef* port, uint16_t pin);

    // Creates the queue, starts the timebase, configures the pin and
    // enables the interrupt - in that order, so no edge can arrive
    // before there is somewhere to put it. Call once, before the
    // scheduler starts.
    bool init();

    // Drains the queue and recomputes. Call periodically from one task.
    void update();

    // Latest speed. Zero once the stall timeout has expired.
    uint16_t getRpm() const;
    bool isStalled() const;

    // Total whole revolutions since init().
    uint32_t getRevolutions() const;

    // Every accepted edge since init(), counted as update() drains it -
    // PULSES_PER_REV of them per revolution. Unlike getRevolutions() it is
    // not held back by the whole-revolution timing and loses nothing to a
    // stall, so it is the one to measure distance with: a move from rest
    // registers from its first edge, and a stop discards no part turn.
    uint32_t getEdges() const;

    // Diagnostics: low-reading edges rejected by the arming check or
    // TACHO__MIN_EDGE_TICKS, and edges lost to a full queue. Spikes on a
    // high line are not counted - they cannot be told apart from real
    // rising edges, which take the same path. The glitch count rising
    // with motor duty means PWM noise is reaching the pin; it only
    // matters if the reading moves with it. Dropped edges should stay
    // at zero.
    uint16_t getGlitchCount() const;
    uint16_t getDroppedEdges() const;

    //-------------------------------------
    // ISR entry. Call from HAL_GPIO_EXTI_Callback() for this object's
    // pin. Not application interface.
    void onEdge();
    //-------------------------------------

  private:
    TIM_TypeDef*  timebase;
    GPIO_TypeDef* port;
    uint16_t      pin;

    TIM_HandleTypeDef timerHandle;

    QueueHandle_t queue;
    StaticQueue_t staticQueue;
    uint8_t       queueStorageArea[TACHO__QUEUE_LENGTH * sizeof(uint32_t)];

    // ISR side: the previous accepted edge, for the glitch guard.
    volatile uint32_t lastEdgeTick;
    volatile bool     hasLastEdge;

    // ISR side: true once the line has been seen high since the last
    // accepted edge. Cleared by accepting one. See "Edge filtering".
    volatile bool     armed;
    volatile uint16_t glitchCount;
    volatile uint16_t droppedEdges;

    // Published to readers.
    volatile uint16_t rpm;
    volatile bool     stalled;
    volatile uint32_t revolutions;
    volatile uint32_t edges;

    // Task side only, carried across update() calls: the edge that
    // opened the revolution currently being timed, and how many edges
    // have arrived since.
    uint32_t revStartTick;
    bool     hasRevStart;
    uint8_t  edgesSinceRevStart;

    // Task side only: the moving average window. A ring of the last
    // TACHO__AVG_REVS revolution times with a running sum; avgCount is
    // how many slots are valid, which is fewer until the ring fills.
    uint32_t avgTicks[TACHO__AVG_REVS];
    uint8_t  avgIndex;
    uint8_t  avgCount;
    uint32_t avgTicksSum;

    // Task side only: when the last edge of any kind was seen, for the
    // stall timeout. Valid only while hasSeenEdge.
    uint32_t lastSeenTick;
    bool     hasSeenEdge;

    bool startTimebase();
    bool startPin();
    void pushRevTicks(uint32_t ticks);
};

#endif /* TACHOMETER_HPP_ */
