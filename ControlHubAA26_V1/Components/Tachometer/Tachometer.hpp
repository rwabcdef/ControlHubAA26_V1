/*
 * Tachometer.hpp
 *
 * Speed sensing for one motor, from a magnet-and-Hall-sensor wheel.
 *
 *   magnet passes --> 3144 open collector pulls low --> level shifter (3 V)
 *                 --> GPIO EXTI falling edge --> read TIM5->CNT --> queue
 *                 --> drained by the control task, turned into RPM
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
// to follow a change - the window is AVG_REVS revolutions long, so ~1 s
// at 240 RPM with 4. A control loop sees that as lag. Limited to 16 to
// keep the arithmetic in update() inside 32 bits.
#define TACHO__AVG_REVS 4

static_assert((TACHO__AVG_REVS >= 1) && (TACHO__AVG_REVS <= 16),
  "TACHO__AVG_REVS must be 1..16");

// No edge for this long means stopped. Must be longer than the slowest
// pulse interval the motor can legitimately produce, or a slow crawl
// reads as a stall: at 2 pulses/rev this is 60 RPM.
#define TACHO__STALL_TIMEOUT_MS 500

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

    // Diagnostics: edges rejected by the glitch guard, and edges lost to
    // a full queue. Both should stay at zero in normal running.
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
    volatile uint16_t glitchCount;
    volatile uint16_t droppedEdges;

    // Published to readers.
    volatile uint16_t rpm;
    volatile bool     stalled;
    volatile uint32_t revolutions;

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
