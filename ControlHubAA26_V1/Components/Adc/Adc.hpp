/*
 * Adc.hpp
 *
 * Continuous multi-channel sampling for one ADC. The conversions are paced
 * by a timer and carried by DMA, so nothing in software touches the sample
 * path until a whole batch is ready to be averaged.
 *
 *   TIM TRGO --> ADC scan (rank 1..N) --> DMA --> buffer --> average --> queue
 *
 * Why scan + DMA rather than one conversion at a time
 * ---------------------------------------------------
 * The ADC has a single data register and a single EOC flag shared by the
 * whole sequence, so in scan mode each conversion overwrites the previous
 * one. Without DMA every result would have to be collected within one
 * conversion time or be lost. DMA is what makes a multi-channel sequence
 * usable at all - it is not an optimisation.
 *
 * Buffer layout
 * -------------
 * One buffer for the whole ADC, interleaved by RANK, because the DMA does
 * one transfer per conversion and walks the memory pointer as it goes. For
 * a 4 channel sequence of IN0, IN3, IN4, IN5:
 *
 *   index:  0    1    2    3    4    5    6    7   ...
 *   value: IN0  IN3  IN4  IN5  IN0  IN3  IN4  IN5  ...
 *          `------ scan 0 -----'`------ scan 1 -----'
 *
 * There is no per-channel buffer, and no hardware able to demultiplex into
 * one - a single channel is read by striding the flat buffer by numChannels.
 *
 * Ping-pong
 * ---------
 * The DMA runs circular over SCANS rows and raises two interrupts per lap:
 * half-complete after the first SCANS/2 rows, complete after the rest. Each
 * one hands over the half the DMA is NOT currently writing, so a batch can
 * be averaged with no risk of it being overwritten underneath, and the
 * transfer never has to be stopped or restarted.
 *
 * With the stock 320 Hz trigger and SCANS = 64 that is one averaged sample
 * set every 32 rows = 100 ms, i.e. 10 Hz out of 32x oversampling. The F4 has
 * no hardware oversampler, so the averaging is done here.
 *
 * Threading
 * ---------
 * The averaging runs in the DMA interrupt - a few hundred adds, microseconds
 * at these rates - and publishes a Sample to queue with xQueueOverwriteFromISR.
 * queue holds exactly one item: for periodic sensor data a stale reading is
 * worthless, so the newest always wins and a slow reader can never block the
 * ISR or stall the sample path.
 *
 * getCount() / getMillivolts() read the last published values without
 * touching the queue, for callers that want the current value rather than
 * the next one - a SerLink instant handler, say. Each is a single half-word
 * load, which the M4 does atomically, so a reader can see values that are
 * one publish stale but never a torn value.
 *
 * run() is the supervisor and must be called in a loop from its own task.
 * See the note on overruns in Adc.cpp.
 *
 * Memory
 * ------
 * The sample buffer must live somewhere the DMA controller can reach. On the
 * F4 that means ordinary SRAM: DMA1 and DMA2 cannot access CCM RAM at
 * 0x10000000, and a buffer placed there fails silently. Plain statics land in
 * SRAM, so this only matters if something is later given an explicit section.
 */

#ifndef ADC_HPP_
#define ADC_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

class Adc
{
  public:
    // Ranks this class can carry. Raising it costs buffer, so it is kept to
    // what an F4 ADC can actually reach on this board.
    static const uint8_t MAX_CHANNELS = 8;

    // Rows in the circular buffer. Half of this is what each interrupt
    // averages, so it sets the output rate for a given trigger frequency:
    //   output Hz = trigger Hz / (SCANS / 2)
    static const uint16_t SCANS = 64;
    static const uint16_t SCANS_PER_HALF = SCANS / 2;

    // Full scale for the 12 bit resolution MX_ADCx_Init() selects.
    static const uint16_t COUNT_MAX = 4095;

    // VREF+ is tied to VDD on this board, so this is nominal rather than
    // measured. Sample the internal VREFINT channel and call
    // setReferenceMillivolts() if absolute accuracy matters.
    static const uint16_t DEFAULT_REFERENCE_MV = 3300;

    // Distinct Adc objects the static HAL callbacks can dispatch to - one
    // per ADC peripheral (ADC1, ADC2, ADC3).
    static const uint8_t MAX_INSTANCES = 3;

    // One averaged set of readings, in rank order.
    class Sample
    {
      public:
        uint16_t count[MAX_CHANNELS];
        uint8_t  numChannels;

        // Increments once per published set. Lets a consumer tell a fresh
        // set from a repeat, and is what run() watches to spot a stall.
        uint32_t sequence;
    };

    // hadc and trigger must already be initialised by their MX_..._Init()
    // functions, with the ADC set to scan numChannels ranks, triggered by
    // that timer's TRGO, and DMAContinuousRequests enabled.
    Adc(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* trigger, uint8_t numChannels);

    // Creates queue and registers this object for the HAL callbacks. Call
    // once, before start(), and before the scheduler starts.
    bool init();

    // Arms the DMA and starts the trigger. Sampling is free running from
    // here on - no task is needed to keep it going.
    bool start();

    // Supervisor. Call repeatedly from one task; it blocks internally, so
    // the loop needs nothing else. See Adc.cpp for what it recovers from.
    void run();

    // Blocks until a set is available (or timeoutMs elapses). Because queue
    // is one deep and overwritten, this always yields the newest set.
    bool read(Sample* sample, uint32_t timeoutMs);

    // Last published values, without consuming anything. channel is the
    // rank index, 0 based: 0 is rank 1, the first channel of the sequence.
    // Out of range channels read 0.
    uint16_t getCount(uint8_t channel) const;
    uint16_t getMillivolts(uint8_t channel) const;

    uint8_t getNumChannels() const;

    // Diagnostics: overruns seen, and restarts performed by run().
    uint16_t getOverrunCount() const;
    uint16_t getRestartCount() const;

    void setReferenceMillivolts(uint16_t millivolts);

    // Published sets, one deep. Read via read(), or block on it directly.
    QueueHandle_t queue;

    //-------------------------------------
    // Called by the HAL callbacks in Adc.cpp. Not application interface.
    void onHalfComplete();
    void onComplete();
    void onError();
    static Adc* instanceFor(const ADC_HandleTypeDef* hadc);
    //-------------------------------------

  private:
    static Adc*   instances[MAX_INSTANCES];
    static uint8_t numInstances;

    ADC_HandleTypeDef* hadc;
    TIM_HandleTypeDef* trigger;
    uint8_t numChannels;
    uint16_t referenceMillivolts;

    // Flat, not [SCANS][MAX_CHANNELS]: rows are packed at the real channel
    // count, so the stride is numChannels and the DMA length is
    // SCANS * numChannels.
    uint16_t buffer[SCANS * MAX_CHANNELS];

    // Written by the interrupt, read by anyone. Half-word loads are atomic
    // on this core, so no lock is needed for a per-channel read.
    volatile uint16_t latestCount[MAX_CHANNELS];
    volatile uint32_t sequence;

    volatile bool     errorFlag;
    volatile uint16_t overrunCount;
    volatile uint16_t restartCount;

    StaticQueue_t staticQueue;
    uint8_t queueStorageArea[sizeof(Sample)];

    // Averages SCANS_PER_HALF rows from startRow and publishes the result.
    void average(uint16_t startRow);

    bool restart();
};

#endif /* ADC_HPP_ */
