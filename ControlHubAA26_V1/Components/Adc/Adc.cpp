/*
 * Adc.cpp
 *
 * See Adc.hpp for the sampling model and the buffer layout.
 */

#include "Adc.hpp"
#include "task.h"

// How often run() checks the sampling is still alive. Must be comfortably
// longer than the publish period, or every check would look like a stall.
#define ADC__SUPERVISE_PERIOD_MS 500

Adc* Adc::instances[Adc::MAX_INSTANCES] = { nullptr };
uint8_t Adc::numInstances = 0;

Adc::Adc(ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* trigger, uint8_t numChannels)
  : queue(nullptr),
    hadc(hadc),
    trigger(trigger),
    numChannels(numChannels),
    referenceMillivolts(DEFAULT_REFERENCE_MV),
    sequence(0),
    errorFlag(false),
    overrunCount(0),
    restartCount(0)
{
  if(this->numChannels > MAX_CHANNELS)
  {
    this->numChannels = MAX_CHANNELS;
  }

  for(uint8_t ch = 0U; ch < MAX_CHANNELS; ch++)
  {
    latestCount[ch] = 0U;
  }

  /* Only stores its arguments - no HAL and no RTOS - so a file scope Adc is
     safe. The constructor runs from __libc_init_array, before HAL_Init() and
     before the scheduler. The real work is in init() and start(). */
}

bool Adc::init()
{
  if((hadc == nullptr) || (trigger == nullptr) || (numChannels == 0U))
  {
    return false;
  }

  if(numInstances >= MAX_INSTANCES)
  {
    return false;
  }

  /* One deep: the newest set always wins, so a slow reader cannot block the
     interrupt or stall the sample path. */
  queue = xQueueCreateStatic(1, sizeof(Sample), queueStorageArea, &staticQueue);
  if(queue == nullptr)
  {
    return false;
  }

  /* Registered before start(), so no callback can arrive with this object
     still unknown to instanceFor(). */
  instances[numInstances] = this;
  numInstances++;

  return true;
}

bool Adc::start()
{
  /* ADC first, timer second. The other way round the first TRGO arrives with
     no DMA armed to service it, which sets the overrun flag - and once OVR is
     set the ADC stops issuing DMA requests permanently, with no other
     symptom than readings that never change again.

     HAL_ADC_Start_DMA() takes a uint32_t*, but the transfer width is set by
     the DMA config (half-word here), not by that type. The cast is what every
     ST example does. */
  if(HAL_ADC_Start_DMA(hadc, (uint32_t*)buffer,
       (uint32_t)SCANS * (uint32_t)numChannels) != HAL_OK)
  {
    return false;
  }

  if(HAL_TIM_Base_Start(trigger) != HAL_OK)
  {
    return false;
  }

  return true;
}

/* Recovers from the one failure this arrangement has: an ADC overrun. If a
   conversion completes before the DMA has taken the previous result, OVR is
   set and the ADC stops issuing DMA requests for good. Nothing else reports
   it - the buffer simply stops being written and every reading freezes at its
   last value, which looks exactly like a sensor that has gone quiet.

   MX_ADC1_Init() leaves the ADC global interrupt disabled, so HAL never calls
   HAL_ADC_ErrorCallback() for OVR. The flag is therefore polled here instead.
   Enabling ADCx_IRQn in CubeMX would make it immediate, but at these rates a
   check twice a second is ample.

   The sequence test catches the same stall from any other cause - a DMA error,
   or a trigger that stopped - without needing to know which. */
void Adc::run()
{
  const uint32_t seenSequence = sequence;

  vTaskDelay(pdMS_TO_TICKS(ADC__SUPERVISE_PERIOD_MS));

  const bool overrun = (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR) != 0U);
  const bool stalled = (sequence == seenSequence);

  if(overrun)
  {
    overrunCount++;
  }

  if(overrun || stalled || errorFlag)
  {
    errorFlag = false;
    restart();
  }
}

bool Adc::restart()
{
  restartCount++;

  HAL_TIM_Base_Stop(trigger);
  HAL_ADC_Stop_DMA(hadc);
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);

  return start();
}

bool Adc::read(Sample* sample, uint32_t timeoutMs)
{
  if((queue == nullptr) || (sample == nullptr))
  {
    return false;
  }

  return (xQueueReceive(queue, sample, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
}

uint16_t Adc::getCount(uint8_t channel) const
{
  if(channel >= numChannels)
  {
    return 0U;
  }

  return latestCount[channel];
}

uint16_t Adc::getMillivolts(uint8_t channel) const
{
  /* 4095 counts full scale, so the product peaks at 4095 * 3300, well inside
     32 bits. Integer division truncates, which is the right way round for a
     measurement - it never reads high. */
  const uint32_t counts = (uint32_t)getCount(channel);

  return (uint16_t)((counts * (uint32_t)referenceMillivolts) / (uint32_t)COUNT_MAX);
}

uint8_t Adc::getNumChannels() const
{
  return numChannels;
}

uint16_t Adc::getOverrunCount() const
{
  return overrunCount;
}

uint16_t Adc::getRestartCount() const
{
  return restartCount;
}

void Adc::setReferenceMillivolts(uint16_t millivolts)
{
  if(millivolts != 0U)
  {
    referenceMillivolts = millivolts;
  }
}

void Adc::onHalfComplete()
{
  /* Rows 0 .. SCANS_PER_HALF-1 are complete; the DMA is now filling the
     second half. */
  average(0U);
}

void Adc::onComplete()
{
  /* Second half complete; the DMA has wrapped and is filling the first. */
  average(SCANS_PER_HALF);
}

void Adc::onError()
{
  /* Left for run() to act on, in task context. HAL_ADC_Stop_DMA() polls for
     the stream to disable using HAL_GetTick(), which does not advance inside
     an interrupt at or above the tick's own priority - restarting from here
     could spin for the whole HAL timeout. */
  errorFlag = true;
}

Adc* Adc::instanceFor(const ADC_HandleTypeDef* hadc)
{
  for(uint8_t i = 0U; i < numInstances; i++)
  {
    if((instances[i] != nullptr) && (instances[i]->hadc == hadc))
    {
      return instances[i];
    }
  }

  return nullptr;
}

void Adc::average(uint16_t startRow)
{
  uint32_t accumulator[MAX_CHANNELS] = { 0U };

  for(uint16_t row = startRow; row < (startRow + SCANS_PER_HALF); row++)
  {
    const uint16_t base = (uint16_t)(row * numChannels);

    for(uint8_t ch = 0U; ch < numChannels; ch++)
    {
      accumulator[ch] += buffer[base + ch];
    }
  }

  Sample sample;

  for(uint8_t ch = 0U; ch < MAX_CHANNELS; ch++)
  {
    sample.count[ch] = (ch < numChannels)
      ? (uint16_t)(accumulator[ch] / (uint32_t)SCANS_PER_HALF)
      : 0U;

    if(ch < numChannels)
    {
      latestCount[ch] = sample.count[ch];
    }
  }

  sequence++;
  sample.numChannels = numChannels;
  sample.sequence = sequence;

  BaseType_t higherPriorityTaskWoken = pdFALSE;

  xQueueOverwriteFromISR(queue, &sample, &higherPriorityTaskWoken);

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/* -------------------------------------------------------------------------- */
/* HAL __weak callback overrides.
 *
 * The extern "C" is mandatory. Without it these compile to mangled symbols,
 * HAL's __weak definitions stay live, the image links cleanly and the
 * callbacks are never called - see the worked example in app_main.cpp.
 *
 * These run in the DMA stream interrupt. MX_DMA_Init() sets that to preemption
 * priority 5, which equals configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, so
 * the FromISR call in average() is legal. Lowering that number would break it.
 */

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  Adc* adc = Adc::instanceFor(hadc);

  if(adc != nullptr)
  {
    adc->onHalfComplete();
  }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  Adc* adc = Adc::instanceFor(hadc);

  if(adc != nullptr)
  {
    adc->onComplete();
  }
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  Adc* adc = Adc::instanceFor(hadc);

  if(adc != nullptr)
  {
    adc->onError();
  }
}
