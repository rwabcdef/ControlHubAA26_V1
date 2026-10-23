/*
 * Rtc.cpp
 *
 * See Rtc.hpp.
 */

#include "Rtc.hpp"
#include "task.h"

Rtc::Rtc(RTC_HandleTypeDef* hrtc, uint32_t markerRegister)
{
  this->hrtc = hrtc;
  this->markerRegister = markerRegister;
  this->mutex = nullptr;
}

void Rtc::init()
{
  this->mutex = xSemaphoreCreateMutexStatic(&this->mutexBuffer);
  configASSERT(this->mutex != nullptr);

  /* MX_RTC_Init() leaves a running calendar alone, and so also skips the
     resync it would otherwise do on leaving init mode. RSF is in ISR, which
     is write protected. */
  __HAL_RTC_WRITEPROTECTION_DISABLE(this->hrtc);
  (void)HAL_RTC_WaitForSynchro(this->hrtc);
  __HAL_RTC_WRITEPROTECTION_ENABLE(this->hrtc);
}

bool Rtc::lock()
{
  if ((this->mutex == nullptr) ||
      (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
  {
    return false;
  }
  (void)xSemaphoreTake(this->mutex, portMAX_DELAY);
  return true;
}

void Rtc::unlock(bool locked)
{
  if (locked)
  {
    (void)xSemaphoreGive(this->mutex);
  }
}

bool Rtc::isValid(const DateTime& dt)
{
  static const uint8_t DAYS_IN_MONTH[12] =
    { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  if ((dt.year < YEAR_MIN) || (dt.year > YEAR_MAX) ||
      (dt.month < 1U) || (dt.month > 12U) ||
      (dt.hour > 23U) || (dt.minute > 59U) || (dt.second > 59U))
  {
    return false;
  }

  /* Every fourth year is a leap year across 2000..2099 (2000 is one, being
     divisible by 400) - the same rule the hardware uses. */
  uint8_t days = DAYS_IN_MONTH[dt.month - 1U];
  if ((dt.month == 2U) && ((dt.year % 4U) == 0U))
  {
    days = 29U;
  }
  return (dt.day >= 1U) && (dt.day <= days);
}

uint8_t Rtc::weekday(uint16_t year, uint8_t month, uint8_t day)
{
  /* Sakamoto's method: 0 = Sunday .. 6 = Saturday. January and February
     count as the end of the previous year, so the leap day falls last. */
  static const uint8_t MONTH_OFFSET[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

  uint32_t y = year;
  if (month < 3U)
  {
    y -= 1U;
  }
  uint32_t w = (y + y / 4U - y / 100U + y / 400U +
                MONTH_OFFSET[month - 1U] + day) % 7U;

  return (w == 0U) ? (uint8_t)RTC_WEEKDAY_SUNDAY : (uint8_t)w;
}

bool Rtc::setDateTime(const DateTime& dt)
{
  if (!isValid(dt))
  {
    return false;
  }

  /* Both registers in BCD, as RTC_TR / RTC_DR lay them out (RM0090 26.6.1,
     26.6.2): 24 hour format, so PM stays 0. */
  uint32_t tr = ((uint32_t)RTC_ByteToBcd2(dt.hour)   << RTC_TR_HU_Pos) |
                ((uint32_t)RTC_ByteToBcd2(dt.minute) << RTC_TR_MNU_Pos) |
                ((uint32_t)RTC_ByteToBcd2(dt.second) << RTC_TR_SU_Pos);

  uint32_t dr = ((uint32_t)RTC_ByteToBcd2((uint8_t)(dt.year - YEAR_MIN)) << RTC_DR_YU_Pos) |
                ((uint32_t)weekday(dt.year, dt.month, dt.day)             << RTC_DR_WDU_Pos) |
                ((uint32_t)RTC_ByteToBcd2(dt.month)                      << RTC_DR_MU_Pos) |
                ((uint32_t)RTC_ByteToBcd2(dt.day)                        << RTC_DR_DU_Pos);

  bool locked = this->lock();

  /* One init mode for both, so the calendar cannot tick between them - see
     Rtc.hpp. Leaving it restarts the counter from the new time and waits
     for the shadow registers, so a read straight after sees it. */
  __HAL_RTC_WRITEPROTECTION_DISABLE(this->hrtc);

  bool ok = (RTC_EnterInitMode(this->hrtc) == HAL_OK);
  if (ok)
  {
    this->hrtc->Instance->TR = tr & RTC_TR_RESERVED_MASK;
    this->hrtc->Instance->DR = dr & RTC_DR_RESERVED_MASK;
    ok = (RTC_ExitInitMode(this->hrtc) == HAL_OK);
  }

  __HAL_RTC_WRITEPROTECTION_ENABLE(this->hrtc);

  if (ok)
  {
    HAL_RTCEx_BKUPWrite(this->hrtc, this->markerRegister, RTC_MARKER);
  }

  this->unlock(locked);
  return ok;
}

bool Rtc::getDateTime(DateTime& dt)
{
  RTC_TimeTypeDef time = {};
  RTC_DateTypeDef date = {};

  bool locked = this->lock();

  /* Time first: reading TR freezes DR until it is read, so the pair is one
     instant. */
  bool ok = (HAL_RTC_GetTime(this->hrtc, &time, RTC_FORMAT_BIN) == HAL_OK) &&
            (HAL_RTC_GetDate(this->hrtc, &date, RTC_FORMAT_BIN) == HAL_OK);

  this->unlock(locked);

  if (!ok)
  {
    dt = DateTime{};
    return false;
  }

  dt.year   = (uint16_t)(YEAR_MIN + date.Year);
  dt.month  = date.Month;
  dt.day    = date.Date;
  dt.hour   = time.Hours;
  dt.minute = time.Minutes;
  dt.second = time.Seconds;

  return this->isSet();
}

bool Rtc::isSet() const
{
  return HAL_RTCEx_BKUPRead(this->hrtc, this->markerRegister) == RTC_MARKER;
}

uint32_t Rtc::getFatTime()
{
  DateTime dt;
  if (!this->getDateTime(dt))
  {
    dt = DateTime{ 1980U, 1U, 1U, 0U, 0U, 0U };
  }

  return ((uint32_t)(dt.year - 1980U) << 25) |
         ((uint32_t)dt.month          << 21) |
         ((uint32_t)dt.day            << 16) |
         ((uint32_t)dt.hour           << 11) |
         ((uint32_t)dt.minute         << 5)  |
         ((uint32_t)dt.second / 2U);
}
