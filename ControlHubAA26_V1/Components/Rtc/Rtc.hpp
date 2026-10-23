/*
 * Rtc.hpp
 *
 * The STM32F439's own real time clock: a calendar, to the second, counted
 * in hardware from the 32.768 kHz LSE crystal (X2 on the NUCLEO-144, on
 * PC14/PC15).
 *
 *   setDateTime(dt) --> check fields --> RTC_TR + RTC_DR, one init mode
 *                                    --> "set" marker in a backup register
 *   getDateTime(dt) --> RTC_TR, then RTC_DR (shadow registers)
 *   getFatTime()    --> getDateTime(), packed for FatFs's get_fattime()
 *
 * What survives what
 * ------------------
 * The RTC and its backup registers are in the backup domain, which a
 * reset - the reset button, a reflash, a watchdog - does not touch. So
 * the time carries on through all of those. The NUCLEO has no backup
 * battery on VBAT, so a power cycle does reset the backup domain: the
 * calendar restarts from 2000-01-01 00:00:00 and the marker is gone.
 * isSet() says which of the two happened.
 *
 * After a reset the shadow registers may hold a stale copy until the
 * hardware next updates them, so init() waits for that (RSF, RM0090
 * 26.3.6) before anything reads the clock.
 *
 * The marker is RTC_MARKER in a backup register, rather than the RTC's
 * own INITS flag, because INITS only means "the year is not 00", and
 * 2000 is a valid year here.
 *
 * MX_RTC_Init() does not disturb a running clock: HAL_RTC_Init() skips
 * the prescaler setup when the calendar is already initialised, and the
 * .ioc has "Activate Calendar" unticked, so nothing writes a default date
 * at boot. Keep it unticked - ticked, CubeMX generates a SetTime/SetDate
 * that resets the time on every reset.
 *
 * Why not HAL_RTC_SetTime() + HAL_RTC_SetDate()
 * ---------------------------------------------
 * Each enters and leaves init mode on its own, and the calendar runs in
 * between. Set 23:59:59 on the 5th that way and a midnight can land in the
 * gap: the time rolls to 00:00:00 on the 6th, and the date write then
 * puts it back to the 5th - a day out. setDateTime() writes both registers
 * in the one init mode, with the counter stopped (RM0090 26.3.5).
 *
 * The weekday is worked out from the date; the hardware needs one, the
 * callers should not have to supply it.
 *
 * Reading
 * -------
 * The calendar is read through shadow registers, which the hardware
 * copies every two RTCCLK cycles. Reading RTC_TR freezes RTC_DR until it
 * is read too, so getDateTime() reads time then date and the pair is
 * always consistent - HAL_RTC_GetTime() must be followed by
 * HAL_RTC_GetDate() for the same reason, or the next read is stale.
 *
 * Accuracy
 * --------
 * The crystal is +-20 ppm, about 1.7 s a day, before any calibration.
 * The RTC's smooth calibration (RTC_CALR) could trim it; nothing does yet.
 *
 * Threading
 * ---------
 * Any task: every call that touches the RTC holds a mutex, because the
 * HAL's own lock on hrtc is not one, and because a write would otherwise
 * interleave with a read's register pair. Each call takes microseconds.
 * get_fattime() runs from sdCardTask inside FatFs, which is fine.
 * Before the scheduler starts there is only one thread, so the mutex is
 * skipped. Never from an ISR.
 *
 * isSet() is one backup register read, so it takes no mutex.
 */

#ifndef RTC_HPP_
#define RTC_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Local time, 24 hour clock. The hardware keeps a two digit year, so
// year is 2000..2099.
struct DateTime
{
  uint16_t year;    // 2000..2099
  uint8_t  month;   // 1..12
  uint8_t  day;     // 1..28/29/30/31
  uint8_t  hour;    // 0..23
  uint8_t  minute;  // 0..59
  uint8_t  second;  // 0..59
};

class Rtc
{
  public:
    static const uint16_t YEAR_MIN = 2000U;
    static const uint16_t YEAR_MAX = 2099U;

    // hrtc: main.c's handle, already through MX_RTC_Init().
    // markerRegister: the backup register (RTC_BKP_DRn) that holds the
    // "set" marker. Nothing else may use it.
    //
    // Stores its arguments only, so it is safe as a global constructed
    // before main().
    Rtc(RTC_HandleTypeDef* hrtc, uint32_t markerRegister);

    // Creates the mutex and waits for the shadow registers to resync.
    // Before the scheduler starts, after MX_RTC_Init().
    void init();

    // Sets the clock. False, with the clock untouched, if any field is
    // out of range (including the 31st of a 30 day month, or 29 Feb in
    // a non leap year) or the RTC does not respond.
    bool setDateTime(const DateTime& dt);

    // Reads the clock into dt. Always fills dt; the return is isSet() at
    // the time of the read, so false means dt is counting from the power
    // up default (2000-01-01 00:00:00), not real time. Also false, with
    // dt zeroed, if the RTC does not respond.
    bool getDateTime(DateTime& dt);

    // True once setDateTime() has succeeded since the last power up.
    bool isSet() const;

    // The time packed the way FatFs's get_fattime() returns it:
    //   bits 31-25 year - 1980, 24-21 month, 20-16 day,
    //        15-11 hour, 10-5 minute, 4-0 second / 2.
    // Until the clock has been set, the FAT epoch, 1980-01-01 00:00:00,
    // so a file is not stamped with a made up date.
    uint32_t getFatTime();

    // Field range check, as setDateTime() does it.
    static bool isValid(const DateTime& dt);

    // 1..7, Monday..Sunday - the RTC's (and ISO 8601's) numbering.
    static uint8_t weekday(uint16_t year, uint8_t month, uint8_t day);

  private:
    // Written to the marker register by setDateTime(). "RTC1".
    static const uint32_t RTC_MARKER = 0x52544331UL;

    // The mutex, if the scheduler is running. lock() returns whether it
    // took it, for unlock().
    bool lock();
    void unlock(bool locked);

    RTC_HandleTypeDef* hrtc;
    uint32_t markerRegister;

    SemaphoreHandle_t mutex;
    StaticSemaphore_t mutexBuffer;
};

#endif /* RTC_HPP_ */
