/*
 * Logger.hpp
 *
 * A text log file on the SD card: data and events, typically the motor's,
 * one line each, stamped from the RTC. Lines are queued by whoever logs
 * them, and written in batches by the task that owns the card.
 *
 *   debug/info/error(text), any task --> RTC time + text --> queue
 *   run(), owner task, each pass     --> enough queued, or long enough
 *                                        since the last write?
 *                                    --> f_open, one f_write per line,
 *                                        f_close
 *
 * A line in the file is
 *
 *   <timestamp> <LEVEL> <text>\r\n      e.g.  14-30-05 INFO  motorB start
 *
 * the timestamp being whatever the caller's DateTimeFormatter makes of the
 * RTC's time when the line was logged (not when it was written).
 *
 * When it writes
 * --------------
 * run() writes everything queued when either
 *
 *   - writeNumThreshold or more lines are queued, or
 *   - writeTimeThreshold_mS or more have passed since the last write
 *
 * and at least one line is queued. The second one means a line logged
 * after a quiet spell is written on the next pass, and one logged soon
 * after a write waits at most writeTimeThreshold_mS. Each write opens and
 * closes the file, so what has been written is on the card - a card
 * pulled out, or the power lost, takes only what is still queued.
 *
 * Overwrite, then append
 * ----------------------
 * The first write after boot, and the first after setFilePath(), creates
 * the file or empties an existing one (FA_CREATE_ALWAYS); later writes
 * add to its end (FA_OPEN_APPEND). So each boot starts a fresh log, and
 * the previous boot's is lost unless it was copied off or the path
 * changed. Lines queued before a setFilePath() go to the new file.
 *
 * Nothing is written - and nothing taken off the queue - while there is
 * no mounted volume (run()'s volumeReady), or if the file will not open.
 * Lines then wait, and once the queue is full, new ones are dropped and
 * counted (getDropped()). A write that fails part way loses that batch's
 * remaining lines; getLastResult() keeps the FRESULT.
 *
 * A close that fails
 * ------------------
 * In this FatFs (R0.12c), f_close() flushes first and releases the file's
 * _FS_LOCK entry only if that flush worked. A FIL given up after a failed
 * close would leave the file locked for writing until the next mount -
 * every open of it, the Logger's own and SDC00's, refused with FR_LOCKED
 * (16). So a FIL whose close failed is kept (isFileOpen()), and run()
 * retries the close - at most once per writeTimeThreshold_mS - before
 * anything else, until it works. FR_INVALID_OBJECT from a retry means the
 * volume has been mounted again since, which cleared the lock, and counts
 * as closed.
 *
 * One leak is out of reach here: an FA_OPEN_APPEND open that fails while
 * seeking to the end (a disk error reading the last sector) keeps its lock
 * and gives no FIL to close. Only a remount - taking the card out and
 * back - clears that.
 *
 * Sharing the card with SdCard
 * ----------------------------
 * Logger keeps its own FIL, so it does not disturb SdCard's open file.
 * ffconf.h's _FS_LOCK (2) allows exactly the two open at once. FatFs
 * refuses to open a file for writing that is already open, so while the
 * log file is open through SdCard (the SDC00 socket), writes are refused
 * with FR_LOCKED (16) and the lines wait for it to be closed.
 *
 * Levels
 * ------
 * debug < info < error. Lines below the minimum level (the constructor's,
 * or setLevel()'s) are dropped at once, without reading the clock, so
 * debug lines left in the code cost one comparison. Lower case, because
 * the Debug build defines DEBUG as a macro.
 *
 * Threading
 * ---------
 * debug(), info(), error(), setFilePath(), setLevel() and the getters:
 * any task, never an ISR (the clock read takes Rtc's mutex). Logging
 * never blocks: a full queue drops the line.
 *
 * run(): one owner task, the one that owns the SD card (sdCardTask) - it
 * calls FatFs, and so blocks on the SD DMA for milliseconds. Its line
 * buffer is ~100 bytes of the owner's stack.
 */

#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "fatfs.h"
#include "Rtc.hpp"

class Logger
{
  public:
    enum class level : uint8_t { debug, info, error };

    // Writes dt into dst, at most TIMESTAMP_MAX chars, no NUL needed, and
    // returns how many it wrote - typically HH-MM-SS, 8. Runs in the
    // logging task.
    typedef uint8_t (*DateTimeFormatter)(const DateTime& dt, char* dst);

    static const uint8_t TEXT_MAX      = 64U;   // per line; longer is cut short
    static const uint8_t TIMESTAMP_MAX = 24U;   // a DateTimeFormatter's dst
    static const uint8_t FILE_PATH_MAX = 64U;   // incl. the NUL; longer is refused

    // One queued line. The queue handed to init() must have items of
    // sizeof(Logger::Entry).
    struct Entry
    {
      DateTime time;
      level    lvl;
      uint8_t  len;
      char     text[TEXT_MAX];
    };

    // rtc: the timestamps' clock; nullptr leaves lines unstamped.
    // formatter: nullptr also leaves them unstamped.
    //
    // Stores its arguments only, so it is safe as a global constructed
    // before main(). defaultFilePath is copied.
    Logger(const char* defaultFilePath, Rtc* rtc, DateTimeFormatter formatter,
           uint8_t writeNumThreshold, uint32_t writeTimeThreshold_mS = 1000U,
           level minLevel = level::info);

    // queue: created by the caller (xQueueCreateStatic), queueLen items of
    // sizeof(Entry). writeNumThreshold is capped at queueLen, or it could
    // never be reached. Before the scheduler starts.
    void init(QueueHandle_t queue, uint8_t queueLen);

    // Queue a line at that level. Any task; never blocks.
    void debug(const char* text);
    void info(const char* text);
    void error(const char* text);

    // A new file for the lines from now on; the next write creates or
    // empties it. False, path unchanged, if empty or too long. Any task.
    bool setFilePath(const char* path);

    // The minimum level logged from now on. Any task.
    void setLevel(level minLevel);

    // Owner task, each pass - every few tens of ms. volumeReady: whether
    // the card is mounted (SdCard::isMounted()). True on a pass that
    // wrote.
    bool run(bool volumeReady);

    uint32_t getDropped() const;      // lines lost to a full queue
    uint32_t getWritten() const;      // lines written since boot
    FRESULT  getLastResult() const;   // of the last write's FatFs calls
    bool     isFileOpen() const;      // a failed close waiting to be retried

  private:
    void log(level lvl, const char* text);
    FRESULT write();
    FRESULT closeFile();
    uint16_t formatLine(const Entry& entry, char* dst);

    Rtc* rtc;
    DateTimeFormatter formatter;
    uint8_t writeNumThreshold;
    TickType_t writeTimeThresholdTicks;
    volatile level minLevel;

    QueueHandle_t queue;
    TickType_t lastWriteTick;

    // Set by setFilePath() in any task, read by write() in the owner's;
    // both under a critical section. pathGeneration tells write() whether
    // the path changed while it had the file open, so a new path's
    // overwrite is not lost.
    char filePath[FILE_PATH_MAX];
    bool overwriteNext;
    uint32_t pathGeneration;

    FIL file;
    volatile bool fileOpen;   // set only while a close is failing - see above
    volatile FRESULT lastResult;
    volatile uint32_t dropped;
    volatile uint32_t written;
};

#endif /* LOGGER_HPP_ */
