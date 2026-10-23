/*
 * Logger.cpp
 *
 * See Logger.hpp.
 */

#include "Logger.hpp"
#include <string.h>
#include "task.h"

namespace
{
  const char LINE_END[] = "\r\n";

  // Padded to one width, so the text column lines up.
  const char* levelName(Logger::level lvl)
  {
    switch(lvl)
    {
      case Logger::level::debug: return "DEBUG";
      case Logger::level::error: return "ERROR";
      case Logger::level::info:
      default:                   return "INFO ";
    }
  }
  const uint8_t LEVEL_NAME_LEN = 5U;
}

Logger::Logger(const char* defaultFilePath, Rtc* rtc, DateTimeFormatter formatter,
               uint8_t writeNumThreshold, uint32_t writeTimeThreshold_mS, level minLevel)
{
  this->rtc = rtc;
  this->formatter = formatter;
  this->writeNumThreshold = (writeNumThreshold == 0U) ? 1U : writeNumThreshold;
  this->writeTimeThresholdTicks = pdMS_TO_TICKS(writeTimeThreshold_mS);
  this->minLevel = minLevel;

  this->queue = nullptr;
  this->lastWriteTick = 0U;

  strncpy(this->filePath, defaultFilePath, FILE_PATH_MAX - 1U);
  this->filePath[FILE_PATH_MAX - 1U] = '\0';
  this->overwriteNext = true;
  this->pathGeneration = 0U;

  memset(&this->file, 0, sizeof(this->file));
  this->fileOpen = false;
  this->lastResult = FR_OK;
  this->dropped = 0U;
  this->written = 0U;
}

void Logger::init(QueueHandle_t queue, uint8_t queueLen)
{
  configASSERT(queue != nullptr);
  this->queue = queue;
  if(this->writeNumThreshold > queueLen)
  {
    this->writeNumThreshold = queueLen;
  }
  this->lastWriteTick = xTaskGetTickCount();
}

void Logger::debug(const char* text) { this->log(level::debug, text); }
void Logger::info(const char* text)  { this->log(level::info, text); }
void Logger::error(const char* text) { this->log(level::error, text); }

void Logger::log(level lvl, const char* text)
{
  if((this->queue == nullptr) || (lvl < this->minLevel) || (text == nullptr))
  {
    return;
  }

  Entry entry;
  entry.lvl = lvl;
  entry.time = DateTime{};
  if(this->rtc != nullptr)
  {
    /* Stamped now, not when written. The return (set since power up or
       not) does not matter here - an unset clock still orders the lines. */
    (void)this->rtc->getDateTime(entry.time);
  }

  size_t len = strlen(text);
  if(len > TEXT_MAX) { len = TEXT_MAX; }
  memcpy(entry.text, text, len);
  entry.len = (uint8_t)len;

  if(xQueueSend(this->queue, &entry, 0) != pdTRUE)
  {
    this->dropped = this->dropped + 1U;
  }
}

bool Logger::setFilePath(const char* path)
{
  if((path == nullptr) || (path[0] == '\0') || (strlen(path) >= FILE_PATH_MAX))
  {
    return false;
  }

  taskENTER_CRITICAL();
  strcpy(this->filePath, path);
  this->overwriteNext = true;
  this->pathGeneration++;
  taskEXIT_CRITICAL();
  return true;
}

void Logger::setLevel(level minLevel)
{
  this->minLevel = minLevel;
}

bool Logger::run(bool volumeReady)
{
  if((this->queue == nullptr) || !volumeReady)
  {
    return false;
  }

  /* Unsigned subtraction carries through the tick counter's wrap. */
  TickType_t sinceWrite = xTaskGetTickCount() - this->lastWriteTick;

  /* A close that failed holds the file's lock - nothing can open the file
     until it works. See A close that fails, in Logger.hpp. */
  if(this->fileOpen)
  {
    if(sinceWrite < this->writeTimeThresholdTicks)
    {
      return false;
    }
    this->lastWriteTick = xTaskGetTickCount();
    this->lastResult = this->closeFile();
    if(this->fileOpen)
    {
      return false;
    }
  }

  UBaseType_t waiting = uxQueueMessagesWaiting(this->queue);
  if(waiting == 0U)
  {
    return false;
  }

  if((waiting < this->writeNumThreshold) && (sinceWrite < this->writeTimeThresholdTicks))
  {
    return false;
  }

  this->lastResult = this->write();
  return true;
}

FRESULT Logger::write()
{
  char path[FILE_PATH_MAX];
  bool overwrite;
  uint32_t generation;

  taskENTER_CRITICAL();
  memcpy(path, this->filePath, FILE_PATH_MAX);
  overwrite = this->overwriteNext;
  generation = this->pathGeneration;
  taskEXIT_CRITICAL();

  /* Counted before opening: lines logged while this batch is written wait
     for the next one, so a fast logger cannot keep this loop going. */
  UBaseType_t count = uxQueueMessagesWaiting(this->queue);

  FRESULT result = f_open(&this->file, path,
                          FA_WRITE | (overwrite ? FA_CREATE_ALWAYS : FA_OPEN_APPEND));
  if(result != FR_OK)
  {
    return result;   // nothing taken off the queue - tried again next pass
  }

  if(overwrite)
  {
    taskENTER_CRITICAL();
    if(this->pathGeneration == generation)
    {
      this->overwriteNext = false;
    }
    taskEXIT_CRITICAL();
  }

  char line[TIMESTAMP_MAX + 1U + LEVEL_NAME_LEN + 1U + TEXT_MAX + sizeof(LINE_END)];
  Entry entry;

  for(UBaseType_t i = 0U; (i < count) && (result == FR_OK); i++)
  {
    if(xQueueReceive(this->queue, &entry, 0) != pdTRUE)
    {
      break;
    }

    UINT len = this->formatLine(entry, line);
    UINT bytesWritten = 0U;
    result = f_write(&this->file, line, len, &bytesWritten);
    if((result == FR_OK) && (bytesWritten < len))
    {
      result = FR_DENIED;   // the volume is full
    }
    if(result == FR_OK)
    {
      this->written = this->written + 1U;
    }
  }

  /* Closed even after a failed write. If the close itself fails, the file
     stays locked, so the FIL is kept for run() to close again. The close's
     result only counts if everything before it went. */
  this->fileOpen = true;
  FRESULT closeResult = this->closeFile();
  if(result == FR_OK)
  {
    result = closeResult;
  }

  this->lastWriteTick = xTaskGetTickCount();
  return result;
}

FRESULT Logger::closeFile()
{
  FRESULT result = f_close(&this->file);

  /* FR_INVALID_OBJECT: the volume was mounted again, which threw the lock
     away with the old mount - nothing is left to close. */
  if((result == FR_OK) || (result == FR_INVALID_OBJECT))
  {
    this->fileOpen = false;
    return FR_OK;
  }
  return result;
}

uint16_t Logger::formatLine(const Entry& entry, char* dst)
{
  uint16_t n = 0U;

  if((this->rtc != nullptr) && (this->formatter != nullptr))
  {
    uint8_t stampLen = this->formatter(entry.time, &dst[n]);
    if(stampLen > TIMESTAMP_MAX) { stampLen = TIMESTAMP_MAX; }
    n += stampLen;
    dst[n++] = ' ';
  }

  memcpy(&dst[n], levelName(entry.lvl), LEVEL_NAME_LEN);
  n += LEVEL_NAME_LEN;
  dst[n++] = ' ';

  memcpy(&dst[n], entry.text, entry.len);
  n += entry.len;

  memcpy(&dst[n], LINE_END, sizeof(LINE_END) - 1U);
  n += (uint16_t)(sizeof(LINE_END) - 1U);

  return n;
}

uint32_t Logger::getDropped() const    { return this->dropped; }
uint32_t Logger::getWritten() const    { return this->written; }
FRESULT  Logger::getLastResult() const { return this->lastResult; }
bool     Logger::isFileOpen() const    { return this->fileOpen; }
