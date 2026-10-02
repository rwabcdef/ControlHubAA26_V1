/*
 * SdCard.cpp
 *
 * See SdCard.hpp.
 */

#include "SdCard.hpp"

// ST's driver table, ff_gen_drv.c. See mount() for why it is touched here.
extern "C" Disk_drvTypeDef disk;

SdCard::SdCard(SD_HandleTypeDef* hsd, FATFS* fs, const char* drivePath,
               GPIO_TypeDef* detectPort, uint16_t detectPin,
               GPIO_PinState detectActive)
  : hsd(hsd), fs(fs), drivePath(drivePath),
    detectPort(detectPort), detectPin(detectPin), detectActive(detectActive),
    file(),
    mounted(false), fileOpen(false), fileWritable(false), fileSize(0U),
    lastResult(FR_OK),
    lastMountMs(0U),
    present(false), candidate(false), candidateSinceMs(0U)
{
}

void SdCard::init()
{
  if(this->detectPort == nullptr)
  {
    return;
  }

  /* The pin is not in the .ioc, so MX_GPIO_Init() may not have clocked
     its port. Harmless if it has. */
  if(this->detectPort == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if(this->detectPort == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if(this->detectPort == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if(this->detectPort == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if(this->detectPort == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if(this->detectPort == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else if(this->detectPort == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }

  /* Pulled to the inactive level, so a switch that is open - or a CD line
     that is not connected at all - reads "no card". */
  GPIO_InitTypeDef gpio = {};
  gpio.Pin   = this->detectPin;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = (this->detectActive == GPIO_PIN_RESET) ? GPIO_PULLUP : GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(this->detectPort, &gpio);
}

bool SdCard::isCardPresent() const
{
  if(this->detectPort == nullptr)
  {
    return true;
  }

  return HAL_GPIO_ReadPin(this->detectPort, this->detectPin) == this->detectActive;
}

bool SdCard::poll()
{
  bool reading = this->isCardPresent();
  uint32_t now = HAL_GetTick();

  if(reading != this->candidate)
  {
    /* A new reading - start timing it. */
    this->candidate = reading;
    this->candidateSinceMs = now;
    return false;
  }

  if((reading == this->present) || ((now - this->candidateSinceMs) < DETECT_SETTLE_MS))
  {
    return false;
  }

  this->present = reading;

  if(this->present)
  {
    (void)this->mount();
  }
  else
  {
    /* Already gone, so this is bookkeeping. f_close() does ask the card
       for its status first (CMD13), which just times out in hardware
       after 64 clocks and fails - harmless for a read file, and
       unmounting clears its _FS_LOCK entry anyway. A write file loses
       whatever was written since its last sync - which is why the SDC00
       line writes sync every time (see write()). The result kept is
       FR_NOT_READY, so the status says why it went. */
    this->unmount();
    this->resetInterface();
    (void)this->setResult(FR_NOT_READY);
  }

  return true;
}

void SdCard::resetInterface()
{
  /* Only once HAL_SD_Init() has run (State READY or later). In RESET -
     at boot, or after a previous call - there is nothing to undo, and
     HAL_SD_DeInit() would run MspDeInit on hardware never set up. */
  if(this->hsd->State != HAL_SD_STATE_RESET)
  {
    (void)HAL_SD_DeInit(this->hsd);
  }
}

FRESULT SdCard::mount()
{
  if(this->mounted)
  {
    return this->setResult(FR_OK);
  }

  /* ST's disk_initialize() (diskio.c) calls the driver's initialise only
     while disk.is_initialized[] is 0, sets it on the first success, and
     never clears it. Left alone, a card swapped in after that is never
     initialised: no identification, no address, still in its idle state,
     so sd_diskio.c's first SD_read() polls its status for the full 30 s
     SD_TIMEOUT and fails (FR_DISK_ERR). Clearing it makes every mount a
     full BSP_SD_Init() of whatever card is in the slot now - which is
     right, since mount() only runs for a new card or after a failure.

     The drive number is the first character of the path ("0:/"), as
     FATFS_LinkDriver() wrote it. */
  uint8_t pdrv = (uint8_t)(this->drivePath[0] - '0');
  if(pdrv < _VOLUMES)
  {
    disk.is_initialized[pdrv] = 0U;
  }

  /* And make that initialise the one the boot mount gets. At boot hsd is
     in RESET, so HAL_SD_Init() runs HAL_SD_MspInit() - pins, DMA, NVIC -
     before identifying the card. Any later HAL_SD_Init() skips straight
     to identification on an SDIO still set up for the old card: 4 bit,
     transfer clock running, data path as the last transfer left it - and
     a re-inserted card would not mount from there. De-initialising
     first (which also resets the peripheral, SDIO_MspDeInit 1) puts every
     mount on the boot path. */
  this->resetInterface();

  /* opt 1: mount now. This is what calls disk_initialize(), and so
     BSP_SD_Init() - see the top of SdCard.hpp. */
  uint32_t startMs = HAL_GetTick();
  FRESULT result = f_mount(this->fs, this->drivePath, 1U);
  this->lastMountMs = HAL_GetTick() - startMs;

  if(result != FR_OK)
  {
    /* f_mount() registers the work area (and creates its _FS_REENTRANT
       semaphore) even when the mount fails. Release it, so a card that
       will not mount does not leave a volume registered behind it. */
    (void)f_mount(nullptr, this->drivePath, 0U);
  }

  this->mounted = (result == FR_OK);
  return this->setResult(result);
}

void SdCard::unmount()
{
  (void)this->close();

  if(this->mounted)
  {
    (void)f_mount(nullptr, this->drivePath, 0U);
    this->mounted = false;
  }
}

FRESULT SdCard::open(const char* path)
{
  return this->openFile(path, FA_READ | FA_OPEN_EXISTING, false);
}

FRESULT SdCard::openWrite(const char* path, WriteMode mode)
{
  /* Write only, never FA_READ as well: read() seeks, and a seek in an
     append file would make the next write land mid-file. */
  BYTE flags;
  switch(mode)
  {
    case WriteMode::append:    flags = FA_WRITE | FA_OPEN_APPEND;   break;
    case WriteMode::createNew: flags = FA_WRITE | FA_CREATE_NEW;    break;
    case WriteMode::overwrite:
    default:                   flags = FA_WRITE | FA_CREATE_ALWAYS; break;
  }

  return this->openFile(path, flags, true);
}

FRESULT SdCard::openFile(const char* path, BYTE flags, bool writable)
{
  (void)this->close();

  if(!this->mounted)
  {
    if(!this->isCardPresent())
    {
      return this->setResult(FR_NOT_READY);
    }

    FRESULT result = this->mount();
    if(result != FR_OK)
    {
      return result;
    }
  }

  FRESULT result = f_open(&this->file, path, flags);
  if(result == FR_OK)
  {
    this->fileOpen = true;
    this->fileWritable = writable;
    this->fileSize = (uint32_t)f_size(&this->file);
  }

  return this->setResult(result);
}

FRESULT SdCard::write(const uint8_t* data, uint32_t len, bool sync,
                      uint32_t* bytesWritten)
{
  *bytesWritten = 0U;

  if(!this->fileOpen)
  {
    return this->setResult(FR_INVALID_OBJECT);
  }

  if(!this->fileWritable)
  {
    return this->setResult(FR_DENIED);
  }

  UINT done = 0U;
  FRESULT result = f_write(&this->file, data, (UINT)len, &done);
  *bytesWritten = (uint32_t)done;
  this->fileSize = (uint32_t)f_size(&this->file);

  /* FatFs reports a full volume as success with fewer bytes written. */
  if((result == FR_OK) && (done < len))
  {
    result = FR_DENIED;
  }

  /* f_sync() writes the data, the FAT and the directory entry (size)
     back to the card, as f_close() would, but leaves the file open. */
  if((result == FR_OK) && sync)
  {
    result = f_sync(&this->file);
  }

  return this->setResult(result);
}

FRESULT SdCard::read(uint32_t offset, uint8_t* buffer, uint32_t len,
                     uint32_t* bytesRead)
{
  *bytesRead = 0U;

  if(!this->fileOpen)
  {
    return this->setResult(FR_INVALID_OBJECT);
  }

  /* f_read() would refuse too, but only after the seek had moved the
     write position - see openWrite(). */
  if(this->fileWritable)
  {
    return this->setResult(FR_DENIED);
  }

  /* Read only, so f_lseek() past the end stops at the end rather than
     growing the file - and the f_read() then reads nothing. */
  FRESULT result = f_lseek(&this->file, (FSIZE_t)offset);
  if(result != FR_OK)
  {
    return this->setResult(result);
  }

  UINT got = 0U;
  result = f_read(&this->file, buffer, (UINT)len, &got);
  *bytesRead = (uint32_t)got;

  return this->setResult(result);
}

FRESULT SdCard::close()
{
  if(!this->fileOpen)
  {
    return FR_OK;
  }

  /* For a write file this is where any unsynced data, and the final
     size, reach the card. */
  this->fileOpen = false;
  this->fileWritable = false;
  this->fileSize = 0U;
  return this->setResult(f_close(&this->file));
}

bool SdCard::isMounted() const
{
  return this->mounted;
}

bool SdCard::isFileOpen() const
{
  return this->fileOpen;
}

bool SdCard::isFileWritable() const
{
  return this->fileWritable;
}

uint32_t SdCard::getFileSize() const
{
  return this->fileSize;
}

FRESULT SdCard::getLastResult() const
{
  return (FRESULT)this->lastResult;
}

uint32_t SdCard::getLastMountMs() const
{
  return this->lastMountMs;
}

FRESULT SdCard::setResult(FRESULT result)
{
  this->lastResult = (uint8_t)result;
  return result;
}
