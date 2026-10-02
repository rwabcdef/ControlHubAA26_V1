/*
 * SdCard.hpp
 *
 * An SD card on SDIO, through FatFs: card detect, mount, and reading an
 * existing file. Read only for now - nothing here writes.
 *
 *   poll(), each pass  --> card detect pin, debounced
 *                          inserted --> mount()     (f_mount, forced)
 *                          removed  --> close(), unmount()
 *   open(path)         --> f_open(FA_READ)          (mounts first if needed)
 *   read(offset, ...)  --> f_lseek + f_read on the open file
 *   close()            --> f_close
 *
 * The layers underneath
 * ---------------------
 * All CubeMX generated, in FATFS/: fatfs.c links SD_Driver to the
 * volume ("0:/", SDPath) in MX_FATFS_Init(); sd_diskio.c is the FatFs
 * disk layer (DMA, RTOS template); bsp_driver_sd.c is the thin BSP over
 * HAL_SD. The SDIO peripheral itself (PC8-PC12, PD2, DMA2 streams 3/6)
 * is the .ioc's, and the extras CubeMX does not do - the SDIO interrupt,
 * bus pull-ups, the slower transfer clock and the DMA scratch buffer -
 * are in USER CODE blocks; see the notes in stm32f4xx_hal_msp.c,
 * stm32f4xx_it.c, main.c (SDIO_Init 2) and sd_diskio.c.
 *
 * MX_SDIO_SD_Init() does not touch the card. It is brought up by the
 * first disk_initialize(), which is the forced f_mount() in mount() -
 * BSP_SD_Init(): HAL_SD_Init() at 400 kHz, then the 4 bit bus at the
 * transfer clock. sd_diskio.c refuses that before the scheduler is
 * running, so mount() can only work from a task.
 *
 * Every mount is a cold start, the same as at boot: the SDIO is
 * de-initialised and reset first, and ST's diskio.c - which on its own
 * only runs the driver's initialise once per boot - is made to run it
 * again. Without both, a re-inserted card never mounts; see mount().
 *
 * Card detect
 * -----------
 * The card socket's CD switch on a plain GPIO input, read by
 * isCardPresent(). The pin is not claimed in the .ioc (like PB8 and PF4,
 * see main_tasks.cpp), so init() configures it, with an internal pull
 * away from the active level so an open switch reads "absent".
 *
 * BSP_SD_Init() asks BSP_SD_IsDetected() before touching the card. The
 * generated one is a weak stub that always answers "present"; the strong
 * one in main_tasks.cpp answers from isCardPresent(), so a mount with no
 * card fails at once (FR_NOT_READY) rather than timing out on the bus.
 *
 * detectPort == nullptr means there is no CD line: the card always reads
 * present, and poll() mounts once at boot. A card pulled out then is not
 * noticed until an operation fails.
 *
 * poll() debounces: a new reading has to hold for DETECT_SETTLE_MS before
 * it counts. That covers the contact bounce, and on insertion gives the
 * card time to get power before it is spoken to.
 *
 * Results
 * -------
 * Every FatFs call's FRESULT is kept, and getLastResult() returns it, so
 * a status query can say why the last thing failed: FR_NOT_READY (3) is
 * no card or no answer from it, FR_NO_FILESYSTEM (13) a card that is not
 * FAT/FAT32 (exFAT is off in ffconf.h), FR_NO_FILE (4) a wrong name.
 *
 * Threading
 * ---------
 * One owner task. FatFs is reentrant here (_FS_REENTRANT), but the FIL
 * this class keeps is not, and every call blocks on the SD DMA for
 * milliseconds - up to sd_diskio.c's 30 s SD_TIMEOUT if the card stops
 * answering mid transfer - so nothing that services a link may call in.
 * Everything except the getters is owner-task only.
 *
 * isCardPresent() is one GPIO register read and the other getters read
 * single words, so they are safe from any task. Each getter is
 * individually consistent, not as a set: a status built from several can
 * straddle a change.
 *
 * Stack: FatFs keeps its long file name buffer on the caller's stack
 * (_USE_LFN 2, (_MAX_LFN + 1) * 2 = 512 bytes), on top of its own frames
 * and the disk layer's. Give the owner task 2 KB.
 */

#ifndef SDCARD_HPP_
#define SDCARD_HPP_

#include <stdint.h>
#include "main.h"
#include "fatfs.h"

class SdCard
{
  public:
    // How long a new card detect reading has to hold before poll() acts
    // on it - see Card detect, above.
    static const uint32_t DETECT_SETTLE_MS = 250U;

    // hsd: the SDIO handle, main.c's - SdCard de-initialises it between
    // cards (see resetInterface()). fs: the volume's work area - fatfs.c's SDFatFS. drivePath: its
    // logical drive, fatfs.c's SDPath ("0:/"), filled in by
    // MX_FATFS_Init(); it is only read from mount() on, so the pointer can
    // be taken before that has run. detectPort == nullptr: no CD line.
    //
    // Stores its arguments only, so it is safe as a global constructed
    // before main().
    SdCard(SD_HandleTypeDef* hsd, FATFS* fs, const char* drivePath,
           GPIO_TypeDef* detectPort, uint16_t detectPin,
           GPIO_PinState detectActive);

    // Configures the card detect pin. Before the scheduler starts.
    void init();

    // The card detect pin, now, undebounced. Any task.
    bool isCardPresent() const;

    // Owner task, regularly - every few tens of ms is plenty. Mounts a
    // card that has been inserted, closes and unmounts one that has gone.
    // True on the pass the debounced presence changes - i.e. when a card
    // has been mounted (or failed to) or taken away - so the caller can
    // publish the new state.
    bool poll();

    // Owner task. Forced mount, so the card is initialised and its
    // volume read now, not on first use. FR_OK if mounted.
    FRESULT mount();

    // Owner task. Closes any open file and releases the volume. Safe to
    // call with nothing mounted.
    void unmount();

    // Owner task. Opens an existing file for reading, closing any file
    // already open. Mounts first if a card is present but not mounted,
    // so it also retries a mount that failed.
    FRESULT open(const char* path);

    // Owner task. Reads up to len bytes from offset in the open file into
    // buffer, and sets *bytesRead. An offset past the end reads 0 bytes
    // and is not an error; neither is a short read at the end.
    // FR_INVALID_OBJECT if no file is open.
    FRESULT read(uint32_t offset, uint8_t* buffer, uint32_t len,
                 uint32_t* bytesRead);

    // Owner task. FR_OK, and harmless, if no file is open.
    FRESULT close();

    // Getters - any task, see Threading.
    bool isMounted() const;
    bool isFileOpen() const;
    uint32_t getFileSize() const;   // of the open file; 0 if none
    FRESULT getLastResult() const;

    // How long the last f_mount() in mount() took, success or not. A
    // mount is normally tens of ms; one that ran into a transfer the disk
    // layer never heard the end of shows up here as ~30000 (SD_TIMEOUT).
    uint32_t getLastMountMs() const;

  private:
    FRESULT setResult(FRESULT result);

    // HAL_SD_DeInit(), if hsd has been initialised: SDIO powered down and
    // reset, pins back to analog, DMA and interrupt off. Before every
    // mount, so the next HAL_SD_Init() takes the same path as at boot,
    // and on removal, so an empty socket is not left clocked while the
    // next card goes in. See mount().
    void resetInterface();

    SD_HandleTypeDef* hsd;
    FATFS*        fs;
    const char*   drivePath;
    GPIO_TypeDef* detectPort;
    uint16_t      detectPin;
    GPIO_PinState detectActive;

    // Not on the owner's stack: a FIL carries a 512 byte sector buffer.
    FIL           file;

    volatile bool    mounted;
    volatile bool    fileOpen;
    volatile uint32_t fileSize;
    volatile uint8_t lastResult;    // an FRESULT
    volatile uint32_t lastMountMs;

    // poll()'s debounce: the debounced state, the raw reading being
    // timed, and when that reading was first seen (HAL_GetTick()).
    bool     present;
    bool     candidate;
    uint32_t candidateSinceMs;
};

#endif /* SDCARD_HPP_ */
