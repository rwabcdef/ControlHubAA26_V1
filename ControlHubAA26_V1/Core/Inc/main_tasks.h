

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void initTasks();

// The RTC's time, packed for FatFs's get_fattime(). See Rtc::getFatTime().
uint32_t rtcGetFatTime(void);

#ifdef __cplusplus
}
#endif