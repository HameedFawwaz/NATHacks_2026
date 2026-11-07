#ifndef CORE_COMMON_TIME_H
#define CORE_COMMON_TIME_H

#include <stdint.h>

/* Monotonic milliseconds, suitable for measuring intervals (not wall-clock time). */
uint64_t core_now_ms(void);

#endif /* CORE_COMMON_TIME_H */
