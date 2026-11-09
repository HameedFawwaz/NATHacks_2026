/* core_now_ms() on the ESP32: the monotonic microsecond timer since boot. */
#include "core/common/time.h"

#include "esp_timer.h"

uint64_t core_now_ms(void) {
    return (uint64_t)(esp_timer_get_time() / 1000);
}
