#include "mavlink_internal.h"

/* The single shared per-channel status array for this library; see mavlink_internal.h. */
static mavlink_status_t g_channel_status[MAVLINK_COMM_NUM_BUFFERS];

mavlink_status_t *mavlink_get_channel_status(uint8_t chan) {
    return &g_channel_status[chan % MAVLINK_COMM_NUM_BUFFERS];
}
