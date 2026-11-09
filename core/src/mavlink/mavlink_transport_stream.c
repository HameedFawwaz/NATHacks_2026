/*
 * Caller-supplied byte stream backend. Portable: no OS calls at all, so it is
 * what the ESP32 build links against (its UART driver supplies read/write).
 */
#include "mavlink_internal.h"

#include "core/common/log.h"

static bool stream_open(core_mavlink_conn_t *conn) {
    const core_mavlink_stream_config_t *s = &conn->cfg.transport.stream;
    if (!s->read || !s->write) {
        CORE_LOG_ERROR("stream transport needs both read and write callbacks");
        return false;
    }
    return true;
}

static void stream_close(core_mavlink_conn_t *conn) {
    (void)conn; /* the caller owns the underlying device */
}

static int stream_read(core_mavlink_conn_t *conn, uint8_t *buf, size_t len) {
    const core_mavlink_stream_config_t *s = &conn->cfg.transport.stream;
    int n = s->read(s->ctx, buf, len);
    if (n < 0) {
        return -1;
    }
    return (size_t)n > len ? (int)len : n;
}

static bool stream_can_send(const core_mavlink_conn_t *conn) {
    return conn->state != CORE_MAVLINK_CONN_DISCONNECTED;
}

static bool stream_write(core_mavlink_conn_t *conn, const uint8_t *buf, size_t len) {
    const core_mavlink_stream_config_t *s = &conn->cfg.transport.stream;
    if (!s->write(s->ctx, buf, len)) {
        CORE_LOG_WARN("mavlink stream write failed");
        return false;
    }
    return true;
}

const core_mavlink_transport_ops_t core_mavlink_transport_stream_ops = {
    .name = "stream",
    .open = stream_open,
    .close = stream_close,
    .destroy = NULL,
    .read = stream_read,
    .can_send = stream_can_send,
    .write = stream_write,
};
