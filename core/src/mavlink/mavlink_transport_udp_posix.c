/* UDP backend (POSIX sockets). Not built for ESP-IDF targets. */
#include "mavlink_internal.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "core/common/log.h"

typedef struct {
    int fd; /* -1 when closed */
    bool peer_known;
    bool peer_pinned;            /* configured remote: never replaced by inbound traffic */
    struct sockaddr_in peer;
    struct sockaddr_in last_from; /* sender of the most recent datagram */
    /* Sender of the frame being parsed, and whether every byte of it so far
     * came from that sender. */
    struct sockaddr_in pending_peer;
    bool pending_active;
    bool pending_consistent;
} udp_state_t;

static bool udp_open(core_mavlink_conn_t *conn) {
    const core_mavlink_udp_config_t *udp_cfg = &conn->cfg.transport.udp;
    udp_state_t *st = conn->transport;
    if (!st) {
        st = calloc(1, sizeof(*st));
        if (!st) {
            CORE_LOG_ERROR("out of memory opening udp transport");
            return false;
        }
        st->fd = -1;
        conn->transport = st;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        CORE_LOG_ERROR("socket() failed: %s", strerror(errno));
        return false;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        CORE_LOG_ERROR("fcntl(O_NONBLOCK) failed: %s", strerror(errno));
        close(fd);
        return false;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(udp_cfg->bind_port);
    const char *addr_str = (udp_cfg->bind_addr[0] != '\0') ? udp_cfg->bind_addr : "0.0.0.0";
    if (inet_pton(AF_INET, addr_str, &bind_addr.sin_addr) != 1) {
        CORE_LOG_ERROR("invalid bind address: %s", addr_str);
        close(fd);
        return false;
    }

    if (bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        CORE_LOG_ERROR("bind() failed on %s:%u: %s", addr_str, udp_cfg->bind_port, strerror(errno));
        close(fd);
        return false;
    }

    st->fd = fd;
    st->peer_known = false;
    st->peer_pinned = false;
    st->pending_active = false;
    st->pending_consistent = false;
    memset(&st->peer, 0, sizeof(st->peer));

    /* If a remote peer was configured explicitly, trust it; otherwise the peer
     * is learned from the first accepted frame (standard SITL pattern, since
     * UDP has no notion of "connected" on its own). */
    if (udp_cfg->remote_port != 0) {
        struct sockaddr_in peer;
        memset(&peer, 0, sizeof(peer));
        peer.sin_family = AF_INET;
        peer.sin_port = htons(udp_cfg->remote_port);
        const char *remote_str = (udp_cfg->remote_addr[0] != '\0') ? udp_cfg->remote_addr : "127.0.0.1";
        if (inet_pton(AF_INET, remote_str, &peer.sin_addr) == 1) {
            st->peer = peer;
            st->peer_known = true;
            st->peer_pinned = true;
        }
    }
    return true;
}

static void udp_close(core_mavlink_conn_t *conn) {
    udp_state_t *st = conn->transport;
    if (st && st->fd >= 0) {
        close(st->fd);
        st->fd = -1;
    }
    if (st) {
        st->peer_known = false;
        st->pending_active = false;
        st->pending_consistent = false;
    }
}

static void udp_destroy(core_mavlink_conn_t *conn) {
    free(conn->transport);
    conn->transport = NULL;
}

static int udp_read(core_mavlink_conn_t *conn, uint8_t *buf, size_t len) {
    udp_state_t *st = conn->transport;
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    ssize_t n = recvfrom(st->fd, buf, len, MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
    if (n <= 0) {
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            CORE_LOG_WARN("mavlink recvfrom error: %s", strerror(errno));
        }
        return 0; /* a UDP error is not a dead link; keep listening */
    }
    st->last_from = from;
    return (int)n;
}

/* Learn an unconfigured peer only from a whole, validated MAVLink frame.
 * Track the source across every byte so a frame split or spliced across
 * datagrams/senders cannot attribute its authentication to only the final
 * datagram. */
static void udp_rx_byte(core_mavlink_conn_t *conn, uint8_t byte, uint8_t parse_state) {
    udp_state_t *st = conn->transport;
    bool idle = parse_state == MAVLINK_PARSE_STATE_UNINIT || parse_state == MAVLINK_PARSE_STATE_IDLE;
    if (idle && (byte == MAVLINK_STX || byte == MAVLINK_STX_MAVLINK1)) {
        st->pending_peer = st->last_from;
        st->pending_active = true;
        st->pending_consistent = true;
    } else if (!idle && st->pending_active &&
               (st->last_from.sin_addr.s_addr != st->pending_peer.sin_addr.s_addr ||
                st->last_from.sin_port != st->pending_peer.sin_port)) {
        st->pending_consistent = false;
    }
}

static void udp_rx_frame_end(core_mavlink_conn_t *conn, bool accepted) {
    udp_state_t *st = conn->transport;
    if (!st->pending_active) {
        return;
    }
    if (accepted && st->pending_consistent && !st->peer_known && !st->peer_pinned) {
        st->peer = st->pending_peer;
        st->peer_known = true;
    }
    st->pending_active = false;
    st->pending_consistent = false;
}

static bool udp_can_send(const core_mavlink_conn_t *conn) {
    const udp_state_t *st = conn->transport;
    return st && st->fd >= 0 && st->peer_known;
}

static bool udp_write(core_mavlink_conn_t *conn, const uint8_t *buf, size_t len) {
    udp_state_t *st = conn->transport;
    ssize_t sent = sendto(st->fd, buf, len, 0, (struct sockaddr *)&st->peer, sizeof(st->peer));
    if (sent < 0 || (size_t)sent != len) {
        CORE_LOG_WARN("mavlink sendto failed");
        return false;
    }
    return true;
}

const core_mavlink_transport_ops_t core_mavlink_transport_udp_ops = {
    .name = "udp",
    .open = udp_open,
    .close = udp_close,
    .destroy = udp_destroy,
    .read = udp_read,
    .rx_byte = udp_rx_byte,
    .rx_frame_end = udp_rx_frame_end,
    .can_send = udp_can_send,
    .write = udp_write,
};
