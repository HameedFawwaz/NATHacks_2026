/*
 * Serial (tty) backend for POSIX hosts: a companion computer wired to a
 * flight controller's TELEM port, or a USB serial adapter. Not built for
 * ESP-IDF targets, which use the stream backend over their UART driver.
 */
#include "mavlink_internal.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "core/common/log.h"

/* How long one frame may wait for room in the tty's output buffer. */
#define SERIAL_WRITE_TIMEOUT_MS 50

typedef struct {
    int fd; /* -1 when closed */
} serial_state_t;

static bool baud_to_speed(uint32_t baud, speed_t *out) {
    switch (baud) {
    case 9600: *out = B9600; return true;
    case 19200: *out = B19200; return true;
    case 38400: *out = B38400; return true;
    case 57600: *out = B57600; return true;
    case 115200: *out = B115200; return true;
    case 230400: *out = B230400; return true;
#ifdef B460800
    case 460800: *out = B460800; return true;
#endif
#ifdef B921600
    case 921600: *out = B921600; return true;
#endif
    default: return false;
    }
}

static bool serial_open(core_mavlink_conn_t *conn) {
    const core_mavlink_serial_config_t *cfg = &conn->cfg.transport.serial;
    serial_state_t *st = conn->transport;
    if (!st) {
        st = calloc(1, sizeof(*st));
        if (!st) {
            CORE_LOG_ERROR("out of memory opening serial transport");
            return false;
        }
        st->fd = -1;
        conn->transport = st;
    }

    speed_t speed;
    if (!baud_to_speed(cfg->baud_rate, &speed)) {
        CORE_LOG_ERROR("unsupported serial baud rate %u", (unsigned)cfg->baud_rate);
        return false;
    }
    if (cfg->device[0] == '\0') {
        CORE_LOG_ERROR("serial transport needs a device path");
        return false;
    }

    int fd = open(cfg->device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        CORE_LOG_ERROR("open(%s) failed: %s", cfg->device, strerror(errno));
        return false;
    }
    /* A second reader on the same port would steal bytes out of frames. */
    if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
        CORE_LOG_ERROR("%s is in use by another program: %s", cfg->device, strerror(errno));
        close(fd);
        return false;
    }
    (void)ioctl(fd, TIOCEXCL); /* best effort: not every pseudo-terminal honours it */

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) {
        CORE_LOG_ERROR("%s is not a serial device: %s", cfg->device, strerror(errno));
        close(fd);
        return false;
    }
    cfmakeraw(&tio); /* no echo, no CR/LF translation of frame bytes, 8 data bits */
    tio.c_cflag &= ~(tcflag_t)(PARENB | CSTOPB | CRTSCTS | HUPCL);
    tio.c_cflag |= CS8 | CREAD | CLOCAL;
    tio.c_iflag &= ~(tcflag_t)(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (cfsetispeed(&tio, speed) < 0 || cfsetospeed(&tio, speed) < 0 || tcsetattr(fd, TCSANOW, &tio) < 0) {
        CORE_LOG_ERROR("configuring %s failed: %s", cfg->device, strerror(errno));
        close(fd);
        return false;
    }
    (void)tcflush(fd, TCIFLUSH); /* drop whatever arrived before we were listening */

    st->fd = fd;
    CORE_LOG_INFO("mavlink serial %s open at %u baud", cfg->device, (unsigned)cfg->baud_rate);
    return true;
}

static void serial_close(core_mavlink_conn_t *conn) {
    serial_state_t *st = conn->transport;
    if (st && st->fd >= 0) {
        close(st->fd);
        st->fd = -1;
    }
}

static void serial_destroy(core_mavlink_conn_t *conn) {
    free(conn->transport);
    conn->transport = NULL;
}

static int serial_read(core_mavlink_conn_t *conn, uint8_t *buf, size_t len) {
    serial_state_t *st = conn->transport;
    for (;;) {
        ssize_t n = read(st->fd, buf, len);
        if (n > 0) {
            return (int)n;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return 0;
        }
        /* With VMIN=VTIME=0 a quiet port reads 0 bytes, so poll to tell that
         * apart from a hangup (USB adapter unplugged, pty closed). */
        if (n == 0) {
            struct pollfd p = {.fd = st->fd, .events = POLLIN};
            if (poll(&p, 1, 0) > 0 && (p.revents & (POLLHUP | POLLERR | POLLNVAL))) {
                CORE_LOG_WARN("mavlink serial hung up");
                return -1;
            }
            return 0;
        }
        CORE_LOG_WARN("mavlink serial read error: %s", strerror(errno));
        return -1;
    }
}

static bool serial_can_send(const core_mavlink_conn_t *conn) {
    const serial_state_t *st = conn->transport;
    return st && st->fd >= 0;
}

/* Writes the whole frame, waiting briefly for buffer space. A frame cut short
 * by the timeout leaves a fragment the receiver's parser will resync past. */
static bool serial_write(core_mavlink_conn_t *conn, const uint8_t *buf, size_t len) {
    serial_state_t *st = conn->transport;
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(st->fd, buf + off, len - off);
        if (n > 0) {
            off += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd p = {.fd = st->fd, .events = POLLOUT};
            if (poll(&p, 1, SERIAL_WRITE_TIMEOUT_MS) > 0 && (p.revents & POLLOUT)) {
                continue;
            }
        }
        CORE_LOG_WARN("mavlink serial write failed after %zu of %zu bytes", off, len);
        return false;
    }
    return true;
}

const core_mavlink_transport_ops_t core_mavlink_transport_serial_ops = {
    .name = "serial",
    .open = serial_open,
    .close = serial_close,
    .destroy = serial_destroy,
    .read = serial_read,
    .can_send = serial_can_send,
    .write = serial_write,
};
