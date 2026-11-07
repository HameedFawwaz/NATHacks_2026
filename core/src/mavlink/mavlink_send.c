#include "core/mavlink/mavlink_send.h"
#include "mavlink_internal.h"

#include <string.h>
#include <sys/socket.h>

#include "core/common/log.h"
#include "core/common/time.h"

bool core_mavlink_internal_send_raw(core_mavlink_conn_t *conn, const mavlink_message_t *msg) {
    if (!conn || conn->sock_fd < 0) {
        CORE_LOG_WARN("mavlink send failed: connection not open");
        return false;
    }
    if (!conn->peer_known) {
        CORE_LOG_WARN("mavlink send failed: no known peer yet");
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);

    ssize_t sent = sendto(conn->sock_fd, buffer, len, 0,
                           (struct sockaddr *)&conn->peer_addr, sizeof(conn->peer_addr));
    if (sent < 0 || (uint16_t)sent != len) {
        CORE_LOG_WARN("mavlink sendto failed");
        return false;
    }
    return true;
}

bool core_mavlink_send_heartbeat(core_mavlink_conn_t *conn) {
    if (!conn) {
        return false;
    }
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(conn->cfg.system_id, conn->cfg.component_id, &msg,
                                MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                                0, 0, MAV_STATE_ACTIVE);
    return core_mavlink_internal_send_raw(conn, &msg);
}

bool core_mavlink_send_statustext(core_mavlink_conn_t *conn, core_mav_severity_t severity, const char *text) {
    if (!conn || !text) {
        return false;
    }

    /* MAVLink's text field is a fixed 50-byte array copied verbatim (not by
     * strlen) by the vendored pack function, so this buffer must always be
     * exactly that size and zero-padded - passing a shorter C string
     * directly would let the library read past its end. */
    char text_buf[50];
    memset(text_buf, 0, sizeof(text_buf));
    strncpy(text_buf, text, sizeof(text_buf));

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(conn->cfg.system_id, conn->cfg.component_id, &msg,
                                 (uint8_t)severity, text_buf, 0, 0);
    return core_mavlink_internal_send_raw(conn, &msg);
}

bool core_mavlink_send_named_value_float(core_mavlink_conn_t *conn, const char *name, float value) {
    if (!conn || !name) {
        return false;
    }

    /* Same fixed-width-copy caveat as statustext above, but 10 bytes. */
    char name_buf[10];
    memset(name_buf, 0, sizeof(name_buf));
    strncpy(name_buf, name, sizeof(name_buf));

    mavlink_message_t msg;
    mavlink_msg_named_value_float_pack(conn->cfg.system_id, conn->cfg.component_id, &msg,
                                        (uint32_t)core_now_ms(), name_buf, value);
    return core_mavlink_internal_send_raw(conn, &msg);
}

bool core_mavlink_publish_trust_score(core_mavlink_conn_t *conn, const core_trust_score_t *score) {
    if (!conn || !score) {
        return false;
    }

    bool ok = true;
    ok = core_mavlink_send_named_value_float(conn, "TRUST", score->overall_trust) && ok;
    ok = core_mavlink_send_named_value_float(conn, "TRUST_NAV", score->nav_trust) && ok;
    ok = core_mavlink_send_named_value_float(conn, "TRUST_LNK", score->link_trust) && ok;
    ok = core_mavlink_send_named_value_float(conn, "TRUST_CRY", score->crypto_trust) && ok;
    return ok;
}
