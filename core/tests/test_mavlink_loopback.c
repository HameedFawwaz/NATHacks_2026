/*
 * Integration test for the mavlink connection/telemetry/send layer, run over
 * real loopback UDP sockets so it doesn't need an actual SITL instance.
 *
 * This file plays the role of a "fake flight controller": it's allowed to
 * include the vendored MAVLink headers directly (unlike nav/link/autonomy/
 * crypto) because it's exercising the boundary this layer is responsible for,
 * not consuming it as another module would.
 */
#include <arpa/inet.h>
#include <assert.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "common/mavlink.h"

#include "core/mavlink/mavlink.h"

static int open_fake_vehicle_socket(struct sockaddr_in *conn_addr_out) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(fd >= 0);

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(0); /* any available port */
    inet_pton(AF_INET, "127.0.0.1", &bind_addr.sin_addr);
    assert(bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);

    memset(conn_addr_out, 0, sizeof(*conn_addr_out));
    conn_addr_out->sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &conn_addr_out->sin_addr);

    /* Never collide with a developer's running SITL/core on the default port. */
    int probe = socket(AF_INET, SOCK_DGRAM, 0);
    assert(probe >= 0);
    assert(bind(probe, (struct sockaddr *)conn_addr_out, sizeof(*conn_addr_out)) == 0);
    socklen_t addr_len = sizeof(*conn_addr_out);
    assert(getsockname(probe, (struct sockaddr *)conn_addr_out, &addr_len) == 0);
    close(probe);

    return fd;
}

static void send_fake_heartbeat(int fake_fd, const struct sockaddr_in *conn_addr) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                0, 0, MAV_STATE_ACTIVE);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t sent = sendto(fake_fd, buf, len, 0, (const struct sockaddr *)conn_addr, sizeof(*conn_addr));
    assert(sent == (ssize_t)len);
}

static void send_fake_gps(int fake_fd, const struct sockaddr_in *conn_addr) {
    mavlink_message_t msg;
    /* time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible,
     * alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw (MAVLink2 extensions) */
    mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                  0, 3, /* GPS_FIX_TYPE_3D_FIX */
                                  473977420, -1223974560, /* lat/lon degE7: ~47.39774N, -122.39746E */
                                  10000, 100, 100, 50, 0, 8,
                                  0, 0, 0, 0, 0, 0);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ssize_t sent = sendto(fake_fd, buf, len, 0, (const struct sockaddr *)conn_addr, sizeof(*conn_addr));
    assert(sent == (ssize_t)len);
}

static bool poll_until(core_mavlink_conn_t *conn, core_telemetry_t *telemetry, int max_iterations,
                        bool (*condition)(const core_telemetry_t *)) {
    for (int i = 0; i < max_iterations; i++) {
        core_mavlink_conn_poll(conn);
        core_mavlink_conn_get_telemetry(conn, telemetry);
        if (condition(telemetry)) {
            return true;
        }
        usleep(1000); /* 1ms */
    }
    return false;
}

static bool heartbeat_valid(const core_telemetry_t *t) { return t->heartbeat.valid; }
static bool gps_valid(const core_telemetry_t *t) { return t->gps_raw.valid; }

int main(void) {
    struct sockaddr_in conn_addr;
    int fake_fd = open_fake_vehicle_socket(&conn_addr);

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "127.0.0.1", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = ntohs(conn_addr.sin_port);
    cfg.transport.udp.remote_port = 0; /* learn peer from first inbound packet */
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 1000;
    cfg.heartbeat_timeout_ms = 5000;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    assert(conn != NULL);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTING);

    /* --- heartbeat decode --- */
    send_fake_heartbeat(fake_fd, &conn_addr);
    core_telemetry_t telemetry;
    bool ok = poll_until(conn, &telemetry, 2000, heartbeat_valid);
    assert(ok && "timed out waiting for heartbeat to be decoded");
    assert(telemetry.heartbeat.type == MAV_TYPE_QUADROTOR);
    assert(telemetry.heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED);

    /* --- GPS decode --- */
    send_fake_gps(fake_fd, &conn_addr);
    ok = poll_until(conn, &telemetry, 2000, gps_valid);
    assert(ok && "timed out waiting for GPS to be decoded");
    assert(telemetry.gps_raw.fix_type == 3);
    assert(telemetry.gps_raw.lat == 473977420);
    assert(telemetry.gps_raw.lon == -1223974560);
    assert(telemetry.gps_raw.satellites_visible == 8);

    /* --- send path: conn should now know the fake vehicle's address --- */
    assert(core_mavlink_send_heartbeat(conn));

    uint8_t recv_buf[MAVLINK_MAX_PACKET_LEN];
    ssize_t n = -1;
    for (int i = 0; i < 2000 && n < 0; i++) {
        n = recv(fake_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT);
        if (n < 0) {
            usleep(1000);
        }
    }
    assert(n > 0 && "timed out waiting for the connection's outbound heartbeat");

    mavlink_message_t recv_msg;
    mavlink_status_t recv_status;
    memset(&recv_status, 0, sizeof(recv_status));
    bool decoded_any = false;
    for (ssize_t i = 0; i < n; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_1, recv_buf[i], &recv_msg, &recv_status)) {
            decoded_any = true;
            assert(recv_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT);
            assert(recv_msg.sysid == cfg.system_id);
            assert(recv_msg.compid == cfg.component_id);
        }
    }
    assert(decoded_any && "failed to decode the connection's outbound heartbeat");

    /* Signing rejection counters are private when enforcement is off. */
    core_link_security_t security = {0};
    security.rejected_unsigned = 2;
    security.rejected_bad_sig = 3;
    security.rejected_replay = 4;
    assert(!core_mavlink_publish_link_security(conn, &security));
    assert(recv(fake_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT) < 0);

    /* When enabled, publish all three counts under their stable short names. */
    security.signing_enabled = true;
    assert(core_mavlink_publish_link_security(conn, &security));
    const char *expected_names[] = {"SIG_UNSGN", "SIG_BADSG", "SIG_REPLY"};
    const float expected_values[] = {2.0f, 3.0f, 4.0f};
    for (size_t i = 0; i < 3; i++) {
        n = -1;
        for (int attempt = 0; attempt < 2000 && n < 0; attempt++) {
            n = recv(fake_fd, recv_buf, sizeof(recv_buf), MSG_DONTWAIT);
            if (n < 0) usleep(1000);
        }
        assert(n > 0 && "timed out waiting for a signing counter");
        memset(&recv_status, 0, sizeof(recv_status));
        decoded_any = false;
        for (ssize_t j = 0; j < n; j++) {
            if (mavlink_parse_char(MAVLINK_COMM_1, recv_buf[j], &recv_msg, &recv_status)) {
                char name[11] = {0};
                float value = 0;
                assert(recv_msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
                mavlink_msg_named_value_float_get_name(&recv_msg, name);
                value = mavlink_msg_named_value_float_get_value(&recv_msg);
                assert(strcmp(name, expected_names[i]) == 0);
                assert(value == expected_values[i]);
                decoded_any = true;
            }
        }
        assert(decoded_any);
    }

    core_mavlink_conn_close(conn);
    close(fake_fd);

    printf("test_mavlink_loopback: all assertions passed\n");
    return 0;
}
