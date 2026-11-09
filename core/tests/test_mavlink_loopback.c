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
#include <math.h>
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

static void send_packed(int fake_fd, const struct sockaddr_in *conn_addr, const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    ssize_t sent = sendto(fake_fd, buf, len, 0, (const struct sockaddr *)conn_addr, sizeof(*conn_addr));
    assert(sent == (ssize_t)len);
}

/* A heartbeat as the autopilot forwards it from a ground station or companion. */
static void send_other_heartbeat(int fake_fd, const struct sockaddr_in *conn_addr, uint8_t sysid, uint8_t type) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(sysid, 190, &msg, type, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    send_packed(fake_fd, conn_addr, &msg);
}

static void send_rc_rssi(int fake_fd, const struct sockaddr_in *conn_addr, uint8_t sysid, uint8_t compid, uint8_t rssi) {
    mavlink_message_t msg;
    mavlink_msg_rc_channels_pack(sysid, compid, &msg, 0, 8, 1500, 1500, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 rssi);
    send_packed(fake_fd, conn_addr, &msg);
}

static void send_home(int fake_fd, const struct sockaddr_in *conn_addr, uint8_t sysid, uint8_t compid, int32_t lat) {
    mavlink_home_position_t home = {0};
    home.latitude = lat;
    home.longitude = -1223974560;
    home.altitude = 123000;
    mavlink_message_t msg;
    mavlink_msg_home_position_encode(sysid, compid, &msg, &home);
    send_packed(fake_fd, conn_addr, &msg);
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
static bool two_operator_heartbeats(const core_telemetry_t *t) { return t->gcs_heartbeat.count >= 2; }
static bool rc_rssi_200(const core_telemetry_t *t) { return t->rc_status.valid && t->rc_status.rssi == 200; }
static bool home_valid(const core_telemetry_t *t) { return t->home_position.valid; }
static bool home_shifted(const core_telemetry_t *t) { return t->home_position.lat == 473978420; }

static void recovery_command(int fd, const struct sockaddr_in *addr, uint8_t sys, uint8_t comp,
                             uint8_t target_comp, float enabled, float nonce) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(sys, comp, &msg, 1, target_comp, CORE_MAV_CMD_RECOVERY_CONTROL,
                                  0, enabled, nonce, 0, 0, 0, 0, 0);
    send_packed(fd, addr, &msg);
}

static void test_recovery_control_is_opt_in_and_targeted(bool allow) {
    struct sockaddr_in addr;
    int fd = open_fake_vehicle_socket(&addr);
    core_mavlink_conn_config_t cfg = {0};
    strcpy(cfg.transport.udp.bind_addr, "127.0.0.1");
    cfg.transport.udp.bind_port = ntohs(addr.sin_port);
    cfg.system_id = 1;
    cfg.component_id = 191;
    cfg.allow_recovery_control = allow;
    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    assert(conn);
    recovery_command(fd, &addr, 255, 190, 191, 0, 1);
    recovery_command(fd, &addr, 254, 1, 191, 0, 1);
    recovery_command(fd, &addr, 254, 190, 1, 0, 1);
    recovery_command(fd, &addr, 254, 190, 191, NAN, 1);
    recovery_command(fd, &addr, 254, 190, 191, 0, NAN);
    recovery_command(fd, &addr, 254, 190, 191, 0, 1.5f);
    recovery_command(fd, &addr, 254, 190, 191, 0, 0);
    for (int i = 0; i < 20; i++) { core_mavlink_conn_poll(conn); usleep(1000); }
    core_telemetry_t t;
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.recovery_control.count == 0);
    recovery_command(fd, &addr, 254, 190, 191, 0, 41);
    recovery_command(fd, &addr, 254, 190, 191, 1, 42);
    for (int i = 0; i < 20; i++) { core_mavlink_conn_poll(conn); usleep(1000); }
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.recovery_control.count == (allow ? 2u : 0u));
    if (allow) assert(t.recovery_control.enabled && t.recovery_control.nonce == 42);
    recovery_command(fd, &addr, 254, 190, 191, 2, 43);
    for (int i = 0; i < 20; i++) { core_mavlink_conn_poll(conn); usleep(1000); }
    core_mavlink_conn_get_telemetry(conn, &t);
    assert(t.recovery_control.count == (allow ? 3u : 0u));
    if (allow) assert(t.recovery_control.reset_estimate && t.recovery_control.nonce == 43);
    core_mavlink_conn_close(conn);
    close(fd);
}

/* After a heartbeat timeout the connection reopens, and the first frame back
 * may well not be a heartbeat. That frame must not find the timeout already
 * expired from before the outage: the connection used to drop again on the
 * spot, closing the socket on the heartbeat that followed, and flap like that
 * for about 10 s after every outage in SITL. */
static void test_reconnect_waits_for_a_fresh_heartbeat(void) {
    struct sockaddr_in conn_addr;
    int fake_fd = open_fake_vehicle_socket(&conn_addr);

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "127.0.0.1", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = ntohs(conn_addr.sin_port);
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 50;
    cfg.heartbeat_timeout_ms = 300;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    assert(conn != NULL);
    core_telemetry_t telemetry;
    send_fake_heartbeat(fake_fd, &conn_addr);
    assert(poll_until(conn, &telemetry, 2000, heartbeat_valid));
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED);

    usleep(400000); /* the vehicle falls silent past the timeout */
    core_mavlink_conn_poll(conn);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_DISCONNECTED);
    usleep(60000);
    core_mavlink_conn_poll(conn); /* reopens */
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTING);

    send_fake_gps(fake_fd, &conn_addr); /* back, but a heartbeat isn't first */
    assert(poll_until(conn, &telemetry, 2000, gps_valid));
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED &&
           "a frame after an outage must not hit the outage's own timeout");

    uint64_t before = telemetry.heartbeat.last_update_ms;
    usleep(2000);
    send_fake_heartbeat(fake_fd, &conn_addr);
    for (int i = 0; i < 2000 && telemetry.heartbeat.last_update_ms == before; i++) {
        core_mavlink_conn_poll(conn);
        core_mavlink_conn_get_telemetry(conn, &telemetry);
        usleep(1000);
    }
    assert(telemetry.heartbeat.last_update_ms != before && "the heartbeat that follows is received");
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED);

    usleep(400000); /* and silence after reconnecting still times out */
    core_mavlink_conn_poll(conn);
    assert(core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_DISCONNECTED);

    core_mavlink_conn_close(conn);
    close(fake_fd);
}

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

    /* No home is trusted until the autopilot has identified itself. */
    send_home(fake_fd, &conn_addr, 1, MAV_COMP_ID_AUTOPILOT1, 473977420);
    core_telemetry_t telemetry;
    assert(!poll_until(conn, &telemetry, 20, home_valid));

    /* --- heartbeat decode --- */
    send_fake_heartbeat(fake_fd, &conn_addr);
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

    /* --- operator GCS heartbeats: only sysid 255 of type GCS counts --- */
    send_other_heartbeat(fake_fd, &conn_addr, 254, MAV_TYPE_GCS);              /* another GCS, e.g. the dashboard */
    send_other_heartbeat(fake_fd, &conn_addr, 255, MAV_TYPE_ONBOARD_CONTROLLER); /* not a GCS */
    send_other_heartbeat(fake_fd, &conn_addr, 255, MAV_TYPE_GCS);
    send_other_heartbeat(fake_fd, &conn_addr, 255, MAV_TYPE_GCS);
    ok = poll_until(conn, &telemetry, 2000, two_operator_heartbeats);
    assert(ok && "timed out waiting for operator heartbeats");
    for (int i = 0; i < 200; i++) { /* let every one of them be processed before counting */
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
    core_mavlink_conn_get_telemetry(conn, &telemetry);
    assert(telemetry.gcs_heartbeat.count == 2 && "other GCSes and non-GCS types must not count");
    assert(telemetry.gcs_heartbeat.sysid == 255);
    assert(telemetry.heartbeat.type == MAV_TYPE_QUADROTOR && "a GCS heartbeat must not overwrite the vehicle's");

    /* --- RC_CHANNELS rssi: only from the autopilot itself --- */
    send_rc_rssi(fake_fd, &conn_addr, 1, MAV_COMP_ID_AUTOPILOT1, 200);
    ok = poll_until(conn, &telemetry, 2000, rc_rssi_200);
    assert(ok && "timed out waiting for RC_CHANNELS rssi");
    send_rc_rssi(fake_fd, &conn_addr, 1, MAV_COMP_ID_ONBOARD_COMPUTER, 17); /* another component */
    send_rc_rssi(fake_fd, &conn_addr, 7, MAV_COMP_ID_AUTOPILOT1, 17);        /* another vehicle */
    for (int i = 0; i < 200; i++) {
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
    core_mavlink_conn_get_telemetry(conn, &telemetry);
    assert(telemetry.rc_status.rssi == 200 && "RC_CHANNELS from anything but the autopilot must be ignored");

    /* HOME_POSITION defines the estimate's coordinate reference. Only the
     * identified autopilot may set it, and an arming-time shift must refresh it. */
    send_home(fake_fd, &conn_addr, 1, MAV_COMP_ID_AUTOPILOT1, 473977420);
    assert(poll_until(conn, &telemetry, 2000, home_valid));
    assert(telemetry.home_position.lat == 473977420);
    assert(telemetry.home_position.lon == -1223974560);
    assert(telemetry.home_position.alt_mm == 123000);
    send_home(fake_fd, &conn_addr, 7, MAV_COMP_ID_AUTOPILOT1, 1);
    send_home(fake_fd, &conn_addr, 1, MAV_COMP_ID_ONBOARD_COMPUTER, 2);
    for (int i = 0; i < 20; i++) {
        core_mavlink_conn_poll(conn);
        usleep(1000);
    }
    core_mavlink_conn_get_telemetry(conn, &telemetry);
    assert(telemetry.home_position.lat == 473977420);
    send_home(fake_fd, &conn_addr, 1, MAV_COMP_ID_AUTOPILOT1, 473978420);
    assert(poll_until(conn, &telemetry, 2000, home_shifted));

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

    core_mavlink_conn_close(conn);
    close(fake_fd);

    test_reconnect_waits_for_a_fresh_heartbeat();
    test_recovery_control_is_opt_in_and_targeted(false);
    test_recovery_control_is_opt_in_and_targeted(true);

    printf("test_mavlink_loopback: all assertions passed\n");
    return 0;
}
