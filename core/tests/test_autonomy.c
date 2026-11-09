/*
 * Decision and command tests for the autonomy layer, run over loopback UDP
 * with a fake vehicle. The clock is passed in, so hold times and timeouts are
 * exercised without actually waiting for them.
 *
 * Like test_mavlink_loopback, this plays the flight controller and so may use
 * the vendored MAVLink headers directly.
 */
#include <arpa/inet.h>
#include <assert.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "common/mavlink.h"

#include "core/autonomy/autonomy.h"
#include "core/common/time.h"

#define MODE_GUIDED 4
#define MODE_RTL 6
#define MODE_LAND 9
#define MODE_BRAKE 17

typedef struct {
    int fd;
    struct sockaddr_in conn_addr;
    core_mavlink_conn_t *conn;
    core_autonomy_t autonomy;
    core_trust_score_t score;
    core_telemetry_t telemetry;
    uint64_t now;
    FILE *log;
    uint64_t vehicle_heard_ms; /* if set, overrides the vehicle heartbeat's real-time stamp */
    /* What the fake vehicle received on the most recent tick. */
    int mode_commands, source_commands;
    uint32_t last_mode;
    int last_source;
} harness_t;

static void vehicle_send(harness_t *h, const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    ssize_t sent = sendto(h->fd, buf, len, 0, (struct sockaddr *)&h->conn_addr, sizeof(h->conn_addr));
    assert(sent == (ssize_t)len);
}

static void sync_conn(harness_t *h, bool (*done)(const harness_t *)) {
    for (int i = 0; i < 2000; i++) {
        core_mavlink_conn_poll(h->conn);
        core_mavlink_conn_get_telemetry(h->conn, &h->telemetry);
        if (done(h)) {
            return;
        }
        usleep(1000);
    }
    assert(!"timed out waiting for loopback delivery");
}

static uint64_t g_prev_hb, g_prev_ack;
static bool new_heartbeat(const harness_t *h) { return h->telemetry.heartbeat.last_update_ms != g_prev_hb; }
static bool new_ack(const harness_t *h) { return h->telemetry.command_ack.last_update_ms != g_prev_ack; }

static void vehicle_heartbeat(harness_t *h, uint32_t mode, bool armed) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0), mode,
                                MAV_STATE_ACTIVE);
    usleep(1500); /* so last_update_ms visibly changes */
    /* tick() can override the harness copy with its simulated clock. Compare
     * against the connection's actual timestamp, or the old armed heartbeat
     * may appear "new" before this datagram has even arrived. */
    core_mavlink_conn_get_telemetry(h->conn, &h->telemetry);
    g_prev_hb = h->telemetry.heartbeat.last_update_ms;
    vehicle_send(h, &msg);
    sync_conn(h, new_heartbeat);
}

static void vehicle_ack(harness_t *h, uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, command, result, 0, 0, 1, 191);
    usleep(1500);
    g_prev_ack = h->telemetry.command_ack.last_update_ms;
    vehicle_send(h, &msg);
    sync_conn(h, new_ack);
}

/* Collects whatever the code under test sent. Loopback delivery isn't
 * synchronous, so this reads until the socket has been quiet for a moment. */
static void vehicle_drain(harness_t *h) {
    h->mode_commands = 0;
    h->source_commands = 0;

    static mavlink_status_t status;
    struct pollfd pfd = {.fd = h->fd, .events = POLLIN};
    uint8_t buf[512];
    while (poll(&pfd, 1, 5) > 0) {
        ssize_t n = recv(h->fd, buf, sizeof(buf), 0);
        assert(n > 0);
        for (ssize_t i = 0; i < n; i++) {
            mavlink_message_t msg;
            if (!mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status) || msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
                continue;
            }
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            assert(cmd.target_system == 1);
            assert(cmd.target_component == MAV_COMP_ID_AUTOPILOT1);
            if (cmd.command == MAV_CMD_DO_SET_MODE) {
                assert(cmd.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                h->mode_commands++;
                h->last_mode = (uint32_t)cmd.param2;
            } else if (cmd.command == CORE_MAV_CMD_SET_EKF_SOURCE_SET) {
                h->source_commands++;
                h->last_source = (int)cmd.param1;
            }
        }
    }
}

static void tick(harness_t *h, uint64_t advance_ms) {
    h->now += advance_ms;
    core_mavlink_conn_get_telemetry(h->conn, &h->telemetry);
    if (h->vehicle_heard_ms) {
        h->telemetry.heartbeat.last_update_ms = h->vehicle_heard_ms;
    }
    core_autonomy_tick(&h->autonomy, h->conn, &h->score, &h->telemetry, h->now);
    vehicle_drain(h);
}

/* Sets a score as a live producer would, i.e. reported as of the test clock. */
static void report(harness_t *h, core_trust_source_t source, float value) {
    core_trust_score_set_component(&h->score, source, value);
    switch (source) {
    case CORE_TRUST_SOURCE_NAV: h->score.nav_update_ms = 0; break;
    case CORE_TRUST_SOURCE_LINK: h->score.link_update_ms = 0; break;
    case CORE_TRUST_SOURCE_HARDWARE: h->score.hardware_update_ms = 0; break;
    default: break;
    }
}

static void reset(harness_t *h, uint32_t mode, bool armed) {
    core_autonomy_config_t cfg = core_autonomy_default_config();
    if (h->log) {
        fclose(h->log);
    }
    h->log = tmpfile();
    assert(h->log);
    core_autonomy_init(&h->autonomy, &cfg, h->log);
    core_trust_score_init(&h->score);
    h->now = 1000; /* well behind the real clock that stamps inbound acks */
    h->vehicle_heard_ms = 0;
    h->last_mode = 0;
    h->last_source = 0;
    vehicle_heartbeat(h, mode, armed);
}

static void test_nominal_commands_nothing(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->mode_commands == 0 && h->source_commands == 0);
    assert(h->score.overall_trust == 1.0f);
}

static void test_recovery_toggle_gates_commands_not_detection(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    assert(h->autonomy.enabled);
    core_autonomy_set_enabled(&h->autonomy, false, h->now);
    report(h, CORE_TRUST_SOURCE_NAV, 0.0f);
    tick(h, 20);
    assert(!h->autonomy.enabled);
    assert(h->mode_commands == 0 && h->source_commands == 0);
    assert(h->score.overall_trust == 0.0f && "disabled responses must not hide the attack");
    tick(h, 60000);
    assert(h->mode_commands == 0 && h->source_commands == 0);
    core_autonomy_set_enabled(&h->autonomy, true, h->now);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);
    assert(h->source_commands == 1 && h->last_source == 2);

    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD);
    core_autonomy_set_enabled(&h->autonomy, false, h->now);
    tick(h, 60000);
    core_autonomy_set_enabled(&h->autonomy, true, h->now);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD && "disabled time is not a return timeout");
    assert(h->mode_commands == 0 && "do not replay an old pending hold after enabling");
}

static void test_gnss_failover_and_recovery(harness_t *h) {
    reset(h, MODE_GUIDED, true);

    report(h, CORE_TRUST_SOURCE_NAV, 0.1f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);
    assert(h->source_commands == 1 && h->last_source == 2);
    assert(h->mode_commands == 0 && "failover must let the current flight continue");
    assert(h->score.overall_trust < 0.11f);

    vehicle_ack(h, CORE_MAV_CMD_SET_EKF_SOURCE_SET, MAV_RESULT_ACCEPTED);
    tick(h, 20);
    assert(h->autonomy.active_source_set == 2);
    tick(h, 2000);
    assert(h->source_commands == 0 && "no resend once acknowledged");

    /* Above the threshold, but not for long enough. */
    report(h, CORE_TRUST_SOURCE_NAV, 0.9f);
    tick(h, 20);
    tick(h, 4000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);

    /* A dip restarts the recovery hold, so flapping can't win. */
    report(h, CORE_TRUST_SOURCE_NAV, 0.5f);
    tick(h, 20);
    report(h, CORE_TRUST_SOURCE_NAV, 0.9f);
    tick(h, 20);
    tick(h, 4000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);

    tick(h, 1100);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->source_commands == 1 && h->last_source == 1);
}

static void test_failover_rejected_lands(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_NAV, 0.0f);
    tick(h, 20);
    vehicle_ack(h, CORE_MAV_CMD_SET_EKF_SOURCE_SET, MAV_RESULT_DENIED);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);
    assert(h->mode_commands == 1 && h->last_mode == MODE_LAND);
}

static void test_failover_unacknowledged_lands(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_NAV, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);
    tick(h, 1000);
    assert(h->source_commands == 1 && "unacknowledged command is retried");
    tick(h, 4100);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);
}

static void test_link_loss_escalates_and_stops_commanding(harness_t *h) {
    reset(h, MODE_GUIDED, true);

    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD);
    assert(h->mode_commands == 1 && h->last_mode == MODE_BRAKE);
    assert(h->source_commands == 0 && "nav is still trusted");

    tick(h, 1000);
    assert(h->mode_commands == 1 && "retried until the heartbeat confirms");

    vehicle_heartbeat(h, MODE_BRAKE, true);
    tick(h, 20);
    assert(h->mode_commands == 0);

    /* A pilot overriding the mode is not fought. */
    vehicle_heartbeat(h, MODE_GUIDED, true);
    tick(h, 2000);
    assert(h->mode_commands == 0);

    tick(h, 28000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_RETURN);
    assert(h->mode_commands == 1 && h->last_mode == MODE_RTL);
}

static void test_link_recovery_leaves_mode_alone(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    vehicle_heartbeat(h, MODE_BRAKE, true);
    tick(h, 20);

    report(h, CORE_TRUST_SOURCE_LINK, 1.0f);
    tick(h, 20);
    tick(h, 5100);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->mode_commands == 0);
}

static void test_link_loss_does_not_interrupt_landing(harness_t *h) {
    reset(h, MODE_LAND, true);
    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD);
    assert(h->mode_commands == 0);
    tick(h, 2000);
    assert(h->mode_commands == 0);
}

static void test_multiple_untrusted_land_is_latched(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_NAV, 0.0f);
    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);
    assert(h->mode_commands == 1 && h->last_mode == MODE_LAND);
    assert(h->source_commands == 1 && h->last_source == 2);

    /* Recovery doesn't resume flight once descending... */
    report(h, CORE_TRUST_SOURCE_NAV, 1.0f);
    report(h, CORE_TRUST_SOURCE_LINK, 1.0f);
    tick(h, 20);
    tick(h, 6000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);

    /* ...only touchdown and disarm clear it. */
    vehicle_heartbeat(h, MODE_LAND, false);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
}

static void test_hardware_alone_lands(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_HARDWARE, 0.1f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);
}

static void test_crypto_does_not_select_behavior(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_CRYPTO, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->score.overall_trust == 0.0f);
}

static void test_stale_producer_is_untrusted(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    tick(h, 60000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL && "a producer that never reported is ignored");

    h->score.nav_update_ms = h->now;
    tick(h, 2000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    tick(h, 1100);
    assert(h->autonomy.behavior == CORE_AUTONOMY_GNSS_FAILOVER);
}

/* Stamps the vehicle as heard at the test clock's now, for the tests about
 * its heartbeat going stale. Others leave this 0 and the clock far behind the
 * real-time stamps, so the vehicle never looks stale to them. */
static void heard(harness_t *h) { h->vehicle_heard_ms = h->now; }

/* Reports nav and link as fresh as of the test clock. */
static void producers(harness_t *h, float nav, float link) {
    core_trust_score_set_component(&h->score, CORE_TRUST_SOURCE_NAV, nav);
    core_trust_score_set_component(&h->score, CORE_TRUST_SOURCE_LINK, link);
    h->score.nav_update_ms = h->now;
    h->score.link_update_ms = h->now;
}

/* When the autopilot's own heartbeat stops reaching us, its armed state and mode
 * are unknown and the link monitor can't judge anything, so a stale input then
 * is our blindness, not an attack. Acting on it would brake or RTL the vehicle
 * once the companion link returned, with the operator in control all along. In
 * SITL, before this, an 8 s cut went LINK_HOLD then SAFE_LAND. */
static void test_stale_vehicle_heartbeat_freezes_decisions(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    h->now = 100000;
    heard(h);
    producers(h, 1.0f, 1.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);

    /* The autopilot falls silent, and with it every producer it feeds: link
     * unreported, nav starved of GPS. */
    tick(h, 3500);
    producers(h, 0.0f, 1.0f);
    h->score.link_update_ms = h->now - 3500;
    tick(h, 30000);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL && "no verdict from stale vehicle state");
    assert(h->mode_commands == 0 && h->source_commands == 0);

    /* It comes back. Nav, starved through the outage, still reads 0 for a tick
     * or two; in SITL that alone switched the EKF off GPS for 6.5 s. */
    for (int i = 0; i < 50; i++) { /* 1 s */
        h->now += 20;
        heard(h);
        producers(h, 0.0f, 1.0f);
        tick(h, 0);
    }
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL && "producers get a stale period to catch up");
    for (int i = 0; i < 110; i++) { /* 2.2 s more, nav caught up */
        h->now += 20;
        heard(h);
        producers(h, 1.0f, 1.0f);
        tick(h, 0);
    }
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->mode_commands == 0 && h->source_commands == 0);

    /* Real link loss after that is acted on at once. */
    heard(h);
    producers(h, 1.0f, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD);
    assert(h->mode_commands == 1 && h->last_mode == MODE_BRAKE);
    h->vehicle_heard_ms = 0;
}

/* An attack already under way when the autopilot comes back is acted on once
 * the settling period is over: it delays, never cancels. */
static void test_untrusted_after_resume_is_acted_on(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    h->now = 100000;
    heard(h);
    producers(h, 1.0f, 1.0f);
    tick(h, 20);
    tick(h, 3500); /* stale */
    bool acted = false;
    uint64_t back = h->now;
    for (int i = 0; i < 250 && !acted; i++) { /* 5 s with the link lost */
        h->now += 20;
        heard(h);
        producers(h, 1.0f, 0.0f);
        tick(h, 0);
        acted = h->autonomy.behavior == CORE_AUTONOMY_LINK_HOLD;
    }
    assert(acted);
    assert(h->now - back >= 3000 && h->now - back <= 3100 && "acted on as soon as the settling period ended");
    assert(h->mode_commands == 1 && h->last_mode == MODE_BRAKE);
    h->vehicle_heard_ms = 0;
}

static void test_disarmed_commands_no_mode(harness_t *h) {
    reset(h, MODE_GUIDED, false);
    report(h, CORE_TRUST_SOURCE_NAV, 0.0f);
    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_NOMINAL);
    assert(h->mode_commands == 0 && h->source_commands == 0);

    vehicle_heartbeat(h, MODE_GUIDED, true);
    tick(h, 20);
    assert(h->autonomy.behavior == CORE_AUTONOMY_SAFE_LAND);
}

static void test_gcs_heartbeat_is_not_the_vehicle(harness_t *h) {
    reset(h, MODE_BRAKE, true);

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                                MAV_STATE_ACTIVE);
    vehicle_send(h, &msg);
    /* Follow with a vehicle heartbeat: once it's in, the GCS one has been handled too. */
    vehicle_heartbeat(h, MODE_BRAKE, true);
    assert(h->telemetry.heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA);
    assert(h->telemetry.heartbeat.custom_mode == MODE_BRAKE);

    report(h, CORE_TRUST_SOURCE_LINK, 0.0f);
    tick(h, 20);
    assert(h->mode_commands == 0 && "already holding");
}

static void test_transition_log(harness_t *h) {
    reset(h, MODE_GUIDED, true);
    report(h, CORE_TRUST_SOURCE_LINK, 0.25f);
    tick(h, 20);

    char contents[512];
    rewind(h->log);
    size_t n = fread(contents, 1, sizeof(contents) - 1, h->log);
    contents[n] = '\0';
    assert(strstr(contents, "mono_ms,from,to,reason,nav_trust,link_trust,hardware_trust\n") == contents);
    assert(strstr(contents, ",NOMINAL,LINK_HOLD,link untrusted,1.000,0.250,1.000\n"));
}

int main(void) {
    harness_t h;
    memset(&h, 0, sizeof(h));

    h.fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(h.fd >= 0);
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &bind_addr.sin_addr);
    assert(bind(h.fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);

    h.conn_addr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &h.conn_addr.sin_addr);
    int probe = socket(AF_INET, SOCK_DGRAM, 0);
    assert(probe >= 0);
    assert(bind(probe, (struct sockaddr *)&h.conn_addr, sizeof(h.conn_addr)) == 0);
    socklen_t addr_len = sizeof(h.conn_addr);
    assert(getsockname(probe, (struct sockaddr *)&h.conn_addr, &addr_len) == 0);
    close(probe);

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "127.0.0.1", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = ntohs(h.conn_addr.sin_port);
    cfg.system_id = 1;
    cfg.component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg.reconnect_interval_ms = 1000;
    cfg.heartbeat_timeout_ms = 60000;
    h.conn = core_mavlink_conn_open(&cfg);
    assert(h.conn);

    test_nominal_commands_nothing(&h);
    test_recovery_toggle_gates_commands_not_detection(&h);
    test_gnss_failover_and_recovery(&h);
    test_failover_rejected_lands(&h);
    test_failover_unacknowledged_lands(&h);
    test_link_loss_escalates_and_stops_commanding(&h);
    test_link_recovery_leaves_mode_alone(&h);
    test_link_loss_does_not_interrupt_landing(&h);
    test_multiple_untrusted_land_is_latched(&h);
    test_hardware_alone_lands(&h);
    test_crypto_does_not_select_behavior(&h);
    test_stale_producer_is_untrusted(&h);
    test_stale_vehicle_heartbeat_freezes_decisions(&h);
    test_untrusted_after_resume_is_acted_on(&h);
    test_disarmed_commands_no_mode(&h);
    test_gcs_heartbeat_is_not_the_vehicle(&h);
    test_transition_log(&h);

    core_mavlink_conn_close(h.conn);
    fclose(h.log);
    close(h.fd);

    printf("test_autonomy: all assertions passed\n");
    return 0;
}
