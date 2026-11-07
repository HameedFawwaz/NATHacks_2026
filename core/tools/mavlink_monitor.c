/*
 * Manual connectivity check against a live ArduPilot SITL instance: opens a
 * MAVLink connection, polls it, and prints the heartbeat plus whatever other
 * telemetry we've decoded each time a new heartbeat arrives. This is not an
 * automated test (it needs SITL actually running) - see core/tests/ for the
 * loopback-based automated test of this same layer.
 *
 * Usage:
 *   mavlink_monitor [port]      (default port: 14551)
 *
 * Point ArduPilot SITL at the same port, e.g.:
 *   sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
 *
 * (14550 is typically already used by MAVProxy's own GCS link / Mission
 * Planner / QGroundControl, so SITL setups conventionally add a second
 * --out for a companion computer on 14551.)
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/common/time.h"
#include "core/mavlink/mavlink.h"

static volatile sig_atomic_t g_running = 1;

static void handle_sigint(int signum) {
    (void)signum;
    g_running = 0;
}

static void print_telemetry_snapshot(const core_telemetry_t *t) {
    printf("[HEARTBEAT] type=%u autopilot=%u base_mode=0x%02x custom_mode=%u system_status=%u\n",
           t->heartbeat.type, t->heartbeat.autopilot, t->heartbeat.base_mode,
           t->heartbeat.custom_mode, t->heartbeat.system_status);

    if (t->gps_raw.valid) {
        printf("  [GPS_RAW_INT] fix_type=%u lat=%d lon=%d alt_mm=%d sats=%u\n",
               t->gps_raw.fix_type, t->gps_raw.lat, t->gps_raw.lon,
               t->gps_raw.alt_mm, t->gps_raw.satellites_visible);
    }
    if (t->global_position.valid) {
        printf("  [GLOBAL_POSITION_INT] lat=%d lon=%d alt_mm=%d rel_alt_mm=%d hdg_cdeg=%u\n",
               t->global_position.lat, t->global_position.lon, t->global_position.alt_mm,
               t->global_position.relative_alt_mm, t->global_position.hdg_cdeg);
    }
    if (t->attitude.valid) {
        printf("  [ATTITUDE] roll=%.3f pitch=%.3f yaw=%.3f (rad)\n",
               t->attitude.roll, t->attitude.pitch, t->attitude.yaw);
    }
    if (t->radio_status.valid) {
        printf("  [RADIO_STATUS] rssi=%u remrssi=%u rxerrors=%u\n",
               t->radio_status.rssi, t->radio_status.remrssi, t->radio_status.rxerrors);
    }
    fflush(stdout);
}

int main(int argc, char **argv) {
    uint16_t port = 14551;
    if (argc > 1) {
        port = (uint16_t)atoi(argv[1]);
    }

    signal(SIGINT, handle_sigint);

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "0.0.0.0", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = port;
    cfg.transport.udp.remote_port = 0; /* learn SITL's address from the first inbound packet */
    cfg.system_id = 1;
    cfg.component_id = 191; /* MAV_COMP_ID_ONBOARD_COMPUTER */
    cfg.reconnect_interval_ms = 1000;
    cfg.heartbeat_timeout_ms = 5000;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    if (!conn) {
        fprintf(stderr, "failed to open mavlink connection on udp port %u\n", port);
        return 1;
    }

    printf("listening for MAVLink on udp 0.0.0.0:%u\n", port);
    printf("point ArduPilot SITL at this port, e.g.: sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:%u\n", port);
    printf("waiting for heartbeat...\n");

    uint64_t last_heartbeat_seen_ms = 0;
    uint64_t last_own_heartbeat_sent_ms = 0;

    while (g_running) {
        core_mavlink_conn_poll(conn);

        core_telemetry_t telemetry;
        core_mavlink_conn_get_telemetry(conn, &telemetry);

        if (telemetry.heartbeat.valid && telemetry.heartbeat.last_update_ms != last_heartbeat_seen_ms) {
            last_heartbeat_seen_ms = telemetry.heartbeat.last_update_ms;
            print_telemetry_snapshot(&telemetry);
        }

        uint64_t now = core_now_ms();
        if (now - last_own_heartbeat_sent_ms >= 1000) {
            core_mavlink_send_heartbeat(conn); /* lets SITL/MAVProxy see us as a connected peer */
            last_own_heartbeat_sent_ms = now;
        }

        usleep(20000); /* 20ms poll cadence */
    }

    printf("\nshutting down\n");
    core_mavlink_conn_close(conn);
    return 0;
}
