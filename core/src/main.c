/*
 * The real integration loop on a computer: connects to the flight controller
 * over MAVLink and runs core's detection + autonomy loop (src/app/core_app.c)
 * against it. The ESP32 firmware (firmware/esp32) runs the same loop over
 * its UART instead.
 *
 * Usage:
 *   core_main [port] [--allow-recovery-control] [--gps-free-backup]
 *   core_main --serial DEVICE[:BAUD] [...]
 *
 * Point ArduPilot SITL at the same port (see core/README.md), or chain it
 * through core/sitl/gnss_spoof_proxy to inject a spoofing scenario on top of
 * a real SITL instance. --serial reads a tty instead (default 115200 baud),
 * e.g. a flight controller's telemetry port.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/app/core_app.h"

static volatile sig_atomic_t g_running = 1;

static void handle_sigint(int signum) {
    (void)signum;
    g_running = 0;
}

static void print_line(void *ctx, const char *line) {
    (void)ctx;
    printf("%s\n", line);
    fflush(stdout);
}

int main(int argc, char **argv) {
    uint16_t port = 14551;
    const char *serial = NULL;
    bool allow_recovery_control = false;
    bool gps_free_backup = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--allow-recovery-control") == 0) {
            allow_recovery_control = true;
        } else if (strcmp(argv[i], "--gps-free-backup") == 0) {
            gps_free_backup = true;
        } else if (strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            serial = argv[++i];
        } else if (argv[i][0] != '-') {
            port = (uint16_t)atoi(argv[i]);
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    signal(SIGINT, handle_sigint);

    core_mavlink_conn_config_t conn_cfg;
    memset(&conn_cfg, 0, sizeof(conn_cfg));
    if (serial) {
        conn_cfg.type = CORE_MAVLINK_TRANSPORT_SERIAL;
        strncpy(conn_cfg.transport.serial.device, serial, sizeof(conn_cfg.transport.serial.device) - 1);
        conn_cfg.transport.serial.baud_rate = 115200;
        char *colon = strrchr(conn_cfg.transport.serial.device, ':');
        if (colon) {
            *colon = '\0';
            conn_cfg.transport.serial.baud_rate = (uint32_t)strtoul(colon + 1, NULL, 10);
        }
    } else {
        conn_cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
        strncpy(conn_cfg.transport.udp.bind_addr, allow_recovery_control ? "127.0.0.1" : "0.0.0.0",
                sizeof(conn_cfg.transport.udp.bind_addr) - 1);
        conn_cfg.transport.udp.bind_port = port;
        conn_cfg.transport.udp.remote_port = 0; /* learn peer from first inbound packet */
    }
    conn_cfg.system_id = 1;
    conn_cfg.component_id = 191; /* MAV_COMP_ID_ONBOARD_COMPUTER */
    conn_cfg.reconnect_interval_ms = 1000;
    conn_cfg.heartbeat_timeout_ms = 5000;
    conn_cfg.allow_recovery_control = allow_recovery_control;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&conn_cfg);
    if (!conn) {
        if (serial) {
            fprintf(stderr, "failed to open mavlink connection on serial %s\n", serial);
        } else {
            fprintf(stderr, "failed to open mavlink connection on udp port %u\n", port);
        }
        return 1;
    }

    FILE *autonomy_log = fopen("autonomy_transitions.csv", "w");
    if (!autonomy_log) {
        fprintf(stderr, "[WARN] could not open autonomy_transitions.csv, logging transitions to stderr only\n");
    }

    core_app_config_t app_cfg = {
        .allow_recovery_control = allow_recovery_control,
        .gps_free_backup = gps_free_backup,
        .autonomy_log = autonomy_log,
        .print = print_line,
    };
    static core_app_t app;
    core_app_init(&app, conn, &app_cfg);

    printf("drone survivability core - GNSS spoofing and lost-link detection + autonomy\n");
    if (serial) {
        printf("reading MAVLink on serial %s at %u baud\n", conn_cfg.transport.serial.device,
               conn_cfg.transport.serial.baud_rate);
    } else {
        printf("listening for MAVLink on udp %s:%u\n", conn_cfg.transport.udp.bind_addr, port);
        printf("point ArduPilot SITL (optionally via core/sitl/gnss_spoof_proxy) at this port\n");
    }

    while (g_running) {
        core_app_tick(&app);
        usleep(20000); /* 20ms poll cadence */
    }

    printf("\nshutting down\n");
    core_mavlink_conn_close(conn);
    if (autonomy_log) {
        fclose(autonomy_log);
    }
    return 0;
}
