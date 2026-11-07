/*
 * Runs the autonomy decision loop against a live vehicle, with trust scores
 * read from stdin in place of the nav/link detectors, which aren't merged yet.
 *
 * Usage:
 *   autonomy_runner [port] [transition_log.csv]   (defaults: 14551, autonomy_transitions.csv)
 *
 * Then type one score per line: nav 0.1 | link 0 | hw 1 | crypto 1
 *
 * A score that has been typed is re-reported every tick, because autonomy
 * treats a silent producer as untrusted. One never typed stays unmonitored.
 */
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/autonomy/autonomy.h"
#include "core/common/time.h"
#include "core/mavlink/mavlink.h"

static volatile sig_atomic_t g_running = 1;

static void handle_sigint(int signum) {
    (void)signum;
    g_running = 0;
}

typedef struct {
    const char *name;
    core_trust_source_t source;
    float value;
    bool reported;
} stand_in_t;

static stand_in_t g_inputs[] = {
    {"nav", CORE_TRUST_SOURCE_NAV, 1.0f, false},
    {"link", CORE_TRUST_SOURCE_LINK, 1.0f, false},
    {"hw", CORE_TRUST_SOURCE_HARDWARE, 1.0f, false},
    {"crypto", CORE_TRUST_SOURCE_CRYPTO, 1.0f, false},
};

static void apply_line(const char *line) {
    char name[16];
    float value;
    if (sscanf(line, "%15s %f", name, &value) != 2) {
        fprintf(stderr, "expected: <nav|link|hw|crypto> <score>\n");
        return;
    }
    for (size_t i = 0; i < sizeof(g_inputs) / sizeof(g_inputs[0]); i++) {
        if (strcmp(name, g_inputs[i].name) == 0) {
            g_inputs[i].value = value;
            g_inputs[i].reported = true;
            return;
        }
    }
    fprintf(stderr, "unknown input: %s\n", name);
}

/* Handles whole lines only; stdin is a terminal or a line-flushing pipe. */
static void read_stdin(void) {
    struct pollfd pfd = {.fd = STDIN_FILENO, .events = POLLIN};
    if (poll(&pfd, 1, 0) <= 0 || !(pfd.revents & (POLLIN | POLLHUP))) {
        return;
    }

    char buf[256];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf) - 1);
    if (n <= 0) {
        return;
    }
    buf[n] = '\0';

    char *save = NULL;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        apply_line(line);
    }
}

int main(int argc, char **argv) {
    uint16_t port = (argc > 1) ? (uint16_t)atoi(argv[1]) : 14551;
    const char *log_path = (argc > 2) ? argv[2] : "autonomy_transitions.csv";

    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    FILE *log = fopen(log_path, "w");
    if (!log) {
        fprintf(stderr, "failed to open %s: %s\n", log_path, strerror(errno));
        return 1;
    }

    core_mavlink_conn_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type = CORE_MAVLINK_TRANSPORT_UDP;
    strncpy(cfg.transport.udp.bind_addr, "0.0.0.0", sizeof(cfg.transport.udp.bind_addr) - 1);
    cfg.transport.udp.bind_port = port;
    cfg.system_id = 1;
    cfg.component_id = 191; /* MAV_COMP_ID_ONBOARD_COMPUTER */
    cfg.reconnect_interval_ms = 1000;
    cfg.heartbeat_timeout_ms = 5000;

    core_mavlink_conn_t *conn = core_mavlink_conn_open(&cfg);
    if (!conn) {
        fprintf(stderr, "failed to open mavlink connection on udp port %u\n", port);
        fclose(log);
        return 1;
    }

    core_autonomy_config_t autonomy_cfg = core_autonomy_default_config();
    core_autonomy_t autonomy;
    core_autonomy_init(&autonomy, &autonomy_cfg, log);

    core_trust_score_t score;
    core_trust_score_init(&score);

    uint64_t last_slow_tick_ms = 0;

    while (g_running) {
        core_mavlink_conn_poll(conn);
        read_stdin();

        for (size_t i = 0; i < sizeof(g_inputs) / sizeof(g_inputs[0]); i++) {
            if (g_inputs[i].reported) {
                core_trust_score_set_component(&score, g_inputs[i].source, g_inputs[i].value);
            }
        }

        core_telemetry_t telemetry;
        core_mavlink_conn_get_telemetry(conn, &telemetry);

        uint64_t now = core_now_ms();
        core_autonomy_tick(&autonomy, conn, &score, &telemetry, now);

        if (now - last_slow_tick_ms >= 1000 && core_mavlink_conn_get_state(conn) == CORE_MAVLINK_CONN_CONNECTED) {
            last_slow_tick_ms = now;
            core_mavlink_send_heartbeat(conn);
            core_mavlink_publish_trust_score(conn, &score);
        }

        usleep(20000); /* 20ms poll cadence */
    }

    core_mavlink_conn_close(conn);
    fclose(log);
    return 0;
}
