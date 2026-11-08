/*
 * Transparent MAVLink UDP relay between a live ArduPilot SITL instance and
 * our detector's listening port, mutating only GPS_RAW_INT messages to
 * inject one of the scripted spoofing scenarios the detector is meant to
 * catch. Everything else passes through unmodified.
 *
 * Usage:
 *   sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14551
 *   gnss_spoof_proxy --scenario=jump
 *   core_main 14552          (our detector now sees the mutated stream)
 *
 * Flags:
 *   --sitl-port=N         (default 14551)  port SITL's --out sends to; we bind here
 *   --out-addr=ADDR       (default 127.0.0.1)
 *   --out-port=N          (default 14552)  where the detector is listening
 *   --scenario=none|drift|jump|replay   (default none)
 *   --drift-rate-mps=F --drift-heading-deg=F --drift-start-sec=F
 *   --jump-at-sec=F --jump-offset-m=F --jump-heading-deg=F
 *   --replay-at-sec=F --replay-buffer-sec=F --replay-mode=looped-live-timestamp|stale-timestamp
 *
 * Simplification (documented, not a bug): assumes SITL's --out stream sends
 * one MAVLink message at a time (true in practice for ArduPilot's own UDP
 * output) - a datagram carrying multiple messages back-to-back is still
 * handled correctly (each parsed message is relayed/mutated independently),
 * just not specifically tested against.
 *
 * Known caveat: a mutated GPS_RAW_INT is re-packed via the MAVLink pack
 * helpers, which assign it a fresh sequence number from this process's own
 * counter rather than preserving SITL's original one. Passed-through
 * messages keep their original sequence numbers untouched. A receiver doing
 * strict seq-continuity checking across the whole stream may see this as a
 * handful of out-of-order/dropped packets - harmless for our own detector,
 * which doesn't rely on MAVLink sequence continuity for anything.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "common/mavlink.h"

#define METERS_PER_DEG_LAT 111320.0
#define REPLAY_BUFFER_CAPACITY 256

typedef enum { SCENARIO_NONE, SCENARIO_DRIFT, SCENARIO_JUMP, SCENARIO_REPLAY } scenario_t;
typedef enum { REPLAY_MODE_LIVE_TIMESTAMP, REPLAY_MODE_STALE_TIMESTAMP } replay_mode_t;

typedef struct {
    uint16_t sitl_port;
    char out_addr[64];
    uint16_t out_port;
    scenario_t scenario;

    double drift_rate_mps, drift_heading_deg, drift_start_sec;
    double jump_at_sec, jump_offset_m, jump_heading_deg;
    double replay_at_sec, replay_buffer_sec;
    replay_mode_t replay_mode;
} proxy_args_t;

typedef struct {
    mavlink_gps_raw_int_t fixes[REPLAY_BUFFER_CAPACITY];
    int count;
    int write_idx;
} replay_ring_t;

/* The window of history being looped during replay. Frozen once, the first
 * time replay activates - NOT recomputed from the (still-growing) ring
 * buffer on every call, otherwise the "replayed" window would just slide
 * forward in lockstep with real time and never actually replay anything
 * stale. The frozen window's samples are COPIED here (not referenced by
 * index into the live ring), since replay_ring_push keeps overwriting the
 * ring's contents while a replay is in progress (capacity 256 @ 5Hz =
 * 51.2s) - without the copy, a long-running replay would silently start
 * mixing in newer, non-frozen data once the ring wraps around. */
typedef struct {
    bool frozen;
    mavlink_gps_raw_int_t snapshot[REPLAY_BUFFER_CAPACITY];
    int snapshot_count;
    int play_idx;
} replay_playback_t;

static volatile sig_atomic_t g_running = 1;
static void handle_sigint(int signum) {
    (void)signum;
    g_running = 0;
}

static void proxy_args_default(proxy_args_t *a) {
    memset(a, 0, sizeof(*a));
    a->sitl_port = 14551;
    strncpy(a->out_addr, "127.0.0.1", sizeof(a->out_addr) - 1);
    a->out_port = 14552;
    a->scenario = SCENARIO_NONE;
    a->drift_rate_mps = 0.5;
    a->drift_heading_deg = 45.0;
    a->drift_start_sec = 5.0;
    a->jump_at_sec = 10.0;
    a->jump_offset_m = 50.0;
    a->jump_heading_deg = 90.0;
    a->replay_at_sec = 10.0;
    a->replay_buffer_sec = 5.0;
    a->replay_mode = REPLAY_MODE_LIVE_TIMESTAMP;
}

static bool arg_value(const char *arg, const char *key, const char **out_value) {
    size_t key_len = strlen(key);
    if (strncmp(arg, key, key_len) == 0 && arg[key_len] == '=') {
        *out_value = arg + key_len + 1;
        return true;
    }
    return false;
}

static void parse_args(int argc, char **argv, proxy_args_t *a) {
    const char *v;
    for (int i = 1; i < argc; i++) {
        if (arg_value(argv[i], "--sitl-port", &v)) a->sitl_port = (uint16_t)atoi(v);
        else if (arg_value(argv[i], "--out-addr", &v)) strncpy(a->out_addr, v, sizeof(a->out_addr) - 1);
        else if (arg_value(argv[i], "--out-port", &v)) a->out_port = (uint16_t)atoi(v);
        else if (arg_value(argv[i], "--scenario", &v)) {
            if (strcmp(v, "drift") == 0) a->scenario = SCENARIO_DRIFT;
            else if (strcmp(v, "jump") == 0) a->scenario = SCENARIO_JUMP;
            else if (strcmp(v, "replay") == 0) a->scenario = SCENARIO_REPLAY;
            else a->scenario = SCENARIO_NONE;
        } else if (arg_value(argv[i], "--drift-rate-mps", &v)) a->drift_rate_mps = atof(v);
        else if (arg_value(argv[i], "--drift-heading-deg", &v)) a->drift_heading_deg = atof(v);
        else if (arg_value(argv[i], "--drift-start-sec", &v)) a->drift_start_sec = atof(v);
        else if (arg_value(argv[i], "--jump-at-sec", &v)) a->jump_at_sec = atof(v);
        else if (arg_value(argv[i], "--jump-offset-m", &v)) a->jump_offset_m = atof(v);
        else if (arg_value(argv[i], "--jump-heading-deg", &v)) a->jump_heading_deg = atof(v);
        else if (arg_value(argv[i], "--replay-at-sec", &v)) a->replay_at_sec = atof(v);
        else if (arg_value(argv[i], "--replay-buffer-sec", &v)) a->replay_buffer_sec = atof(v);
        else if (arg_value(argv[i], "--replay-mode", &v)) {
            a->replay_mode = (strcmp(v, "stale-timestamp") == 0) ? REPLAY_MODE_STALE_TIMESTAMP
                                                                  : REPLAY_MODE_LIVE_TIMESTAMP;
        } else {
            fprintf(stderr, "unrecognized argument: %s\n", argv[i]);
        }
    }
}

static int open_nonblocking_udp(const char *bind_addr, uint16_t bind_port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("fcntl");
        close(fd);
        return -1;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(bind_port);
    inet_pton(AF_INET, bind_addr, &addr.sin_addr);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    return fd;
}

static void send_message(int fd, const struct sockaddr_in *dest, const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sendto(fd, buf, len, 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void replay_ring_push(replay_ring_t *ring, const mavlink_gps_raw_int_t *gps) {
    ring->fixes[ring->write_idx] = *gps;
    ring->write_idx = (ring->write_idx + 1) % REPLAY_BUFFER_CAPACITY;
    if (ring->count < REPLAY_BUFFER_CAPACITY) ring->count++;
}

/* Index of the oldest sample still within buffer_sec of the newest one (or
 * the oldest sample available, whichever is less history). */
static int replay_ring_window_start(const replay_ring_t *ring, double buffer_sec) {
    if (ring->count == 0) return 0;
    int newest_idx = (ring->write_idx - 1 + REPLAY_BUFFER_CAPACITY) % REPLAY_BUFFER_CAPACITY;
    uint64_t newest_time_usec = ring->fixes[newest_idx].time_usec;
    uint64_t window_usec = (uint64_t)(buffer_sec * 1e6);

    int idx = newest_idx;
    for (int steps = 1; steps < ring->count; steps++) {
        int candidate = (newest_idx - steps + REPLAY_BUFFER_CAPACITY * ring->count) % REPLAY_BUFFER_CAPACITY;
        uint64_t age = newest_time_usec - ring->fixes[candidate].time_usec;
        if (age > window_usec) break;
        idx = candidate;
    }
    return idx;
}

static void apply_drift(mavlink_gps_raw_int_t *gps, double elapsed_s, const proxy_args_t *a) {
    if (elapsed_s < a->drift_start_sec) return;
    double drift_elapsed = elapsed_s - a->drift_start_sec;
    double offset_m = a->drift_rate_mps * drift_elapsed;
    double heading_rad = a->drift_heading_deg * M_PI / 180.0;

    double lat = (double)gps->lat / 1e7, lon = (double)gps->lon / 1e7;
    lat += (offset_m * cos(heading_rad)) / METERS_PER_DEG_LAT;
    lon += (offset_m * sin(heading_rad)) / (METERS_PER_DEG_LAT * cos(lat * M_PI / 180.0));
    gps->lat = (int32_t)(lat * 1e7);
    gps->lon = (int32_t)(lon * 1e7);

    /* Self-consistent: reported velocity reflects real motion + drift, so
     * position and velocity don't contradict each other (the harder case). */
    double real_vel_mps = gps->vel / 100.0;
    double real_cog_rad = (gps->cog == UINT16_MAX ? 0.0 : gps->cog) * M_PI / 18000.0;
    double vn = real_vel_mps * cos(real_cog_rad) + a->drift_rate_mps * cos(heading_rad);
    double ve = real_vel_mps * sin(real_cog_rad) + a->drift_rate_mps * sin(heading_rad);
    double new_speed = sqrt(vn * vn + ve * ve);
    double new_cog = atan2(ve, vn) * 18000.0 / M_PI;
    if (new_cog < 0) new_cog += 36000.0;
    gps->vel = (uint16_t)(new_speed * 100.0);
    gps->cog = (uint16_t)new_cog;
}

static void apply_jump(mavlink_gps_raw_int_t *gps, double elapsed_s, const proxy_args_t *a) {
    if (elapsed_s < a->jump_at_sec) return;
    double heading_rad = a->jump_heading_deg * M_PI / 180.0;

    double lat = (double)gps->lat / 1e7, lon = (double)gps->lon / 1e7;
    lat += (a->jump_offset_m * cos(heading_rad)) / METERS_PER_DEG_LAT;
    lon += (a->jump_offset_m * sin(heading_rad)) / (METERS_PER_DEG_LAT * cos(lat * M_PI / 180.0));
    gps->lat = (int32_t)(lat * 1e7);
    gps->lon = (int32_t)(lon * 1e7);
    /* vel/cog deliberately left as the real reported values - the resulting
     * position/velocity mismatch is the point. */
}

static bool apply_replay(mavlink_gps_raw_int_t *gps, double elapsed_s, const proxy_args_t *a,
                          const replay_ring_t *ring, replay_playback_t *playback) {
    if (elapsed_s < a->replay_at_sec || ring->count == 0) return false;

    if (!playback->frozen) {
        int window_start = replay_ring_window_start(ring, a->replay_buffer_sec);
        int newest_idx = (ring->write_idx - 1 + REPLAY_BUFFER_CAPACITY) % REPLAY_BUFFER_CAPACITY;
        int span = newest_idx - window_start;
        if (span < 0) span += REPLAY_BUFFER_CAPACITY;
        int window_count = span + 1;

        for (int i = 0; i < window_count; i++) {
            int src_idx = (window_start + i) % REPLAY_BUFFER_CAPACITY;
            playback->snapshot[i] = ring->fixes[src_idx];
        }
        playback->snapshot_count = window_count;
        playback->play_idx = 0;
        playback->frozen = true;
    }

    int idx = playback->play_idx % playback->snapshot_count;
    playback->play_idx = playback->play_idx + 1;

    uint64_t original_time_usec = playback->snapshot[idx].time_usec;
    *gps = playback->snapshot[idx];
    if (a->replay_mode == REPLAY_MODE_LIVE_TIMESTAMP) {
        /* Re-stamp to look fresh - only KF divergence should catch this one. */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        gps->time_usec = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    } else {
        gps->time_usec = original_time_usec; /* left stale on purpose */
    }
    return true;
}

int main(int argc, char **argv) {
    proxy_args_t args;
    proxy_args_default(&args);
    parse_args(argc, argv, &args);

    signal(SIGINT, handle_sigint);

    int sock_sitl = open_nonblocking_udp("0.0.0.0", args.sitl_port);
    if (sock_sitl < 0) return 1;

    int sock_out = open_nonblocking_udp("0.0.0.0", 0);
    if (sock_out < 0) {
        close(sock_sitl);
        return 1;
    }

    struct sockaddr_in out_dest;
    memset(&out_dest, 0, sizeof(out_dest));
    out_dest.sin_family = AF_INET;
    out_dest.sin_port = htons(args.out_port);
    inet_pton(AF_INET, args.out_addr, &out_dest.sin_addr);

    bool sitl_peer_known = false;
    struct sockaddr_in sitl_peer;
    memset(&sitl_peer, 0, sizeof(sitl_peer));

    mavlink_status_t sitl_parse_status;
    memset(&sitl_parse_status, 0, sizeof(sitl_parse_status));

    replay_ring_t replay_ring;
    memset(&replay_ring, 0, sizeof(replay_ring));
    replay_playback_t replay_playback;
    memset(&replay_playback, 0, sizeof(replay_playback));

    struct timespec start_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);

    printf("gnss_spoof_proxy: relaying udp %u -> %s:%u, scenario=%d\n", args.sitl_port, args.out_addr,
           args.out_port, (int)args.scenario);

    uint8_t buf[1024];

    while (g_running) {
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        double elapsed_s = (now_ts.tv_sec - start_ts.tv_sec) + (now_ts.tv_nsec - start_ts.tv_nsec) / 1e9;

        for (int reads = 0; reads < 64; reads++) {
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            ssize_t n = recvfrom(sock_sitl, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
            if (n <= 0) break;

            if (!sitl_peer_known) {
                sitl_peer = from;
                sitl_peer_known = true;
            }

            for (ssize_t i = 0; i < n; i++) {
                mavlink_message_t msg;
                if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &sitl_parse_status)) {
                    continue;
                }

                if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    replay_ring_push(&replay_ring, &gps); /* always capture ground truth */

                    bool replaying = false;
                    switch (args.scenario) {
                    case SCENARIO_DRIFT:
                        apply_drift(&gps, elapsed_s, &args);
                        break;
                    case SCENARIO_JUMP:
                        apply_jump(&gps, elapsed_s, &args);
                        break;
                    case SCENARIO_REPLAY:
                        replaying = apply_replay(&gps, elapsed_s, &args, &replay_ring, &replay_playback);
                        break;
                    case SCENARIO_NONE:
                    default:
                        break;
                    }
                    (void)replaying;

                    mavlink_message_t out_msg;
                    mavlink_msg_gps_raw_int_pack(msg.sysid, msg.compid, &out_msg, gps.time_usec, gps.fix_type,
                                                  gps.lat, gps.lon, gps.alt, gps.eph, gps.epv, gps.vel, gps.cog,
                                                  gps.satellites_visible, gps.alt_ellipsoid, gps.h_acc, gps.v_acc,
                                                  gps.vel_acc, gps.hdg_acc, gps.yaw);
                    send_message(sock_out, &out_dest, &out_msg);
                } else {
                    send_message(sock_out, &out_dest, &msg); /* transparent passthrough */
                }
            }
        }

        for (int reads = 0; reads < 64; reads++) {
            ssize_t n = recvfrom(sock_out, buf, sizeof(buf), MSG_DONTWAIT, NULL, NULL);
            if (n <= 0) break;
            if (sitl_peer_known) {
                sendto(sock_sitl, buf, n, 0, (struct sockaddr *)&sitl_peer, sizeof(sitl_peer));
            }
        }

        usleep(5000);
    }

    printf("\ngnss_spoof_proxy: shutting down\n");
    close(sock_sitl);
    close(sock_out);
    return 0;
}
