/*
 * Lost-link detection against synthetic telemetry: a simulated clock, an
 * autopilot heartbeat, an operator heartbeat that can be dropped, and optional
 * radio data. Ticks every 20 ms like core_main.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/link/link_monitor.h"

#define TICK_MS 20

typedef struct {
    core_link_monitor_t mon;
    core_telemetry_t t;
    uint64_t now;
    uint64_t next_gcs_ms;
    uint32_t gcs_interval_ms;
    float drop;            /* probability each operator heartbeat is lost */
    uint32_t jitter_ms;    /* each heartbeat lands up to this early or late, as on a real link */
    uint32_t batch;        /* deliver heartbeats in groups of this many, as a bursty transport would */
    uint32_t pending;
    bool gcs_on;           /* the operator is sending */
    bool autopilot_on;
    int rf;                /* 0 none, 1 RC rssi, 2 RADIO_STATUS */
    uint8_t rssi;
    uint16_t noise;

    core_link_result_t last;
    int transitions;
    float min_trust, max_trust;
    core_link_state_t worst;
} harness_t;

static int severity(core_link_state_t s) {
    return s == CORE_LINK_UNKNOWN ? -1 : (int)s;
}

static void setup(harness_t *h) {
    memset(h, 0, sizeof(*h));
    core_link_config_t cfg;
    core_link_config_default(&cfg);
    core_link_monitor_init(&h->mon, &cfg);
    h->now = 1000;
    h->gcs_interval_ms = 1000;
    h->next_gcs_ms = h->now + 100;
    h->gcs_on = true;
    h->autopilot_on = true;
    h->rssi = 180;
    h->noise = 40;
    srand(1);
}

static void reset_stats(harness_t *h) {
    h->transitions = 0;
    h->min_trust = 1.0f;
    h->max_trust = 0.0f;
    h->worst = CORE_LINK_NOMINAL;
}

static void run(harness_t *h, uint32_t ms) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += TICK_MS) {
        h->now += TICK_MS;
        if (h->autopilot_on && h->now % 1000 < TICK_MS) {
            h->t.heartbeat.valid = true;
            h->t.heartbeat.last_update_ms = h->now;
        }
        if (h->now >= h->next_gcs_ms) {
            h->next_gcs_ms += h->gcs_interval_ms;
            if (h->jitter_ms) {
                h->next_gcs_ms += (uint64_t)(rand() % (int)(2 * h->jitter_ms + 1)) - h->jitter_ms;
            }
            bool dropped = (float)rand() / (float)RAND_MAX < h->drop;
            if (h->gcs_on && h->autopilot_on && !dropped) {
                h->pending++;
            }
            if (h->pending > 0 && (h->batch <= 1 || h->pending >= h->batch)) {
                h->t.gcs_heartbeat.valid = true;
                h->t.gcs_heartbeat.sysid = 255;
                h->t.gcs_heartbeat.count += h->pending;
                h->t.gcs_heartbeat.last_update_ms = h->now;
                h->pending = 0;
            }
        }
        if (h->autopilot_on && h->now % 500 < TICK_MS) {
            if (h->rf == 1) {
                h->t.rc_status.valid = true;
                h->t.rc_status.rssi = h->rssi;
                h->t.rc_status.last_update_ms = h->now;
            } else if (h->rf == 2) {
                h->t.radio_status.valid = true;
                h->t.radio_status.rssi = h->rssi;
                h->t.radio_status.noise = h->noise;
                h->t.radio_status.last_update_ms = h->now;
            }
        }

        h->last = core_link_monitor_update(&h->mon, &h->t, h->now);
        if (h->last.state_changed) h->transitions++;
        if (h->last.state != CORE_LINK_UNKNOWN) {
            if (h->last.link_trust < h->min_trust) h->min_trust = h->last.link_trust;
            if (h->last.link_trust > h->max_trust) h->max_trust = h->last.link_trust;
        }
        if (severity(h->last.state) > severity(h->worst)) h->worst = h->last.state;
    }
}

static void settle(harness_t *h) {
    run(h, 15000);
    assert(h->last.state == CORE_LINK_NOMINAL);
    assert(h->last.link_trust > 0.95f);
    reset_stats(h);
}

static void test_unknown_until_operator_heard(void) {
    harness_t h;
    setup(&h);
    h.gcs_on = false;
    run(&h, 10000);
    assert(h.last.state == CORE_LINK_UNKNOWN && "no operator ever heard is not a lost link");

    h.gcs_on = true;
    run(&h, 2000);
    assert(h.last.state == CORE_LINK_NOMINAL);
}

static void test_steady_link_stays_nominal(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    run(&h, 60000);
    assert(h.transitions == 0);
    assert(h.min_trust > 0.95f);
}

static void test_jittery_link_is_nominal_from_the_start(void) {
    /* Found in SITL: a few seconds in, one heartbeat landing just outside the
     * window read as a third of them missing. */
    for (unsigned seed = 1; seed <= 20; seed++) {
        harness_t h;
        setup(&h);
        srand(seed);
        h.jitter_ms = 150;
        reset_stats(&h);
        run(&h, 90000);
        assert(h.worst == CORE_LINK_NOMINAL);
        assert(h.min_trust > 0.95f);
    }
}

static void test_brief_dropouts_dont_flap(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    for (int i = 0; i < 6; i++) {
        h.gcs_on = false;
        run(&h, 2000);
        h.gcs_on = true;
        run(&h, 8000);
    }
    assert(h.worst <= CORE_LINK_DEGRADED && "a 2 s dropout is not a lost link");
    assert(h.min_trust >= 0.4f && "and must not cross autonomy's untrusted threshold");
}

static void test_light_random_loss_tolerated(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.drop = 0.1f;
    run(&h, 120000);
    assert(h.worst <= CORE_LINK_DEGRADED);
    assert(h.min_trust >= 0.4f);
}

static void test_steady_moderate_loss_doesnt_flap(void) {
    /* Found in SITL: 30% loss sits near the DEGRADED threshold, and with one
     * threshold both ways the state bounced between DEGRADED and NOMINAL. */
    for (unsigned seed = 1; seed <= 10; seed++) {
        harness_t h;
        setup(&h);
        srand(seed);
        h.jitter_ms = 150;
        settle(&h);
        h.drop = 0.3f;
        run(&h, 120000);
        assert(h.worst >= CORE_LINK_DEGRADED);
        assert(h.transitions <= 3 && "into DEGRADED and stays there, at most one excursion to LOST");
        assert(h.last.state != CORE_LINK_NOMINAL);
    }
}

static void test_heavy_loss_is_degraded_and_untrusted(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.drop = 0.5f;
    run(&h, 30000);
    assert(h.worst >= CORE_LINK_DEGRADED);
    assert(h.min_trust < 0.4f && "sustained heavy loss must reach autonomy");
}

static void test_silence_without_radio_is_lost(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.gcs_on = false; /* silence counts from the last heartbeat, up to 1 s before this */
    run(&h, 4800);
    assert(h.last.silence_ms < 6000);
    assert(h.last.state != CORE_LINK_LOST && "not yet: under lost_after_ms");
    run(&h, 2400);
    assert(h.last.state == CORE_LINK_LOST);
    assert(h.last.link_trust == 0.0f);
    assert(strstr(h.last.reason, "no radio data"));
}

static void test_rssi_separates_jamming_from_loss(void) {
    harness_t jam, lost;
    setup(&jam);
    jam.rf = 1;
    settle(&jam);
    jam.gcs_on = false; /* packets stop, but the radio still hears strong RF */
    run(&jam, 8000);
    assert(jam.last.state == CORE_LINK_JAMMED);
    assert(jam.last.rf_source == CORE_LINK_RF_RSSI);
    assert(strstr(jam.last.reason, "RF energy"));

    setup(&lost);
    lost.rf = 1;
    settle(&lost);
    lost.gcs_on = false;
    lost.rssi = 10; /* the operator's radio is gone: nothing on the air */
    run(&lost, 8000);
    assert(lost.last.state == CORE_LINK_LOST);
    assert(lost.worst == CORE_LINK_LOST && "never called jamming");
}

static void test_noise_floor_separates_jamming(void) {
    harness_t h;
    setup(&h);
    h.rf = 2;
    settle(&h);

    /* Noise rising while packets still arrive is interference, not yet loss. */
    h.noise = 120;
    run(&h, 3000);
    assert(h.last.state == CORE_LINK_DEGRADED);
    assert(h.last.interference);
    assert(strstr(h.last.reason, "interference"));

    h.gcs_on = false;
    run(&h, 8000);
    assert(h.last.state == CORE_LINK_JAMMED);
    assert(strstr(h.last.reason, "noise +"));

    /* A loss with a quiet noise floor is not jamming. */
    harness_t q;
    setup(&q);
    q.rf = 2;
    settle(&q);
    q.gcs_on = false;
    q.rssi = 10;
    run(&q, 8000);
    assert(q.last.state == CORE_LINK_LOST);
}

static void test_recovery_is_gradual_and_doesnt_flap(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.gcs_on = false;
    run(&h, 10000);
    assert(h.last.state == CORE_LINK_LOST);

    reset_stats(&h);
    h.gcs_on = true;
    run(&h, 2000);
    assert(h.last.link_trust < 0.4f && "trust climbs back slowly, not in one step");
    run(&h, 10000);
    assert(h.last.link_trust >= 0.7f && "trust, which autonomy acts on, is back within about 10 s");
    assert(h.last.state == CORE_LINK_DEGRADED && "the label waits for a clean record that excludes the outage");
    run(&h, 30000);
    assert(h.last.state == CORE_LINK_NOMINAL);
    assert(h.transitions <= 2 && "LOST -> DEGRADED -> NOMINAL, no bouncing");
}

static void test_slow_gcs_is_learned(void) {
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 2000; /* a GCS that sends every 2 s is not 50% loss */
    run(&h, 30000);
    reset_stats(&h);
    run(&h, 60000);
    assert(h.worst == CORE_LINK_NOMINAL);
    assert(h.min_trust > 0.9f);
}

static void test_fast_gcs_doesnt_overflow(void) {
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 100; /* more heartbeats per window than the buffer holds */
    run(&h, 30000);
    reset_stats(&h);
    run(&h, 30000);
    assert(h.worst == CORE_LINK_NOMINAL);
    assert(h.min_trust > 0.9f);
}

static void test_autopilot_silence_is_unknown_not_lost(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.autopilot_on = false; /* nothing is forwarded, so operator silence means nothing */
    run(&h, 20000);
    assert(h.last.state == CORE_LINK_UNKNOWN);
    assert(h.worst <= CORE_LINK_DEGRADED && "never claimed the operator was lost");

    h.autopilot_on = true;
    run(&h, 3000);
    assert(h.last.state == CORE_LINK_NOMINAL && "comes back without a false outage");
}

/* --- From review of PR #21 --- */

static void test_faster_gcs_is_relearned(void) {
    /* A GCS that speeds up must not leave loss detection tuned to the old rate. */
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 5000;
    run(&h, 60000);
    h.gcs_interval_ms = 1000;
    h.next_gcs_ms = h.now + 1000;
    run(&h, 40000);
    assert(h.mon.interval_ms < 1300);
    h.gcs_on = false;
    run(&h, 9000);
    assert(h.last.state == CORE_LINK_LOST && "a loss must be caught on the new rate's timescale");
}

static void test_slower_gcs_mid_flight_reads_as_loss(void) {
    /* Documented policy: halving the rate looks exactly like 50% loss, so it's
     * treated as one rather than learned away. */
    harness_t h;
    setup(&h);
    settle(&h);
    h.gcs_interval_ms = 2000;
    run(&h, 30000);
    assert(h.last.state == CORE_LINK_DEGRADED);
}

static void test_reconnect_relearns_rate(void) {
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 100;
    settle(&h);
    h.autopilot_on = false;
    run(&h, 5000);
    h.autopilot_on = true;
    h.gcs_interval_ms = 1000; /* a different GCS setup after the reconnect */
    h.next_gcs_ms = h.now + 500;
    run(&h, 60000);
    assert(h.last.state == CORE_LINK_NOMINAL && "the old 100 ms interval must not outlive the reconnect");
    assert(h.last.link_trust > 0.95f);
}

static void test_heartbeat_in_first_poll_counts(void) {
    /* If the only operator heartbeat ever seen arrives in the very first poll
     * and is thrown away, the link stays UNKNOWN forever and is never reported. */
    harness_t h;
    setup(&h);
    h.t.heartbeat.valid = true;
    h.t.heartbeat.last_update_ms = h.now;
    h.t.gcs_heartbeat.valid = true;
    h.t.gcs_heartbeat.count = 1;
    h.t.gcs_heartbeat.last_update_ms = h.now;
    h.gcs_on = false;
    run(&h, 20000);
    assert(h.last.state == CORE_LINK_LOST);
}

static void test_batched_arrivals_dont_hide_loss(void) {
    harness_t h;
    setup(&h);
    h.batch = 3; /* three at a time, every 3 s */
    run(&h, 30000);
    assert(h.mon.interval_ms > 800 && h.mon.interval_ms < 1200 && "learn the heartbeat rate, not the batch rate");
    h.batch = 1;
    h.drop = 0.67f;
    reset_stats(&h);
    run(&h, 60000);
    assert(h.min_trust < 0.4f && "losing two thirds of them must be caught");
}

static void test_autopilot_outage_keeps_distrust(void) {
    harness_t h;
    setup(&h);
    settle(&h);
    h.gcs_on = false;
    run(&h, 10000);
    assert(h.last.state == CORE_LINK_LOST);

    h.autopilot_on = false;
    run(&h, 5000);
    h.autopilot_on = true;
    h.gcs_on = true;
    run(&h, 2000);
    assert(h.last.link_trust < 0.4f && "an observation gap is not evidence the link recovered");
    assert(h.last.state != CORE_LINK_NOMINAL);
}

static void test_gradual_noise_rise_is_interference(void) {
    harness_t h;
    setup(&h);
    h.rf = 2;
    settle(&h);
    for (int i = 0; i < 20; i++) { /* 5 units a second, 40 -> 140 */
        h.noise += 5;
        run(&h, 1000);
    }
    assert(h.last.interference && "the floor must not follow a rising jammer up");
    h.gcs_on = false;
    run(&h, 8000);
    assert(h.last.state == CORE_LINK_JAMMED);
}

static void test_zero_rssi_is_no_radio_data(void) {
    /* ArduPilot reports 0 when RSSI_TYPE=5 is set but no radio has reported for
     * 5 s: that's an absent radio, not a weak signal. */
    harness_t h;
    setup(&h);
    h.rf = 1;
    h.rssi = 0;
    run(&h, 30000);
    assert(h.last.state == CORE_LINK_NOMINAL);
    assert(h.last.rf_source == CORE_LINK_RF_NONE);
}

int main(void) {
    test_unknown_until_operator_heard();
    test_steady_link_stays_nominal();
    test_jittery_link_is_nominal_from_the_start();
    test_brief_dropouts_dont_flap();
    test_light_random_loss_tolerated();
    test_steady_moderate_loss_doesnt_flap();
    test_heavy_loss_is_degraded_and_untrusted();
    test_silence_without_radio_is_lost();
    test_rssi_separates_jamming_from_loss();
    test_noise_floor_separates_jamming();
    test_recovery_is_gradual_and_doesnt_flap();
    test_slow_gcs_is_learned();
    test_fast_gcs_doesnt_overflow();
    test_autopilot_silence_is_unknown_not_lost();
    test_faster_gcs_is_relearned();
    test_slower_gcs_mid_flight_reads_as_loss();
    test_reconnect_relearns_rate();
    test_heartbeat_in_first_poll_counts();
    test_batched_arrivals_dont_hide_loss();
    test_autopilot_outage_keeps_distrust();
    test_gradual_noise_rise_is_interference();
    test_zero_rssi_is_no_radio_data();
    printf("test_link_monitor: all passed\n");
    return 0;
}
