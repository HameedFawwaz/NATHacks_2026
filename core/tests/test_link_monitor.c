/*
 * Lost-link detection against synthetic telemetry: a simulated clock, an
 * autopilot heartbeat, an operator heartbeat that can be dropped, delayed,
 * jittered or batched, and optional radio data. Ticks every 20 ms like
 * core_main.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/link/link_monitor.h"

#define TICK_MS 20
#define QUEUE 256

typedef struct {
    core_link_monitor_t mon;
    core_telemetry_t t;
    uint64_t now;

    uint64_t next_gcs_ms;
    uint32_t gcs_interval_ms;  /* what the GCS actually sends */
    float drop;                /* probability each operator heartbeat is lost */
    uint32_t drop_every, sent; /* or exactly every Nth */
    uint32_t jitter_ms;        /* each one delivered up to this late or early */
    const uint32_t *delays;    /* or a repeating pattern of delivery delays */
    int delays_len, delays_i;
    uint32_t batch;            /* delivered in groups of this many */
    uint64_t queue[QUEUE];     /* delivery times of heartbeats in flight */
    int queued;
    uint32_t ready;

    bool gcs_on, autopilot_on;
    int rf;                    /* 0 none, 1 RC rssi, 2 RADIO_STATUS */
    uint8_t rssi;
    uint16_t noise;

    core_link_result_t last;
    int transitions;
    int back_to_nominal; /* changes into NOMINAL: the flapping that matters */
    float min_trust;
    core_link_state_t worst;
} harness_t;

static int severity(core_link_state_t s) {
    return s == CORE_LINK_UNKNOWN ? -1 : (int)s;
}

static void reset_stats(harness_t *h) {
    h->transitions = 0;
    h->back_to_nominal = 0;
    h->min_trust = 1.0f;
    h->worst = CORE_LINK_NOMINAL;
}

static void setup_with(harness_t *h, uint32_t configured_interval_ms) {
    memset(h, 0, sizeof(*h));
    core_link_config_t cfg;
    core_link_config_default(&cfg);
    if (configured_interval_ms) {
        cfg.heartbeat_interval_ms = configured_interval_ms;
    }
    core_link_monitor_init(&h->mon, &cfg);
    h->now = 1000;
    h->gcs_interval_ms = cfg.heartbeat_interval_ms;
    h->next_gcs_ms = h->now + 100;
    h->gcs_on = true;
    h->autopilot_on = true;
    h->rssi = 180;
    h->noise = 40;
    srand(1);
    reset_stats(h);
}

static void setup(harness_t *h) {
    setup_with(h, 0);
}

static void deliver(harness_t *h, uint32_t n) {
    h->t.gcs_heartbeat.valid = true;
    h->t.gcs_heartbeat.sysid = 255;
    h->t.gcs_heartbeat.count += n;
    h->t.gcs_heartbeat.last_update_ms = h->now;
}

static void run(harness_t *h, uint32_t ms) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += TICK_MS) {
        h->now += TICK_MS;
        if (h->autopilot_on && h->now % 1000 < TICK_MS) {
            h->t.heartbeat.valid = true;
            h->t.heartbeat.last_update_ms = h->now;
        }

        /* The GCS sends on its own schedule... */
        if (h->now >= h->next_gcs_ms) {
            h->next_gcs_ms += h->gcs_interval_ms;
            bool dropped = (float)rand() / (float)RAND_MAX < h->drop;
            if (h->drop_every && ++h->sent % h->drop_every == 0) dropped = true;
            if (h->gcs_on && !dropped && h->queued < QUEUE) {
                uint64_t at = h->now;
                if (h->jitter_ms) at += (uint64_t)(rand() % (int)(2 * h->jitter_ms + 1));
                if (h->delays_len) at += h->delays[h->delays_i++ % h->delays_len];
                h->queue[h->queued++] = at;
            }
        }
        /* ...and the link delivers each when it's due, unless the autopilot is
         * down, when nothing is forwarded at all. */
        for (int i = 0; i < h->queued;) {
            if (h->queue[i] <= h->now) {
                h->queue[i] = h->queue[--h->queued];
                if (h->autopilot_on) h->ready++;
            } else {
                i++;
            }
        }
        if (h->ready > 0 && (h->batch <= 1 || h->ready >= h->batch)) {
            deliver(h, h->ready);
            h->ready = 0;
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
        if (h->last.state_changed) {
            h->transitions++;
            if (h->last.state == CORE_LINK_NOMINAL) h->back_to_nominal++;
        }
        if (h->last.state != CORE_LINK_UNKNOWN && h->last.link_trust < h->min_trust) h->min_trust = h->last.link_trust;
        if (severity(h->last.state) > severity(h->worst)) h->worst = h->last.state;
    }
}

static void settle(harness_t *h) {
    run(h, 40000);
    assert(h->last.state == CORE_LINK_NOMINAL);
    assert(h->last.link_trust > 0.95f);
    reset_stats(h);
}

/* --- Basics --- */

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

static void test_jitter_stays_nominal(void) {
    /* Found in SITL: a few seconds in, one heartbeat landing just outside the
     * window read as a third of them missing. Also the second review's 5 Hz case. */
    struct { uint32_t interval, jitter; } cases[] = {{1000, 150}, {1000, 350}, {200, 80}, {100, 40}};
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        for (unsigned seed = 1; seed <= 10; seed++) {
            harness_t h;
            setup_with(&h, cases[c].interval);
            srand(seed);
            h.jitter_ms = cases[c].jitter;
            run(&h, 90000);
            assert(h.worst == CORE_LINK_NOMINAL);
            assert(h.min_trust > 0.8f);
        }
    }
}

static void test_bursty_lossless_delivery_stays_nominal(void) {
    /* Third review: delays of 1000, 500, 0 ms leave gaps of 500, 500, 2000 ms
     * with nothing lost. A learned interval read that as half the heartbeats
     * missing and then locked itself in. */
    static const uint32_t pattern[] = {1000, 500, 0};
    harness_t h;
    setup(&h);
    h.delays = pattern;
    h.delays_len = 3;
    run(&h, 180000);
    assert(h.worst == CORE_LINK_NOMINAL);
    assert(h.min_trust > 0.8f);
}

static void test_batched_arrivals_count_and_loss_is_caught(void) {
    harness_t h;
    setup(&h);
    h.batch = 3; /* three at a time */
    run(&h, 40000);
    assert(h.last.state == CORE_LINK_NOMINAL && "a batched link that loses nothing is healthy");
    h.batch = 1;
    h.drop = 0.67f;
    reset_stats(&h);
    run(&h, 60000);
    assert(h.min_trust < 0.4f && "losing two thirds of them must be caught");
}

/* --- Dropouts and loss --- */

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
        /* Random loss sometimes drops 6 in a row, a real 6 s silence, so LOST
         * may come and go; what must not happen is looking healthy again. */
        assert(h.back_to_nominal == 0 && "never bounces back to NOMINAL under steady loss");
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

/* --- Heartbeat rate --- */

static void test_configured_slow_gcs(void) {
    harness_t h;
    setup_with(&h, 5000);
    run(&h, 180000);
    assert(h.worst == CORE_LINK_NOMINAL && "a GCS configured at 0.2 Hz is healthy at 0.2 Hz");
    h.gcs_on = false;
    run(&h, 36000); /* 6 intervals, plus up to one since the last heartbeat */
    assert(h.last.state == CORE_LINK_LOST);
}

static void test_unconfigured_slow_gcs_reads_as_loss(void) {
    /* Documented: a GCS slower than configured looks exactly like loss. */
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 2000;
    run(&h, 60000);
    assert(h.last.state == CORE_LINK_DEGRADED);
}

static void test_faster_gcs_than_configured(void) {
    /* A 10 Hz GCS with the default 1 Hz configured: healthy, and a total loss
     * is still caught by silence. */
    harness_t h;
    setup(&h);
    h.gcs_interval_ms = 100;
    run(&h, 60000);
    assert(h.worst == CORE_LINK_NOMINAL);
    h.gcs_on = false;
    run(&h, 7000);
    assert(h.last.state == CORE_LINK_LOST);
}

/* --- Radio --- */

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

static void test_returning_signal_isnt_jamming(void) {
    /* Found in SITL: when the operator came back, RSSI rose about a second
     * before the next heartbeat, which briefly looked like strong RF and no
     * packets, and was announced as jamming. */
    harness_t h;
    setup(&h);
    h.rf = 1;
    settle(&h);
    h.gcs_on = false;
    h.rssi = 10;
    run(&h, 10000);
    assert(h.last.state == CORE_LINK_LOST);
    reset_stats(&h);
    h.rssi = 180;   /* the radio hears the operator again... */
    run(&h, 1200);
    h.gcs_on = true; /* ...and the heartbeats follow */
    run(&h, 45000);
    assert(h.worst == CORE_LINK_LOST && "never JAMMED on the way back");
    assert(h.last.state == CORE_LINK_NOMINAL);
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

static void test_noise_calibrates_despite_weak_signal(void) {
    /* Third review: calibration waited for NOMINAL, which a weak signal
     * prevents, so jamming then read as a clean loss. */
    harness_t h;
    setup(&h);
    h.rf = 2;
    h.rssi = 40; /* weak: the link is DEGRADED from the start */
    run(&h, 30000);
    assert(h.mon.noise_floor_known);
    h.noise = 140;
    h.gcs_on = false;
    run(&h, 8000);
    assert(h.last.state == CORE_LINK_JAMMED);
}

static void test_bad_noise_start_recalibrates(void) {
    /* A radio reporting 0 for its first 15 s must not leave every later normal
     * reading looking like interference. */
    harness_t h;
    setup(&h);
    h.rf = 2;
    h.noise = 0;
    run(&h, 15000);
    h.noise = 40;
    run(&h, 120000);
    assert(!h.last.interference);
    assert(h.last.state == CORE_LINK_NOMINAL);
    h.gcs_on = false;
    h.rssi = 10;
    run(&h, 8000);
    assert(h.last.state == CORE_LINK_LOST && "a clean loss, not jamming");
}

static void test_jammer_samples_dont_enter_calibration(void) {
    /* Fourth review: with loss not yet measurable, early silence counted as
     * healthy, so samples from a starting jammer got into the median. */
    harness_t h;
    setup(&h);
    h.rf = 2;
    run(&h, 2400);   /* five normal samples, too early for loss to be measured */
    h.gcs_on = false;
    h.noise = 140;   /* jamming starts */
    int before = h.mon.noise_cal_len;
    run(&h, 20000);
    assert(h.mon.noise_cal_len <= before + 1 && "samples taken in silence don't calibrate");
    assert(!h.mon.noise_floor_known || h.mon.noise_floor < 60);
}

static void test_noise_with_loss_isnt_baselined(void) {
    /* Fourth review: raised noise costing 10% of heartbeats was re-baselined,
     * hiding a later jam. Only noise that costs (almost) nothing is. */
    harness_t h;
    setup(&h);
    h.rf = 2;
    settle(&h);
    h.noise = 140;
    h.drop_every = 10; /* exactly 10% */
    run(&h, 75000);
    assert(h.mon.noise_floor < 60);
    h.gcs_on = false;
    run(&h, 9000);
    assert(h.last.state == CORE_LINK_JAMMED);
}

static void test_unconfirmed_interference_is_said_so(void) {
    harness_t h;
    setup(&h);
    h.rf = 1;
    settle(&h);
    h.gcs_on = false;
    h.rssi = 10;
    run(&h, 6500);    /* LOST, with nothing on the air... */
    h.rssi = 180;     /* ...then RF appears, not yet confirmed */
    run(&h, 600);
    assert(h.last.state == CORE_LINK_LOST);
    assert(strstr(h.last.reason, "unconfirmed") && "pending interference is not 'no interference'");
    run(&h, 2500);
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

/* --- The autopilot --- */

static void test_heartbeat_in_first_poll_counts(void) {
    /* If the only operator heartbeat ever seen arrives in the very first poll
     * and is thrown away, the link stays UNKNOWN forever and is never reported. */
    harness_t h;
    setup(&h);
    h.t.heartbeat.valid = true;
    h.t.heartbeat.last_update_ms = h.now;
    deliver(&h, 1);
    h.gcs_on = false;
    run(&h, 20000);
    assert(h.last.state == CORE_LINK_LOST);
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

int main(void) {
    test_unknown_until_operator_heard();
    test_steady_link_stays_nominal();
    test_jitter_stays_nominal();
    test_bursty_lossless_delivery_stays_nominal();
    test_batched_arrivals_count_and_loss_is_caught();
    test_brief_dropouts_dont_flap();
    test_light_random_loss_tolerated();
    test_steady_moderate_loss_doesnt_flap();
    test_heavy_loss_is_degraded_and_untrusted();
    test_silence_without_radio_is_lost();
    test_recovery_is_gradual_and_doesnt_flap();
    test_configured_slow_gcs();
    test_unconfigured_slow_gcs_reads_as_loss();
    test_faster_gcs_than_configured();
    test_rssi_separates_jamming_from_loss();
    test_returning_signal_isnt_jamming();
    test_noise_floor_separates_jamming();
    test_gradual_noise_rise_is_interference();
    test_noise_calibrates_despite_weak_signal();
    test_bad_noise_start_recalibrates();
    test_jammer_samples_dont_enter_calibration();
    test_noise_with_loss_isnt_baselined();
    test_unconfirmed_interference_is_said_so();
    test_zero_rssi_is_no_radio_data();
    test_heartbeat_in_first_poll_counts();
    test_autopilot_silence_is_unknown_not_lost();
    test_autopilot_outage_keeps_distrust();
    printf("test_link_monitor: all passed\n");
    return 0;
}
