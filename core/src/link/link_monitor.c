#include "core/link/link_monitor.h"

#include <stdio.h>
#include <string.h>

#define RSSI_UNKNOWN 255
#define RSSI_MAX 254.0f
/* Per new RADIO_STATUS sample while the link is healthy. At SiK's 1-2 Hz that's
 * a time constant of about a minute, so a jammer ramping up over tens of
 * seconds is seen as interference before the floor can follow it. */
#define NOISE_FLOOR_ALPHA 0.01f
/* After warm-up, gaps over this many intervals are outages and not learned. The
 * median takes care of the odd dropped heartbeat. */
#define LEARN_GAP_MAX 3
#define MIN_LOSS_INTERVALS 10     /* the window always spans this many heartbeats, so +-1 from jitter stays under loss_ok */
#define WARMUP_MAX_GAP_MS 5000  /* until then, accept any gap up to this, so a slow GCS can be learned */
/* Learned intervals are kept to this range. MAVLink GCSes send 1 Hz; a learned
 * interval above 2 s is more likely loss during warm-up than a real rate, and
 * the cap bounds how slowly a loss can be detected (6 intervals). */
#define MIN_INTERVAL_MS 100
#define MAX_INTERVAL_MS 2000
#define LOSS_MIN_GAPS 16 /* loss isn't measured until the interval rests on this many gaps */
#define NOISE_CAL_SAMPLES 10    /* the noise floor starts as the median of this many samples */

void core_link_config_default(core_link_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->window_ms = 10000;
    cfg->default_interval_ms = 1000; /* MAVLink's usual GCS heartbeat rate */
    cfg->silence_grace_ms = 3000;
    cfg->lost_after_ms = 6000;
    cfg->loss_ok = 0.15f;
    cfg->loss_bad = 0.6f;
    cfg->rssi_ok = 0.35f;
    cfg->rssi_bad = 0.10f;
    cfg->rssi_floor_score = 0.5f;
    cfg->jam_rssi_min = 0.30f;
    cfg->jam_noise_margin = 20; /* SiK units are about 0.5 dB, so about 10 dB */
    cfg->interference_cap = 0.6f;
    cfg->rf_stale_ms = 3000;
    cfg->degraded_below = 0.7f; /* autonomy's trusted_at */
    cfg->nominal_at = 0.85f;
    cfg->nominal_window_ms = 30000;
    cfg->recover_per_s = 0.1f;
    cfg->state_hold_ms = 3000;
    cfg->autopilot_late_ms = 2000;
    cfg->autopilot_stale_ms = 3000;
}

/* After the autopilot has been silent: what was heard before no longer
 * describes the link, and the GCS may be a different one, so its rate is
 * learned again. Trust is kept, so a link that was lost recovers gradually
 * rather than being declared healthy by the gap. */
static void reset_history(core_link_monitor_t *mon) {
    mon->first_heard_ms = 0;
    mon->last_heard_ms = 0;
    mon->arrivals_head = 0;
    mon->arrivals_len = 0;
    mon->gaps_head = 0;
    mon->gaps_len = 0;
    mon->interval_ms = mon->cfg.default_interval_ms;
    mon->noise_floor_known = false; /* the radio may have changed too */
    mon->noise_cal_len = 0;
    mon->state = CORE_LINK_UNKNOWN;
    mon->easing_since_ms = 0;
}

void core_link_monitor_init(core_link_monitor_t *mon, const core_link_config_t *cfg) {
    memset(mon, 0, sizeof(*mon));
    mon->cfg = *cfg;
    mon->blind = true;
    mon->trust = 1.0f;
    reset_history(mon);
}

const char *core_link_state_name(core_link_state_t state) {
    switch (state) {
    case CORE_LINK_UNKNOWN: return "UNKNOWN";
    case CORE_LINK_NOMINAL: return "NOMINAL";
    case CORE_LINK_DEGRADED: return "DEGRADED";
    case CORE_LINK_LOST: return "LOST";
    case CORE_LINK_JAMMED: return "JAMMED";
    }
    return "UNKNOWN";
}

/* Linear from 1 at good to floor at bad; good and bad may be in either order. */
static float ramp(float value, float good, float bad, float floor) {
    float t;
    if (good > bad) {
        if (value >= good) return 1.0f;
        if (value <= bad) return floor;
        t = (good - value) / (good - bad);
    } else {
        if (value <= good) return 1.0f;
        if (value >= bad) return floor;
        t = (value - good) / (bad - good);
    }
    return 1.0f - t * (1.0f - floor);
}

static void learn_gap(core_link_monitor_t *mon, uint32_t gap) {
    /* Warm-up lasts until loss can be measured, and learns in any state, so a
     * link that falters early can't stop its own interval being learned. */
    bool warming_up = mon->gaps_len < LOSS_MIN_GAPS;
    if (gap == 0) {
        return;
    }
    if (gap > (warming_up ? WARMUP_MAX_GAP_MS : LEARN_GAP_MAX * mon->interval_ms)) {
        return;
    }
    if (!warming_up && mon->state != CORE_LINK_NOMINAL) {
        return; /* gaps from a lossy link would teach it that loss is normal */
    }
    mon->gaps[mon->gaps_head] = gap;
    mon->gaps_head = (mon->gaps_head + 1) % CORE_LINK_GAPS;
    if (mon->gaps_len < CORE_LINK_GAPS) {
        mon->gaps_len++;
    }
    if (mon->gaps_len >= 3) {
        /* The median of many gaps: unbiased under symmetric jitter, unmoved by an
         * occasional dropped heartbeat's double gap, and needing no clipping band
         * (a band that follows the estimate feeds back into it). */
        uint32_t sorted[CORE_LINK_GAPS];
        int n = mon->gaps_len;
        memcpy(sorted, mon->gaps, sizeof(sorted[0]) * (size_t)n);
        for (int i = 1; i < n; i++) {
            for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
                uint32_t tmp = sorted[j];
                sorted[j] = sorted[j - 1];
                sorted[j - 1] = tmp;
            }
        }
        uint32_t median = n % 2 ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2;
        mon->interval_ms = median < MIN_INTERVAL_MS ? MIN_INTERVAL_MS : (median > MAX_INTERVAL_MS ? MAX_INTERVAL_MS : median);
    }
}

static void record_arrivals(core_link_monitor_t *mon, const core_gcs_heartbeat_t *hb) {
    if (!hb->valid) {
        return;
    }
    uint32_t fresh = hb->count - mon->seen_count;
    mon->seen_count = hb->count;
    if (fresh == 0) {
        return;
    }
    if (fresh > CORE_LINK_ARRIVALS) {
        fresh = CORE_LINK_ARRIVALS;
    }

    /* Several in one tick share the tick's timestamp. Learn the average spacing
     * of what arrived, not the time between batches. */
    uint64_t at = hb->last_update_ms;
    if (mon->last_heard_ms != 0 && at > mon->last_heard_ms) {
        learn_gap(mon, (uint32_t)((at - mon->last_heard_ms) / fresh));
    }
    for (uint32_t i = 0; i < fresh; i++) {
        mon->arrivals[mon->arrivals_head] = at;
        mon->arrivals_head = (mon->arrivals_head + 1) % CORE_LINK_ARRIVALS;
        if (mon->arrivals_len < CORE_LINK_ARRIVALS) {
            mon->arrivals_len++;
        }
    }
    if (mon->first_heard_ms == 0) {
        mon->first_heard_ms = at;
    }
    mon->last_heard_ms = at;
}

/* Fraction of expected heartbeats missing over the window, or -1 while it can't
 * be measured yet. */
static float measure_loss(const core_link_monitor_t *mon, uint64_t now_ms, uint64_t window) {
    if (window < MIN_LOSS_INTERVALS * (uint64_t)mon->interval_ms) {
        window = MIN_LOSS_INTERVALS * (uint64_t)mon->interval_ms; /* a slow GCS gets a longer window */
    }
    uint64_t span = now_ms - mon->first_heard_ms;
    if (span > window) {
        span = window;
    }
    /* A GCS fast enough to overflow the buffer gets a shorter window instead. */
    uint64_t fits = (uint64_t)(CORE_LINK_ARRIVALS - 1) * mon->interval_ms;
    if (span > fits) {
        span = fits;
    }
    /* Over only a few heartbeats, one landing just outside the window by jitter
     * reads as a large loss, and an interval learned from a few gaps can be well
     * off. Until both have enough behind them, only silence counts, which still
     * catches a total loss. */
    if (span < MIN_LOSS_INTERVALS * (uint64_t)mon->interval_ms || mon->gaps_len < LOSS_MIN_GAPS) {
        return -1.0f;
    }
    int received = 0;
    for (int i = 0; i < mon->arrivals_len; i++) {
        if (now_ms - mon->arrivals[i] < span) {
            received++;
        }
    }
    /* Rounded down: the window catches a whole number of heartbeats, and the
     * fraction left over is phase, not loss. */
    float expected = (float)(span / mon->interval_ms);
    float loss = 1.0f - (float)received / expected;
    return loss < 0.0f ? 0.0f : (loss > 1.0f ? 1.0f : loss);
}

typedef struct {
    core_link_rf_source_t source;
    float rssi;        /* normalized, valid unless source is NONE */
    bool noise_high;   /* RADIO_STATUS noise above the learned floor */
    int noise_excess;
} rf_view_t;

static rf_view_t read_rf(core_link_monitor_t *mon, const core_telemetry_t *t, uint64_t now_ms, bool healthy) {
    rf_view_t rf = {0};
    const core_radio_status_t *radio = &t->radio_status;
    const core_rc_status_t *rc = &t->rc_status;

    if (radio->valid && now_ms - radio->last_update_ms <= mon->cfg.rf_stale_ms) {
        rf.source = CORE_LINK_RF_NOISE;
        rf.rssi = radio->rssi == RSSI_UNKNOWN ? 0.0f : (float)radio->rssi / RSSI_MAX;
        /* Judged against the floor before this sample, which is then learned
         * once, and only from a healthy link that shows no interference. */
        if (mon->noise_floor_known) {
            rf.noise_excess = (int)((float)radio->noise - mon->noise_floor);
            rf.noise_high = rf.noise_excess >= (int)mon->cfg.jam_noise_margin;
        }
        bool new_sample = radio->last_update_ms != mon->last_radio_ms;
        mon->last_radio_ms = radio->last_update_ms;
        if (!new_sample || !healthy) {
            return rf;
        }
        if (!mon->noise_floor_known) {
            /* Calibrate from the median of several samples, so one odd reading
             * can't set a baseline that every later sample is judged against. */
            mon->noise_cal[mon->noise_cal_len++] = radio->noise;
            if (mon->noise_cal_len == NOISE_CAL_SAMPLES) {
                uint16_t sorted[NOISE_CAL_SAMPLES];
                memcpy(sorted, mon->noise_cal, sizeof(sorted));
                for (int i = 1; i < NOISE_CAL_SAMPLES; i++) { /* insertion sort, 10 items */
                    for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
                        uint16_t tmp = sorted[j];
                        sorted[j] = sorted[j - 1];
                        sorted[j - 1] = tmp;
                    }
                }
                mon->noise_floor = (float)sorted[NOISE_CAL_SAMPLES / 2];
                mon->noise_floor_known = true;
            }
        } else if (!rf.noise_high) {
            mon->noise_floor += NOISE_FLOOR_ALPHA * ((float)radio->noise - mon->noise_floor);
        }
        return rf;
    }
    /* 0 is what ArduPilot sends when RSSI_TYPE=5 but no radio has reported for
     * 5 s: no radio, rather than a weak signal. */
    if (rc->valid && rc->rssi != RSSI_UNKNOWN && rc->rssi != 0 && now_ms - rc->last_update_ms <= mon->cfg.rf_stale_ms) {
        rf.source = CORE_LINK_RF_RSSI;
        rf.rssi = (float)rc->rssi / RSSI_MAX;
    }
    return rf;
}

static int severity(core_link_state_t state) {
    switch (state) {
    case CORE_LINK_NOMINAL: return 0;
    case CORE_LINK_DEGRADED: return 1;
    case CORE_LINK_LOST: return 2;
    case CORE_LINK_JAMMED: return 3;
    case CORE_LINK_UNKNOWN: break;
    }
    return -1;
}

core_link_result_t core_link_monitor_update(core_link_monitor_t *mon, const core_telemetry_t *telemetry, uint64_t now_ms) {
    const core_link_config_t *cfg = &mon->cfg;
    core_link_result_t r;
    memset(&r, 0, sizeof(r));
    core_link_state_t previous = mon->state;

    /* Without the autopilot nothing it forwards arrives, so silence from the
     * operator would mean nothing. Start over once it's back. */
    bool autopilot_heard = telemetry->heartbeat.valid &&
                           now_ms - telemetry->heartbeat.last_update_ms <= cfg->autopilot_stale_ms;
    if (!autopilot_heard) {
        if (!mon->blind) {
            reset_history(mon);
            mon->blind = true;
        }
        mon->seen_count = telemetry->gcs_heartbeat.count;
        mon->last_update_ms = now_ms;
        r.state = CORE_LINK_UNKNOWN;
        r.state_changed = previous != CORE_LINK_UNKNOWN;
        snprintf(r.reason, sizeof(r.reason), "autopilot silent");
        return r;
    }
    /* While blind, seen_count followed the count, so whatever is new now
     * arrived with the autopilot's return and counts. */
    mon->blind = false;

    /* When the autopilot goes quiet, the operator's forwarded heartbeats stop
     * with it. Until it's late enough to call UNKNOWN, hold the last verdict
     * rather than blame the operator link. */
    if (now_ms - telemetry->heartbeat.last_update_ms > cfg->autopilot_late_ms && mon->state != CORE_LINK_UNKNOWN) {
        mon->last_update_ms = now_ms;
        r.state = mon->state;
        r.link_trust = mon->trust;
        r.silence_ms = (uint32_t)(now_ms - mon->last_heard_ms);
        snprintf(r.reason, sizeof(r.reason), "autopilot late, holding");
        return r;
    }

    record_arrivals(mon, &telemetry->gcs_heartbeat);
    if (mon->last_heard_ms == 0) {
        mon->last_update_ms = now_ms;
        r.state = CORE_LINK_UNKNOWN;
        snprintf(r.reason, sizeof(r.reason), "operator not heard yet");
        return r;
    }

    r.silence_ms = (uint32_t)(now_ms - mon->last_heard_ms);
    float loss = measure_loss(mon, now_ms, cfg->window_ms);
    r.loss = loss < 0.0f ? 0.0f : loss;
    /* Ten heartbeats can't tell steady 30% loss from a clean link, so calling it
     * NOMINAL again takes a longer clean record. */
    float long_loss = measure_loss(mon, now_ms, cfg->nominal_window_ms);
    /* Scaled up for a GCS that legitimately sends slowly. */
    uint32_t grace_ms = cfg->silence_grace_ms > 3 * mon->interval_ms ? cfg->silence_grace_ms : 3 * mon->interval_ms;
    uint32_t lost_ms = cfg->lost_after_ms > 6 * mon->interval_ms ? cfg->lost_after_ms : 6 * mon->interval_ms;
    bool silent_long = r.silence_ms >= lost_ms;
    bool silent_short = r.silence_ms > grace_ms;

    rf_view_t rf = read_rf(mon, telemetry, now_ms, mon->state == CORE_LINK_NOMINAL && !silent_short);
    r.rf_source = rf.source;
    /* With RSSI only, strong RF energy is suspicious just when nothing gets through. */
    r.interference = rf.noise_high || (rf.source == CORE_LINK_RF_RSSI && silent_short && rf.rssi >= cfg->jam_rssi_min);

    float loss_score = ramp(r.loss, cfg->loss_ok, cfg->loss_bad, 0.0f);
    float silence_score = ramp((float)r.silence_ms, (float)grace_ms, (float)lost_ms, 0.0f);
    float rssi_score = rf.source == CORE_LINK_RF_NONE ? 1.0f : ramp(rf.rssi, cfg->rssi_ok, cfg->rssi_bad, cfg->rssi_floor_score);
    float raw = loss_score;
    if (silence_score < raw) raw = silence_score;
    if (rssi_score < raw) raw = rssi_score;
    if (r.interference && raw > cfg->interference_cap) raw = cfg->interference_cap;

    /* Drop at once, recover at a bounded rate. */
    float dt = mon->last_update_ms ? (float)(now_ms - mon->last_update_ms) / 1000.0f : 0.0f;
    mon->last_update_ms = now_ms;
    if (raw < mon->trust) {
        mon->trust = raw;
    } else {
        float limit = mon->trust + cfg->recover_per_s * dt;
        mon->trust = raw < limit ? raw : limit;
    }

    core_link_state_t indicated = CORE_LINK_NOMINAL;
    if (silent_long) {
        indicated = r.interference ? CORE_LINK_JAMMED : CORE_LINK_LOST;
    } else if (mon->trust < cfg->degraded_below ||
               (mon->state != CORE_LINK_NOMINAL && mon->state != CORE_LINK_UNKNOWN &&
                (mon->trust < cfg->nominal_at || long_loss < 0.0f || long_loss > cfg->loss_ok))) {
        indicated = CORE_LINK_DEGRADED;
    }

    if (mon->state == CORE_LINK_UNKNOWN || severity(indicated) >= severity(mon->state)) {
        mon->state = indicated;
        mon->easing_since_ms = 0;
    } else if (mon->easing_since_ms == 0) {
        mon->easing_since_ms = now_ms;
    } else if (now_ms - mon->easing_since_ms >= cfg->state_hold_ms) {
        mon->state = indicated;
        mon->easing_since_ms = 0;
    }

    r.state = mon->state;
    r.link_trust = mon->trust;
    r.state_changed = r.state != previous;

    unsigned silent_s = r.silence_ms / 1000;
    switch (r.state) {
    case CORE_LINK_JAMMED:
        if (rf.noise_high) {
            snprintf(r.reason, sizeof(r.reason), "silent %us, noise +%d", silent_s, rf.noise_excess);
        } else {
            snprintf(r.reason, sizeof(r.reason), "silent %us, RF energy present", silent_s);
        }
        break;
    case CORE_LINK_LOST:
        snprintf(r.reason, sizeof(r.reason), "silent %us, %s", silent_s,
                 rf.source == CORE_LINK_RF_NONE ? "no radio data" : "no interference");
        break;
    case CORE_LINK_DEGRADED:
        if (r.interference) {
            snprintf(r.reason, sizeof(r.reason), "interference, loss %d%%", (int)(r.loss * 100.0f + 0.5f));
        } else if (rssi_score < loss_score && rssi_score < silence_score) {
            snprintf(r.reason, sizeof(r.reason), "weak signal");
        } else if (silent_short) {
            snprintf(r.reason, sizeof(r.reason), "silent %us", silent_s);
        } else {
            snprintf(r.reason, sizeof(r.reason), "loss %d%%", (int)(r.loss * 100.0f + 0.5f));
        }
        break;
    case CORE_LINK_NOMINAL:
        snprintf(r.reason, sizeof(r.reason), "heartbeats regular");
        break;
    case CORE_LINK_UNKNOWN:
        break;
    }
    return r;
}
