#include "core/link/link_monitor.h"

#include <stdio.h>
#include <string.h>

#define RSSI_UNKNOWN 255
#define RSSI_MAX 254.0f
#define NOISE_FLOOR_ALPHA 0.05f /* per RADIO_STATUS sample while the link is healthy */
#define WARMUP_GAPS 5           /* gaps learned before the interval needs a healthy link */
/* After warm-up, only gaps near the current interval are learned. A dropped
 * heartbeat leaves a gap of at least twice it; clipping both sides equally
 * keeps jitter from biasing the mean. */
#define LEARN_GAP_LOW 0.5f
#define LEARN_GAP_HIGH 1.5f
#define MIN_LOSS_INTERVALS 10     /* the window always spans this many heartbeats, so +-1 from jitter stays under loss_ok */
#define WARMUP_MAX_GAP_MS 5000  /* until then, accept any gap up to this, so a slow GCS can be learned */

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
    cfg->autopilot_stale_ms = 3000;
}

static void reset_history(core_link_monitor_t *mon) {
    mon->first_heard_ms = 0;
    mon->last_heard_ms = 0;
    mon->arrivals_head = 0;
    mon->arrivals_len = 0;
    mon->trust = 1.0f;
    mon->state = CORE_LINK_UNKNOWN;
    mon->easing_since_ms = 0;
}

void core_link_monitor_init(core_link_monitor_t *mon, const core_link_config_t *cfg) {
    memset(mon, 0, sizeof(*mon));
    mon->cfg = *cfg;
    mon->interval_ms = cfg->default_interval_ms;
    mon->blind = true;
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
    bool warming_up = mon->gaps_len < WARMUP_GAPS;
    if (gap == 0) {
        return;
    }
    if (warming_up ? gap > WARMUP_MAX_GAP_MS
                   : (gap < LEARN_GAP_LOW * mon->interval_ms || gap > LEARN_GAP_HIGH * mon->interval_ms)) {
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
        /* The mean, not the median: what matters is the heartbeat rate, and with
         * jitter a median of a few gaps can be well off it. Outages never get here. */
        uint64_t sum = 0;
        for (int i = 0; i < mon->gaps_len; i++) {
            sum += mon->gaps[i];
        }
        uint32_t mean = (uint32_t)(sum / (uint64_t)mon->gaps_len);
        mon->interval_ms = mean < 100 ? 100 : (mean > 5000 ? 5000 : mean);
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

    /* Several in one tick share the tick's timestamp. */
    uint64_t at = hb->last_update_ms;
    if (mon->last_heard_ms != 0 && at > mon->last_heard_ms) {
        learn_gap(mon, (uint32_t)(at - mon->last_heard_ms));
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
     * reads as a large loss, so wait for enough of them. */
    if (span < MIN_LOSS_INTERVALS * (uint64_t)mon->interval_ms) {
        return 0.0f;
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
        if (healthy) {
            if (!mon->noise_floor_known) {
                mon->noise_floor = (float)radio->noise;
                mon->noise_floor_known = true;
            } else {
                mon->noise_floor += NOISE_FLOOR_ALPHA * ((float)radio->noise - mon->noise_floor);
            }
        }
        if (mon->noise_floor_known) {
            rf.noise_excess = (int)((float)radio->noise - mon->noise_floor);
            rf.noise_high = rf.noise_excess >= (int)mon->cfg.jam_noise_margin;
        }
        return rf;
    }
    if (rc->valid && rc->rssi != RSSI_UNKNOWN && now_ms - rc->last_update_ms <= mon->cfg.rf_stale_ms) {
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
    if (mon->blind) {
        mon->blind = false;
        mon->seen_count = telemetry->gcs_heartbeat.count; /* only count what arrives from now */
    }

    record_arrivals(mon, &telemetry->gcs_heartbeat);
    if (mon->last_heard_ms == 0) {
        mon->last_update_ms = now_ms;
        r.state = CORE_LINK_UNKNOWN;
        snprintf(r.reason, sizeof(r.reason), "operator not heard yet");
        return r;
    }

    r.silence_ms = (uint32_t)(now_ms - mon->last_heard_ms);
    r.loss = measure_loss(mon, now_ms, cfg->window_ms);
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
               (mon->state != CORE_LINK_NOMINAL && (mon->trust < cfg->nominal_at || long_loss > cfg->loss_ok))) {
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
