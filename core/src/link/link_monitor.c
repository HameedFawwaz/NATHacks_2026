#include "core/link/link_monitor.h"

#include <stdio.h>
#include <string.h>

#define RSSI_UNKNOWN 255
#define RSSI_MAX 254.0f
/* Per new RADIO_STATUS sample while the link is healthy. At SiK's 1-2 Hz that's
 * a time constant of about a minute, so a jammer ramping up over tens of
 * seconds is seen as interference before the floor can follow it. */
#define NOISE_FLOOR_ALPHA 0.01f
#define NOISE_CAL_SAMPLES 10 /* the floor starts as the median of this many samples */
/* "Interference" that costs not one heartbeat for this long is the new normal,
 * or a floor calibrated wrongly: calibrate again. */
#define NOISE_RECAL_MS 60000
/* The window always spans this many heartbeats, so being off by one from jitter
 * stays under loss_ok. */
#define MIN_LOSS_INTERVALS 10

void core_link_config_default(core_link_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->heartbeat_interval_ms = 1000; /* what MAVProxy, QGroundControl and Mission Planner send */
    cfg->window_ms = 10000;
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
    cfg->jam_confirm_ms = 2000;
    cfg->degraded_below = 0.7f; /* autonomy's trusted_at */
    cfg->nominal_at = 0.85f;
    cfg->nominal_window_ms = 30000;
    cfg->recover_per_s = 0.1f;
    cfg->state_hold_ms = 3000;
    cfg->autopilot_late_ms = 2000;
    cfg->autopilot_stale_ms = 3000;
}

/* After the autopilot has been silent, what was heard before no longer
 * describes the link, and the radio may be a different one. Trust is kept, so a
 * link that was lost recovers gradually rather than being declared healthy by
 * the gap. */
static void reset_history(core_link_monitor_t *mon) {
    mon->first_heard_ms = 0;
    mon->last_heard_ms = 0;
    mon->arrivals_head = 0;
    mon->arrivals_len = 0;
    mon->noise_floor_known = false;
    mon->noise_cal_len = 0;
    mon->noisy_but_healthy_since_ms = 0;
    mon->interference_since_ms = 0;
    mon->state = CORE_LINK_UNKNOWN;
    mon->easing_since_ms = 0;
}

void core_link_monitor_init(core_link_monitor_t *mon, const core_link_config_t *cfg) {
    memset(mon, 0, sizeof(*mon));
    mon->cfg = *cfg;
    if (mon->cfg.heartbeat_interval_ms == 0) {
        mon->cfg.heartbeat_interval_ms = 1000;
    }
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
    for (uint32_t i = 0; i < fresh; i++) {
        mon->arrivals[mon->arrivals_head] = hb->last_update_ms;
        mon->arrivals_head = (mon->arrivals_head + 1) % CORE_LINK_ARRIVALS;
        if (mon->arrivals_len < CORE_LINK_ARRIVALS) {
            mon->arrivals_len++;
        }
    }
    if (mon->first_heard_ms == 0) {
        mon->first_heard_ms = hb->last_update_ms;
    }
    mon->last_heard_ms = hb->last_update_ms;
}

/* Fraction of expected heartbeats missing over the window, or -1 while there
 * hasn't been enough of the link yet to say. */
static float measure_loss(const core_link_monitor_t *mon, uint64_t now_ms, uint64_t window) {
    uint64_t interval = mon->cfg.heartbeat_interval_ms;
    if (window < MIN_LOSS_INTERVALS * interval) {
        window = MIN_LOSS_INTERVALS * interval;
    }
    /* A GCS fast enough to overflow the buffer gets a shorter window instead. */
    uint64_t fits = (uint64_t)(CORE_LINK_ARRIVALS - 1) * interval;
    if (window > fits) {
        window = fits;
    }
    uint64_t span = now_ms - mon->first_heard_ms;
    if (span < MIN_LOSS_INTERVALS * interval) {
        return -1.0f;
    }
    if (span > window) {
        span = window;
    }
    int received = 0;
    for (int i = 0; i < mon->arrivals_len; i++) {
        if (now_ms - mon->arrivals[i] < span) {
            received++;
        }
    }
    /* Rounded down: the window catches a whole number of heartbeats, and the
     * fraction left over is phase, not loss. */
    float expected = (float)(span / interval);
    float loss = 1.0f - (float)received / expected;
    return loss < 0.0f ? 0.0f : (loss > 1.0f ? 1.0f : loss);
}

typedef struct {
    core_link_rf_source_t source;
    float rssi;        /* normalized, valid unless source is NONE */
    bool noise_known;  /* RADIO_STATUS with a calibrated floor */
    bool noise_high;   /* noise above that floor */
    int noise_excess;
} rf_view_t;

static void calibrate_noise(core_link_monitor_t *mon, uint16_t noise) {
    /* The median of several samples, so one odd reading can't set the baseline
     * that every later sample is judged against. */
    mon->noise_cal[mon->noise_cal_len++] = noise;
    if (mon->noise_cal_len < NOISE_CAL_SAMPLES) {
        return;
    }
    uint16_t sorted[NOISE_CAL_SAMPLES];
    memcpy(sorted, mon->noise_cal, sizeof(sorted));
    for (int i = 1; i < NOISE_CAL_SAMPLES; i++) {
        for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            uint16_t tmp = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = tmp;
        }
    }
    mon->noise_floor = (float)sorted[NOISE_CAL_SAMPLES / 2];
    mon->noise_floor_known = true;
    mon->noise_cal_len = 0;
}

/* heard_well: heartbeats are arriving with no more than loss_ok missing. That,
 * not the overall state, gates learning, so a weak signal can't stop the noise
 * floor being calibrated. */
static rf_view_t read_rf(core_link_monitor_t *mon, const core_telemetry_t *t, uint64_t now_ms, bool heard_well) {
    rf_view_t rf = {0};
    const core_radio_status_t *radio = &t->radio_status;
    const core_rc_status_t *rc = &t->rc_status;

    if (radio->valid && now_ms - radio->last_update_ms <= mon->cfg.rf_stale_ms) {
        rf.source = CORE_LINK_RF_NOISE;
        rf.rssi = radio->rssi == RSSI_UNKNOWN ? 0.0f : (float)radio->rssi / RSSI_MAX;
        /* Judged against the floor before this sample, which is then learned. */
        if (mon->noise_floor_known) {
            rf.noise_known = true;
            rf.noise_excess = (int)((float)radio->noise - mon->noise_floor);
            rf.noise_high = rf.noise_excess >= (int)mon->cfg.jam_noise_margin;
        }

        bool new_sample = radio->last_update_ms != mon->last_radio_ms;
        mon->last_radio_ms = radio->last_update_ms;
        if (!(rf.noise_high && heard_well)) {
            mon->noisy_but_healthy_since_ms = 0;
        } else if (mon->noisy_but_healthy_since_ms == 0) {
            mon->noisy_but_healthy_since_ms = now_ms;
        } else if (now_ms - mon->noisy_but_healthy_since_ms >= NOISE_RECAL_MS) {
            mon->noise_floor_known = false;
            mon->noise_cal_len = 0;
            mon->noisy_but_healthy_since_ms = 0;
        }
        if (new_sample && heard_well) {
            if (!mon->noise_floor_known) {
                calibrate_noise(mon, radio->noise);
            } else if (!rf.noise_high) {
                mon->noise_floor += NOISE_FLOOR_ALPHA * ((float)radio->noise - mon->noise_floor);
            }
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

    uint32_t interval = cfg->heartbeat_interval_ms;
    r.silence_ms = (uint32_t)(now_ms - mon->last_heard_ms);
    float loss = measure_loss(mon, now_ms, cfg->window_ms);
    r.loss = loss < 0.0f ? 0.0f : loss;
    /* Ten heartbeats can't tell steady 30% loss from a clean link, so calling it
     * NOMINAL again takes a longer clean record. */
    float long_loss = measure_loss(mon, now_ms, cfg->nominal_window_ms);
    /* Scaled up for a GCS configured to send slowly. */
    uint32_t grace_ms = cfg->silence_grace_ms > 3 * interval ? cfg->silence_grace_ms : 3 * interval;
    uint32_t lost_ms = cfg->lost_after_ms > 6 * interval ? cfg->lost_after_ms : 6 * interval;
    bool silent_long = r.silence_ms >= lost_ms;
    bool silent_short = r.silence_ms > grace_ms;

    bool heard_well = !silent_short && r.loss <= cfg->loss_ok;
    rf_view_t rf = read_rf(mon, telemetry, now_ms, heard_well);
    r.rf_source = rf.source;
    /* Without a calibrated noise floor, fall back to RSSI: strong RF energy is
     * suspicious just when nothing gets through. */
    bool rssi_rule = rf.source == CORE_LINK_RF_RSSI || (rf.source == CORE_LINK_RF_NOISE && !rf.noise_known);
    r.interference = rf.noise_high || (rssi_rule && silent_short && rf.rssi >= cfg->jam_rssi_min);
    /* Calling it jamming takes interference that lasts: a returning signal shows
     * strong RF a moment before its next heartbeat. */
    if (!r.interference) {
        mon->interference_since_ms = 0;
    } else if (mon->interference_since_ms == 0) {
        mon->interference_since_ms = now_ms;
    }
    bool jam_confirmed = r.interference && now_ms - mon->interference_since_ms >= cfg->jam_confirm_ms;

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
        indicated = jam_confirmed ? CORE_LINK_JAMMED : CORE_LINK_LOST;
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
