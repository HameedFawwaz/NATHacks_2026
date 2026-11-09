#ifndef CORE_LINK_MONITOR_H
#define CORE_LINK_MONITOR_H

/*
 * Lost-link and jamming detection for the operator's control link.
 *
 * The companion computer isn't on that link, but the autopilot forwards the
 * operator GCS's heartbeats to it (telemetry.gcs_heartbeat), so their arrival
 * pattern measures the link: the fraction missing over a window, and how long
 * it has been silent. Radio data, when there is any, separates a link that was
 * jammed from one that was simply lost:
 *
 *   RADIO_STATUS noise   the radio's noise floor. Only reaches us if the radio
 *                        is wired to the companion computer: ArduPilot never
 *                        forwards RADIO_STATUS.
 *   RC_CHANNELS rssi     the telemetry radio's RSSI as relayed by ArduPilot
 *                        with RSSI_TYPE=5 (or the RC receiver's otherwise).
 *
 *   State      Means                                        link_trust
 *   ---------  -------------------------------------------  ----------
 *   UNKNOWN    no operator heard yet, or the autopilot      not reported
 *              itself is silent so nothing can be judged
 *   NOMINAL    heartbeats arriving as expected              about 1
 *   DEGRADED   heartbeats missing, weak signal, or          below 0.7
 *              interference while still connected
 *   LOST       silent, and the radio shows no jamming       0
 *   JAMMED     silent while the radio shows interference    0
 *
 * Telling JAMMED from LOST is a heuristic. With only RSSI to go on, "silent
 * but the radio still hears strong RF energy" is read as jamming; check it
 * against the real radio before relying on the distinction.
 *
 * Against flapping: loss is measured over a window, a short silence costs
 * nothing, the score drops at once but recovers at a bounded rate, and the
 * state only steps down to a less severe one after holding there.
 *
 * There is no backup link to switch to on this hardware, so the response is
 * left to autonomy, which reads link_trust.
 */

#include <stdbool.h>
#include <stdint.h>

#include "core/mavlink/mavlink_telemetry.h"

typedef enum {
    CORE_LINK_UNKNOWN = 0,
    CORE_LINK_NOMINAL,
    CORE_LINK_DEGRADED,
    CORE_LINK_LOST,
    CORE_LINK_JAMMED
} core_link_state_t;

typedef enum {
    CORE_LINK_RF_NONE = 0, /* no radio data */
    CORE_LINK_RF_RSSI,     /* RC_CHANNELS.rssi */
    CORE_LINK_RF_NOISE     /* RADIO_STATUS, with a noise floor */
} core_link_rf_source_t;

typedef struct {
    uint32_t window_ms;           /* loss is measured over this long (at least 10 heartbeat intervals) */
    uint32_t default_interval_ms; /* expected heartbeat interval until one is learned */
    uint32_t silence_grace_ms;    /* silence this short costs nothing (at least 3 heartbeat intervals) */
    uint32_t lost_after_ms;       /* silence this long is LOST or JAMMED (at least 6 intervals) */

    float loss_ok;  /* loss at or below this costs nothing... */
    float loss_bad; /* ...and at or above this, trust is 0 */

    float rssi_ok;          /* normalized RSSI (0..1) at or above this costs nothing... */
    float rssi_bad;         /* ...and at or below this, the score is rssi_floor_score */
    float rssi_floor_score; /* weak signal alone never makes the link untrusted */
    float jam_rssi_min;     /* silent with RSSI at or above this reads as jamming */
    uint16_t jam_noise_margin;   /* RADIO_STATUS noise this far above the learned floor is interference */
    float interference_cap;      /* trust ceiling while interference is seen but packets still arrive */
    uint32_t rf_stale_ms;        /* radio data older than this is ignored */

    float degraded_below;   /* trust below this is DEGRADED... */
    float nominal_at;       /* ...and it takes this much to be NOMINAL again, so it can't bounce, */
    uint32_t nominal_window_ms; /* ...with loss no worse than loss_ok over this longer window */
    float recover_per_s;    /* how fast trust may rise */
    uint32_t state_hold_ms; /* a less severe state must hold this long to be entered */
    uint32_t autopilot_late_ms;  /* autopilot heartbeat this late => hold the verdict: silence may be its fault */
    uint32_t autopilot_stale_ms; /* no autopilot heartbeat for this long => UNKNOWN */
} core_link_config_t;

typedef struct {
    core_link_state_t state;
    float link_trust;          /* meaningful unless state is UNKNOWN */
    float loss;                /* fraction of expected heartbeats missing over the window */
    uint32_t silence_ms;       /* since the operator was last heard */
    core_link_rf_source_t rf_source;
    bool interference;         /* the radio shows signs of jamming right now */
    bool state_changed;        /* state differs from the previous update */
    char reason[40];           /* short, so "LINK <STATE>: <reason>" fits a STATUSTEXT */
} core_link_result_t;

#define CORE_LINK_ARRIVALS 64
#define CORE_LINK_GAPS 9

typedef struct {
    core_link_config_t cfg;

    bool blind; /* the autopilot was silent at the last update */
    uint32_t seen_count;
    uint64_t first_heard_ms, last_heard_ms; /* 0 = not heard */
    uint64_t arrivals[CORE_LINK_ARRIVALS];
    int arrivals_head, arrivals_len;

    uint32_t interval_ms;
    uint32_t gaps[CORE_LINK_GAPS];
    int gaps_head, gaps_len;

    float noise_floor;
    bool noise_floor_known;
    uint64_t last_radio_ms; /* the RADIO_STATUS sample already folded into the floor */

    float trust;
    uint64_t last_update_ms;

    core_link_state_t state;
    uint64_t easing_since_ms; /* when a less severe state was first indicated, 0 = not */
} core_link_monitor_t;

void core_link_config_default(core_link_config_t *cfg);

void core_link_monitor_init(core_link_monitor_t *mon, const core_link_config_t *cfg);

/* Call once per main-loop tick with the latest telemetry. */
core_link_result_t core_link_monitor_update(core_link_monitor_t *mon, const core_telemetry_t *telemetry, uint64_t now_ms);

const char *core_link_state_name(core_link_state_t state);

#endif /* CORE_LINK_MONITOR_H */
