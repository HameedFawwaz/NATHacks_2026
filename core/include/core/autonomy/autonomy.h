#ifndef CORE_AUTONOMY_H
#define CORE_AUTONOMY_H

/*
 * Decision layer: reads the nav/link/hardware trust scores and drives the
 * vehicle's response, without waiting for an operator who may be the one under
 * attack.
 *
 *   Untrusted                Behavior       Action
 *   ----------------------   -------------  --------------------------------
 *   nothing                  NOMINAL        none; the operator keeps control
 *   nav                      GNSS_FAILOVER  switch EKF to the non-GPS source set
 *   link                     LINK_HOLD      BRAKE, giving the link time to return
 *   link, for link_hold_ms   LINK_RETURN    RTL
 *   hardware, or 2+ of the   SAFE_LAND      LAND (stays latched until disarmed)
 *   above, or failover failed
 *
 * While disarmed the behavior is always NOMINAL and no flight mode is commanded.
 * Each mode is commanded once per behavior entry and retried only until the
 * heartbeat confirms it, so a pilot with working RC can still override it.
 *
 * The hold is BRAKE rather than LOITER because LOITER still takes altitude from
 * the pilot's throttle stick: in SITL, with nobody on the sticks, it flew into
 * the ground.
 *
 * Crypto trust feeds overall_trust but does not select a behavior.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "core/common/trust_score.h"
#include "core/mavlink/mavlink.h"

typedef enum {
    CORE_AUTONOMY_NOMINAL = 0,
    CORE_AUTONOMY_GNSS_FAILOVER,
    CORE_AUTONOMY_LINK_HOLD,
    CORE_AUTONOMY_LINK_RETURN,
    CORE_AUTONOMY_SAFE_LAND
} core_autonomy_behavior_t;

typedef struct {
    float untrusted_below;       /* a subsystem becomes untrusted below this score... */
    float trusted_at;            /* ...and needs this score to recover */
    uint32_t recovery_hold_ms;   /* ...held this long, so a flapping attacker can't cycle behaviors */
    uint32_t input_stale_ms;     /* a producer silent this long counts as untrusted; one that has never reported is ignored */
    uint32_t link_hold_ms;       /* time in LINK_HOLD before escalating to LINK_RETURN */
    uint32_t command_retry_ms;
    uint32_t failover_timeout_ms; /* unacknowledged source switch for this long => SAFE_LAND */

    uint8_t primary_source_set;  /* EK3_SRC set using GPS */
    uint8_t fallback_source_set; /* EK3_SRC set using non-GPS navigation */

    /* Firmware-specific custom_mode numbers. */
    uint32_t hold_mode;
    uint32_t return_mode;
    uint32_t land_mode;
} core_autonomy_config_t;

typedef struct {
    core_autonomy_config_t cfg;
    bool enabled; /* detectors still run while automatic commands are disabled */
    uint64_t disabled_since_ms;
    FILE *log; /* transition log, NULL = STATUSTEXT and stderr only */

    core_autonomy_behavior_t behavior;
    uint64_t behavior_since_ms;

    bool nav_untrusted, link_untrusted, hardware_untrusted;
    uint64_t nav_recovering_since_ms, link_recovering_since_ms, hardware_recovering_since_ms; /* 0 = not recovering */

    bool failover_failed;

    /* The vehicle's heartbeat went stale: decisions are held until it's back,
     * and then until resume_settle_until_ms. */
    bool vehicle_stale;
    uint64_t resume_settle_until_ms;

    bool mode_pending;
    uint32_t pending_mode;
    uint64_t last_mode_send_ms;

    uint8_t active_source_set;
    uint8_t wanted_source_set;
    uint64_t source_requested_ms; /* when wanted_source_set first differed from active */
    uint64_t last_source_send_ms;
} core_autonomy_t;

/* Thresholds and timings for the demo, with ArduCopter mode numbers. */
core_autonomy_config_t core_autonomy_default_config(void);

/*
 * log, if not NULL, receives one CSV row per transition and is flushed on each
 * so it survives a crash. It stays owned by the caller. The header row is
 * written here, so pass a newly opened file.
 */
void core_autonomy_init(core_autonomy_t *ctx, const core_autonomy_config_t *cfg, FILE *log);

/* Pauses new automatic commands, not the aircraft's existing mode/source.
 * Hold/retry timers do not expire merely because responses were disabled. */
void core_autonomy_set_enabled(core_autonomy_t *ctx, bool enabled, uint64_t now_ms);

/* Clear decision latches only on a fresh disarmed vehicle, after the caller
 * has obtained an ACK selecting the primary source. Preserves enable/log/cfg;
 * does not change detector scores or send any flight command. */
bool core_autonomy_reset_ground(core_autonomy_t *ctx, const core_telemetry_t *telemetry, uint64_t now_ms);

/*
 * Runs one decision step: updates overall_trust in score, then sends whatever
 * commands the current behavior still needs. Non-blocking; call it once per
 * main-loop tick after core_mavlink_conn_poll(). Does nothing until a vehicle
 * heartbeat has been received, because armed state is unknown before that, nor
 * while that heartbeat is more than input_stale_ms old, nor for input_stale_ms
 * after it returns, so producers fed by the autopilot can catch up first.
 */
void core_autonomy_tick(core_autonomy_t *ctx, core_mavlink_conn_t *conn, core_trust_score_t *score,
                        const core_telemetry_t *telemetry, uint64_t now_ms);

const char *core_autonomy_behavior_name(core_autonomy_behavior_t behavior);

#endif /* CORE_AUTONOMY_H */
