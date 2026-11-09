#include "core/autonomy/autonomy.h"

#include <string.h>

#include "core/common/log.h"

#define MAV_MODE_FLAG_ARMED 128 /* MAV_MODE_FLAG_SAFETY_ARMED, without exposing the vendored enum */

core_autonomy_config_t core_autonomy_default_config(void) {
    core_autonomy_config_t cfg = {
        .untrusted_below = 0.4f,
        .trusted_at = 0.7f,
        .recovery_hold_ms = 5000,
        .input_stale_ms = 3000,
        .link_hold_ms = 30000,
        .command_retry_ms = 1000,
        .failover_timeout_ms = 5000,
        .primary_source_set = 1,
        .fallback_source_set = 2,
        .hold_mode = 17,  /* COPTER_MODE_BRAKE */
        .return_mode = 6, /* COPTER_MODE_RTL */
        .land_mode = 9,   /* COPTER_MODE_LAND */
    };
    return cfg;
}

void core_autonomy_init(core_autonomy_t *ctx, const core_autonomy_config_t *cfg, FILE *log) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->log = log;
    ctx->behavior = CORE_AUTONOMY_NOMINAL;
    ctx->active_source_set = cfg->primary_source_set;
    ctx->wanted_source_set = cfg->primary_source_set;
    if (log) {
        fprintf(log, "mono_ms,from,to,reason,nav_trust,link_trust,hardware_trust\n");
        fflush(log);
    }
}

const char *core_autonomy_behavior_name(core_autonomy_behavior_t behavior) {
    switch (behavior) {
    case CORE_AUTONOMY_NOMINAL: return "NOMINAL";
    case CORE_AUTONOMY_GNSS_FAILOVER: return "GNSS_FAILOVER";
    case CORE_AUTONOMY_LINK_HOLD: return "LINK_HOLD";
    case CORE_AUTONOMY_LINK_RETURN: return "LINK_RETURN";
    case CORE_AUTONOMY_SAFE_LAND: return "SAFE_LAND";
    }
    return "UNKNOWN";
}

/* Drops immediately on a low or stale score; recovers only after trusted_at has held for recovery_hold_ms. */
static void update_untrusted(const core_autonomy_config_t *cfg, float value, uint64_t updated_ms, uint64_t now_ms,
                             bool *untrusted, uint64_t *recovering_since_ms) {
    bool stale = updated_ms != 0 && now_ms > updated_ms && now_ms - updated_ms > cfg->input_stale_ms;

    if (stale || value < cfg->untrusted_below) {
        *untrusted = true;
        *recovering_since_ms = 0;
    } else if (*untrusted) {
        if (value < cfg->trusted_at) {
            *recovering_since_ms = 0;
        } else if (*recovering_since_ms == 0) {
            *recovering_since_ms = now_ms;
        } else if (now_ms - *recovering_since_ms >= cfg->recovery_hold_ms) {
            *untrusted = false;
            *recovering_since_ms = 0;
        }
    }
}

static core_autonomy_behavior_t select_behavior(const core_autonomy_t *ctx, bool armed, uint64_t now_ms,
                                                const char **reason) {
    if (!armed) {
        *reason = "disarmed";
        return CORE_AUTONOMY_NOMINAL;
    }
    if (ctx->behavior == CORE_AUTONOMY_SAFE_LAND) {
        *reason = "landing latched";
        return CORE_AUTONOMY_SAFE_LAND;
    }
    if (ctx->failover_failed) {
        *reason = "gnss failover failed";
        return CORE_AUTONOMY_SAFE_LAND;
    }

    int untrusted_count = ctx->nav_untrusted + ctx->link_untrusted + ctx->hardware_untrusted;
    if (untrusted_count >= 2) {
        *reason = "multiple subsystems untrusted";
        return CORE_AUTONOMY_SAFE_LAND;
    }
    if (ctx->hardware_untrusted) {
        *reason = "hardware untrusted";
        return CORE_AUTONOMY_SAFE_LAND;
    }
    if (ctx->nav_untrusted) {
        *reason = "gnss untrusted";
        return CORE_AUTONOMY_GNSS_FAILOVER;
    }
    if (ctx->link_untrusted) {
        if (ctx->behavior == CORE_AUTONOMY_LINK_RETURN ||
            (ctx->behavior == CORE_AUTONOMY_LINK_HOLD && now_ms - ctx->behavior_since_ms >= ctx->cfg.link_hold_ms)) {
            *reason = "link still lost";
            return CORE_AUTONOMY_LINK_RETURN;
        }
        *reason = "link untrusted";
        return CORE_AUTONOMY_LINK_HOLD;
    }
    *reason = "all subsystems trusted";
    return CORE_AUTONOMY_NOMINAL;
}

static void log_transition(core_autonomy_t *ctx, core_mavlink_conn_t *conn, const core_trust_score_t *score,
                           core_autonomy_behavior_t to, const char *reason, uint64_t now_ms) {
    const char *from_name = core_autonomy_behavior_name(ctx->behavior);
    const char *to_name = core_autonomy_behavior_name(to);

    CORE_LOG_WARN("autonomy: %s -> %s (%s)", from_name, to_name, reason);
    if (ctx->log) {
        fprintf(ctx->log, "%llu,%s,%s,%s,%.3f,%.3f,%.3f\n", (unsigned long long)now_ms, from_name, to_name, reason,
                score->nav_trust, score->link_trust, score->hardware_trust);
        fflush(ctx->log);
    }

    char text[50];
    snprintf(text, sizeof(text), "AUTONOMY %s: %s", to_name, reason);
    core_mavlink_send_statustext(conn, CORE_MAV_SEVERITY_WARNING, text);
}

static void enter_behavior(core_autonomy_t *ctx, core_autonomy_behavior_t behavior, uint64_t now_ms) {
    ctx->behavior = behavior;
    ctx->behavior_since_ms = now_ms;

    /* NOMINAL and GNSS_FAILOVER deliberately leave the current mode alone: after
     * a recovery the operator decides what happens next, and a failover should
     * let the current flight continue. */
    switch (behavior) {
    case CORE_AUTONOMY_LINK_HOLD:
        ctx->pending_mode = ctx->cfg.hold_mode;
        ctx->mode_pending = true;
        break;
    case CORE_AUTONOMY_LINK_RETURN:
        ctx->pending_mode = ctx->cfg.return_mode;
        ctx->mode_pending = true;
        break;
    case CORE_AUTONOMY_SAFE_LAND:
        ctx->pending_mode = ctx->cfg.land_mode;
        ctx->mode_pending = true;
        break;
    case CORE_AUTONOMY_NOMINAL:
    case CORE_AUTONOMY_GNSS_FAILOVER:
        ctx->mode_pending = false;
        break;
    }
    ctx->last_mode_send_ms = 0;
}

/* land > return > hold > anything else. */
static int mode_rank(const core_autonomy_config_t *cfg, uint32_t mode) {
    if (mode == cfg->land_mode) return 3;
    if (mode == cfg->return_mode) return 2;
    if (mode == cfg->hold_mode) return 1;
    return 0;
}

static void drive_mode(core_autonomy_t *ctx, core_mavlink_conn_t *conn, const core_telemetry_t *telemetry,
                       uint64_t now_ms) {
    if (!ctx->mode_pending) {
        return;
    }
    if (telemetry->heartbeat.custom_mode == ctx->pending_mode) {
        ctx->mode_pending = false;
        return;
    }
    if (ctx->last_mode_send_ms == 0 || now_ms - ctx->last_mode_send_ms >= ctx->cfg.command_retry_ms) {
        ctx->last_mode_send_ms = now_ms;
        core_mavlink_send_set_mode(conn, ctx->pending_mode);
    }
}

static void drive_source_set(core_autonomy_t *ctx, core_mavlink_conn_t *conn, const core_telemetry_t *telemetry,
                             bool armed, uint64_t now_ms) {
    uint8_t wanted = ctx->wanted_source_set;
    if (ctx->nav_untrusted) {
        /* Don't switch on the ground, but do switch as soon as the vehicle arms. */
        if (armed) {
            wanted = ctx->cfg.fallback_source_set;
        }
    } else {
        wanted = ctx->cfg.primary_source_set;
    }

    if (wanted != ctx->wanted_source_set) {
        ctx->wanted_source_set = wanted;
        ctx->source_requested_ms = now_ms;
        ctx->last_source_send_ms = 0;
    }
    if (ctx->active_source_set == ctx->wanted_source_set) {
        return;
    }

    const core_command_ack_t *ack = &telemetry->command_ack;
    if (ctx->last_source_send_ms != 0 && ack->valid && ack->command == CORE_MAV_CMD_SET_EKF_SOURCE_SET &&
        ack->last_update_ms >= ctx->last_source_send_ms) {
        if (ack->result == CORE_MAV_RESULT_ACCEPTED) {
            CORE_LOG_WARN("autonomy: EKF source set %u active", ctx->wanted_source_set);
            ctx->active_source_set = ctx->wanted_source_set;
            return;
        }
        if (ack->result != CORE_MAV_RESULT_TEMPORARILY_REJECTED && ctx->nav_untrusted) {
            ctx->failover_failed = true;
        }
    }

    if (ctx->nav_untrusted && now_ms - ctx->source_requested_ms >= ctx->cfg.failover_timeout_ms) {
        ctx->failover_failed = true;
    }

    if (ctx->last_source_send_ms == 0 || now_ms - ctx->last_source_send_ms >= ctx->cfg.command_retry_ms) {
        ctx->last_source_send_ms = now_ms;
        core_mavlink_send_set_ekf_source_set(conn, ctx->wanted_source_set);
    }
}

void core_autonomy_tick(core_autonomy_t *ctx, core_mavlink_conn_t *conn, core_trust_score_t *score,
                        const core_telemetry_t *telemetry, uint64_t now_ms) {
    float overall = score->nav_trust;
    if (score->link_trust < overall) overall = score->link_trust;
    if (score->crypto_trust < overall) overall = score->crypto_trust;
    if (score->hardware_trust < overall) overall = score->hardware_trust;
    core_trust_score_set_component(score, CORE_TRUST_SOURCE_AUTONOMY, overall);

    if (!telemetry->heartbeat.valid) {
        return;
    }
    /* A stale vehicle heartbeat means armed state and mode are unknown, and
     * every producer fed by the autopilot (link above all) stops reporting with
     * it. Deciding then would blame our own blindness on an attack, and the
     * command would land once the autopilot came back. Hold everything instead,
     * and once it's back, give those producers one stale period to see fresh
     * data: nav, starved of GPS through the outage, reads 0 for a moment. An
     * attack still under way is acted on when that period ends. */
    uint64_t hb_ms = telemetry->heartbeat.last_update_ms;
    if (now_ms > hb_ms && now_ms - hb_ms > ctx->cfg.input_stale_ms) {
        ctx->vehicle_stale = true;
        return;
    }
    if (ctx->vehicle_stale) {
        ctx->vehicle_stale = false;
        ctx->resume_settle_until_ms = now_ms + ctx->cfg.input_stale_ms;
        CORE_LOG_WARN("autonomy: vehicle heard again, deciding again in %u ms", (unsigned)ctx->cfg.input_stale_ms);
    }
    if (now_ms < ctx->resume_settle_until_ms) {
        return;
    }
    bool armed = (telemetry->heartbeat.base_mode & MAV_MODE_FLAG_ARMED) != 0;

    update_untrusted(&ctx->cfg, score->nav_trust, score->nav_update_ms, now_ms,
                     &ctx->nav_untrusted, &ctx->nav_recovering_since_ms);
    update_untrusted(&ctx->cfg, score->link_trust, score->link_update_ms, now_ms,
                     &ctx->link_untrusted, &ctx->link_recovering_since_ms);
    update_untrusted(&ctx->cfg, score->hardware_trust, score->hardware_update_ms, now_ms,
                     &ctx->hardware_untrusted, &ctx->hardware_recovering_since_ms);

    drive_source_set(ctx, conn, telemetry, armed, now_ms);

    if (!armed) {
        ctx->failover_failed = false;
    }

    const char *reason = "";
    core_autonomy_behavior_t next = select_behavior(ctx, armed, now_ms, &reason);
    if (next != ctx->behavior) {
        log_transition(ctx, conn, score, next, reason, now_ms);
        enter_behavior(ctx, next, now_ms);
        /* Never pull a vehicle that is already landing or returning back up to a weaker response. */
        if (ctx->mode_pending && mode_rank(&ctx->cfg, telemetry->heartbeat.custom_mode) >= mode_rank(&ctx->cfg, ctx->pending_mode)) {
            ctx->mode_pending = false;
        }
    }

    drive_mode(ctx, conn, telemetry, now_ms);
}
