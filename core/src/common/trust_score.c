#include "core/common/trust_score.h"
#include "core/common/time.h"

static float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool core_trust_component_live(uint64_t update_ms, uint64_t now_ms) {
    return update_ms != 0 && now_ms >= update_ms && now_ms - update_ms <= CORE_TRUST_LIVE_MS;
}

void core_trust_score_init(core_trust_score_t *score) {
    score->nav_trust = 1.0f;
    score->link_trust = 1.0f;
    score->crypto_trust = 1.0f;
    score->hardware_trust = 1.0f;
    score->overall_trust = 1.0f;
    score->last_updated_by = CORE_TRUST_SOURCE_UNKNOWN;
    score->last_update_ms = core_now_ms();
    score->nav_update_ms = 0;
    score->link_update_ms = 0;
    score->hardware_update_ms = 0;
}

void core_trust_score_set_component(core_trust_score_t *score, core_trust_source_t source, float value) {
    float clamped = clamp01(value);
    uint64_t now = core_now_ms();

    switch (source) {
    case CORE_TRUST_SOURCE_NAV:
        score->nav_trust = clamped;
        score->nav_update_ms = now;
        break;
    case CORE_TRUST_SOURCE_LINK:
        score->link_trust = clamped;
        score->link_update_ms = now;
        break;
    case CORE_TRUST_SOURCE_CRYPTO:
        score->crypto_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_HARDWARE:
        score->hardware_trust = clamped;
        score->hardware_update_ms = now;
        break;
    case CORE_TRUST_SOURCE_AUTONOMY:
        score->overall_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_UNKNOWN:
    default:
        break;
    }

    score->last_updated_by = source;
    score->last_update_ms = now;
}
