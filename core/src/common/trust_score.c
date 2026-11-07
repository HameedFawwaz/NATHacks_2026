#include "core/common/trust_score.h"
#include "core/common/time.h"

static float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

void core_trust_score_init(core_trust_score_t *score) {
    score->nav_trust = 1.0f;
    score->link_trust = 1.0f;
    score->crypto_trust = 1.0f;
    score->overall_trust = 1.0f;
    score->last_updated_by = CORE_TRUST_SOURCE_UNKNOWN;
    score->last_update_ms = core_now_ms();
}

void core_trust_score_set_component(core_trust_score_t *score, core_trust_source_t source, float value) {
    float clamped = clamp01(value);

    switch (source) {
    case CORE_TRUST_SOURCE_NAV:
        score->nav_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_LINK:
        score->link_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_CRYPTO:
        score->crypto_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_AUTONOMY:
        score->overall_trust = clamped;
        break;
    case CORE_TRUST_SOURCE_UNKNOWN:
    default:
        break;
    }

    score->last_updated_by = source;
    score->last_update_ms = core_now_ms();
}
