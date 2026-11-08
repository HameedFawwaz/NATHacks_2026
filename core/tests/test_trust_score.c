#include <assert.h>

#include "core/common/trust_score.h"

static void test_init_sets_all_to_one(void) {
    core_trust_score_t score;
    core_trust_score_init(&score);

    assert(score.nav_trust == 1.0f);
    assert(score.link_trust == 1.0f);
    assert(score.crypto_trust == 1.0f);
    assert(score.overall_trust == 1.0f);
    assert(score.last_updated_by == CORE_TRUST_SOURCE_UNKNOWN);
}

static void test_set_component_clamps(void) {
    core_trust_score_t score;
    core_trust_score_init(&score);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_NAV, -5.0f);
    assert(score.nav_trust == 0.0f);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_LINK, 5.0f);
    assert(score.link_trust == 1.0f);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_CRYPTO, 0.42f);
    assert(score.crypto_trust == 0.42f);
    assert(score.last_updated_by == CORE_TRUST_SOURCE_CRYPTO);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_AUTONOMY, 0.75f);
    assert(score.overall_trust == 0.75f);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_HARDWARE, 0.25f);
    assert(score.hardware_trust == 0.25f);
}

static void test_component_timestamps(void) {
    core_trust_score_t score;
    core_trust_score_init(&score);
    assert(score.nav_update_ms == 0);
    assert(score.link_update_ms == 0);
    assert(score.hardware_update_ms == 0);

    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_NAV, 0.5f);
    assert(score.nav_update_ms != 0);
    assert(score.link_update_ms == 0);

    /* Autonomy writing overall_trust must not make a dead producer look alive. */
    core_trust_score_set_component(&score, CORE_TRUST_SOURCE_AUTONOMY, 0.5f);
    assert(score.link_update_ms == 0);
}

int main(void) {
    test_init_sets_all_to_one();
    test_set_component_clamps();
    test_component_timestamps();
    return 0;
}
