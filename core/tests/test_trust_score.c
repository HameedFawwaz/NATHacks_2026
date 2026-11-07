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
}

int main(void) {
    test_init_sets_all_to_one();
    test_set_component_clamps();
    return 0;
}
