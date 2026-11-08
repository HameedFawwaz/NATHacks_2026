#ifndef CORE_COMMON_TRUST_SCORE_H
#define CORE_COMMON_TRUST_SCORE_H

#include <stdint.h>

/* Which module last wrote a component of the trust score. */
typedef enum {
    CORE_TRUST_SOURCE_UNKNOWN = 0,
    CORE_TRUST_SOURCE_NAV,
    CORE_TRUST_SOURCE_LINK,
    CORE_TRUST_SOURCE_CRYPTO,
    CORE_TRUST_SOURCE_AUTONOMY,
    CORE_TRUST_SOURCE_HARDWARE
} core_trust_source_t;

/*
 * Shared hand-off point between modules: nav/link/crypto/hardware each write their own
 * component via core_trust_score_set_component(), autonomy reads all of them
 * and writes overall_trust, and the mavlink layer only reads this struct to
 * publish it outward - it never computes trust itself.
 *
 * All scores are in [0, 1]; 1.0 = fully trusted, 0.0 = fully compromised/lost.
 */
typedef struct {
    float nav_trust;
    float link_trust;
    float crypto_trust;
    float hardware_trust;
    float overall_trust;
    core_trust_source_t last_updated_by;
    uint64_t last_update_ms;

    /*
     * When each input last reported, 0 = has never reported. These are separate
     * from last_update_ms because autonomy writes overall_trust every tick,
     * which would otherwise hide a nav/link/hardware producer that has died.
     */
    uint64_t nav_update_ms;
    uint64_t link_update_ms;
    uint64_t hardware_update_ms;
} core_trust_score_t;

/* Sets every component to 1.0 (fully trusted) and stamps last_update_ms = now. */
void core_trust_score_init(core_trust_score_t *score);

/* Clamps value to [0, 1] before writing it into the given component. */
void core_trust_score_set_component(core_trust_score_t *score, core_trust_source_t source, float value);

#endif /* CORE_COMMON_TRUST_SCORE_H */
