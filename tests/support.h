/* Shared test support: tiny check harness, seeded PRNG, an independent
 * reference implementation of the derech cost/search spec, and a
 * structural validator for returned paths.
 *
 * The reference is written FROM THE SPEC in derech.h, not from the
 * library source — it exists to cross-check the real implementation.
 * The one deliberate coupling: per-tag folding iterates bits 0..63
 * ascending, as documented, so double-precision products round
 * identically in both implementations. */

#ifndef DERECH_TEST_SUPPORT_H
#define DERECH_TEST_SUPPORT_H

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "derech.h"

/* ------------------------------------------------------------------ */
/* Harness                                                             */
/* ------------------------------------------------------------------ */

extern int t_checks;
extern int t_fails;

void t_check_impl(int ok, const char *expr, const char *file, int line);
int t_done(const char *suite); /* prints summary, returns exit code */

#define T_CHECK(cond) t_check_impl((cond) ? 1 : 0, #cond, __FILE__, __LINE__)

/* ------------------------------------------------------------------ */
/* Deterministic PRNG (splitmix64) — never seeded from the clock       */
/* ------------------------------------------------------------------ */

typedef struct t_rng {
	uint64_t s;
} t_rng;

uint64_t t_rng_next(t_rng *r);
uint32_t t_rng_below(t_rng *r, uint32_t bound);

/* ------------------------------------------------------------------ */
/* Reference model                                                     */
/* ------------------------------------------------------------------ */

typedef struct ref_map {
	uint32_t w, h;
	const float *pass;   /* w*h row-major passability */
	const uint64_t *tags;/* w*h tag words; NULL = all zero */
} ref_map;

#define REF_INF UINT64_MAX

/* Exact Dijkstra distance field of perceived Q8 costs from (sx, sy).
 * Returns a malloc'd array of w*h uint64 (REF_INF = unreachable);
 * caller frees. */
uint64_t *ref_solve_field(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy);

/* Optimal perceived Q8 cost from start to goal, or REF_INF. */
uint64_t ref_solve(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy);

/* ------------------------------------------------------------------ */
/* Builders                                                            */
/* ------------------------------------------------------------------ */

/* Zeroed neutral profile with struct_size set. */
derech_profile_desc t_neutral_desc(void);

/* Build a real map from a reference map (asserts internally). */
derech_map *t_build_map(const ref_map *m);

/* Convenience: zeroed request start->goal for profile 0, epsilon 1. */
derech_request t_req(uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy);

/* ------------------------------------------------------------------ */
/* Validation                                                          */
/* ------------------------------------------------------------------ */

/* Structurally validates result i against the reference model: step
 * adjacency and bounds, connectivity, profile blocking, corner rule,
 * per-step true ticks, total consistency, and endpoint agreement with
 * the reported status.  Records failures via T_CHECK. */
void t_validate_result(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy,
	const derech_results *res, uint32_t i);

#endif /* DERECH_TEST_SUPPORT_H */
