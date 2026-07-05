/* Goal fields, the field cache, and component labels.
 *
 * Field-answered requests are observable through expansions == 0 (no
 * search ran) and exact-optimal costs; classic searches report
 * expansions > 0.  White-box checks inspect the cache via
 * derech_internal.h. */

#include <math.h>
#include <stdlib.h>

#include "derech_internal.h" /* white-box: cache accounting */
#include "support.h"

#define SQRT2_TICKS 1.4140625f /* 362 / 256 */

static derech_results *run_batch(derech_map *m, const derech_request *q,
	uint32_t n)
{
	derech_results *res = NULL;

	T_CHECK(derech_find_paths(m, q, n, &res) == DERECH_OK);
	T_CHECK(res != NULL);
	return res;
}

/* Connected weighted terrain (no walls), deterministic. */
static void fill_weighted(t_rng *rng, uint32_t n, float *pass,
	uint64_t *tags)
{
	static const float levels[3] = { 1.0f, 0.5f, 0.25f };

	for (uint32_t i = 0; i < n; i++) {
		pass[i] = levels[t_rng_below(rng, 3)];
		tags[i] = t_rng_below(rng, 4); /* bits 0..1 */
	}
}

static void test_group_uses_field(void)
{
	enum { W = 24, H = 24, NREQ = 8 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0xF1E1DULL };
	ref_map rm = { W, H, pass, tags };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[NREQ];
	derech_results *res;

	fill_weighted(&rng, W * H, pass, tags);
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	for (uint32_t i = 0; i < NREQ; i++) {
		q[i] = t_req(i, 2 * i + 1, 20, 20); /* distinct starts */
		q[i].epsilon = 1.25f;
	}
	res = run_batch(m, q, NREQ);
	for (uint32_t i = 0; i < NREQ; i++) {
		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_expansions(res, i) == 0);
		/* fields are exact even though epsilon allowed 1.25x */
		T_CHECK((uint64_t)llround((double)
			derech_result_total_perceived(res, i) * 256.0) ==
			ref_solve(&rm, &d, q[i].start_x, q[i].start_y, 20, 20));
		t_validate_result(&rm, &d, q[i].start_x, q[i].start_y, 20, 20,
			res, i);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_below_threshold_solves(void)
{
	derech_map *m = derech_map_create(16, 16, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q[3];
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 3; i++) {
		q[i] = t_req(i, 0, 15, 15);
	}
	res = run_batch(m, q, 3);
	for (uint32_t i = 0; i < 3; i++) {
		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_expansions(res, i) > 0);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_cache_hit_and_invalidation(void)
{
	enum { W = 24, H = 24 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0xCAC4EULL };
	ref_map rm = { W, H, pass, tags };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[4];
	derech_request one;
	derech_results *res;

	fill_weighted(&rng, W * H, pass, tags);
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(i, 0, 20, 12);
	}
	res = run_batch(m, q, 4); /* builds and caches the field */
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);

	/* a later SINGLE request to the same (goal, profile) rides the
	 * cached field */
	one = t_req(3, 19, 20, 12);
	res = run_batch(m, &one, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	T_CHECK((uint64_t)llround((double)
		derech_result_total_perceived(res, 0) * 256.0) ==
		ref_solve(&rm, &d, 3, 19, 20, 12));
	derech_results_destroy(res);

	/* any edit flushes the cache; the single request now searches */
	T_CHECK(derech_map_set_passability_at(m, 0, 23, 0.5f) == DERECH_OK);
	res = run_batch(m, &one, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_expansions(res, 0) > 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_reverse_cost_asymmetry(void)
{
	/* THE regression for the reverse-graph subtlety: entering the goal
	 * tile costs the GOAL's passability, not the start's */
	float pass2[2] = { 1.0f, 0.25f };
	ref_map rm2 = { 2, 1, pass2, NULL };
	float pass4[4] = { 1.0f, 1.0f, 1.0f, 0.25f };
	ref_map rm4 = { 2, 2, pass4, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[4];
	derech_results *res;

	m = t_build_map(&rm2);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, 0, 1, 0);
	}
	res = run_batch(m, q, 4);
	for (uint32_t i = 0; i < 4; i++) {
		T_CHECK(derech_result_expansions(res, i) == 0);
		T_CHECK(derech_result_total_perceived(res, i) == 4.0f);
		T_CHECK(derech_result_total_ticks(res, i) == 4.0f);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);

	/* diagonal vs two straight steps into an expensive goal: the field
	 * must pick the genuinely cheaper two-step route (5.0 < 4 * sqrt2) */
	m = t_build_map(&rm4);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, 0, 1, 1);
	}
	res = run_batch(m, q, 4);
	for (uint32_t i = 0; i < 4; i++) {
		T_CHECK(derech_result_expansions(res, i) == 0);
		T_CHECK(derech_result_length(res, i) == 2);
		T_CHECK(derech_result_total_perceived(res, i) == 5.0f);
		T_CHECK((uint64_t)llround((double)
			derech_result_total_perceived(res, i) * 256.0) ==
			ref_solve(&rm4, &d, 0, 0, 1, 1));
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void seal_goal(float *pass, uint32_t w, uint32_t gx, uint32_t gy)
{
	for (uint32_t y = gy - 1; y <= gy + 1; y++) {
		for (uint32_t x = gx - 1; x <= gx + 1; x++) {
			if (x != gx || y != gy) {
				pass[y * w + x] = 0.0f;
			}
		}
	}
}

static void test_sealed_goal_labels_and_fallback(void)
{
	enum { W = 9, H = 9 };
	float pass[W * H];
	ref_map rm = { W, H, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[4];
	derech_results *res;

	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
	}
	seal_goal(pass, W, 4, 4);
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	/* non-partial: component labels answer in O(1), no sweep */
	q[0] = t_req(0, 0, 4, 4);
	res = run_batch(m, q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	T_CHECK(derech_result_length(res, 0) == 0);
	derech_results_destroy(res);

	/* partial: still needs the closest-approach sweep */
	q[0].flags = DERECH_REQ_ALLOW_PARTIAL;
	res = run_batch(m, q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) > 0);
	T_CHECK(derech_result_length(res, 0) > 0);
	t_validate_result(&rm, &d, 0, 0, 4, 4, res, 0);
	derech_results_destroy(res);

	/* a partial GROUP builds the (tiny) field, extraction sees the
	 * infinite distance, and every member falls back to a search */
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(i, 0, 4, 4);
		q[i].flags = DERECH_REQ_ALLOW_PARTIAL;
	}
	res = run_batch(m, q, 4);
	for (uint32_t i = 0; i < 4; i++) {
		T_CHECK(derech_result_status(res, i) ==
			DERECH_PATH_UNREACHABLE);
		T_CHECK(derech_result_expansions(res, i) > 0);
		T_CHECK(derech_result_length(res, i) > 0);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_blocked_start_labels(void)
{
	enum { W = 9, H = 9 };
	float pass[W * H];
	ref_map rm = { W, H, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q;
	derech_results *res;

	/* blocked start with an open neighbor: escapable, must be solved */
	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
	}
	pass[0] = 0.0f; /* (0,0) */
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(0, 0, 8, 8);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	derech_results_destroy(res);
	derech_map_destroy(m);

	/* start sealed inside a blocked 3x3: labels prove it in O(1) */
	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 0; y <= 2; y++) {
		for (uint32_t x = 0; x <= 2; x++) {
			pass[y * W + x] = 0.0f;
		}
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(1, 1, 8, 8);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_field_cost_cap(void)
{
	derech_map *m = derech_map_create(9, 1, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q[4];
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, 0, 8, 0);
	}
	q[0].max_perceived_cost = 5.0f;
	q[1].max_perceived_cost = 5.0f;
	res = run_batch(m, q, 4);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(res, 0) == 0);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	T_CHECK(derech_result_status(res, 2) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_perceived(res, 2) == 8.0f);
	T_CHECK(derech_result_expansions(res, 2) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_blocked_start_via_field(void)
{
	derech_map *m = derech_map_create(9, 1, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q[4];
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_set_passability_at(m, 0, 0, 0.0f) == DERECH_OK);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, 0, 8, 0);
	}
	res = run_batch(m, q, 4);
	for (uint32_t i = 0; i < 4; i++) {
		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_expansions(res, i) == 0);
		T_CHECK(derech_result_length(res, i) == 8);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_threshold_one(void)
{
	derech_map_opts o;
	derech_map *m;
	derech_profile_desc d = t_neutral_desc();
	derech_request q;
	derech_results *res;

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.field_group_threshold = 1;
	m = derech_map_create(16, 16, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(0, 0, 15, 15);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_expansions(res, 0) == 0); /* field even alone */
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_profile_specific_fields(void)
{
	/* same goal, two profiles: separate fields, different routes */
	enum { W = 5, H = 5 };
	float pass[W * H];
	uint64_t tags[W * H];
	ref_map rm = { W, H, pass, tags };
	derech_profile_desc walker = t_neutral_desc();
	derech_profile_desc swimmer = t_neutral_desc();
	derech_map *m;
	derech_request q[8];
	derech_results *res;

	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
		tags[i] = 0;
	}
	for (uint32_t y = 0; y < 4; y++) {
		tags[y * W + 2] = 1; /* water column, bridge at y = 4 */
	}
	walker.block_mask = 1;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &walker) == 0);
	T_CHECK(derech_profile_register(m, &swimmer) == 1);

	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, i, 4, 2);
		q[i].profile_id = 0;
		q[i + 4] = t_req(0, i, 4, 2);
		q[i + 4].profile_id = 1;
	}
	res = run_batch(m, q, 8);
	for (uint32_t i = 0; i < 8; i++) {
		const derech_profile_desc *d = i < 4 ? &walker : &swimmer;

		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_expansions(res, i) == 0);
		T_CHECK((uint64_t)llround((double)
			derech_result_total_perceived(res, i) * 256.0) ==
			ref_solve(&rm, d, q[i].start_x, q[i].start_y, 4, 2));
		t_validate_result(&rm, d, q[i].start_x, q[i].start_y, 4, 2,
			res, i);
	}
	/* the walker's detour over the bridge is strictly longer */
	T_CHECK(derech_result_total_perceived(res, 2) >
		derech_result_total_perceived(res, 6));
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_lru_eviction(void)
{
	/* 64x64 fields are ~20.5 KB; a 1 MiB budget holds ~51.  Build 60
	 * distinct fields and verify the cache trims back under budget. */
	enum { W = 64, H = 64, GOALS = 60 };
	derech_map_opts o;
	derech_map *m;
	derech_profile_desc d = t_neutral_desc();
	derech_request q[4 * GOALS];
	derech_results *res;
	uint32_t cached = 0;

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.field_cache_mb = 1;
	m = derech_map_create(W, H, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	for (uint32_t g = 0; g < GOALS; g++) {
		for (uint32_t i = 0; i < 4; i++) {
			q[g * 4 + i] = t_req(i, 0, g % W, 1 + g / W);
		}
	}
	res = run_batch(m, q, 4 * GOALS);
	for (uint32_t i = 0; i < 4 * GOALS; i++) {
		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
	}
	derech_results_destroy(res);

	T_CHECK(m->field_bytes <= m->field_budget_bytes);
	for (derech_field *f = m->field_lru_head; f != NULL;
		f = f->lru_next) {
		T_CHECK(f->ok);
		T_CHECK(!f->pinned);
		cached++;
	}
	T_CHECK(cached > 0);
	T_CHECK(cached < GOALS);
	derech_map_destroy(m);
}

static void test_field_history_determinism(void)
{
	/* identical batch SEQUENCES on serial and 8-thread maps must yield
	 * identical bytes, through builds, cache hits, and invalidation */
	enum { W = 24, H = 24, NREQ = 16 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0xD17E5ULL };
	derech_map_opts o;
	derech_map *maps[2];
	derech_profile_desc d = t_neutral_desc();
	derech_request q[NREQ];

	fill_weighted(&rng, W * H, pass, tags);
	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	for (uint32_t c = 0; c < 2; c++) {
		o.n_threads = c == 0 ? 1 : 8;
		maps[c] = derech_map_create(W, H, &o);
		T_CHECK(maps[c] != NULL);
		T_CHECK(derech_map_set_passability(maps[c], pass,
			W * H) == DERECH_OK);
		T_CHECK(derech_map_set_tags(maps[c], tags, W * H) ==
			DERECH_OK);
		T_CHECK(derech_profile_register(maps[c], &d) == 0);
	}

	for (uint32_t round = 0; round < 6; round++) {
		derech_results *a = NULL;
		derech_results *b = NULL;

		if (round == 3) { /* invalidate both caches identically */
			T_CHECK(derech_map_set_passability_at(maps[0], 12,
				12, 0.5f) == DERECH_OK);
			T_CHECK(derech_map_set_passability_at(maps[1], 12,
				12, 0.5f) == DERECH_OK);
		}
		/* half the batch converges on one goal, half is random */
		for (uint32_t i = 0; i < NREQ; i++) {
			if (i < NREQ / 2) {
				q[i] = t_req(t_rng_below(&rng, W),
					t_rng_below(&rng, H), 20, 20);
			} else {
				q[i] = t_req(t_rng_below(&rng, W),
					t_rng_below(&rng, H),
					t_rng_below(&rng, W),
					t_rng_below(&rng, H));
			}
			q[i].flags = (t_rng_next(&rng) & 1) ?
				DERECH_REQ_ALLOW_PARTIAL : 0;
		}
		T_CHECK(derech_find_paths(maps[0], q, NREQ, &a) ==
			DERECH_OK);
		T_CHECK(derech_find_paths(maps[1], q, NREQ, &b) ==
			DERECH_OK);
		for (uint32_t i = 0; i < NREQ; i++) {
			uint32_t len = derech_result_length(a, i);

			T_CHECK(derech_result_status(a, i) ==
				derech_result_status(b, i));
			T_CHECK(len == derech_result_length(b, i));
			T_CHECK(derech_result_expansions(a, i) ==
				derech_result_expansions(b, i));
			T_CHECK(derech_result_total_perceived(a, i) ==
				derech_result_total_perceived(b, i));
			if (len > 0 && len == derech_result_length(b, i)) {
				T_CHECK(memcmp(derech_result_steps(a, i),
					derech_result_steps(b, i), (size_t)len *
					2 * sizeof(uint32_t)) == 0);
			}
		}
		derech_results_destroy(a);
		derech_results_destroy(b);
	}
	derech_map_destroy(maps[0]);
	derech_map_destroy(maps[1]);
}

int main(void)
{
	test_group_uses_field();
	test_below_threshold_solves();
	test_cache_hit_and_invalidation();
	test_reverse_cost_asymmetry();
	test_sealed_goal_labels_and_fallback();
	test_blocked_start_labels();
	test_field_cost_cap();
	test_blocked_start_via_field();
	test_threshold_one();
	test_profile_specific_fields();
	test_lru_eviction();
	test_field_history_determinism();
	return t_done("test_field");
}
