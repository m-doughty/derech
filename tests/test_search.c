/* Search semantics: movement rules, weights, preferences, budgets,
 * partials, and the epsilon quality bound.  Expected tick values are
 * exact: every Q8 quantity here divides by 256 without float rounding. */

#include <math.h>
#include <stdlib.h>

#include "support.h"

#define SQRT2_TICKS 1.4140625f /* 362 / 256 */

/* Run one request against a map; asserts the batch itself succeeds. */
static derech_results *solve1(derech_map *m, derech_request q)
{
	derech_results *res = NULL;

	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_OK);
	T_CHECK(res != NULL);
	return res;
}

/* Build an all-passable w*h map with a registered neutral profile. */
static derech_map *open_map(uint32_t w, uint32_t h)
{
	derech_map *m = derech_map_create(w, h, NULL);
	derech_profile_desc d = t_neutral_desc();

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	return m;
}

static void test_trivial(void)
{
	derech_map *m = open_map(3, 3);
	derech_results *res = solve1(m, t_req(1, 1, 1, 1));

	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 0);
	T_CHECK(derech_result_total_ticks(res, 0) == 0.0f);
	T_CHECK(derech_result_total_perceived(res, 0) == 0.0f);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_straight_line(void)
{
	derech_map *m = open_map(5, 5);
	derech_results *res = solve1(m, t_req(0, 0, 4, 0));
	const uint32_t *steps = derech_result_steps(res, 0);
	const float *ticks = derech_result_step_ticks(res, 0);

	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 4);
	T_CHECK(derech_result_total_ticks(res, 0) == 4.0f);
	T_CHECK(derech_result_total_perceived(res, 0) == 4.0f);
	T_CHECK(steps != NULL && ticks != NULL);
	for (uint32_t i = 0; i < 4 && steps != NULL; i++) {
		T_CHECK(steps[i * 2] == i + 1);
		T_CHECK(steps[i * 2 + 1] == 0);
		T_CHECK(ticks[i] == 1.0f);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_diagonal(void)
{
	derech_map *m = open_map(5, 5);
	derech_results *res = solve1(m, t_req(0, 0, 4, 4));
	const float *ticks = derech_result_step_ticks(res, 0);

	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 4);
	T_CHECK(derech_result_total_ticks(res, 0) == 4 * SQRT2_TICKS);
	for (uint32_t i = 0; i < 4 && ticks != NULL; i++) {
		T_CHECK(ticks[i] == SQRT2_TICKS);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_conn4(void)
{
	derech_map *m = derech_map_create(5, 5, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_results *res;
	derech_request q;

	T_CHECK(m != NULL);
	d.connectivity = DERECH_CONN_4;
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(0, 0, 4, 4);
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 8);
	T_CHECK(derech_result_total_ticks(res, 0) == 8.0f);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_wall_detour(void)
{
	/* vertical wall at x = 3 with a gap at y = 5 */
	float pass[7 * 7];
	ref_map rm = { 7, 7, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	uint64_t want;

	for (uint32_t i = 0; i < 49; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 0; y < 7; y++) {
		if (y != 5) {
			pass[y * 7 + 3] = 0.0f;
		}
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	res = solve1(m, t_req(0, 3, 6, 3));
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	t_validate_result(&rm, &d, 0, 3, 6, 3, res, 0);
	want = ref_solve(&rm, &d, 0, 3, 6, 3);
	T_CHECK(want != REF_INF);
	T_CHECK((uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0) ==
		want);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_corner_rules(void)
{
	/* 3x3, tile (1,0) blocked; diagonal (0,0)->(1,1) has one blocked
	 * flank */
	float pass[9];
	derech_profile_desc d;
	derech_map *m;
	derech_results *res;
	derech_request q = t_req(0, 0, 1, 1);
	static const uint8_t rules[3] = { DERECH_CORNER_STRICT,
		DERECH_CORNER_LENIENT, DERECH_CORNER_ALLOW };

	for (uint32_t r = 0; r < 3; r++) {
		ref_map rm = { 3, 3, pass, NULL };

		for (uint32_t i = 0; i < 9; i++) {
			pass[i] = 1.0f;
		}
		pass[0 * 3 + 1] = 0.0f; /* (1,0) */
		m = t_build_map(&rm);
		T_CHECK(m != NULL);
		d = t_neutral_desc();
		d.corner_rule = rules[r];
		T_CHECK(derech_profile_register(m, &d) == 0);
		res = solve1(m, q);
		T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
		if (rules[r] == DERECH_CORNER_STRICT) {
			T_CHECK(derech_result_length(res, 0) == 2);
			T_CHECK(derech_result_total_ticks(res, 0) == 2.0f);
		} else {
			T_CHECK(derech_result_length(res, 0) == 1);
			T_CHECK(derech_result_total_ticks(res, 0) ==
				SQRT2_TICKS);
		}
		t_validate_result(&rm, &d, 0, 0, 1, 1, res, 0);
		derech_results_destroy(res);
		derech_map_destroy(m);
	}
}

static void test_corner_squeeze(void)
{
	/* both flanks of the (0,0)->(1,1) diagonal blocked: only ALLOW may
	 * squeeze through the X */
	float pass[9];
	derech_profile_desc d;
	derech_map *m;
	derech_results *res;
	static const uint8_t rules[3] = { DERECH_CORNER_STRICT,
		DERECH_CORNER_LENIENT, DERECH_CORNER_ALLOW };

	for (uint32_t r = 0; r < 3; r++) {
		ref_map rm = { 3, 3, pass, NULL };

		for (uint32_t i = 0; i < 9; i++) {
			pass[i] = 1.0f;
		}
		pass[0 * 3 + 1] = 0.0f; /* (1,0) */
		pass[1 * 3 + 0] = 0.0f; /* (0,1) */
		m = t_build_map(&rm);
		T_CHECK(m != NULL);
		d = t_neutral_desc();
		d.corner_rule = rules[r];
		T_CHECK(derech_profile_register(m, &d) == 0);
		res = solve1(m, t_req(0, 0, 1, 1));
		if (rules[r] == DERECH_CORNER_ALLOW) {
			T_CHECK(derech_result_status(res, 0) ==
				DERECH_PATH_FOUND);
			T_CHECK(derech_result_length(res, 0) == 1);
		} else {
			T_CHECK(derech_result_status(res, 0) ==
				DERECH_PATH_UNREACHABLE);
			T_CHECK(derech_result_length(res, 0) == 0);
		}
		t_validate_result(&rm, &d, 0, 0, 1, 1, res, 0);
		derech_results_destroy(res);
		derech_map_destroy(m);
	}
}

static void test_weighted_choice(void)
{
	/* slow swamp on the direct row; the fast detour over y=0 wins */
	float pass[5 * 3];
	ref_map rm = { 5, 3, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res;

	for (uint32_t i = 0; i < 15; i++) {
		pass[i] = 1.0f;
	}
	pass[1 * 5 + 1] = 0.25f;
	pass[1 * 5 + 2] = 0.25f;
	pass[1 * 5 + 3] = 0.25f;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	res = solve1(m, t_req(0, 1, 4, 1));
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	/* diag + 2 straight + diag = 2*sqrt2 + 2 */
	T_CHECK(derech_result_total_ticks(res, 0) ==
		2.0f + 2 * SQRT2_TICKS);
	T_CHECK((uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0) ==
		ref_solve(&rm, &d, 0, 1, 4, 1));
	t_validate_result(&rm, &d, 0, 1, 4, 1, res, 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_block_and_require_masks(void)
{
	/* column of water (bit 0) at x = 2, bridge row at y = 4 */
	float pass[5 * 5];
	uint64_t tags[5 * 5];
	ref_map rm = { 5, 5, pass, tags };
	derech_profile_desc walker = t_neutral_desc();
	derech_profile_desc swimmer = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;

	for (uint32_t i = 0; i < 25; i++) {
		pass[i] = 1.0f;
		tags[i] = 0;
	}
	for (uint32_t y = 0; y < 4; y++) {
		tags[y * 5 + 2] = 1; /* water */
	}
	walker.block_mask = 1;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &walker) == 0);
	T_CHECK(derech_profile_register(m, &swimmer) == 1);

	q = t_req(0, 2, 4, 2);
	res = solve1(m, q); /* walker: around via the bridge */
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	t_validate_result(&rm, &walker, 0, 2, 4, 2, res, 0);
	T_CHECK((uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0) ==
		ref_solve(&rm, &walker, 0, 2, 4, 2));
	derech_results_destroy(res);

	q.profile_id = 1; /* swimmer: straight across */
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 4);
	T_CHECK(derech_result_total_ticks(res, 0) == 4.0f);
	t_validate_result(&rm, &swimmer, 0, 2, 4, 2, res, 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_require_mask(void)
{
	/* road (bit 1) on x = 0..3 of a 7x1 strip; carts may not leave it */
	float pass[7];
	uint64_t tags[7];
	ref_map rm = { 7, 1, pass, tags };
	derech_profile_desc cart = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;

	for (uint32_t i = 0; i < 7; i++) {
		pass[i] = 1.0f;
		tags[i] = i <= 3 ? 2 : 0;
	}
	cart.require_mask = 2;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &cart) == 0);

	res = solve1(m, t_req(0, 0, 3, 0));
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 3);
	t_validate_result(&rm, &cart, 0, 0, 3, 0, res, 0);
	derech_results_destroy(res);

	/* off-road goal: unenterable, resolved without searching */
	q = t_req(0, 0, 5, 0);
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 0) == 0);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_mult_preference(void)
{
	/* enemy land (bit 0) on (3,1) and (3,2); (3,0) is the clean gap */
	float pass[7 * 3];
	uint64_t tags[7 * 3];
	ref_map rm = { 7, 3, pass, tags };
	derech_profile_desc neutral = t_neutral_desc();
	derech_profile_desc avoider = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;
	const uint32_t *steps;

	for (uint32_t i = 0; i < 21; i++) {
		pass[i] = 1.0f;
		tags[i] = 0;
	}
	tags[1 * 7 + 3] = 1;
	tags[2 * 7 + 3] = 1;
	avoider.tag_mult[0] = 4.0f;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &neutral) == 0);
	T_CHECK(derech_profile_register(m, &avoider) == 1);

	res = solve1(m, t_req(0, 1, 6, 1)); /* neutral cuts straight */
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_perceived(res, 0) == 6.0f);
	t_validate_result(&rm, &neutral, 0, 1, 6, 1, res, 0);
	derech_results_destroy(res);

	q = t_req(0, 1, 6, 1);
	q.profile_id = 1;
	res = solve1(m, q); /* avoider detours through the gap */
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_perceived(res, 0) ==
		4.0f + 2 * SQRT2_TICKS);
	T_CHECK(derech_result_total_ticks(res, 0) ==
		4.0f + 2 * SQRT2_TICKS);
	steps = derech_result_steps(res, 0);
	for (uint32_t i = 0; steps != NULL &&
		i < derech_result_length(res, 0); i++) {
		T_CHECK(tags[steps[i * 2 + 1] * 7 + steps[i * 2]] == 0);
	}
	T_CHECK((uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0) ==
		ref_solve(&rm, &avoider, 0, 1, 6, 1));
	t_validate_result(&rm, &avoider, 0, 1, 6, 1, res, 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_add_penalty_and_split(void)
{
	/* full wilderness column (bit 1): the +10 penalty is unavoidable,
	 * so perceived and true totals must differ by exactly 10 */
	float pass[7 * 3];
	uint64_t tags[7 * 3];
	ref_map rm = { 7, 3, pass, tags };
	derech_profile_desc hiker = t_neutral_desc();
	derech_map *m;
	derech_results *res;

	for (uint32_t i = 0; i < 21; i++) {
		pass[i] = 1.0f;
		tags[i] = 0;
	}
	tags[0 * 7 + 3] = 2;
	tags[1 * 7 + 3] = 2;
	tags[2 * 7 + 3] = 2;
	hiker.tag_add[1] = 10.0f;
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &hiker) == 0);

	res = solve1(m, t_req(0, 1, 6, 1));
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_ticks(res, 0) == 6.0f);
	T_CHECK(derech_result_total_perceived(res, 0) == 16.0f);
	t_validate_result(&rm, &hiker, 0, 1, 6, 1, res, 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_unreachable_and_partial(void)
{
	/* goal (3,3) sealed by a ring of walls */
	float pass[7 * 7];
	ref_map rm = { 7, 7, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;
	const uint32_t *steps;

	for (uint32_t i = 0; i < 49; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 2; y <= 4; y++) {
		for (uint32_t x = 2; x <= 4; x++) {
			if (x != 3 || y != 3) {
				pass[y * 7 + x] = 0.0f;
			}
		}
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	q = t_req(0, 3, 3, 3);
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 0) == 0);
	/* component labels answer this without a sweep since v0.3 */
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);

	q.flags = DERECH_REQ_ALLOW_PARTIAL;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 0) == 1);
	steps = derech_result_steps(res, 0);
	/* deterministic closest approach: (1,3), one step east */
	T_CHECK(steps != NULL && steps[0] == 1 && steps[1] == 3);
	t_validate_result(&rm, &d, 0, 3, 3, 3, res, 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_blocked_goal_fast_path(void)
{
	derech_map *m = open_map(5, 5);
	derech_results *res;
	derech_request q;

	T_CHECK(derech_map_set_passability_at(m, 4, 4, 0.0f) == DERECH_OK);
	q = t_req(0, 0, 4, 4);
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0); /* no search ran */
	derech_results_destroy(res);

	/* with ALLOW_PARTIAL it walks up next to the blocked goal; the
	 * straight neighbors (4,3)/(3,4) beat the diagonal one on h, and
	 * (4,3) wins the index tie-break */
	q.flags = DERECH_REQ_ALLOW_PARTIAL;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 0) == 4);
	{
		const uint32_t *steps = derech_result_steps(res, 0);

		T_CHECK(steps != NULL && steps[3 * 2] == 4 &&
			steps[3 * 2 + 1] == 3);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_blocked_start_escapable(void)
{
	derech_map *m = open_map(5, 1);
	derech_results *res;

	T_CHECK(derech_map_set_passability_at(m, 0, 0, 0.0f) == DERECH_OK);
	res = solve1(m, t_req(0, 0, 4, 0));
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 4);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_expansion_budget(void)
{
	derech_map *m = open_map(9, 1);
	derech_results *res;
	derech_request q = t_req(0, 0, 8, 0);

	q.max_expansions = 3;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(res, 0) == 0);
	T_CHECK(derech_result_expansions(res, 0) == 3);
	derech_results_destroy(res);

	q.flags = DERECH_REQ_ALLOW_PARTIAL;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(res, 0) == 2); /* start,(1,0),(2,0) */
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_cost_budget(void)
{
	derech_map *m = open_map(9, 1);
	derech_results *res;
	derech_request q = t_req(0, 0, 8, 0);

	q.max_perceived_cost = 3.0f;
	q.flags = DERECH_REQ_ALLOW_PARTIAL;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(res, 0) == 3);
	T_CHECK(derech_result_total_perceived(res, 0) == 3.0f);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_unreachable_vs_cost_budget(void)
{
	/* sealed goal + generous cap: must still say UNREACHABLE, not
	 * budget-exceeded */
	float pass[7 * 7];
	ref_map rm = { 7, 7, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;

	for (uint32_t i = 0; i < 49; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 2; y <= 4; y++) {
		for (uint32_t x = 2; x <= 4; x++) {
			if (x != 3 || y != 3) {
				pass[y * 7 + x] = 0.0f;
			}
		}
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(0, 3, 3, 3);
	q.max_perceived_cost = 100.0f;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_epsilon_bound(void)
{
	/* expensive band across the beeline: eps=1 must find the exact
	 * optimum; eps=2 may cut through but must stay within 2x */
	float pass[9 * 9];
	ref_map rm = { 9, 9, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res;
	derech_request q;
	uint64_t want, got;

	for (uint32_t i = 0; i < 81; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t x = 1; x < 8; x++) {
		pass[4 * 9 + x] = 0.125f;
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	want = ref_solve(&rm, &d, 4, 0, 4, 8);
	T_CHECK(want != REF_INF);

	q = t_req(4, 0, 4, 8);
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	got = (uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0);
	T_CHECK(got == want);
	t_validate_result(&rm, &d, 4, 0, 4, 8, res, 0);
	derech_results_destroy(res);

	q.epsilon = 2.0f;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	got = (uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0);
	T_CHECK(got >= want);
	T_CHECK(got <= (want * 512) >> 8);
	t_validate_result(&rm, &d, 4, 0, 4, 8, res, 0);
	derech_results_destroy(res);

	/* default epsilon (0) is accepted and stays within 1.25x */
	q.epsilon = 0.0f;
	res = solve1(m, q);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	got = (uint64_t)llround(
		(double)derech_result_total_perceived(res, 0) * 256.0);
	T_CHECK(got >= want);
	T_CHECK(got <= (want * 320) >> 8);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_invalid_endpoints(void)
{
	derech_map *m = open_map(4, 4);
	derech_request q[2];
	derech_results *res = NULL;

	q[0] = t_req(9, 0, 3, 3);
	q[1] = t_req(0, 0, 3, 9);
	T_CHECK(derech_find_paths(m, q, 2, &res) == DERECH_OK);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_INVALID_ENDPOINT);
	T_CHECK(derech_result_status(res, 1) ==
		DERECH_PATH_INVALID_ENDPOINT);
	T_CHECK(derech_result_length(res, 0) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_request_validation(void)
{
	derech_map *m = open_map(4, 4);
	derech_request q = t_req(0, 0, 3, 3);
	derech_results *res = NULL;

	q.epsilon = 0.5f;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);
	T_CHECK(res == NULL);
	q.epsilon = nanf("");
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);
	q.epsilon = 300.0f;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);

	q = t_req(0, 0, 3, 3);
	q.flags = 1u << 7;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);

	q = t_req(0, 0, 3, 3);
	q.max_perceived_cost = -1.0f;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);
	q.max_perceived_cost = nanf("");
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);

	q = t_req(0, 0, 3, 3);
	q.profile_id = 7;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_BAD_PROFILE);

	derech_map_destroy(m);
}

int main(void)
{
	test_trivial();
	test_straight_line();
	test_diagonal();
	test_conn4();
	test_wall_detour();
	test_corner_rules();
	test_corner_squeeze();
	test_weighted_choice();
	test_block_and_require_masks();
	test_require_mask();
	test_mult_preference();
	test_add_penalty_and_split();
	test_unreachable_and_partial();
	test_blocked_goal_fast_path();
	test_blocked_start_escapable();
	test_expansion_budget();
	test_cost_budget();
	test_unreachable_vs_cost_budget();
	test_epsilon_bound();
	test_invalid_endpoints();
	test_request_validation();
	return t_done("test_search");
}
