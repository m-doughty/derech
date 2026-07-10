/* Cumulative costs beyond 32-bit Q8, exact accessors, fields, and caps. */

#include <stdint.h>
#include <string.h>

#include "support.h"

static derech_request request(uint32_t sx, uint32_t sy, uint32_t gx,
	uint32_t gy)
{
	derech_request q;

	memset(&q, 0, sizeof(q));
	q.struct_size = (uint32_t)sizeof(q);
	q.start_x = sx;
	q.start_y = sy;
	q.goal_x = gx;
	q.goal_y = gy;
	q.epsilon = 1.0f;
	return q;
}

static derech_map *corridor_map(void)
{
	enum { W = 257 };
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	float pass[W];

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_group_threshold = 4;
	map = derech_map_create(W, 1, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return NULL;
	}
	for (uint32_t i = 0; i < W; i++) {
		pass[i] = 1.0f / 65536.0f;
	}
	T_CHECK(derech_map_set_passability(map, pass, W) == DERECH_OK);
	T_CHECK(derech_profile_register(map, &profile) == 0);
	return map;
}

static void check_corridor_result(const derech_results *results, uint32_t i,
	uint32_t expansions)
{
	const uint64_t expected_q = UINT64_C(1) << 32;

	T_CHECK(derech_result_status(results, i) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(results, i) == 256);
	T_CHECK(derech_result_total_ticks_q8(results, i) == expected_q);
	T_CHECK(derech_result_total_perceived_q8(results, i) == expected_q);
	T_CHECK(derech_result_total_ticks(results, i) == 16777216.0f);
	T_CHECK(derech_result_total_perceived(results, i) == 16777216.0f);
	if (expansions == 0) {
		T_CHECK(derech_result_expansions(results, i) == 0);
	} else {
		T_CHECK(derech_result_expansions(results, i) > 0);
	}
}

static void test_classic_field_and_cache(void)
{
	derech_map *map = corridor_map();
	derech_request q = request(0, 0, 256, 0);
	derech_request batch[4];
	derech_results *results = NULL;

	if (map == NULL) {
		return;
	}
	/* One request is below the field threshold: exercise classic A*. */
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	check_corridor_result(results, 0, 1);
	derech_results_destroy(results);

	/* Four same-goal requests build one exact reverse field. */
	for (uint32_t i = 0; i < 4; i++) {
		batch[i] = q;
	}
	results = NULL;
	T_CHECK(derech_find_paths(map, batch, 4, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	for (uint32_t i = 0; i < 4; i++) {
		check_corridor_result(results, i, 0);
	}
	derech_results_destroy(results);

	/* A later singleton must use the cached field and retain the exact sum. */
	results = NULL;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	check_corridor_result(results, 0, 0);
	derech_results_destroy(results);

	T_CHECK(derech_result_total_ticks_q8(NULL, 0) == 0);
	T_CHECK(derech_result_total_perceived_q8(NULL, 0) == 0);
	derech_map_destroy(map);
}

static void test_cost_caps_above_old_limit(void)
{
	derech_map *map = corridor_map();
	derech_request q = request(0, 0, 256, 0);
	derech_results *results = NULL;

	if (map == NULL) {
		return;
	}
	/* Exactly 2^32 Q8 is now a valid cap and a valid path total. */
	q.max_perceived_cost = 16777216.0f;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	check_corridor_result(results, 0, 1);
	derech_results_destroy(results);

	/* One maximum-cost step less must prune the only route. */
	q.max_perceived_cost = 16711680.0f;
	results = NULL;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_total_perceived_q8(results, 0) == 0);
	derech_results_destroy(results);
	derech_map_destroy(map);
}

static void test_uint32_max_single_step(void)
{
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	derech_request q = request(0, 0, 1, 0);
	derech_request batch[4];
	derech_results *results = NULL;
	float pass[2] = { 1.0f, 1.0f / 65536.0f };
	uint64_t tags[2] = { 0, 1 };

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_group_threshold = 4;
	map = derech_map_create(2, 1, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_map_set_passability(map, pass, 2) == DERECH_OK);
	T_CHECK(derech_map_set_tags(map, tags, 2) == DERECH_OK);
	profile.tag_mult[0] = 65536.0f;
	T_CHECK(derech_profile_register(map, &profile) == 0);

	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(results, 0) == 1);
	T_CHECK(derech_result_total_ticks_q8(results, 0) ==
		(UINT64_C(1) << 24));
	T_CHECK(derech_result_total_perceived_q8(results, 0) ==
		DERECH_MAX_STEP_COST_Q8);
	T_CHECK(derech_result_expansions(results, 0) > 0);
	derech_results_destroy(results);

	for (uint32_t i = 0; i < 4; i++) {
		batch[i] = q;
	}
	results = NULL;
	T_CHECK(derech_find_paths(map, batch, 4, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	for (uint32_t i = 0; i < 4; i++) {
		T_CHECK(derech_result_status(results, i) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_total_perceived_q8(results, i) ==
			DERECH_MAX_STEP_COST_Q8);
		T_CHECK(derech_result_expansions(results, i) == 0);
	}
	derech_results_destroy(results);
	derech_map_destroy(map);
}

static void test_epsilon_bound_above_32_bits(void)
{
	derech_map *map = corridor_map();
	derech_request q = request(0, 0, 256, 0);
	derech_results *results = NULL;
	const uint64_t optimum = UINT64_C(1) << 32;

	if (map == NULL) {
		return;
	}
	q.epsilon = 1.25f;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_perceived_q8(results, 0) >= optimum);
	T_CHECK(derech_result_total_perceived_q8(results, 0) <=
		(optimum * 5) / 4);
	derech_results_destroy(results);
	derech_map_destroy(map);
}

int main(void)
{
	test_classic_field_and_cache();
	test_cost_caps_above_old_limit();
	test_uint32_max_single_step();
	test_epsilon_bound_above_32_bits();
	return t_done("extreme costs");
}
