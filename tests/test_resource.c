/* Goal-set normalization, memory limits/statistics, and cancellation. */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "derech_thread.h"
#include "support.h"

#define MIB (UINT64_C(1024) * 1024)

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

static void test_duplicate_goal_members(void)
{
	derech_map *map = derech_map_create(7, 3, NULL);
	derech_profile_desc profile = t_neutral_desc();
	uint32_t xy[8] = { 2, 1, 2, 1, 5, 1, 5, 1 };
	int32_t normal;
	int32_t adjacent;
	derech_request q;
	derech_results *results = NULL;

	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	normal = derech_goalset_register(map, xy, 4, 0);
	adjacent = derech_goalset_register(map, xy, 4,
		DERECH_GOALSET_ADJACENT);
	T_CHECK(normal == 1);
	T_CHECK(adjacent == 2);
	T_CHECK(derech_goalset_count(map, (uint32_t)normal) == 2);
	T_CHECK(derech_goalset_count(map, (uint32_t)adjacent) == 2);

	q = request(0, 1, 0, 0);
	q.goalset = (uint32_t)normal;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(results, 0) == 2);
	derech_results_destroy(results);

	q.goalset = (uint32_t)adjacent;
	results = NULL;
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(results, 0) == 1);
	derech_results_destroy(results);
	derech_map_destroy(map);
}

static void test_memory_estimates_and_stats(void)
{
	derech_map_opts opts;
	derech_memory_stats estimate;
	derech_memory_stats stats;
	derech_map *map = NULL;
	derech_profile_desc profile = t_neutral_desc();
	derech_request q = request(0, 0, 63, 63);
	derech_results *results = NULL;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 4;
	opts.worker_memory_mb = 64;
	opts.field_cache_mb = 7;
	opts.field_working_mb = 9;
	opts.scratch_retention_mb = 1;
	opts.label_cache_mb = 3;

	memset(&estimate, 0, sizeof(estimate));
	estimate.struct_size = (uint32_t)sizeof(estimate);
	T_CHECK(derech_map_memory_estimate(64, 64, &opts, &estimate) ==
		DERECH_OK);
	T_CHECK(estimate.configured_threads == 4);
	T_CHECK(estimate.terrain_bytes > 0);
	T_CHECK(estimate.worker_bytes > 0);
	T_CHECK(estimate.field_cache_bytes == 7 * MIB);
	T_CHECK(estimate.field_working_bytes == 9 * MIB);
	T_CHECK(estimate.label_cache_bytes == 3 * MIB);

	memset(&stats, 0, sizeof(stats));
	T_CHECK(derech_map_memory_estimate(64, 64, &opts, &stats) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_memory_estimate(0, 64, &opts, &estimate) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_memory_estimate(64, 64, &opts, NULL) ==
		DERECH_E_INVALID_ARG);

	T_CHECK(derech_map_create_ex(64, 64, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.configured_threads == 4);
	T_CHECK(stats.terrain_bytes == estimate.terrain_bytes);
	T_CHECK(stats.field_cache_bytes == 7 * MIB);
	T_CHECK(stats.field_working_bytes == 9 * MIB);
	T_CHECK(stats.label_cache_bytes == 3 * MIB);
	T_CHECK(stats.allocated_contexts <= stats.configured_threads);

	T_CHECK(derech_profile_register(map, &profile) == 0);
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	derech_results_destroy(results);
	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.allocated_contexts >= 1);
	T_CHECK(stats.allocated_contexts <= 4);
	T_CHECK(stats.worker_bytes > 0);
	T_CHECK(stats.retained_scratch_bytes <= 4 * MIB);
	derech_map_destroy(map);
}

static void test_worker_memory_limit(void)
{
	derech_map_opts opts;
	derech_map *map = (derech_map *)(uintptr_t)1;
	derech_status rc;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 4;
	opts.worker_memory_mb = 4;
	rc = derech_map_create_ex(512, 512, &opts, &map);
	T_CHECK(rc == DERECH_E_RESOURCE_LIMIT);
	T_CHECK(map == NULL);

	/* Auto thread selection must reduce parallelism to fit the budget. */
	opts.n_threads = 0;
	map = NULL;
	T_CHECK(derech_map_create_ex(512, 512, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	if (map != NULL) {
		T_CHECK(derech_map_thread_count(map) == 1);
	}
	derech_map_destroy(map);
}

static void test_field_working_limit(void)
{
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	uint32_t xy[2] = { 511, 511 };
	int32_t set_id;
	derech_request q = request(0, 0, 511, 511);
	derech_results *results = (derech_results *)(uintptr_t)1;
	derech_status rc;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_cache_mb = 1;
	opts.field_working_mb = 1;
	map = derech_map_create(512, 512, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	set_id = derech_goalset_register(map, xy, 1, 0);
	T_CHECK(set_id == 1);
	q.goalset = (uint32_t)set_id;
	rc = derech_find_paths(map, &q, 1, &results);
	T_CHECK(rc == DERECH_E_RESOURCE_LIMIT);
	T_CHECK(results == NULL);
	if (rc != DERECH_E_RESOURCE_LIMIT) {
		derech_results_destroy(results);
		results = NULL;
	}

	/* A single-goal crowd may fall back to A* when its field will not fit. */
	{
		derech_request batch[4];

		q.goalset = DERECH_NO_GOALSET;
		for (uint32_t i = 0; i < 4; i++) {
			batch[i] = q;
		}
		results = NULL;
		T_CHECK(derech_find_paths(map, batch, 4, &results) == DERECH_OK);
		T_CHECK(results != NULL);
		if (results != NULL) {
			for (uint32_t i = 0; i < 4; i++) {
				T_CHECK(derech_result_status(results, i) ==
					DERECH_PATH_FOUND);
				T_CHECK(derech_result_expansions(results, i) > 0);
			}
		}
		derech_results_destroy(results);
	}
	derech_map_destroy(map);
}

static void test_field_waves(void)
{
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	derech_request batch[8];
	derech_results *results = NULL;
	derech_memory_stats stats;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 2;
	opts.field_cache_mb = 1;
	opts.field_working_mb = 1;
	opts.field_group_threshold = 4;
	map = derech_map_create(256, 256, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		batch[i * 2] = request(0, 0, 255, 255);
		batch[i * 2 + 1] = request(0, 255, 255, 0);
	}
	T_CHECK(derech_find_paths(map, batch, 8, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	if (results != NULL) {
		for (uint32_t i = 0; i < 8; i++) {
			T_CHECK(derech_result_status(results, i) ==
				DERECH_PATH_FOUND);
			T_CHECK(derech_result_expansions(results, i) == 0);
		}
	}
	derech_results_destroy(results);

	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.field_peak_bytes <= stats.field_working_bytes);
	T_CHECK(stats.field_bytes <= stats.field_cache_bytes);
	derech_map_destroy(map);
}

static void test_tight_field_cache_replacement(void)
{
	enum { SIDE = 128, SETS = 10 };
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	derech_memory_stats stats;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_cache_mb = 1;
	opts.field_working_mb = 1;
	map = derech_map_create(SIDE, SIDE, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	for (uint32_t i = 0; i < SETS; i++) {
		uint32_t xy[2] = { SIDE - 1, i };
		int32_t set_id = derech_goalset_register(map, xy, 1, 0);
		derech_request q = request(0, SIDE - 1, 0, 0);
		derech_results *results = NULL;

		T_CHECK(set_id == (int32_t)i + 1);
		q.goalset = (uint32_t)set_id;
		T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
		T_CHECK(results != NULL);
		if (results != NULL) {
			T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
		}
		derech_results_destroy(results);
	}
	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.field_peak_bytes <= stats.field_working_bytes);
	T_CHECK(stats.field_bytes <= stats.field_cache_bytes);
	derech_map_destroy(map);
}

static void test_label_cache_lru(void)
{
	derech_map_opts opts;
	derech_map *map;
	derech_request batch[6];
	derech_results *results = NULL;
	derech_memory_stats stats;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.label_cache_mb = 1;
	map = derech_map_create(256, 256, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	for (uint32_t i = 0; i < 6; i++) {
		derech_profile_desc profile = t_neutral_desc();

		profile.block_mask = UINT64_C(1) << i;
		T_CHECK(derech_profile_register(map, &profile) == (int32_t)i);
		batch[i] = request(0, 0, 1, 0);
		batch[i].profile_id = i;
	}
	T_CHECK(derech_find_paths(map, batch, 6, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	if (results != NULL) {
		for (uint32_t i = 0; i < 6; i++) {
			T_CHECK(derech_result_status(results, i) ==
				DERECH_PATH_FOUND);
		}
	}
	derech_results_destroy(results);

	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.label_bytes > 0);
	T_CHECK(stats.label_bytes <= stats.label_cache_bytes);
	T_CHECK(stats.label_cache_bytes == MIB);
	derech_map_destroy(map);
}

typedef struct cancel_ctx {
	derech_map *map;
	derech_cancel *cancel;
	int saw_busy;
} cancel_ctx;

static void cancel_when_running(void *opaque)
{
	cancel_ctx *ctx = opaque;

	for (uint32_t i = 0; i < 1000000; i++) {
		derech_memory_stats stats;
		derech_status rc;

		memset(&stats, 0, sizeof(stats));
		stats.struct_size = (uint32_t)sizeof(stats);
		rc = derech_map_get_memory_stats(ctx->map, &stats);
		if (rc == DERECH_E_BUSY) {
			ctx->saw_busy = 1;
			derech_cancel_request(ctx->cancel);
			return;
		}
	}
	derech_cancel_request(ctx->cancel);
}

static void cancel_when_pool_running(void *opaque)
{
	cancel_ctx *ctx = opaque;
	volatile uint32_t spin = 0;

	for (uint32_t i = 0; i < 1000000; i++) {
		derech_memory_stats stats;

		memset(&stats, 0, sizeof(stats));
		stats.struct_size = (uint32_t)sizeof(stats);
		if (derech_map_get_memory_stats(ctx->map, &stats) == DERECH_E_BUSY) {
			ctx->saw_busy = 1;
			break;
		}
	}
	/* Let the small planning phase dispatch its worker tasks before the
	 * cancellation request.  Volatile keeps this delay in release builds. */
	for (uint32_t i = 0; i < 20000000; i++) {
		spin += i;
	}
	(void)spin;
	derech_cancel_request(ctx->cancel);
}

static void test_cancellation(void)
{
	enum { SIDE = 768 };
	derech_map_opts map_opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	derech_request q = request(0, 0, SIDE - 1, SIDE - 1);
	derech_find_opts find_opts;
	derech_cancel *cancel = NULL;
	derech_results *results = (derech_results *)(uintptr_t)1;
	derech_thread thread;
	cancel_ctx ctx;
	derech_status rc;

	T_CHECK(derech_cancel_create(NULL) == DERECH_E_INVALID_ARG);
	T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
	T_CHECK(cancel != NULL);
	if (cancel == NULL) {
		return;
	}
	memset(&map_opts, 0, sizeof(map_opts));
	map_opts.struct_size = (uint32_t)sizeof(map_opts);
	map_opts.default_passability = 1.0f;
	map_opts.n_threads = 1;
	map = derech_map_create(SIDE, SIDE, &map_opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		derech_cancel_destroy(cancel);
		return;
	}
	/* A tiny declared multiplier makes the heuristic weak without making
	 * the untagged terrain cheap, ensuring enough work for true overlap. */
	profile.tag_mult[0] = 1.0f / 65536.0f;
	T_CHECK(derech_profile_register(map, &profile) == 0);

	memset(&find_opts, 0, sizeof(find_opts));
	find_opts.struct_size = (uint32_t)sizeof(find_opts);
	find_opts.cancel = cancel;
	find_opts.reserved0 = 1;
	T_CHECK(derech_find_paths_ex(map, &q, 1, &find_opts, &results) ==
		DERECH_E_INVALID_ARG);
	find_opts.reserved0 = 0;
	find_opts.struct_size = 0;
	T_CHECK(derech_find_paths_ex(map, &q, 1, &find_opts, &results) ==
		DERECH_E_INVALID_ARG);
	find_opts.struct_size = (uint32_t)sizeof(find_opts);

	/* Already-requested tokens cancel before any result is published. */
	derech_cancel_request(cancel);
	results = (derech_results *)(uintptr_t)1;
	T_CHECK(derech_find_paths_ex(map, &q, 1, &find_opts, &results) ==
		DERECH_E_CANCELLED);
	T_CHECK(results == NULL);
	derech_cancel_destroy(cancel);

	/* A fresh token is requested from a second thread only after that
	 * thread observes the map busy inside the real search call. */
	cancel = NULL;
	T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
	find_opts.cancel = cancel;
	ctx.map = map;
	ctx.cancel = cancel;
	ctx.saw_busy = 0;
	if (!derech_thread_create(&thread, cancel_when_running, &ctx)) {
		T_CHECK(0);
		derech_cancel_destroy(cancel);
		derech_map_destroy(map);
		return;
	}
	T_CHECK(1);
	results = NULL;
	do {
		rc = derech_find_paths_ex(map, &q, 1, &find_opts, &results);
	} while (rc == DERECH_E_BUSY);
	derech_thread_join(thread);
	T_CHECK(ctx.saw_busy != 0);
	T_CHECK(rc == DERECH_E_CANCELLED);
	T_CHECK(results == NULL);

	derech_cancel_destroy(cancel);
	derech_cancel_request(NULL);
	derech_cancel_destroy(NULL);
	derech_map_destroy(map);
}

static void test_field_cancellation(void)
{
	enum { SIDE = 768 };
	derech_map_opts map_opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	uint32_t xy[2] = { SIDE - 1, SIDE - 1 };
	derech_request q = request(0, 0, 0, 0);
	derech_find_opts find_opts;
	derech_cancel *cancel = NULL;
	derech_results *results = NULL;
	derech_thread thread;
	cancel_ctx ctx;
	derech_memory_stats stats;
	derech_status rc;
	int32_t set_id;

	memset(&map_opts, 0, sizeof(map_opts));
	map_opts.struct_size = (uint32_t)sizeof(map_opts);
	map_opts.default_passability = 1.0f;
	map_opts.n_threads = 1;
	map = derech_map_create(SIDE, SIDE, &map_opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	set_id = derech_goalset_register(map, xy, 1, 0);
	T_CHECK(set_id == 1);
	q.goalset = (uint32_t)set_id;
	T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
	if (cancel == NULL) {
		derech_map_destroy(map);
		return;
	}
	memset(&find_opts, 0, sizeof(find_opts));
	find_opts.struct_size = (uint32_t)sizeof(find_opts);
	find_opts.cancel = cancel;
	ctx.map = map;
	ctx.cancel = cancel;
	ctx.saw_busy = 0;
	if (!derech_thread_create(&thread, cancel_when_running, &ctx)) {
		T_CHECK(0);
		derech_cancel_destroy(cancel);
		derech_map_destroy(map);
		return;
	}
	do {
		rc = derech_find_paths_ex(map, &q, 1, &find_opts, &results);
	} while (rc == DERECH_E_BUSY);
	derech_thread_join(thread);
	T_CHECK(ctx.saw_busy != 0);
	T_CHECK(rc == DERECH_E_CANCELLED);
	T_CHECK(results == NULL);

	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.field_bytes == 0);
	derech_cancel_destroy(cancel);

	/* Cancellation rolls back the partial field and leaves the map reusable. */
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	if (results != NULL) {
		T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	}
	derech_results_destroy(results);
	derech_map_destroy(map);
}

static void test_pooled_cancellation_and_reuse(void)
{
	enum { SIDE = 768, COUNT = 8 };
	derech_map_opts map_opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	derech_request batch[COUNT];
	derech_find_opts find_opts;
	derech_cancel *cancel = NULL;
	derech_results *results = NULL;
	derech_thread thread;
	cancel_ctx ctx;
	derech_memory_stats stats;
	derech_status rc;

	memset(&map_opts, 0, sizeof(map_opts));
	map_opts.struct_size = (uint32_t)sizeof(map_opts);
	map_opts.default_passability = 1.0f;
	map_opts.n_threads = 4;
	map = derech_map_create(SIDE, SIDE, &map_opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		return;
	}
	profile.tag_mult[0] = 1.0f / 65536.0f;
	T_CHECK(derech_profile_register(map, &profile) == 0);
	for (uint32_t i = 0; i < COUNT; i++) {
		batch[i] = request(0, 0, SIDE - 1 - i * 17,
			SIDE - 1 - i * 29);
		batch[i].flags = DERECH_REQ_ALLOW_PARTIAL;
	}
	T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
	if (cancel == NULL) {
		derech_map_destroy(map);
		return;
	}
	memset(&find_opts, 0, sizeof(find_opts));
	find_opts.struct_size = (uint32_t)sizeof(find_opts);
	find_opts.cancel = cancel;
	ctx.map = map;
	ctx.cancel = cancel;
	ctx.saw_busy = 0;
	if (!derech_thread_create(&thread, cancel_when_pool_running, &ctx)) {
		T_CHECK(0);
		derech_cancel_destroy(cancel);
		derech_map_destroy(map);
		return;
	}
	do {
		rc = derech_find_paths_ex(map, batch, COUNT, &find_opts, &results);
	} while (rc == DERECH_E_BUSY);
	derech_thread_join(thread);
	T_CHECK(ctx.saw_busy != 0);
	T_CHECK(rc == DERECH_E_CANCELLED);
	T_CHECK(results == NULL);

	memset(&stats, 0, sizeof(stats));
	stats.struct_size = (uint32_t)sizeof(stats);
	T_CHECK(derech_map_get_memory_stats(map, &stats) == DERECH_OK);
	T_CHECK(stats.allocated_contexts == 4);
	derech_cancel_destroy(cancel);

	for (uint32_t i = 0; i < 4; i++) {
		batch[i] = request(0, 0, i + 1, i & 1u);
		batch[i].flags = DERECH_REQ_ALLOW_PARTIAL;
	}
	T_CHECK(derech_find_paths(map, batch, 4, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	if (results != NULL) {
		for (uint32_t i = 0; i < 4; i++) {
			T_CHECK(derech_result_status(results, i) == DERECH_PATH_FOUND);
		}
	}
	derech_results_destroy(results);
	derech_map_destroy(map);
}

int main(void)
{
	test_duplicate_goal_members();
	test_memory_estimates_and_stats();
	test_worker_memory_limit();
	test_field_working_limit();
	test_field_waves();
	test_tight_field_cache_replacement();
	test_label_cache_lru();
	test_cancellation();
	test_field_cancellation();
	test_pooled_cancellation_and_reuse();
	return t_done("resources");
}
