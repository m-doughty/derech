/* Deterministic allocation-failure coverage for required allocations.
 *
 * This test links a private DERECH_TESTING build of the core.  The fault
 * hook applies only to library translation units, so this harness and the
 * shared test support continue to use libc normally. */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "derech_internal.h"
#include "support.h"

static derech_map_opts one_thread_opts(void)
{
	derech_map_opts opts;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	return opts;
}

static derech_map *open_map(uint32_t w, uint32_t h)
{
	derech_map_opts opts = one_thread_opts();
	derech_profile_desc desc = t_neutral_desc();
	derech_map *map = NULL;

	derech_test_alloc_disable();
	T_CHECK(derech_map_create_ex(w, h, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	if (map != NULL) {
		T_CHECK(derech_profile_register(map, &desc) == 0);
	}
	return map;
}

static void check_found(derech_results *res)
{
	T_CHECK(res != NULL);
	if (res != NULL) {
		T_CHECK(derech_results_count(res) == 1);
		T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_length(res, 0) > 0);
	}
}

static void retry_found(derech_map *map, const derech_request *req)
{
	derech_results *res = NULL;

	derech_test_alloc_disable();
	T_CHECK(derech_find_paths(map, req, 1, &res) == DERECH_OK);
	check_found(res);
	derech_results_destroy(res);
}

static uint64_t baseline_astar_allocations(void)
{
	derech_map *map = open_map(9, 7);
	derech_request req = t_req(0, 0, 8, 6);
	derech_results *res = NULL;
	uint64_t attempts = 0;

	req.flags = DERECH_REQ_ALLOW_PARTIAL; /* skip optional label cache */
	if (map != NULL) {
		derech_test_alloc_disable();
		T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_OK);
		attempts = derech_test_alloc_attempts();
		check_found(res);
	}
	derech_results_destroy(res);
	derech_map_destroy(map);
	derech_test_alloc_disable();
	return attempts;
}

static void test_create_ex_failures(void)
{
	derech_map_opts opts = one_thread_opts();
	derech_map *map = (derech_map *)(uintptr_t)1;
	uint64_t attempts;

	derech_test_alloc_disable();
	T_CHECK(derech_map_create_ex(0, 4, &opts, &map) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(map == NULL);
	T_CHECK(derech_test_alloc_attempts() == 0);

	T_CHECK(derech_map_create_ex(4, 4, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	attempts = derech_test_alloc_attempts();
	T_CHECK(attempts >= 6);
	derech_map_destroy(map);

	for (uint64_t i = 0; i < attempts; i++) {
		map = (derech_map *)(uintptr_t)1;
		derech_test_alloc_fail_after(i);
		T_CHECK(derech_map_create_ex(4, 4, &opts, &map) ==
			DERECH_E_NOMEM);
		T_CHECK(map == NULL);
		T_CHECK(derech_test_alloc_attempts() >= i + 1);
	}
	derech_test_alloc_disable();
	T_CHECK(derech_map_create(0, 4, &opts) == NULL);

	/* Repeat through pool/thread-bootstrap allocation and partial teardown. */
	opts.n_threads = 2;
	map = NULL;
	derech_test_alloc_disable();
	T_CHECK(derech_map_create_ex(4, 4, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	attempts = derech_test_alloc_attempts();
	T_CHECK(attempts >= 10);
	derech_map_destroy(map);
	for (uint64_t i = 0; i < attempts; i++) {
		map = (derech_map *)(uintptr_t)1;
		derech_test_alloc_fail_after(i);
		T_CHECK(derech_map_create_ex(4, 4, &opts, &map) ==
			DERECH_E_NOMEM);
		T_CHECK(map == NULL);
	}
	derech_test_alloc_disable();
}

static void test_profile_registration_failures(void)
{
	for (uint64_t failure = 0; failure < 2; failure++) {
		derech_map *map = open_map(4, 4);
		derech_profile_desc profile = t_neutral_desc();
		uint64_t generation;

		if (map == NULL) {
			continue;
		}
		profile.block_mask = 1;
		generation = derech_map_generation(map);
		derech_test_alloc_fail_after(failure);
		T_CHECK(derech_profile_register(map, &profile) == DERECH_E_NOMEM);
		T_CHECK(derech_profile_count(map) == 1);
		T_CHECK(derech_map_generation(map) == generation);
		derech_test_alloc_disable();
		T_CHECK(derech_profile_register(map, &profile) == 1);
		derech_map_destroy(map);
	}
}

static void test_goalset_registration_failures(void)
{
	uint32_t xy[2] = { 3, 3 };

	for (uint64_t failure = 0; failure < 3; failure++) {
		derech_map *map = open_map(4, 4);
		uint64_t generation;

		if (map == NULL) {
			continue;
		}
		generation = derech_map_generation(map);
		derech_test_alloc_fail_after(failure);
		T_CHECK(derech_goalset_register(map, xy, 1, 0) ==
			DERECH_E_NOMEM);
		T_CHECK(derech_goalset_count(map, 1) == DERECH_E_BAD_GOALSET);
		T_CHECK(derech_map_generation(map) == generation);
		derech_test_alloc_disable();
		T_CHECK(derech_goalset_register(map, xy, 1, 0) == 1);
		derech_map_destroy(map);
	}
}

static void test_optional_label_failures(void)
{
	derech_map *map = open_map(32, 32);

	if (map == NULL) {
		return;
	}
	for (uint64_t failure = 0; failure < 2; failure++) {
		derech_test_alloc_fail_after(failure);
		T_CHECK(derech_labels_for(map, &map->profiles[0]) == NULL);
		T_CHECK(map->label_class_count == 0);
	}
	derech_test_alloc_disable();
	T_CHECK(derech_labels_for(map, &map->profiles[0]) != NULL);
	derech_labels_finish_call(map, 1);
	T_CHECK(map->label_class_count == 1);
	derech_map_destroy(map);
}

static void test_reverse_field_heap_failure(void)
{
	derech_map *map = open_map(16, 16);
	derech_field *field;
	derech_status error = DERECH_OK;
	derech_request req = t_req(0, 0, 15, 15);

	if (map == NULL) {
		return;
	}
	T_CHECK(derech_contexts_ensure(map, 1) == DERECH_OK);
	field = derech_field_cache_insert(map, 15 * 16 + 15,
		DERECH_NO_GOALSET, 0, &error);
	T_CHECK(field != NULL);
	T_CHECK(error == DERECH_OK);
	if (field == NULL) {
		derech_map_destroy(map);
		return;
	}
	map->ctxs[0].heap_cap = 1;
	derech_test_alloc_fail_after(0);
	derech_field_build(map, &map->ctxs[0], field);
	T_CHECK(field->ok == 0);
	T_CHECK(field->error == DERECH_E_NOMEM);
	derech_field_cache_end_wave(map);
	T_CHECK(map->field_bytes == 0);
	retry_found(map, &req);
	derech_map_destroy(map);
}

static void test_astar_required_allocation_sweep(void)
{
	derech_request req = t_req(0, 0, 8, 6);
	uint64_t attempts = baseline_astar_allocations();

	req.flags = DERECH_REQ_ALLOW_PARTIAL;
	/* result + rows/stage + five planning arrays + four context arrays +
	 * two path buffers + two result arenas */
	T_CHECK(attempts >= 16);
	for (uint64_t i = 0; i < attempts; i++) {
		derech_map *map = open_map(9, 7);
		derech_results *res =
			(derech_results *)(uintptr_t)1;

		if (map == NULL) {
			continue;
		}
		derech_test_alloc_fail_after(i);
		{
			derech_status rc = derech_find_paths(map, &req, 1, &res);

			if (rc != DERECH_E_NOMEM) {
				fprintf(stderr, "astar allocation %llu returned %d "
					"after %llu attempts\n",
					(unsigned long long)i, (int)rc,
					(unsigned long long)
						derech_test_alloc_attempts());
			}
			T_CHECK(rc == DERECH_E_NOMEM);
		}
		T_CHECK(res == NULL);
		retry_found(map, &req);
		derech_map_destroy(map);
	}
	derech_test_alloc_disable();
}

/* With an initialized context and warm output buffers, the first eight
 * find allocations are the result/stage/planning scaffolding. */
#define WARM_SEARCH_PREFIX_ALLOCS 8u

static void test_astar_heap_growth_failure(void)
{
	derech_map *map = open_map(9, 9);
	derech_request req = t_req(4, 4, 8, 8);
	derech_results *res = NULL;

	req.flags = DERECH_REQ_ALLOW_PARTIAL;
	if (map == NULL) {
		return;
	}
	retry_found(map, &req); /* allocate the context and output buffers */
	T_CHECK(map->allocated_contexts == 1);
	map->ctxs[0].heap_cap = 1; /* force the second open-list push to grow */

	derech_test_alloc_fail_after(WARM_SEARCH_PREFIX_ALLOCS);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	T_CHECK(map->ctxs[0].heap_cap == 1);
	retry_found(map, &req);
	T_CHECK(map->ctxs[0].heap_cap > 1);
	derech_map_destroy(map);
}

static void clear_output_buffers(derech_search_ctx *ctx)
{
	free(ctx->out_steps);
	free(ctx->out_ticks);
	ctx->out_steps = NULL;
	ctx->out_ticks = NULL;
	ctx->out_len = 0;
	ctx->out_cap = 0;
}

static void test_path_buffer_failures(void)
{
	derech_map *map = open_map(9, 9);
	derech_request req = t_req(0, 0, 8, 8);
	derech_results *res = NULL;

	req.flags = DERECH_REQ_ALLOW_PARTIAL;
	if (map == NULL) {
		return;
	}
	retry_found(map, &req);
	clear_output_buffers(&map->ctxs[0]);

	/* First path-buffer allocation (interleaved steps). */
	derech_test_alloc_fail_after(WARM_SEARCH_PREFIX_ALLOCS);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	T_CHECK(map->ctxs[0].out_steps == NULL);
	retry_found(map, &req);

	/* The new steps buffer succeeds, then the tick buffer fails.  Neither
	 * replacement is published. */
	clear_output_buffers(&map->ctxs[0]);
	derech_test_alloc_fail_after(WARM_SEARCH_PREFIX_ALLOCS + 1);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	T_CHECK(map->ctxs[0].out_steps == NULL);
	T_CHECK(map->ctxs[0].out_ticks == NULL);
	T_CHECK(map->ctxs[0].out_cap == 0);
	retry_found(map, &req);
	derech_map_destroy(map);
}

static void test_result_arena_failures(void)
{
	derech_map *map = open_map(9, 9);
	derech_request req = t_req(0, 0, 8, 8);
	derech_results *res = NULL;

	req.flags = DERECH_REQ_ALLOW_PARTIAL;
	if (map == NULL) {
		return;
	}
	retry_found(map, &req); /* context and output buffers are now warm */

	/* No search allocation is needed, so these are the two final arenas. */
	derech_test_alloc_fail_after(WARM_SEARCH_PREFIX_ALLOCS);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	retry_found(map, &req);

	derech_test_alloc_fail_after(WARM_SEARCH_PREFIX_ALLOCS + 1);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	retry_found(map, &req);
	derech_map_destroy(map);
}

static void test_tag_interner_rollback(void)
{
	derech_map *map = open_map(64, 1);
	uint64_t initial[64];
	uint64_t update[2] = { UINT64_C(1001), UINT64_C(1002) };
	uint64_t generation;
	uint64_t observed = UINT64_MAX;
	uint32_t committed_count;
	derech_profile_desc second_profile = t_neutral_desc();

	if (map == NULL) {
		return;
	}
	for (uint32_t i = 0; i < 64; i++) {
		/* 0 already exists; this commits 62 additional combinations. */
		initial[i] = i < 63 ? i : 0;
	}
	derech_test_alloc_disable();
	T_CHECK(derech_map_set_tags(map, initial, 64) == DERECH_OK);
	committed_count = map->combo_count;
	T_CHECK(committed_count == 63);
	T_CHECK(derech_profile_register(map, &second_profile) == 1);
	generation = derech_map_generation(map);

	/* Scratch succeeds, the first new word fills slot 63, combo_words and
	 * profile 0 grow, then profile 1 growth fails. */
	derech_test_alloc_fail_after(3);
	T_CHECK(derech_map_set_tags_rect(map, 0, 0, 2, 1, update) ==
		DERECH_E_NOMEM);
	T_CHECK(map->combo_count == committed_count);
	T_CHECK(derech_map_generation(map) == generation);
	T_CHECK(derech_map_get_tags_at(map, 0, 0, &observed) == DERECH_OK);
	T_CHECK(observed == initial[0]);

	derech_test_alloc_disable();
	T_CHECK(derech_map_set_tags_rect(map, 0, 0, 2, 1, update) == DERECH_OK);
	T_CHECK(map->combo_count == committed_count + 2);
	T_CHECK(derech_map_get_tags_at(map, 0, 0, &observed) == DERECH_OK);
	T_CHECK(observed == update[0]);
	derech_map_destroy(map);
}

static derech_map *goalset_map(uint32_t *out_set)
{
	derech_map *map = open_map(16, 16);
	uint32_t xy[2] = { 15, 15 };
	int32_t set_id;

	if (map == NULL) {
		return NULL;
	}
	set_id = derech_goalset_register(map, xy, 1, 0);
	T_CHECK(set_id == 1);
	if (set_id < 1) {
		derech_map_destroy(map);
		return NULL;
	}
	*out_set = (uint32_t)set_id;
	return map;
}

static derech_request goalset_request(uint32_t set_id)
{
	derech_request req = t_req(0, 0, 0, 0);

	req.goalset = set_id;
	return req;
}

static uint64_t baseline_goalset_allocations(void)
{
	uint32_t set_id = 0;
	derech_map *map = goalset_map(&set_id);
	derech_request req = goalset_request(set_id);
	derech_results *res = NULL;
	uint64_t attempts = 0;

	if (map != NULL) {
		derech_test_alloc_disable();
		T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_OK);
		attempts = derech_test_alloc_attempts();
		check_found(res);
		T_CHECK(map->field_bytes > 0);
	}
	derech_results_destroy(res);
	derech_map_destroy(map);
	derech_test_alloc_disable();
	return attempts;
}

static void test_required_field_rollback_sweep(void)
{
	uint64_t attempts = baseline_goalset_allocations();

	/* Includes the field shell, distance/next arrays, context, output,
	 * and final result arenas. */
	T_CHECK(attempts >= 19);
	for (uint64_t i = 0; i < attempts; i++) {
		uint32_t set_id = 0;
		derech_map *map = goalset_map(&set_id);
		derech_request req = goalset_request(set_id);
		derech_results *res =
			(derech_results *)(uintptr_t)1;

		if (map == NULL) {
			continue;
		}
		derech_test_alloc_fail_after(i);
		{
			derech_status rc = derech_find_paths(map, &req, 1, &res);

			if (rc != DERECH_E_NOMEM) {
				fprintf(stderr, "field allocation %llu returned %d "
					"after %llu attempts\n",
					(unsigned long long)i, (int)rc,
					(unsigned long long)
						derech_test_alloc_attempts());
			}
			T_CHECK(rc == DERECH_E_NOMEM);
		}
		T_CHECK(res == NULL);
		T_CHECK(map->field_bytes == 0);
		T_CHECK(map->field_lru_head == NULL);
		T_CHECK(map->field_lru_tail == NULL);

		retry_found(map, &req);
		T_CHECK(map->field_bytes > 0);
		T_CHECK(map->field_lru_head != NULL);
		derech_map_destroy(map);
	}
	derech_test_alloc_disable();
}

static derech_map *two_goalset_map(uint32_t *set_a, uint32_t *set_b,
	derech_request *req_a, derech_request *req_b)
{
	derech_map *map = open_map(20, 20);
	uint32_t a[2] = { 19, 19 };
	uint32_t b[2] = { 0, 19 };
	int32_t a_id, b_id;

	if (map == NULL) {
		return NULL;
	}
	a_id = derech_goalset_register(map, a, 1, 0);
	b_id = derech_goalset_register(map, b, 1, 0);
	T_CHECK(a_id == 1);
	T_CHECK(b_id == 2);
	if (a_id < 1 || b_id < 1) {
		derech_map_destroy(map);
		return NULL;
	}
	*set_a = (uint32_t)a_id;
	*set_b = (uint32_t)b_id;
	*req_a = goalset_request(*set_a);
	*req_b = goalset_request(*set_b);
	return map;
}

static uint64_t second_field_allocations(void)
{
	uint32_t set_a = 0, set_b = 0;
	derech_request req_a, req_b;
	derech_map *map = two_goalset_map(&set_a, &set_b, &req_a, &req_b);
	derech_results *res = NULL;
	uint64_t attempts = 0;

	(void)set_a;
	(void)set_b;
	if (map != NULL) {
		retry_found(map, &req_a);
		derech_test_alloc_disable();
		T_CHECK(derech_find_paths(map, &req_b, 1, &res) == DERECH_OK);
		attempts = derech_test_alloc_attempts();
		check_found(res);
	}
	derech_results_destroy(res);
	derech_map_destroy(map);
	derech_test_alloc_disable();
	return attempts;
}

static void test_failed_batch_preserves_old_field(void)
{
	uint64_t attempts = second_field_allocations();
	uint32_t set_a = 0, set_b = 0;
	derech_request req_a, req_b;
	derech_map *map = two_goalset_map(&set_a, &set_b, &req_a, &req_b);
	derech_results *res = NULL;
	derech_field *old_field;
	uint64_t old_bytes;

	(void)set_a;
	(void)set_b;
	T_CHECK(attempts >= 13);
	if (map == NULL || attempts == 0) {
		derech_map_destroy(map);
		return;
	}
	retry_found(map, &req_a);
	old_field = map->field_lru_head;
	old_bytes = map->field_bytes;
	T_CHECK(old_field != NULL);

	/* The final allocation is the result tick arena, after the new field
	 * has been built and extracted.  Rollback must discard only that field. */
	derech_test_alloc_fail_after(attempts - 1);
	T_CHECK(derech_find_paths(map, &req_b, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	T_CHECK(map->field_bytes == old_bytes);
	T_CHECK(map->field_lru_head == old_field);
	T_CHECK(map->field_lru_tail == old_field);

	retry_found(map, &req_b);
	T_CHECK(map->field_bytes == old_bytes * 2);
	T_CHECK(map->field_lru_head != old_field);
	derech_map_destroy(map);
}

static void test_predicate_refresh_rollback(void)
{
	derech_map *map = open_map(16, 1);
	derech_request req = t_req(0, 0, 0, 0);
	derech_results *res = NULL;
	int32_t set_id;
	derech_field *old_field;
	uint64_t *old_members;
	uint64_t old_epoch;
	uint64_t old_field_bytes;

	if (map == NULL) {
		return;
	}
	T_CHECK(derech_map_set_tags_at(map, 15, 0, 1) == DERECH_OK);
	set_id = derech_goalset_register_tags(map, 1, 0, 0);
	T_CHECK(set_id == 1);
	if (set_id != 1) {
		derech_map_destroy(map);
		return;
	}
	req.goalset = (uint32_t)set_id;
	retry_found(map, &req);
	old_field = map->field_lru_head;
	old_field_bytes = map->field_bytes;
	old_members = map->goalsets[0].members;
	old_epoch = map->goalsets[0].epoch;
	T_CHECK(map->goalsets[0].member_count == 1);

	T_CHECK(derech_map_set_tags_at(map, 14, 0, 1) == DERECH_OK);
	/* result, rows, and stage allocate before transactional rematerialization */
	derech_test_alloc_fail_after(3);
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_E_NOMEM);
	T_CHECK(res == NULL);
	T_CHECK(map->goalsets[0].members == old_members);
	T_CHECK(map->goalsets[0].epoch == old_epoch);
	T_CHECK(map->goalsets[0].member_count == 1);
	T_CHECK(map->field_lru_head == old_field);
	T_CHECK(map->field_bytes == old_field_bytes);

	derech_test_alloc_disable();
	T_CHECK(derech_find_paths(map, &req, 1, &res) == DERECH_OK);
	check_found(res);
	T_CHECK(derech_result_length(res, 0) == 14);
	T_CHECK(map->goalsets[0].member_count == 2);
	T_CHECK(map->goalsets[0].epoch == old_epoch + 1);
	derech_results_destroy(res);
	derech_map_destroy(map);
}

int main(void)
{
	derech_test_alloc_disable();
	test_create_ex_failures();
	test_profile_registration_failures();
	test_goalset_registration_failures();
	test_optional_label_failures();
	test_reverse_field_heap_failure();
	test_astar_required_allocation_sweep();
	test_astar_heap_growth_failure();
	test_path_buffer_failures();
	test_result_arena_failures();
	test_tag_interner_rollback();
	test_required_field_rollback_sweep();
	test_failed_batch_preserves_old_field();
	test_predicate_refresh_rollback();
	derech_test_alloc_disable();
	return t_done("oom");
}
