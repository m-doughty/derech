/* Public ABI layout, version epoch, and size/reserved-field validation. */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "support.h"

_Static_assert(DERECH_ABI_VERSION == 1u, "unexpected ABI epoch");
_Static_assert(sizeof(derech_request) == 64, "request ABI must stay 64 bytes");
_Static_assert(offsetof(derech_request, struct_size) == 0,
	"request.struct_size ABI offset");
_Static_assert(offsetof(derech_request, start_x) == 4,
	"request.start_x ABI offset");
_Static_assert(offsetof(derech_request, start_y) == 8,
	"request.start_y ABI offset");
_Static_assert(offsetof(derech_request, goal_x) == 12,
	"request.goal_x ABI offset");
_Static_assert(offsetof(derech_request, goal_y) == 16,
	"request.goal_y ABI offset");
_Static_assert(offsetof(derech_request, profile_id) == 20,
	"request.profile_id ABI offset");
_Static_assert(offsetof(derech_request, flags) == 24,
	"request.flags ABI offset");
_Static_assert(offsetof(derech_request, max_expansions) == 28,
	"request.max_expansions ABI offset");
_Static_assert(offsetof(derech_request, max_perceived_cost) == 32,
	"request.max_perceived_cost ABI offset");
_Static_assert(offsetof(derech_request, epsilon) == 36,
	"request.epsilon ABI offset");
_Static_assert(offsetof(derech_request, goalset) == 40,
	"request.goalset ABI offset");
_Static_assert(offsetof(derech_request, reserved) == 44,
	"request.reserved ABI offset");
_Static_assert(sizeof(derech_map_opts) == 48,
	"map options ABI must stay 48 bytes");
_Static_assert(offsetof(derech_map_opts, struct_size) == 0,
	"map_opts.struct_size ABI offset");
_Static_assert(offsetof(derech_map_opts, default_passability) == 4,
	"map_opts.default_passability ABI offset");
_Static_assert(offsetof(derech_map_opts, default_tags) == 8,
	"map_opts.default_tags ABI offset");
_Static_assert(offsetof(derech_map_opts, n_threads) == 16,
	"map_opts.n_threads ABI offset");
_Static_assert(offsetof(derech_map_opts, field_cache_mb) == 20,
	"map_opts.field_cache_mb ABI offset");
_Static_assert(offsetof(derech_map_opts, field_group_threshold) == 24,
	"map_opts.field_group_threshold ABI offset");
_Static_assert(offsetof(derech_map_opts, worker_memory_mb) == 28,
	"map_opts.worker_memory_mb ABI offset");
_Static_assert(offsetof(derech_map_opts, field_working_mb) == 32,
	"map_opts.field_working_mb ABI offset");
_Static_assert(offsetof(derech_map_opts, scratch_retention_mb) == 36,
	"map_opts.scratch_retention_mb ABI offset");
_Static_assert(offsetof(derech_map_opts, label_cache_mb) == 40,
	"map_opts.label_cache_mb ABI offset");
_Static_assert(offsetof(derech_map_opts, reserved0) == 44,
	"map_opts.reserved0 ABI offset");
_Static_assert(sizeof(derech_memory_stats) == 88,
	"memory stats ABI must stay 88 bytes");
_Static_assert(offsetof(derech_memory_stats, struct_size) == 0,
	"memory_stats.struct_size ABI offset");
_Static_assert(offsetof(derech_memory_stats, configured_threads) == 4,
	"memory_stats.configured_threads ABI offset");
_Static_assert(offsetof(derech_memory_stats, allocated_contexts) == 8,
	"memory_stats.allocated_contexts ABI offset");
_Static_assert(offsetof(derech_memory_stats, reserved0) == 12,
	"memory_stats.reserved0 ABI offset");
_Static_assert(offsetof(derech_memory_stats, terrain_bytes) == 16,
	"memory_stats.terrain_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, worker_bytes) == 24,
	"memory_stats.worker_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, field_bytes) == 32,
	"memory_stats.field_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, field_peak_bytes) == 40,
	"memory_stats.field_peak_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, field_cache_bytes) == 48,
	"memory_stats.field_cache_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, field_working_bytes) == 56,
	"memory_stats.field_working_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, label_bytes) == 64,
	"memory_stats.label_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, label_cache_bytes) == 72,
	"memory_stats.label_cache_bytes ABI offset");
_Static_assert(offsetof(derech_memory_stats, retained_scratch_bytes) == 80,
	"memory_stats.retained_scratch_bytes ABI offset");
_Static_assert(sizeof(derech_profile_desc) == 552,
	"profile descriptor ABI must stay 552 bytes");
_Static_assert(offsetof(derech_profile_desc, struct_size) == 0,
	"profile.struct_size ABI offset");
_Static_assert(offsetof(derech_profile_desc, connectivity) == 4,
	"profile.connectivity ABI offset");
_Static_assert(offsetof(derech_profile_desc, diagonal_mult) == 8,
	"profile.diagonal_mult ABI offset");
_Static_assert(offsetof(derech_profile_desc, reserved2) == 12,
	"profile.reserved2 ABI offset");
_Static_assert(offsetof(derech_profile_desc, block_mask) == 16,
	"profile.block_mask ABI offset");
_Static_assert(offsetof(derech_profile_desc, require_mask) == 24,
	"profile.require_mask ABI offset");
_Static_assert(offsetof(derech_profile_desc, tag_mult) == 32,
	"profile.tag_mult ABI offset");
_Static_assert(offsetof(derech_profile_desc, tag_add) == 288,
	"profile.tag_add ABI offset");
_Static_assert(offsetof(derech_profile_desc, reserved3) == 544,
	"profile.reserved3 ABI offset");
_Static_assert(offsetof(derech_profile_desc, reserved4) == 548,
	"profile.reserved4 ABI offset");
#if UINTPTR_MAX == UINT64_MAX
_Static_assert(sizeof(derech_find_opts) == 16,
	"64-bit find options ABI must stay 16 bytes");
_Static_assert(offsetof(derech_find_opts, cancel) == 8,
	"64-bit find_opts.cancel ABI offset");
#else
_Static_assert(sizeof(derech_find_opts) == 12,
	"32-bit find options ABI must stay 12 bytes");
_Static_assert(offsetof(derech_find_opts, cancel) == 8,
	"32-bit find_opts.cancel ABI offset");
#endif

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

static void test_versions(void)
{
	T_CHECK(DERECH_VERSION_MAJOR == 0);
	T_CHECK(DERECH_VERSION_MINOR == 5);
	T_CHECK(DERECH_VERSION_PATCH == 0);
	T_CHECK(derech_abi_version() == DERECH_ABI_VERSION);
	T_CHECK(derech_version() == ((0u << 16) | (5u << 8) | 0u));
	T_CHECK(strcmp(derech_version_str(), "0.5.0") == 0);
}

static void test_create_ex(void)
{
	derech_map *map = (derech_map *)(uintptr_t)1;
	derech_map_opts opts;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;

	T_CHECK(derech_map_create_ex(2, 2, &opts, NULL) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_create_ex(0, 2, &opts, &map) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(map == NULL);
	map = (derech_map *)(uintptr_t)1;
	opts.struct_size = 7;
	T_CHECK(derech_map_create_ex(2, 2, &opts, &map) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(map == NULL);

	opts.struct_size = (uint32_t)sizeof(opts);
	opts.reserved0 = 1;
	map = (derech_map *)(uintptr_t)1;
	T_CHECK(derech_map_create_ex(2, 2, &opts, &map) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(map == NULL);
	opts.reserved0 = 0;
	T_CHECK(derech_map_create_ex(2, 2, &opts, &map) == DERECH_OK);
	T_CHECK(map != NULL);
	derech_map_destroy(map);
}

static void test_request_validation(void)
{
	derech_map *map = derech_map_create(3, 1, NULL);
	derech_profile_desc profile = t_neutral_desc();
	_Alignas(derech_profile_desc) unsigned char legacy64[544];
	_Alignas(derech_profile_desc) unsigned char legacy32[540];
	derech_request q = request(0, 0, 2, 0);
	derech_results *results = NULL;
	uint32_t legacy_size;

	T_CHECK(map != NULL);
	profile.reserved2 = 1;
	T_CHECK(derech_profile_register(map, &profile) == DERECH_E_INVALID_ARG);
	profile.reserved2 = 0;
	T_CHECK(derech_profile_register(map, &profile) == 0);

	memset(legacy64, 0, sizeof(legacy64));
	legacy_size = 544;
	memcpy(legacy64, &legacy_size, sizeof(legacy_size));
	memset(legacy64 + 12, 0xA5, 4); /* former 64-bit padding */
	T_CHECK(derech_profile_register(map,
		(const derech_profile_desc *)(const void *)legacy64) == 1);
	memset(legacy32, 0, sizeof(legacy32));
	legacy_size = 540;
	memcpy(legacy32, &legacy_size, sizeof(legacy_size));
	T_CHECK(derech_profile_register(map,
		(const derech_profile_desc *)(const void *)legacy32) == 2);
	T_CHECK(derech_find_paths(map, &q, 1, &results) == DERECH_OK);
	T_CHECK(results != NULL);
	T_CHECK(derech_result_status(results, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_ticks_q8(results, 1) == 0);
	T_CHECK(derech_result_total_perceived_q8(results, 1) == 0);
	derech_results_destroy(results);

	q.struct_size = 0;
	results = (derech_results *)(uintptr_t)1;
	T_CHECK(derech_find_paths(map, &q, 1, &results) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(results == NULL);
	q.struct_size = (uint32_t)sizeof(q);
	q.reserved[3] = 1;
	results = (derech_results *)(uintptr_t)1;
	T_CHECK(derech_find_paths(map, &q, 1, &results) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(results == NULL);

	derech_map_destroy(map);
}

int main(void)
{
	test_versions();
	test_create_ex();
	test_request_validation();
	return t_done("abi");
}
