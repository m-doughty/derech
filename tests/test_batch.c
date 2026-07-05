/* Batch semantics: empty batches, mixed outcomes, accessor bounds, the
 * busy guard, run-to-run determinism, and results outliving map edits. */

#include <stdlib.h>

#include "derech_internal.h" /* white-box: busy flag */
#include "support.h"

static void test_empty_and_null(void)
{
	derech_map *m = derech_map_create(3, 3, NULL);
	derech_results *res = NULL;
	derech_request q = t_req(0, 0, 1, 1);

	T_CHECK(m != NULL);
	T_CHECK(derech_find_paths(NULL, &q, 1, &res) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_find_paths(m, &q, 1, NULL) == DERECH_E_INVALID_ARG);
	T_CHECK(derech_find_paths(m, NULL, 1, &res) ==
		DERECH_E_INVALID_ARG);

	T_CHECK(derech_find_paths(m, NULL, 0, &res) == DERECH_OK);
	T_CHECK(res != NULL);
	T_CHECK(derech_results_count(res) == 0);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_NONE);
	derech_results_destroy(res);

	derech_results_destroy(NULL); /* no-op */
	derech_map_destroy(m);
}

static void test_mixed_batch(void)
{
	/* pocket sealed at the south-east corner */
	float pass[6 * 6];
	ref_map rm = { 6, 6, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_results *res = NULL;
	derech_request q[5];

	for (uint32_t i = 0; i < 36; i++) {
		pass[i] = 1.0f;
	}
	pass[4 * 6 + 5] = 0.0f; /* (5,4) */
	pass[4 * 6 + 4] = 0.0f; /* (4,4) */
	pass[5 * 6 + 4] = 0.0f; /* (4,5) -> (5,5) is sealed */
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	q[0] = t_req(0, 0, 5, 0);
	q[1] = t_req(0, 0, 6, 6); /* out of bounds */
	q[2] = t_req(0, 0, 5, 5); /* sealed */
	q[3] = t_req(2, 2, 2, 2); /* trivial */
	q[4] = t_req(0, 0, 5, 5);
	q[4].flags = DERECH_REQ_ALLOW_PARTIAL;

	T_CHECK(derech_find_paths(m, q, 5, &res) == DERECH_OK);
	T_CHECK(derech_results_count(res) == 5);

	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 5);
	T_CHECK(derech_result_status(res, 1) ==
		DERECH_PATH_INVALID_ENDPOINT);
	T_CHECK(derech_result_status(res, 2) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 2) == 0);
	T_CHECK(derech_result_status(res, 3) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 3) == 0);
	T_CHECK(derech_result_status(res, 4) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_length(res, 4) > 0);

	t_validate_result(&rm, &d, 0, 0, 5, 0, res, 0);
	t_validate_result(&rm, &d, 0, 0, 5, 5, res, 2);
	t_validate_result(&rm, &d, 2, 2, 2, 2, res, 3);
	t_validate_result(&rm, &d, 0, 0, 5, 5, res, 4);

	/* accessor bounds */
	T_CHECK(derech_result_status(res, 5) == DERECH_PATH_NONE);
	T_CHECK(derech_result_length(res, 5) == 0);
	T_CHECK(derech_result_steps(res, 5) == NULL);
	T_CHECK(derech_result_step_ticks(res, 5) == NULL);
	T_CHECK(derech_result_total_ticks(res, 5) == 0.0f);
	T_CHECK(derech_result_total_perceived(res, 5) == 0.0f);
	T_CHECK(derech_result_expansions(res, 5) == 0);

	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_busy_guard(void)
{
	derech_map *m = derech_map_create(4, 4, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_results *res = NULL;
	derech_request q = t_req(0, 0, 3, 3);

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	T_CHECK(derech_busy_acquire(&m->busy));
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_BUSY);
	T_CHECK(res == NULL);
	T_CHECK(derech_map_set_passability_at(m, 0, 0, 0.5f) ==
		DERECH_E_BUSY);
	T_CHECK(derech_map_set_tags_at(m, 0, 0, 1) == DERECH_E_BUSY);
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_BUSY);
	derech_busy_release(&m->busy);

	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_OK);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void fill_random_terrain(t_rng *rng, uint32_t n, float *pass,
	uint64_t *tags)
{
	static const float levels[4] = { 1.0f, 0.5f, 0.25f, 0.125f };

	for (uint32_t i = 0; i < n; i++) {
		pass[i] = t_rng_below(rng, 10) < 2 ? 0.0f :
			levels[t_rng_below(rng, 4)];
		tags[i] = t_rng_below(rng, 8); /* bits 0..2 */
	}
}

static void test_determinism(void)
{
	enum { W = 16, H = 16, NREQ = 8 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0x5EEDULL };
	derech_map *m;
	derech_profile_desc d = t_neutral_desc();
	derech_profile_desc avoid = t_neutral_desc();
	derech_request q[NREQ];
	derech_results *a = NULL;
	derech_results *b = NULL;

	fill_random_terrain(&rng, W * H, pass, tags);
	{
		ref_map rm = { W, H, pass, tags };

		m = t_build_map(&rm);
	}
	T_CHECK(m != NULL);
	avoid.tag_mult[0] = 4.0f;
	avoid.tag_add[1] = 8.0f;
	T_CHECK(derech_profile_register(m, &d) == 0);
	T_CHECK(derech_profile_register(m, &avoid) == 1);

	for (uint32_t i = 0; i < NREQ; i++) {
		q[i] = t_req(t_rng_below(&rng, W), t_rng_below(&rng, H),
			t_rng_below(&rng, W), t_rng_below(&rng, H));
		q[i].profile_id = i % 2;
		q[i].flags = (i % 3 == 0) ? DERECH_REQ_ALLOW_PARTIAL : 0;
		q[i].epsilon = (i % 2 == 0) ? 1.0f : 1.25f;
	}

	T_CHECK(derech_find_paths(m, q, NREQ, &a) == DERECH_OK);
	T_CHECK(derech_find_paths(m, q, NREQ, &b) == DERECH_OK);
	for (uint32_t i = 0; i < NREQ; i++) {
		uint32_t len = derech_result_length(a, i);

		T_CHECK(derech_result_status(a, i) ==
			derech_result_status(b, i));
		T_CHECK(len == derech_result_length(b, i));
		T_CHECK(derech_result_total_ticks(a, i) ==
			derech_result_total_ticks(b, i));
		T_CHECK(derech_result_total_perceived(a, i) ==
			derech_result_total_perceived(b, i));
		T_CHECK(derech_result_expansions(a, i) ==
			derech_result_expansions(b, i));
		if (len > 0) {
			T_CHECK(memcmp(derech_result_steps(a, i),
				derech_result_steps(b, i),
				(size_t)len * 2 * sizeof(uint32_t)) == 0);
			T_CHECK(memcmp(derech_result_step_ticks(a, i),
				derech_result_step_ticks(b, i),
				(size_t)len * sizeof(float)) == 0);
		}
	}
	derech_results_destroy(a);
	derech_results_destroy(b);
	derech_map_destroy(m);
}

static void test_results_outlive_edits(void)
{
	derech_map *m;
	derech_profile_desc d = t_neutral_desc();
	derech_results *res = NULL;
	derech_request q;
	uint32_t saved[16 * 2];
	uint32_t len;

	m = derech_map_create(8, 8, NULL);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	q = t_req(0, 0, 7, 7);
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_OK);
	len = derech_result_length(res, 0);
	T_CHECK(len == 7);
	memcpy(saved, derech_result_steps(res, 0),
		(size_t)len * 2 * sizeof(uint32_t));

	/* mutate the map; the results object must be unaffected */
	T_CHECK(derech_map_set_passability_at(m, 3, 3, 0.0f) == DERECH_OK);
	T_CHECK(derech_map_set_tags_at(m, 2, 2, 99) == DERECH_OK);
	T_CHECK(memcmp(saved, derech_result_steps(res, 0),
		(size_t)len * 2 * sizeof(uint32_t)) == 0);

	derech_results_destroy(res);
	derech_map_destroy(m);
}

/* Build a map with an explicit thread count and three profiles. */
static derech_map *build_threaded(const float *pass, const uint64_t *tags,
	uint32_t w, uint32_t h, uint32_t nt)
{
	derech_map_opts o;
	derech_map *m;
	derech_profile_desc neutral = t_neutral_desc();
	derech_profile_desc avoid = t_neutral_desc();
	derech_profile_desc four = t_neutral_desc();

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.n_threads = nt;
	m = derech_map_create(w, h, &o);
	T_CHECK(m != NULL);
	if (m == NULL) {
		return NULL;
	}
	T_CHECK(derech_map_thread_count(m) == nt);
	T_CHECK(derech_map_set_passability(m, pass, (uint64_t)w * h) ==
		DERECH_OK);
	T_CHECK(derech_map_set_tags(m, tags, (uint64_t)w * h) == DERECH_OK);
	avoid.tag_mult[0] = 4.0f;
	avoid.tag_add[1] = 8.0f;
	four.connectivity = DERECH_CONN_4;
	T_CHECK(derech_profile_register(m, &neutral) == 0);
	T_CHECK(derech_profile_register(m, &avoid) == 1);
	T_CHECK(derech_profile_register(m, &four) == 2);
	return m;
}

/* Bitwise comparison of two results objects, row by row. */
static void compare_results(const derech_results *a,
	const derech_results *b)
{
	uint32_t n = derech_results_count(a);

	T_CHECK(n == derech_results_count(b));
	for (uint32_t i = 0; i < n; i++) {
		uint32_t len = derech_result_length(a, i);

		T_CHECK(derech_result_status(a, i) ==
			derech_result_status(b, i));
		T_CHECK(len == derech_result_length(b, i));
		T_CHECK(derech_result_expansions(a, i) ==
			derech_result_expansions(b, i));
		T_CHECK(derech_result_total_ticks(a, i) ==
			derech_result_total_ticks(b, i));
		T_CHECK(derech_result_total_perceived(a, i) ==
			derech_result_total_perceived(b, i));
		if (len > 0 && len == derech_result_length(b, i)) {
			T_CHECK(memcmp(derech_result_steps(a, i),
				derech_result_steps(b, i),
				(size_t)len * 2 * sizeof(uint32_t)) == 0);
			T_CHECK(memcmp(derech_result_step_ticks(a, i),
				derech_result_step_ticks(b, i),
				(size_t)len * sizeof(float)) == 0);
		}
	}
}

static void make_requests(t_rng *rng, uint32_t w, uint32_t h, uint32_t n,
	derech_request *out)
{
	static const float epsilons[4] = { 1.0f, 1.0f, 1.25f, 2.0f };

	for (uint32_t i = 0; i < n; i++) {
		out[i] = t_req(t_rng_below(rng, w), t_rng_below(rng, h),
			t_rng_below(rng, w), t_rng_below(rng, h));
		out[i].profile_id = t_rng_below(rng, 3);
		out[i].flags = (t_rng_next(rng) & 1) ?
			DERECH_REQ_ALLOW_PARTIAL : 0;
		out[i].epsilon = epsilons[t_rng_below(rng, 4)];
	}
}

/* The M3 guarantee: identical batches produce bitwise-identical results
 * objects at every thread count.  Results are compared after their maps
 * are destroyed, which also re-proves results independence. */
static void test_thread_count_invariance(void)
{
	enum { W = 32, H = 32, NREQ = 24 };
	static const uint32_t counts[4] = { 1, 2, 4, 8 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0x7EAD5ULL };
	derech_request q[NREQ];
	derech_results *res[4] = { NULL, NULL, NULL, NULL };

	fill_random_terrain(&rng, W * H, pass, tags);
	make_requests(&rng, W, H, NREQ, q);

	for (uint32_t c = 0; c < 4; c++) {
		derech_map *m = build_threaded(pass, tags, W, H, counts[c]);

		T_CHECK(derech_find_paths(m, q, NREQ, &res[c]) == DERECH_OK);
		derech_map_destroy(m);
	}
	for (uint32_t c = 1; c < 4; c++) {
		compare_results(res[0], res[c]);
	}
	for (uint32_t c = 0; c < 4; c++) {
		derech_results_destroy(res[c]);
	}
}

/* Pool-reuse stress: many consecutive batches of varying sizes (including
 * 0 and 1, which bypass the pool) on one 8-thread map, with occasional
 * identical mutations, always matching a serial twin. */
static void test_pool_reuse_stress(void)
{
	enum { W = 16, H = 16, MAX_REQ = 33 };
	static const uint32_t sizes[5] = { 0, 1, 3, 17, MAX_REQ };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0xB00CULL };
	derech_map *serial;
	derech_map *threaded;
	derech_request q[MAX_REQ];

	fill_random_terrain(&rng, W * H, pass, tags);
	serial = build_threaded(pass, tags, W, H, 1);
	threaded = build_threaded(pass, tags, W, H, 8);

	for (uint32_t round = 0; round < 50; round++) {
		uint32_t n = sizes[round % 5];
		derech_results *a = NULL;
		derech_results *b = NULL;

		if (round % 7 == 6) {
			uint32_t x = t_rng_below(&rng, W);
			uint32_t y = t_rng_below(&rng, H);
			float p = (t_rng_next(&rng) & 1) ? 0.0f : 0.5f;

			T_CHECK(derech_map_set_passability_at(serial, x, y,
				p) == DERECH_OK);
			T_CHECK(derech_map_set_passability_at(threaded, x, y,
				p) == DERECH_OK);
		}
		make_requests(&rng, W, H, MAX_REQ, q);
		T_CHECK(derech_find_paths(serial, q, n, &a) == DERECH_OK);
		T_CHECK(derech_find_paths(threaded, q, n, &b) == DERECH_OK);
		compare_results(a, b);
		derech_results_destroy(a);
		derech_results_destroy(b);
	}
	derech_map_destroy(serial);
	derech_map_destroy(threaded);
}

int main(void)
{
	test_empty_and_null();
	test_mixed_batch();
	test_busy_guard();
	test_determinism();
	test_results_outlive_edits();
	test_thread_count_invariance();
	test_pool_reuse_stress();
	return t_done("test_batch");
}
