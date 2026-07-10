/* Goal sets (explicit, predicate, adjacent) and dirty-region cache
 * invalidation.  White-box checks (derech_internal.h) verify which
 * cached fields survive which edits — the externally visible behavior
 * (expansions == 0) can't distinguish a surviving field from a rebuilt
 * one. */

#include <math.h>
#include <stdlib.h>

#include "derech_internal.h" /* white-box: field cache, labels */
#include "support.h"

static derech_results *run_batch(derech_map *m, const derech_request *q,
	uint32_t n)
{
	derech_results *res = NULL;

	T_CHECK(derech_find_paths(m, q, n, &res) == DERECH_OK);
	T_CHECK(res != NULL);
	return res;
}

static derech_request set_req(uint32_t sx, uint32_t sy, uint32_t set_id)
{
	derech_request q = t_req(sx, sy, 0, 0);

	q.goalset = set_id;
	return q;
}

static void test_register_validation(void)
{
	derech_map *m = derech_map_create(8, 8, NULL);
	uint32_t xy[4] = { 1, 1, 9, 9 };
	int32_t ids[DERECH_MAX_GOALSETS];

	T_CHECK(m != NULL);
	T_CHECK(derech_goalset_register(NULL, xy, 1, 0) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_goalset_register(m, NULL, 1, 0) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_goalset_register(m, xy, 0, 0) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_goalset_register(m, xy, 65, 0) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_goalset_register(m, xy, 2, 0) == DERECH_E_OOB);
	T_CHECK(derech_goalset_register(m, xy, 1, 0x4) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_goalset_register_tags(m, 0, 0, 0) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_busy_acquire(&m->busy));
	T_CHECK(derech_goalset_register(m, xy, 1, 0) == DERECH_E_BUSY);
	T_CHECK(derech_goalset_register_tags(m, 1, 0, 0) == DERECH_E_BUSY);
	derech_busy_release(&m->busy);

	T_CHECK(derech_goalset_unregister(m, 0) == DERECH_E_BAD_GOALSET);
	T_CHECK(derech_goalset_unregister(m, 1) == DERECH_E_BAD_GOALSET);
	T_CHECK(derech_goalset_count(m, 1) == DERECH_E_BAD_GOALSET);

	for (uint32_t i = 0; i < DERECH_MAX_GOALSETS; i++) {
		ids[i] = derech_goalset_register(m, xy, 1, 0);
		T_CHECK(ids[i] == (int32_t)i + 1);
	}
	T_CHECK(derech_goalset_register(m, xy, 1, 0) ==
		DERECH_E_TOO_MANY_GOALSETS);
	T_CHECK(derech_goalset_count(m, (uint32_t)ids[5]) == 1);
	T_CHECK(derech_goalset_unregister(m, (uint32_t)ids[5]) == DERECH_OK);
	/* the freed slot is reused */
	T_CHECK(derech_goalset_register_tags(m, 1, 0, 0) == ids[5]);
	derech_map_destroy(m);
}

static void test_nearest_by_path(void)
{
	/* two goals: euclidean-near one behind a swamp, far one on clear
	 * ground — nearest-by-path must win */
	float pass[16 * 8];
	ref_map rm = { 16, 8, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	int32_t sid;
	derech_request q;
	derech_results *res;
	uint32_t xy[4] = { 6, 3, 14, 3 }; /* near goal, far goal */

	for (uint32_t i = 0; i < 16 * 8; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 0; y < 8; y++) {
		for (uint32_t x = 4; x <= 6; x++) {
			pass[y * 16 + x] = 0.125f; /* swamp wall */
		}
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	sid = derech_goalset_register(m, xy, 2, 0);
	T_CHECK(sid == 1);

	q = set_req(0, 3, (uint32_t)sid);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	{
		/* exact: equals min over the two members of ref distances */
		uint64_t a = ref_solve(&rm, &d, 0, 3, 6, 3);
		uint64_t b = ref_solve(&rm, &d, 0, 3, 14, 3);
		uint64_t want = a < b ? a : b;
		uint32_t len = derech_result_length(res, 0);
		const uint32_t *steps = derech_result_steps(res, 0);

		T_CHECK((uint64_t)llround((double)
			derech_result_total_perceived(res, 0) * 256.0) ==
			want);
		/* the reached member is the path's last step */
		T_CHECK(len > 0 && steps != NULL);
		if (len > 0 && steps != NULL) {
			uint32_t ex = steps[(len - 1) * 2];
			uint32_t ey = steps[(len - 1) * 2 + 1];

			T_CHECK((ex == 6 && ey == 3) ||
				(ex == 14 && ey == 3));
			t_validate_result(&rm, &d, 0, 3, ex, ey, res, 0);
		}
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_predicate_and_membership_updates(void)
{
	/* trees at two spots; fell the near one and the set must follow */
	enum { W = 12, H = 6 };
	derech_map *m = derech_map_create(W, H, NULL);
	derech_profile_desc d = t_neutral_desc();
	int32_t sid;
	derech_request q;
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	T_CHECK(derech_map_set_tags_at(m, 4, 2, 1) == DERECH_OK);
	T_CHECK(derech_map_set_tags_at(m, 10, 2, 1) == DERECH_OK);
	sid = derech_goalset_register_tags(m, 1, 0, 0);
	T_CHECK(sid == 1);
	T_CHECK(derech_goalset_count(m, (uint32_t)sid) == 2);

	q = set_req(0, 2, (uint32_t)sid);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 4); /* to (4,2) */
	derech_results_destroy(res);

	/* fell the near tree: clear its tag */
	T_CHECK(derech_map_set_tags_at(m, 4, 2, 0) == DERECH_OK);
	T_CHECK(derech_goalset_count(m, (uint32_t)sid) == 1);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 10); /* to (10,2) */
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);

	/* fell the last one: the set is empty */
	T_CHECK(derech_map_set_tags_at(m, 10, 2, 0) == DERECH_OK);
	T_CHECK(derech_goalset_count(m, (uint32_t)sid) == 0);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_adjacent_goals(void)
{
	/* impassable trees (p = 0, tagged): ADJACENT sets end beside them */
	enum { W = 10, H = 5 };
	derech_map *m = derech_map_create(W, H, NULL);
	derech_profile_desc d = t_neutral_desc();
	int32_t sid;
	derech_request q;
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	T_CHECK(derech_map_set_passability_at(m, 6, 2, 0.0f) == DERECH_OK);
	T_CHECK(derech_map_set_tags_at(m, 6, 2, 1) == DERECH_OK);
	sid = derech_goalset_register_tags(m, 1, 0,
		DERECH_GOALSET_ADJACENT);
	T_CHECK(sid == 1);

	q = set_req(0, 2, (uint32_t)sid);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	{
		uint32_t len = derech_result_length(res, 0);
		const uint32_t *steps = derech_result_steps(res, 0);

		T_CHECK(len > 0 && steps != NULL);
		if (len > 0 && steps != NULL) {
			uint32_t ex = steps[(len - 1) * 2];
			uint32_t ey = steps[(len - 1) * 2 + 1];
			uint32_t adx = ex > 6 ? ex - 6 : 6 - ex;
			uint32_t ady = ey > 2 ? ey - 2 : 2 - ey;

			T_CHECK(adx <= 1 && ady <= 1);
			T_CHECK(!(ex == 6 && ey == 2)); /* not ON the tree */
		}
	}
	derech_results_destroy(res);

	/* already beside the tree: trivially arrived */
	q = set_req(5, 2, (uint32_t)sid);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_length(res, 0) == 0);
	derech_results_destroy(res);

	/* non-adjacent set on the impassable tree: unreachable */
	{
		int32_t plain = derech_goalset_register_tags(m, 1, 0, 0);

		T_CHECK(plain == 2);
		q = set_req(0, 2, (uint32_t)plain);
		res = run_batch(m, &q, 1);
		T_CHECK(derech_result_status(res, 0) ==
			DERECH_PATH_UNREACHABLE);
		derech_results_destroy(res);

		/* standing ON the blocked member counts as arrived */
		q = set_req(6, 2, (uint32_t)plain);
		res = run_batch(m, &q, 1);
		T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_length(res, 0) == 0);
		derech_results_destroy(res);
	}
	derech_map_destroy(m);
}

static void test_request_validation(void)
{
	derech_map *m = derech_map_create(6, 6, NULL);
	derech_profile_desc d = t_neutral_desc();
	uint32_t xy[2] = { 5, 5 };
	int32_t sid;
	derech_request q;
	derech_results *res = NULL;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	sid = derech_goalset_register(m, xy, 1, 0);
	T_CHECK(sid == 1);

	q = set_req(0, 0, 99);
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_BAD_GOALSET);
	q = set_req(0, 0, (uint32_t)sid);
	q.flags = DERECH_REQ_ALLOW_PARTIAL;
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_INVALID_ARG);

	/* cost cap: member is 10 steps away, cap at 3 ticks */
	q = set_req(0, 0, (uint32_t)sid);
	q.max_perceived_cost = 3.0f;
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) ==
		DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(res, 0) == 0);
	derech_results_destroy(res);

	/* goal coords are ignored for set requests, even hostile ones */
	q = set_req(0, 0, (uint32_t)sid);
	q.goal_x = 9999;
	q.goal_y = 9999;
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	derech_results_destroy(res);

	/* unregister invalidates the id for later batches */
	T_CHECK(derech_goalset_unregister(m, (uint32_t)sid) == DERECH_OK);
	q = set_req(0, 0, (uint32_t)sid);
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_E_BAD_GOALSET);
	derech_map_destroy(m);
}

/* ------------------------------------------------------------------ */
/* Dirty-region invalidation (white-box)                               */
/* ------------------------------------------------------------------ */

static uint32_t cached_field_count(const derech_map *m)
{
	uint32_t n = 0;

	for (derech_field *f = m->field_lru_head; f != NULL;
		f = f->lru_next) {
		n++;
	}
	return n;
}

static void test_dirty_spatial_selectivity(void)
{
	/* map split by a full wall: a field on the left must survive edits
	 * on the right, and die from edits on the left */
	enum { W = 17, H = 9 };
	float pass[W * H];
	ref_map rm = { W, H, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[4];
	derech_results *res;

	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 0; y < H; y++) {
		pass[y * W + 8] = 0.0f; /* wall at x = 8 */
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);

	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(i, i, 2, 4); /* left-half goal, group of 4 */
	}
	res = run_batch(m, q, 4);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	/* sentinel in a slot single-goal fields never read: a rebuild
	 * calloc()s a fresh struct and zeroes it, so pointer identity
	 * (which the allocator can recycle) is not the test */
	m->field_lru_head->set_epoch = 12345;

	/* right-half edit: irrelevant to the left field */
	T_CHECK(derech_map_set_passability_at(m, 14, 4, 0.5f) == DERECH_OK);
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	T_CHECK(m->field_lru_head->set_epoch == 12345); /* survived */

	/* left-half edit: kills and rebuilds it */
	T_CHECK(derech_map_set_passability_at(m, 4, 4, 0.5f) == DERECH_OK);
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	T_CHECK(m->field_lru_head->set_epoch == 0); /* rebuilt fresh */
	derech_map_destroy(m);
}

static void test_dirty_tag_relevance(void)
{
	/* the explorer scenario: rewriting a tag bit NO profile weighs must
	 * not disturb cached fields; a weighted bit must */
	enum { W = 12, H = 12 };
	derech_map *m = derech_map_create(W, H, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q[4];
	derech_results *res;
	uint64_t reveal[16];

	d.tag_mult[0] = 4.0f; /* profile cares about bit 0 only */
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(i, 0, 10, 10);
	}
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	m->field_lru_head->set_epoch = 777; /* survival sentinel */

	/* "reveal" writes: bit 7, smack in the field's area */
	for (uint32_t i = 0; i < 16; i++) {
		reveal[i] = 1ULL << 7;
	}
	T_CHECK(derech_map_set_tags_rect(m, 4, 4, 4, 4, reveal) ==
		DERECH_OK);
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(m->field_lru_head->set_epoch == 777); /* survived */

	/* a bit the profile weighs: field must go */
	for (uint32_t i = 0; i < 16; i++) {
		reveal[i] = (1ULL << 7) | 1ULL;
	}
	T_CHECK(derech_map_set_tags_rect(m, 4, 4, 4, 4, reveal) ==
		DERECH_OK);
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(m->field_lru_head->set_epoch == 0); /* rebuilt fresh */

	/* rewriting identical values is a no-op: nothing dirty, survives */
	m->field_lru_head->set_epoch = 888;
	T_CHECK(derech_map_set_tags_rect(m, 4, 4, 4, 4, reveal) ==
		DERECH_OK);
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(m->field_lru_head->set_epoch == 888);
	derech_map_destroy(m);
}

static void test_dirty_adjacent_unblock(void)
{
	/* opening a wall tile ADJACENT to the field's area must invalidate
	 * it — the new opening creates shorter routes */
	enum { W = 11, H = 5 };
	float pass[W * H];
	ref_map rm = { W, H, pass, NULL };
	derech_profile_desc d = t_neutral_desc();
	derech_map *m;
	derech_request q[4];
	derech_results *res;
	float long_way, short_way;

	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = 1.0f;
	}
	for (uint32_t y = 0; y < 4; y++) {
		pass[y * W + 5] = 0.0f; /* wall with a gap only at y = 4 */
	}
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(0, 0, 10, 0);
	}
	res = run_batch(m, q, 4);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	long_way = derech_result_total_perceived(res, 0);
	derech_results_destroy(res);

	/* open the wall at y = 0: straight shot */
	T_CHECK(derech_map_set_passability_at(m, 5, 0, 1.0f) == DERECH_OK);
	res = run_batch(m, q, 4);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	short_way = derech_result_total_perceived(res, 0);
	T_CHECK(short_way < long_way);
	T_CHECK(short_way == 10.0f);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_dirty_overflow_fallback(void)
{
	/* more distinct edits than the rect log holds: everything flushes,
	 * correctness intact */
	enum { W = 20, H = 20 };
	derech_map *m = derech_map_create(W, H, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q[4];
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	for (uint32_t i = 0; i < 4; i++) {
		q[i] = t_req(i, 0, 19, 19);
	}
	res = run_batch(m, q, 4);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);

	for (uint32_t k = 0; k < DERECH_MAX_DIRTY_RECTS + 4; k++) {
		T_CHECK(derech_map_set_passability_at(m, k % W, 10,
			0.5f) == DERECH_OK);
	}
	res = run_batch(m, q, 4);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

static void test_new_field_is_most_recent(void)
{
	enum { SIDE = 256 };
	derech_map_opts opts;
	derech_map *m;
	derech_profile_desc d = t_neutral_desc();
	uint32_t a_xy[2] = { SIDE - 1, SIDE - 1 };
	uint32_t b_xy[2] = { SIDE - 1, 0 };
	derech_request first;
	derech_request batch[2];
	derech_results *res;
	int32_t a_id, b_id;

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_cache_mb = 1; /* one 256x256 field fits, two do not */
	m = derech_map_create(SIDE, SIDE, &opts);
	T_CHECK(m != NULL);
	if (m == NULL) {
		return;
	}
	T_CHECK(derech_profile_register(m, &d) == 0);
	a_id = derech_goalset_register(m, a_xy, 1, 0);
	b_id = derech_goalset_register(m, b_xy, 1, 0);
	T_CHECK(a_id == 1);
	T_CHECK(b_id == 2);
	first = set_req(0, 0, (uint32_t)a_id);
	res = run_batch(m, &first, 1);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	T_CHECK(m->field_lru_head->goalset_id == (uint32_t)a_id);

	/* The old field is hit before the new one is inserted.  Commit order
	 * must still leave the later-created field as the retained MRU. */
	batch[0] = first;
	batch[1] = set_req(0, SIDE - 1, (uint32_t)b_id);
	res = run_batch(m, batch, 2);
	derech_results_destroy(res);
	T_CHECK(cached_field_count(m) == 1);
	T_CHECK(m->field_lru_head->goalset_id == (uint32_t)b_id);
	derech_map_destroy(m);
}

static void test_labels_relevance(void)
{
	/* labels survive tag edits outside their masks, flush on
	 * passability changes */
	enum { W = 10, H = 10 };
	derech_map *m = derech_map_create(W, H, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_request q;
	derech_results *res;

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(m, &d) == 0);
	/* a sealed pocket so a label class gets built and used */
	for (uint32_t y = 3; y <= 5; y++) {
		for (uint32_t x = 3; x <= 5; x++) {
			if (x != 4 || y != 4) {
				T_CHECK(derech_map_set_passability_at(m, x, y,
					0.0f) == DERECH_OK);
			}
		}
	}
	q = t_req(0, 0, 4, 4);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	T_CHECK(m->label_class_count == 1);
	/* sentinel in a blocked tile's label slot (never consulted): a
	 * rebuild recomputes it to 0 */
	m->label_classes[0].label[3 * W + 3] = 0x7FFFFFFFu;

	/* tag-only edit on a bit outside the class masks: labels stay */
	T_CHECK(derech_map_set_tags_at(m, 0, 9, 1ULL << 9) == DERECH_OK);
	res = run_batch(m, &q, 1);
	derech_results_destroy(res);
	T_CHECK(m->label_class_count == 1);
	T_CHECK(m->label_classes[0].label[3 * W + 3] == 0x7FFFFFFFu);

	/* passability edit: labels flush, then rebuild on demand */
	T_CHECK(derech_map_set_passability_at(m, 9, 9, 0.5f) == DERECH_OK);
	res = run_batch(m, &q, 1);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	T_CHECK(derech_result_expansions(res, 0) == 0);
	derech_results_destroy(res);
	T_CHECK(m->label_class_count == 1);
	T_CHECK(m->label_classes[0].label[3 * W + 3] == 0); /* rebuilt */
	derech_map_destroy(m);
}

static void test_set_history_determinism(void)
{
	/* identical batch+edit sequences with sets on serial vs 8-thread
	 * maps produce identical bytes */
	enum { W = 20, H = 20, NREQ = 10 };
	float pass[W * H];
	uint64_t tags[W * H];
	t_rng rng = { 0x5E75ULL };
	derech_map_opts o;
	derech_map *maps[2];
	derech_profile_desc d = t_neutral_desc();

	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = (t_rng_below(&rng, 10) < 2) ? 0.0f : 1.0f;
		tags[i] = t_rng_below(&rng, 8) == 0 ? 1 : 0; /* trees */
	}
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
		T_CHECK(derech_goalset_register_tags(maps[c], 1, 0, 0) == 1);
	}

	for (uint32_t round = 0; round < 6; round++) {
		derech_request q[NREQ];
		derech_results *a = NULL;
		derech_results *b = NULL;

		for (uint32_t i = 0; i < NREQ; i++) {
			if (i % 2 == 0) {
				q[i] = set_req(t_rng_below(&rng, W),
					t_rng_below(&rng, H), 1);
			} else {
				q[i] = t_req(t_rng_below(&rng, W),
					t_rng_below(&rng, H),
					t_rng_below(&rng, W),
					t_rng_below(&rng, H));
			}
		}
		if (round % 2 == 1) { /* fell a random tree on both maps */
			uint32_t x = t_rng_below(&rng, W);
			uint32_t y = t_rng_below(&rng, H);

			T_CHECK(derech_map_set_tags_at(maps[0], x, y, 0) ==
				DERECH_OK);
			T_CHECK(derech_map_set_tags_at(maps[1], x, y, 0) ==
				DERECH_OK);
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
			T_CHECK(derech_result_total_perceived(a, i) ==
				derech_result_total_perceived(b, i));
			if (len > 0 && len == derech_result_length(b, i)) {
				T_CHECK(memcmp(derech_result_steps(a, i),
					derech_result_steps(b, i),
					(size_t)len * 2 *
					sizeof(uint32_t)) == 0);
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
	test_register_validation();
	test_nearest_by_path();
	test_predicate_and_membership_updates();
	test_adjacent_goals();
	test_request_validation();
	test_dirty_spatial_selectivity();
	test_dirty_tag_relevance();
	test_dirty_adjacent_unblock();
	test_dirty_overflow_fallback();
	test_new_field_is_most_recent();
	test_labels_relevance();
	test_set_history_determinism();
	return t_done("test_goalset");
}
