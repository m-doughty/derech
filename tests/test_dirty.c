/* Overflow loses rectangle detail, not dependency information. */
#include <math.h>
#include <stdlib.h>

#include "derech_internal.h"
#include "support.h"

enum { SIDE = 32, CELLS = SIDE * SIDE };

typedef struct fixture {
	derech_map *map;
	derech_profile_desc desc[2];
	float pass[CELLS];
	uint64_t tags[CELLS];
} fixture;

static void init(fixture *f)
{
	derech_map_opts opts = { 0 };

	opts.struct_size = sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	opts.field_group_threshold = 1;
	f->map = derech_map_create(SIDE, SIDE, &opts);
	T_CHECK(f->map != NULL);
	f->desc[0] = t_neutral_desc();
	f->desc[0].tag_mult[0] = 2.0f;
	f->desc[1] = t_neutral_desc();
	f->desc[1].block_mask = 2;
	for (uint32_t i = 0; i < 2; i++) {
		T_CHECK(derech_profile_register(f->map, &f->desc[i]) == (int32_t)i);
	}
	for (uint32_t i = 0; i < CELLS; i++) {
		f->pass[i] = 1.0f;
		f->tags[i] = 0;
	}
}

static derech_field *field(derech_map *m, uint32_t profile)
{
	for (derech_field *p = m->field_lru_head; p != NULL; p = p->lru_next) {
		if (p->profile_id == profile) return p;
	}
	return NULL;
}

static void check_retained(derech_map *m, uint32_t profile)
{
	derech_field *p = field(m, profile);

	T_CHECK(p != NULL);
	/* A sentinel in a single-goal field's unused epoch distinguishes reuse
	 * from a rebuild even when the allocator reuses the struct's address. */
	if (p != NULL) T_CHECK(p->set_epoch == 100 + profile);
}

static void build_and_check(fixture *f)
{
	derech_request q[2];
	derech_results *res = NULL;
	ref_map reference = { SIDE, SIDE, f->pass, f->tags };

	for (uint32_t i = 0; i < 2; i++) {
		q[i] = t_req(0, 0, SIDE - 1, SIDE - 1);
		q[i].profile_id = i;
	}
	T_CHECK(derech_find_paths(f->map, q, 2, &res) == DERECH_OK);
	if (res == NULL) return;
	for (uint32_t i = 0; i < 2; i++) {
		derech_field *p = field(f->map, i);
		uint64_t expected = ref_solve(&reference, &f->desc[i], 0, 0,
			SIDE - 1, SIDE - 1);

		T_CHECK(derech_result_status(res, i) == DERECH_PATH_FOUND);
		T_CHECK((uint64_t)llround(derech_result_total_perceived(res, i) *
			256.0) == expected);
		t_validate_result(&reference, &f->desc[i], 0, 0, SIDE - 1,
			SIDE - 1, res, i);
		T_CHECK(p != NULL);
		if (p != NULL) p->set_epoch = 100 + i;
	}
	derech_results_destroy(res);
}

static void tag(fixture *f, uint32_t x, uint32_t y, uint64_t bits)
{
	f->tags[y * SIDE + x] = bits;
	T_CHECK(derech_map_set_tags_at(f->map, x, y, bits) == DERECH_OK);
}

static void noise(fixture *f, uint32_t count, uint64_t bits)
{
	for (uint32_t x = 0; x < count; x++) tag(f, x, 2, bits);
}

static void test_irrelevant_and_full_grid(void)
{
	fixture f;
	derech_results *empty = NULL;

	init(&f);
	build_and_check(&f);
	for (uint32_t cycle = 0; cycle < 3; cycle++) {
		noise(&f, 17, cycle & 1 ? 0 : 32);
		T_CHECK(f.map->dirty.full);
		/* Empty/no-request intervals must leave the outstanding changes intact. */
		T_CHECK(derech_find_paths(f.map, NULL, 0, &empty) == DERECH_OK);
		T_CHECK(derech_results_count(empty) == 0);
		derech_results_destroy(empty);
		T_CHECK(f.map->dirty.full);
		T_CHECK(derech_reconcile(f.map) == DERECH_OK);
		check_retained(f.map, 0);
		check_retained(f.map, 1);
		T_CHECK(f.map->label_class_count == 2);
		T_CHECK(!f.map->dirty.full && f.map->dirty.count == 0);
		build_and_check(&f);
	}
	for (uint32_t i = 0; i < CELLS; i++) f.tags[i] = 64;
	T_CHECK(derech_map_set_tags(f.map, f.tags, CELLS) == DERECH_OK);
	T_CHECK(f.map->dirty.full);
	T_CHECK(derech_reconcile(f.map) == DERECH_OK);
	check_retained(f.map, 0);
	check_retained(f.map, 1);
	T_CHECK(f.map->label_class_count == 2);
	T_CHECK(derech_map_set_tags(f.map, f.tags, CELLS) == DERECH_OK);
	T_CHECK(!f.map->dirty.full && f.map->dirty.count == 0);
	for (uint32_t i = 0; i < CELLS; i++) f.tags[i] |= 1;
	T_CHECK(derech_map_set_tags(f.map, f.tags, CELLS) == DERECH_OK);
	T_CHECK(derech_reconcile(f.map) == DERECH_OK);
	T_CHECK(field(f.map, 0) == NULL);
	check_retained(f.map, 1);
	T_CHECK(f.map->label_class_count == 2); /* costs do not change connectivity */
	build_and_check(&f);
	for (uint32_t i = 0; i < CELLS; i++) f.pass[i] = 0.5f;
	T_CHECK(derech_map_set_passability(f.map, f.pass, CELLS) == DERECH_OK);
	T_CHECK(derech_reconcile(f.map) == DERECH_OK);
	T_CHECK(f.map->field_lru_head == NULL && f.map->label_class_count == 0);
	build_and_check(&f);
	derech_map_destroy(f.map);
}

static void test_relevant_at_and_after_overflow(void)
{
	for (uint32_t before = 16; before <= 17; before++) {
		for (uint32_t kind = 0; kind < 3; kind++) {
			fixture f;

			init(&f);
			build_and_check(&f);
			noise(&f, before, 32);
			if (kind < 2) {
				tag(&f, 20, 20, 1u << kind);
			} else {
				f.pass[20 * SIDE + 20] = 0.5f;
				T_CHECK(derech_map_set_passability_at(f.map, 20, 20,
					0.5f) == DERECH_OK);
			}
			T_CHECK(f.map->dirty.full);
			T_CHECK(derech_reconcile(f.map) == DERECH_OK);
			for (uint32_t profile = 0; profile < 2; profile++) {
				if (kind == 2 || profile == kind) {
					T_CHECK(field(f.map, profile) == NULL);
				} else {
					check_retained(f.map, profile);
				}
			}
			T_CHECK(f.map->label_class_count == (kind == 0 ? 2u : kind == 1 ? 1u : 0u));
			if (kind == 1 && f.map->label_class_count == 1) {
				T_CHECK(f.map->label_classes[0].block_mask == 0);
				T_CHECK(f.map->label_classes[0].require_mask == 0);
			}
			build_and_check(&f);
			derech_map_destroy(f.map);
		}
	}
}

static void test_predicate_cancel_retry(void)
{
	for (uint32_t before = 16; before <= 17; before++) {
		fixture f;
		derech_request q = t_req(0, 0, 0, 0);
		derech_results *res = NULL;
		derech_cancel *cancel = NULL;
		derech_find_opts opts = { 0 };
		uint64_t epoch;
		uint64_t *members;

		init(&f);
		tag(&f, 31, 31, 8);
		T_CHECK(derech_goalset_register_tags(f.map, 8, 0, 0) == 1);
		q.goalset = 1;
		T_CHECK(derech_find_paths(f.map, &q, 1, &res) == DERECH_OK);
		derech_results_destroy(res);
		epoch = f.map->goalsets[0].epoch;
		members = f.map->goalsets[0].members;
		noise(&f, before, 32);
		tag(&f, 4, 0, 8);
		T_CHECK(f.map->dirty.full);
		T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
		derech_cancel_request(cancel);
		opts.struct_size = sizeof(opts);
		opts.cancel = cancel;
		T_CHECK(derech_find_paths_ex(f.map, &q, 1, &opts, &res) == DERECH_E_CANCELLED);
		T_CHECK(res == NULL);
		/* Exercise cancellation inside materialization, after batch entry's guard. */
		f.map->active_cancel = cancel;
		T_CHECK(derech_reconcile(f.map) == DERECH_E_CANCELLED);
		f.map->active_cancel = NULL;
		T_CHECK(f.map->dirty.full);
		T_CHECK(f.map->goalsets[0].members == members);
		T_CHECK(f.map->goalsets[0].epoch == epoch);
		T_CHECK(f.map->goalsets[0].member_count == 1);
		derech_cancel_destroy(cancel);
		T_CHECK(derech_find_paths(f.map, &q, 1, &res) == DERECH_OK);
		T_CHECK(derech_result_status(res, 0) == DERECH_PATH_FOUND);
		T_CHECK(derech_result_length(res, 0) == 4);
		T_CHECK(derech_result_total_perceived(res, 0) == 4.0f);
		T_CHECK(f.map->goalsets[0].member_count == 2);
		T_CHECK(f.map->goalsets[0].epoch == epoch + 1);
		T_CHECK(!f.map->dirty.full && f.map->dirty.count == 0);
		derech_results_destroy(res);
		derech_map_destroy(f.map);
	}
}

int main(void)
{
	test_irrelevant_and_full_grid();
	test_relevant_at_and_after_overflow();
	test_predicate_cancel_retry();
	return t_done("test_dirty");
}
