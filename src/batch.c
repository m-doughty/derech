/* Batch entry point, request validation, planning, per-request solving,
 * ordered results assembly, and the results object.
 *
 * A batch runs in three phases.
 *
 * Planning (single-threaded): per request, run the cheap resolutions —
 * bounds, trivial, blocked goal, component-label quick-NO — then try the
 * field cache; cache misses group by (goal, profile), and groups of at
 * least field_threshold requests get a shared goal field planned.  All
 * cache/LRU/label mutation happens here, so scheduling can never affect
 * routing decisions.
 *
 * Execution (parallel): round one runs field builds, classic A* solves,
 * and extractions from already-cached fields; round two runs extractions
 * from the fields built in round one.  Workers claim tasks off an atomic
 * cursor and touch only their own derech_search_ctx and the staging rows
 * of requests they claimed.
 *
 * Assembly (single-threaded): staged paths are copied into the results
 * arena in request order.  Because routing is content-based, every task
 * is self-contained, and assembly order is fixed, the results object is
 * bitwise-identical for any thread count — the serial path is just the
 * same tasks run in a loop on ctx 0. */

#include "derech_internal.h"

typedef struct derech_res_row {
	derech_path_status status;
	uint64_t off; /* first step index into the shared buffers */
	uint32_t len;
	uint32_t expansions;
	float total_ticks;
	float total_perceived;
} derech_res_row;

struct derech_results {
	uint32_t count;
	derech_res_row *rows;
	uint32_t *steps; /* interleaved x,y — 2 per step */
	float *ticks;    /* 1 per step */
	uint64_t steps_len;
};

/* ------------------------------------------------------------------ */
/* Per-worker path emission                                            */
/* ------------------------------------------------------------------ */

int derech_ctx_out_reserve(derech_search_ctx *ctx, uint64_t extra)
{
	uint64_t need = ctx->out_len + extra;
	uint64_t cap = ctx->out_cap;
	uint32_t *steps;
	float *ticks;

	if (need <= cap) {
		return 1;
	}
	cap = cap == 0 ? 256 : cap;
	while (cap < need) {
		cap *= 2;
	}
	steps = realloc(ctx->out_steps, (size_t)cap * 2 * sizeof(*steps));
	if (steps == NULL) {
		return 0;
	}
	ctx->out_steps = steps;
	ticks = realloc(ctx->out_ticks, (size_t)cap * sizeof(*ticks));
	if (ticks == NULL) {
		return 0;
	}
	ctx->out_ticks = ticks;
	ctx->out_cap = cap;
	return 1;
}

/* Walk parents back from end_idx, then emit the path forward (start tile
 * excluded) into the ctx's out buffers.  Returns 0 only on allocation
 * failure. */
static int emit_path(const derech_map *map, derech_search_ctx *ctx,
	const derech_profile *prof, uint32_t start_idx, uint32_t end_idx,
	derech_stage_row *row)
{
	uint32_t idx = end_idx;
	uint32_t depth = 0;
	uint64_t total_true_q = 0;

	while (idx != start_idx) {
		ctx->path_scratch[depth++] = idx;
		{
			uint32_t d = (uint32_t)ctx->parent[idx] - 1;

			idx -= (uint32_t)((int64_t)derech_dir_dx[d] +
				(int64_t)derech_dir_dy[d] * map->w);
		}
	}

	if (!derech_ctx_out_reserve(ctx, depth)) {
		return 0;
	}
	row->local_off = ctx->out_len;
	row->len = depth;
	for (uint32_t i = 0; i < depth; i++) {
		uint32_t step = ctx->path_scratch[depth - 1 - i];
		uint32_t d = (uint32_t)ctx->parent[step] - 1;
		uint32_t q = derech_true_step_q(map, prof, step, d >= 4);
		uint64_t at = ctx->out_len + i;

		ctx->out_steps[at * 2] = step % map->w;
		ctx->out_steps[at * 2 + 1] = step / map->w;
		ctx->out_ticks[at] = (float)((double)q / 256.0);
		total_true_q += q;
	}
	ctx->out_len += depth;
	row->true_q = total_true_q;
	row->perceived_q = ctx->g[end_idx];
	return 1;
}

/* ------------------------------------------------------------------ */
/* Per-request solve (runs on any worker)                              */
/* ------------------------------------------------------------------ */

void derech_solve_request(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_request *req, derech_stage_row *row)
{
	const derech_profile *prof = &map->profiles[req->profile_id];
	int allow_partial = (req->flags & DERECH_REQ_ALLOW_PARTIAL) != 0;
	uint32_t start_idx, goal_idx, eps_q, max_exp, max_cost_q;
	derech_search_result sr;

	row->worker = worker;
	row->status = DERECH_PATH_INVALID_ENDPOINT;
	if (req->start_x >= map->w || req->start_y >= map->h ||
		req->goal_x >= map->w || req->goal_y >= map->h) {
		return;
	}
	start_idx = req->start_y * map->w + req->start_x;
	goal_idx = req->goal_y * map->w + req->goal_x;

	if (start_idx == goal_idx) {
		row->status = DERECH_PATH_FOUND;
		return;
	}
	/* an unenterable goal can never be reached; skip the search unless
	 * the caller wants the closest approach */
	if (derech_tile_blocked(map, prof, goal_idx) && !allow_partial) {
		row->status = DERECH_PATH_UNREACHABLE;
		return;
	}

	eps_q = req->epsilon == 0.0f ? DERECH_Q_EPS_DEFAULT :
		derech_q_round((double)req->epsilon * 256.0, 1u << 16);
	max_exp = req->max_expansions == 0 ? map->n : req->max_expansions;
	max_cost_q = req->max_perceived_cost == 0.0f ? UINT32_MAX :
		derech_q_round((double)req->max_perceived_cost * 256.0,
			UINT32_MAX - 1);

	derech_search(map, ctx, prof, start_idx, goal_idx, eps_q, max_exp,
		max_cost_q, &sr);

	row->status = sr.status;
	row->expansions = sr.expansions;
	if (sr.status != DERECH_PATH_FOUND &&
		(!allow_partial || sr.end_idx == start_idx)) {
		return;
	}
	if (!emit_path(map, ctx, prof, start_idx, sr.end_idx, row)) {
		row->oom = 1;
	}
}

void derech_run_task(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_task *task, const derech_request *reqs,
	derech_stage_row *stage)
{
	switch (task->kind) {
	case DERECH_TASK_SOLVE:
		derech_solve_request(map, ctx, worker, &reqs[task->req_idx],
			&stage[task->req_idx]);
		break;
	case DERECH_TASK_FIELD_BUILD:
		derech_field_build(map, ctx, task->field);
		break;
	default: /* DERECH_TASK_EXTRACT */
		derech_field_extract(map, ctx, worker, task->field,
			&reqs[task->req_idx], &stage[task->req_idx]);
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Batch solve                                                         */
/* ------------------------------------------------------------------ */

static derech_status validate_requests(const derech_map *map,
	const derech_request *reqs, uint32_t n_reqs)
{
	for (uint32_t i = 0; i < n_reqs; i++) {
		const derech_request *q = &reqs[i];

		if (q->profile_id >= map->profile_count) {
			return DERECH_E_BAD_PROFILE;
		}
		if ((q->flags & ~(uint32_t)DERECH_REQ_ALLOW_PARTIAL) != 0) {
			return DERECH_E_INVALID_ARG;
		}
		if (q->goalset != DERECH_NO_GOALSET) {
			if (q->goalset > DERECH_MAX_GOALSETS ||
				!map->goalsets[q->goalset - 1].used) {
				return DERECH_E_BAD_GOALSET;
			}
			/* no gradient exists toward an unreachable SET, so
			 * closest-approach partials are undefined */
			if ((q->flags & DERECH_REQ_ALLOW_PARTIAL) != 0) {
				return DERECH_E_INVALID_ARG;
			}
		}
		if (q->epsilon != 0.0f &&
			!(q->epsilon >= 1.0f && q->epsilon <= 256.0f)) {
			return DERECH_E_INVALID_ARG;
		}
		if (!(q->max_perceived_cost >= 0.0f) ||
			q->max_perceived_cost > 16000000.0f) {
			return DERECH_E_INVALID_ARG;
		}
	}
	return DERECH_OK;
}

/* Copy staged paths into the results arena in request order. */
static derech_status assemble_results(const derech_map *map,
	const derech_stage_row *stage, uint32_t n_reqs, derech_results *r)
{
	uint64_t total = 0;

	for (uint32_t i = 0; i < n_reqs; i++) {
		if (stage[i].oom) {
			return DERECH_E_NOMEM;
		}
		total += stage[i].len;
	}
	if (total > 0) {
		r->steps = malloc((size_t)total * 2 * sizeof(*r->steps));
		r->ticks = malloc((size_t)total * sizeof(*r->ticks));
		if (r->steps == NULL || r->ticks == NULL) {
			return DERECH_E_NOMEM;
		}
	}

	for (uint32_t i = 0; i < n_reqs; i++) {
		const derech_stage_row *s = &stage[i];
		derech_res_row *row = &r->rows[i];

		row->status = s->status;
		row->expansions = s->expansions;
		row->len = s->len;
		row->off = r->steps_len;
		if (s->len > 0) {
			const derech_search_ctx *ctx = &map->ctxs[s->worker];

			memcpy(&r->steps[r->steps_len * 2],
				&ctx->out_steps[s->local_off * 2],
				(size_t)s->len * 2 * sizeof(*r->steps));
			memcpy(&r->ticks[r->steps_len],
				&ctx->out_ticks[s->local_off],
				(size_t)s->len * sizeof(*r->ticks));
			r->steps_len += s->len;
			row->total_ticks =
				(float)((double)s->true_q / 256.0);
			row->total_perceived =
				(float)((double)s->perceived_q / 256.0);
		}
	}
	return DERECH_OK;
}

/* ------------------------------------------------------------------ */
/* Planning                                                            */
/* ------------------------------------------------------------------ */

enum {
	PLAN_DONE = 0,  /* stage row already resolved or task already queued */
	PLAN_GROUP = 1  /* candidate for (goal, profile) grouping            */
};

typedef struct plan_group {
	uint64_t key; /* (goal_idx << 32) | profile_id */
	uint32_t count;
	uint8_t used;
	uint8_t decided;
	derech_field *field; /* NULL = members solve individually */
} plan_group;

static plan_group *plan_group_slot(plan_group *tab, uint32_t cap,
	uint64_t key)
{
	uint64_t h = key;
	uint32_t i;

	h ^= h >> 30;
	h *= 0xbf58476d1ce4e5b9ULL;
	h ^= h >> 27;
	i = (uint32_t)h & (cap - 1);
	while (tab[i].used && tab[i].key != key) {
		i = (i + 1) & (cap - 1);
	}
	return &tab[i];
}

static void run_tasks(derech_map *map, const derech_task *tasks, uint32_t n,
	const derech_request *reqs, derech_stage_row *stage)
{
	if (n == 0) {
		return;
	}
	if (map->pool != NULL && n > 1) {
		derech_pool_run(map->pool, map, tasks, n, reqs, stage);
	} else {
		for (uint32_t k = 0; k < n; k++) {
			derech_run_task(map, &map->ctxs[0], 0, &tasks[k],
				reqs, stage);
		}
	}
}

/* True when the component labels prove the request unreachable.  A
 * blocked start is escapable, so it counts as reachable-maybe if any
 * neighbor shares the goal's component. */
static int labels_prove_unreachable(const derech_map *map,
	const derech_labels *lc, uint32_t start_idx, uint32_t goal_idx)
{
	uint32_t lg = lc->label[goal_idx];
	uint32_t ls = lc->label[start_idx];
	uint32_t x, y;

	if (ls == lg) {
		return 0;
	}
	if (ls != 0) {
		return 1; /* both enterable, different components */
	}
	x = start_idx % map->w;
	y = start_idx / map->w;
	for (uint32_t d = 0; d < 8; d++) {
		int64_t nx = (int64_t)x + derech_dir_dx[d];
		int64_t ny = (int64_t)y + derech_dir_dy[d];

		if (nx < 0 || ny < 0 || nx >= (int64_t)map->w ||
			ny >= (int64_t)map->h) {
			continue;
		}
		if (lc->label[(uint32_t)ny * map->w + (uint32_t)nx] == lg) {
			return 0;
		}
	}
	return 1;
}

static derech_status plan_and_execute(derech_map *map,
	const derech_request *reqs, uint32_t n_reqs, derech_stage_row *stage)
{
	derech_task *t1 = NULL;
	derech_task *t2 = NULL;
	uint8_t *action = NULL;
	plan_group *groups = NULL;
	uint32_t group_cap = 16;
	uint32_t n1 = 0, n2 = 0;
	derech_status rc = DERECH_E_NOMEM;

	/* apply pending edits: refresh predicate goal sets, drop caches the
	 * edits could actually affect */
	derech_reconcile(map);

	while (group_cap < 2ull * n_reqs) {
		group_cap *= 2;
	}
	t1 = malloc((size_t)n_reqs * 2 * sizeof(*t1));
	t2 = malloc((size_t)n_reqs * sizeof(*t2));
	action = malloc(n_reqs);
	groups = calloc(group_cap, sizeof(*groups));
	if (t1 == NULL || t2 == NULL || action == NULL || groups == NULL) {
		goto done;
	}

	for (uint32_t i = 0; i < n_reqs; i++) {
		const derech_request *q = &reqs[i];
		derech_stage_row *row = &stage[i];
		const derech_profile *prof;
		int allow_partial =
			(q->flags & DERECH_REQ_ALLOW_PARTIAL) != 0;
		uint32_t start_idx, goal_idx;
		derech_field *cached;

		action[i] = PLAN_DONE;
		if (q->start_x >= map->w || q->start_y >= map->h) {
			row->status = DERECH_PATH_INVALID_ENDPOINT;
			continue;
		}
		start_idx = q->start_y * map->w + q->start_x;
		prof = &map->profiles[q->profile_id];

		if (q->goalset != DERECH_NO_GOALSET) {
			/* set queries are always field-backed: look up the
			 * cached field or group for a build */
			cached = derech_field_cache_lookup(map, 0, q->goalset,
				q->profile_id);
			if (cached != NULL) {
				t1[n1].kind = DERECH_TASK_EXTRACT;
				t1[n1].req_idx = i;
				t1[n1].field = cached;
				n1++;
			} else {
				uint64_t key = (((uint64_t)0x80000000u |
					q->goalset) << 32) | q->profile_id;
				plan_group *g = plan_group_slot(groups,
					group_cap, key);

				if (!g->used) {
					g->used = 1;
					g->key = key;
				}
				g->count++;
				action[i] = PLAN_GROUP;
			}
			continue;
		}

		if (q->goal_x >= map->w || q->goal_y >= map->h) {
			row->status = DERECH_PATH_INVALID_ENDPOINT;
			continue;
		}
		goal_idx = q->goal_y * map->w + q->goal_x;
		if (start_idx == goal_idx) {
			row->status = DERECH_PATH_FOUND;
			continue;
		}
		if (derech_tile_blocked(map, prof, goal_idx)) {
			if (!allow_partial) {
				row->status = DERECH_PATH_UNREACHABLE;
			} else {
				/* fields cannot help with a blocked goal */
				t1[n1].kind = DERECH_TASK_SOLVE;
				t1[n1].req_idx = i;
				t1[n1].field = NULL;
				n1++;
			}
			continue;
		}
		if (!allow_partial) {
			const derech_labels *lc = derech_labels_for(map, prof);

			if (lc != NULL && labels_prove_unreachable(map, lc,
				start_idx, goal_idx)) {
				row->status = DERECH_PATH_UNREACHABLE;
				continue;
			}
		}
		cached = derech_field_cache_lookup(map, goal_idx,
			DERECH_NO_GOALSET, q->profile_id);
		if (cached != NULL) {
			t1[n1].kind = DERECH_TASK_EXTRACT;
			t1[n1].req_idx = i;
			t1[n1].field = cached;
			n1++;
			continue;
		}
		{
			uint64_t key = ((uint64_t)goal_idx << 32) |
				q->profile_id;
			plan_group *g = plan_group_slot(groups, group_cap,
				key);

			if (!g->used) {
				g->used = 1;
				g->key = key;
			}
			g->count++;
			action[i] = PLAN_GROUP;
		}
	}

	for (uint32_t i = 0; i < n_reqs; i++) {
		const derech_request *q = &reqs[i];
		int is_set = q->goalset != DERECH_NO_GOALSET;
		uint64_t key;
		plan_group *g;

		if (action[i] != PLAN_GROUP) {
			continue;
		}
		if (is_set) {
			key = (((uint64_t)0x80000000u | q->goalset) << 32) |
				q->profile_id;
		} else {
			key = ((uint64_t)(q->goal_y * map->w + q->goal_x)
				<< 32) | q->profile_id;
		}
		g = plan_group_slot(groups, group_cap, key);
		if (!g->decided) {
			g->decided = 1;
			/* set queries are always field-backed — they exist to
			 * be reused; single goals need a crowd to justify it */
			if (is_set || g->count >= map->field_threshold) {
				g->field = derech_field_cache_insert(map,
					is_set ? 0 : q->goal_y * map->w +
						q->goal_x,
					is_set ? q->goalset :
						DERECH_NO_GOALSET,
					q->profile_id);
				if (g->field != NULL) {
					t1[n1].kind = DERECH_TASK_FIELD_BUILD;
					t1[n1].req_idx = 0;
					t1[n1].field = g->field;
					n1++;
				}
			}
		}
		if (g->field != NULL) {
			t2[n2].kind = DERECH_TASK_EXTRACT;
			t2[n2].req_idx = i;
			t2[n2].field = g->field;
			n2++;
		} else if (is_set) {
			/* shell allocation failed: sets have no per-request
			 * fallback search */
			stage[i].oom = 1;
		} else {
			/* insert failure just leaves the group on the classic
			 * A* path */
			t1[n1].kind = DERECH_TASK_SOLVE;
			t1[n1].req_idx = i;
			t1[n1].field = NULL;
			n1++;
		}
	}

	run_tasks(map, t1, n1, reqs, stage);
	run_tasks(map, t2, n2, reqs, stage);
	derech_field_cache_finish_batch(map);
	rc = DERECH_OK;
done:
	free(t1);
	free(t2);
	free(action);
	free(groups);
	return rc;
}

derech_status derech_find_paths(derech_map *map, const derech_request *reqs,
	uint32_t n_reqs, derech_results **out)
{
	derech_results *r;
	derech_stage_row *stage = NULL;
	derech_status rc;

	if (map == NULL || out == NULL || (reqs == NULL && n_reqs > 0)) {
		return DERECH_E_INVALID_ARG;
	}
	*out = NULL;
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	rc = validate_requests(map, reqs, n_reqs);
	if (rc != DERECH_OK) {
		derech_busy_release(&map->busy);
		return rc;
	}

	r = calloc(1, sizeof(*r));
	if (r == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	r->count = n_reqs;
	if (n_reqs == 0) {
		derech_busy_release(&map->busy);
		*out = r;
		return DERECH_OK;
	}

	r->rows = calloc(n_reqs, sizeof(*r->rows));
	stage = calloc(n_reqs, sizeof(*stage));
	if (r->rows == NULL || stage == NULL) {
		free(stage);
		derech_busy_release(&map->busy);
		derech_results_destroy(r);
		return DERECH_E_NOMEM;
	}

	for (uint32_t i = 0; i < map->n_threads; i++) {
		map->ctxs[i].out_len = 0;
	}

	rc = plan_and_execute(map, reqs, n_reqs, stage);
	if (rc != DERECH_OK) {
		free(stage);
		derech_busy_release(&map->busy);
		derech_results_destroy(r);
		return rc;
	}

	rc = assemble_results(map, stage, n_reqs, r);
	free(stage);
	derech_busy_release(&map->busy);
	if (rc != DERECH_OK) {
		derech_results_destroy(r);
		return rc;
	}
	*out = r;
	return DERECH_OK;
}

/* ------------------------------------------------------------------ */
/* Results accessors                                                   */
/* ------------------------------------------------------------------ */

void derech_results_destroy(derech_results *results)
{
	if (results == NULL) {
		return;
	}
	free(results->rows);
	free(results->steps);
	free(results->ticks);
	free(results);
}

uint32_t derech_results_count(const derech_results *results)
{
	return results == NULL ? 0 : results->count;
}

static const derech_res_row *row_at(const derech_results *results,
	uint32_t i)
{
	if (results == NULL || i >= results->count) {
		return NULL;
	}
	return &results->rows[i];
}

derech_path_status derech_result_status(const derech_results *results,
	uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	return row == NULL ? DERECH_PATH_NONE : row->status;
}

uint32_t derech_result_length(const derech_results *results, uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	return row == NULL ? 0 : row->len;
}

const uint32_t *derech_result_steps(const derech_results *results, uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	if (row == NULL || row->len == 0) {
		return NULL;
	}
	return &results->steps[row->off * 2];
}

const float *derech_result_step_ticks(const derech_results *results,
	uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	if (row == NULL || row->len == 0) {
		return NULL;
	}
	return &results->ticks[row->off];
}

float derech_result_total_ticks(const derech_results *results, uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	return row == NULL ? 0.0f : row->total_ticks;
}

float derech_result_total_perceived(const derech_results *results, uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	return row == NULL ? 0.0f : row->total_perceived;
}

uint32_t derech_result_expansions(const derech_results *results, uint32_t i)
{
	const derech_res_row *row = row_at(results, i);

	return row == NULL ? 0 : row->expansions;
}
