/* Goal fields (reverse Dijkstra), the per-map field cache, and
 * connected-component labels.
 *
 * Reverse-graph correctness: the forward edge v->u costs
 * perceived(u, diagonal) — you pay to ENTER u.  So the field Dijkstra
 * runs from the goal and, when settling u, relaxes every in-bounds
 * neighbor v with candidate dist[u] + perceived(u, diag(v,u)).  Corner
 * rules are symmetric in (v, u) — the two flanking tiles are the same
 * set seen from either end — so they can be checked from u with the
 * reversed direction.  Tiles blocked for the profile still RECEIVE
 * distances (a blocked start is escapable, matching A*) but are never
 * expanded (no forward edge enters them).
 *
 * All cache and label mutation happens in the single-threaded planning
 * phase of derech_find_paths; builds and extractions run on workers but
 * touch only their own field / ctx. */

#include "derech_internal.h"

/* Direction index pointing the opposite way (E<->W, SE<->NW, ...). */
static const uint8_t DIR_OPPOSITE[8] = { 1, 0, 3, 2, 7, 6, 5, 4 };

/* ------------------------------------------------------------------ */
/* Field build: reverse Dijkstra                                       */
/* ------------------------------------------------------------------ */

/* Reuses the ctx heap (entries carry f = g = dist for the total order). */
static int field_heap_push(derech_search_ctx *ctx, uint64_t *len,
	uint32_t dist, uint32_t idx)
{
	derech_heap_entry *h;
	derech_heap_entry e;
	uint64_t i = *len;

	if (i == ctx->heap_cap) {
		uint64_t cap = ctx->heap_cap * 2;

		h = realloc(ctx->heap, (size_t)cap * sizeof(*h));
		if (h == NULL) {
			return 0;
		}
		ctx->heap = h;
		ctx->heap_cap = cap;
	}
	e.f = dist;
	e.g = dist;
	e.idx = idx;
	h = ctx->heap;
	while (i > 0) {
		uint64_t up = (i - 1) / 2;

		if (!(e.f < h[up].f || (e.f == h[up].f && e.idx < h[up].idx))) {
			break;
		}
		h[i] = h[up];
		i = up;
	}
	h[i] = e;
	(*len)++;
	return 1;
}

static derech_heap_entry field_heap_pop(derech_search_ctx *ctx,
	uint64_t *len)
{
	derech_heap_entry *h = ctx->heap;
	derech_heap_entry top = h[0];
	derech_heap_entry e = h[--(*len)];
	uint64_t i = 0;

	while (1) {
		uint64_t l = 2 * i + 1;
		uint64_t r = l + 1;
		uint64_t best = i;
		const derech_heap_entry *cmp = &e;

		if (l < *len && (h[l].f < cmp->f ||
			(h[l].f == cmp->f && h[l].idx < cmp->idx))) {
			best = l;
			cmp = &h[l];
		}
		if (r < *len && (h[r].f < cmp->f ||
			(h[r].f == cmp->f && h[r].idx < cmp->idx))) {
			best = r;
		}
		if (best == i) {
			break;
		}
		h[i] = h[best];
		i = best;
	}
	h[i] = e;
	return top;
}

/* Corner rule for the diagonal pair (v, u), evaluated from u with the
 * direction u->v; the flanks are the same two tiles either way. */
static int field_corner_forbidden(const derech_map *map,
	const derech_profile *prof, uint32_t u_idx, uint32_t d)
{
	uint32_t flank_a, flank_b;
	int ba, bb;

	if (prof->corner_rule == DERECH_CORNER_ALLOW) {
		return 0;
	}
	flank_a = derech_dir_dx[d] > 0 ? u_idx + 1 : u_idx - 1;
	flank_b = derech_dir_dy[d] > 0 ? u_idx + map->w : u_idx - map->w;
	ba = derech_tile_blocked(map, prof, flank_a);
	bb = derech_tile_blocked(map, prof, flank_b);
	if (prof->corner_rule == DERECH_CORNER_STRICT) {
		return ba || bb;
	}
	return ba && bb; /* DERECH_CORNER_LENIENT */
}

void derech_field_build(const derech_map *map, derech_search_ctx *ctx,
	derech_field *field)
{
	const derech_profile *prof = &map->profiles[field->profile_id];
	const uint32_t w = map->w;
	const uint32_t n_dirs = prof->connectivity == DERECH_CONN_4 ? 4 : 8;
	uint64_t heap_len = 0;

	for (uint32_t i = 0; i < map->n; i++) {
		field->dist_q[i] = DERECH_G_INFINITE;
	}
	memset(field->next_dir, 0, map->n);

	if (field->goalset_id != DERECH_NO_GOALSET) {
		/* multi-source: every seed starts at 0.  Blocked seeds keep
		 * their zero (a start ON one is trivially arrived) but never
		 * expand — no forward edge can enter them. */
		const derech_goalset *gs =
			&map->goalsets[field->goalset_id - 1];

		for (uint32_t idx = 0; idx < map->n; idx++) {
			if (((gs->seeds[idx / 64] >> (idx % 64)) & 1) == 0) {
				continue;
			}
			field->dist_q[idx] = 0;
			if (!derech_tile_blocked(map, prof, idx) &&
				!field_heap_push(ctx, &heap_len, 0, idx)) {
				return; /* ok stays 0 */
			}
		}
	} else {
		/* an unenterable goal has no forward edges into it: the
		 * all-infinite field is the correct (and complete) answer */
		if (derech_tile_blocked(map, prof, field->goal_idx)) {
			field->ok = 1;
			return;
		}
		field->dist_q[field->goal_idx] = 0;
		if (!field_heap_push(ctx, &heap_len, 0, field->goal_idx)) {
			return; /* ok stays 0; requests fall back to A* */
		}
	}

	while (heap_len > 0) {
		derech_heap_entry e = field_heap_pop(ctx, &heap_len);
		uint32_t u = e.idx;
		uint32_t x, y;
		uint32_t perc_straight, perc_diag;

		if (e.f != field->dist_q[u]) {
			continue; /* stale duplicate */
		}
		x = u % w;
		y = u / w;
		perc_straight = derech_perceived_step_q(map, prof, u, 0);
		perc_diag = derech_perceived_step_q(map, prof, u, 1);

		for (uint32_t d = 0; d < n_dirs; d++) {
			int64_t vx = (int64_t)x + derech_dir_dx[d];
			int64_t vy = (int64_t)y + derech_dir_dy[d];
			uint32_t v, step_q;
			uint64_t cand64;
			uint32_t cand;

			if (vx < 0 || vy < 0 || vx >= (int64_t)w ||
				vy >= (int64_t)map->h) {
				continue;
			}
			v = (uint32_t)vy * w + (uint32_t)vx;
			if (d >= 4 &&
				field_corner_forbidden(map, prof, u, d)) {
				continue;
			}
			step_q = d >= 4 ? perc_diag : perc_straight;
			cand64 = (uint64_t)field->dist_q[u] + step_q;
			if (cand64 >= DERECH_G_INFINITE) {
				cand = DERECH_G_INFINITE - 1; /* saturate */
			} else {
				cand = (uint32_t)cand64;
			}
			if (cand >= field->dist_q[v]) {
				continue;
			}
			field->dist_q[v] = cand;
			field->next_dir[v] = (uint8_t)(DIR_OPPOSITE[d] + 1);
			/* blocked tiles receive distances but never expand:
			 * no forward edge can enter them */
			if (derech_tile_blocked(map, prof, v)) {
				continue;
			}
			if (!field_heap_push(ctx, &heap_len, cand, v)) {
				return; /* ok stays 0 */
			}
		}
	}
	field->ok = 1;
}

/* ------------------------------------------------------------------ */
/* Extraction                                                          */
/* ------------------------------------------------------------------ */

/* Answer a request from a finished field, falling back to a classic
 * solve for the semantics a field cannot provide (partial paths, and
 * anything if the build failed). */
void derech_field_extract(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_field *field, const derech_request *req,
	derech_stage_row *row)
{
	const derech_profile *prof = &map->profiles[field->profile_id];
	int allow_partial = (req->flags & DERECH_REQ_ALLOW_PARTIAL) != 0;
	uint32_t start_idx = req->start_y * map->w + req->start_x;
	uint32_t dist;
	uint32_t max_cost_q;

	row->worker = worker;
	if (!field->ok) {
		/* build failure is an allocation failure; set queries have
		 * no per-request search to fall back to */
		if (req->goalset != DERECH_NO_GOALSET) {
			row->oom = 1;
			return;
		}
		derech_solve_request(map, ctx, worker, req, row);
		return;
	}
	dist = field->dist_q[start_idx];

	if (dist == DERECH_G_INFINITE) {
		/* the field is exact: this start genuinely cannot reach the
		 * goal.  Only a closest-approach partial needs a search (and
		 * planning rejects the partial flag for set queries). */
		if (allow_partial && req->goalset == DERECH_NO_GOALSET) {
			derech_solve_request(map, ctx, worker, req, row);
			return;
		}
		row->status = DERECH_PATH_UNREACHABLE;
		return;
	}

	max_cost_q = req->max_perceived_cost == 0.0f ? UINT32_MAX :
		derech_q_round((double)req->max_perceived_cost * 256.0,
			UINT32_MAX - 1);
	if (dist > max_cost_q) {
		if (allow_partial && req->goalset == DERECH_NO_GOALSET) {
			derech_solve_request(map, ctx, worker, req, row);
			return;
		}
		row->status = DERECH_PATH_BUDGET_EXCEEDED;
		return;
	}

	/* walk the next-hop chain; dist strictly decreases along it, so it
	 * terminates at a seed (next_dir == 0, dist == 0) in at most n
	 * steps — for single-goal fields the goal is that seed */
	{
		int32_t offs[8];
		uint32_t idx = start_idx;
		uint32_t depth = 0;
		uint64_t total_true_q = 0;

		for (uint32_t d = 0; d < 8; d++) {
			offs[d] = (int32_t)derech_dir_dx[d] +
				(int32_t)derech_dir_dy[d] * (int32_t)map->w;
		}
		while (field->next_dir[idx] != 0 && depth < map->n) {
			uint32_t d = (uint32_t)field->next_dir[idx] - 1;

			idx = (uint32_t)((int64_t)idx + offs[d]);
			if (!derech_ctx_out_reserve(ctx, 1)) {
				ctx->out_len -= depth;
				row->oom = 1;
				return;
			}
			{
				uint32_t q = derech_true_step_q(map, prof,
					idx, d >= 4);
				uint64_t at = ctx->out_len;

				ctx->out_steps[at * 2] = idx % map->w;
				ctx->out_steps[at * 2 + 1] = idx / map->w;
				ctx->out_ticks[at] =
					(float)((double)q / 256.0);
				ctx->out_len++;
				total_true_q += q;
			}
			depth++;
		}
		if (field->dist_q[idx] != 0) {
			/* impossible unless the field is corrupt; be safe */
			ctx->out_len -= depth;
			if (req->goalset != DERECH_NO_GOALSET) {
				row->oom = 1;
				return;
			}
			derech_solve_request(map, ctx, worker, req, row);
			return;
		}
		row->status = DERECH_PATH_FOUND;
		row->len = depth;
		row->local_off = ctx->out_len - depth;
		row->true_q = total_true_q;
		row->perceived_q = dist;
		row->expansions = 0;
	}
}

/* ------------------------------------------------------------------ */
/* Field cache (planning phase only)                                   */
/* ------------------------------------------------------------------ */

/* Node indices stay below 2^31, so set ids get the high bit. */
static uint32_t field_key(uint32_t goal_idx, uint32_t goalset_id)
{
	return goalset_id != DERECH_NO_GOALSET ?
		(0x80000000u | goalset_id) : goal_idx;
}

static uint32_t field_hash(uint32_t key, uint32_t profile_id)
{
	uint64_t x = ((uint64_t)key << 32) | profile_id;

	x ^= x >> 30;
	x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27;
	x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	return (uint32_t)x & (DERECH_FIELD_HASH_BUCKETS - 1);
}

static void field_lru_unlink(derech_map *map, derech_field *f)
{
	if (f->lru_prev != NULL) {
		f->lru_prev->lru_next = f->lru_next;
	} else {
		map->field_lru_head = f->lru_next;
	}
	if (f->lru_next != NULL) {
		f->lru_next->lru_prev = f->lru_prev;
	} else {
		map->field_lru_tail = f->lru_prev;
	}
	f->lru_prev = NULL;
	f->lru_next = NULL;
}

static void field_lru_push_front(derech_map *map, derech_field *f)
{
	f->lru_prev = NULL;
	f->lru_next = map->field_lru_head;
	if (map->field_lru_head != NULL) {
		map->field_lru_head->lru_prev = f;
	}
	map->field_lru_head = f;
	if (map->field_lru_tail == NULL) {
		map->field_lru_tail = f;
	}
}

static void field_destroy(derech_map *map, derech_field *f)
{
	uint32_t b = field_hash(field_key(f->goal_idx, f->goalset_id),
		f->profile_id);
	derech_field **link = &map->field_hash[b];

	while (*link != NULL && *link != f) {
		link = &(*link)->hash_next;
	}
	if (*link == f) {
		*link = f->hash_next;
	}
	field_lru_unlink(map, f);
	map->field_bytes -= f->bytes;
	free(f->dist_q);
	free(f->next_dir);
	free(f);
}

void derech_field_cache_flush(derech_map *map)
{
	while (map->field_lru_head != NULL) {
		field_destroy(map, map->field_lru_head);
	}
}

/* Evict unpinned fields from the LRU tail until within budget. */
static void field_cache_trim(derech_map *map)
{
	derech_field *f = map->field_lru_tail;

	while (map->field_bytes > map->field_budget_bytes && f != NULL) {
		derech_field *prev = f->lru_prev;

		if (!f->pinned) {
			field_destroy(map, f);
		}
		f = prev;
	}
}

derech_field *derech_field_cache_lookup(derech_map *map, uint32_t goal_idx,
	uint32_t goalset_id, uint32_t profile_id)
{
	uint32_t key = field_key(goal_idx, goalset_id);
	derech_field *f = map->field_hash[field_hash(key, profile_id)];

	while (f != NULL) {
		if (f->goal_idx == goal_idx && f->goalset_id == goalset_id &&
			f->profile_id == profile_id) {
			field_lru_unlink(map, f);
			field_lru_push_front(map, f);
			f->pinned = 1;
			return f;
		}
		f = f->hash_next;
	}
	return NULL;
}

/* Allocate and register a field shell (dist/next arrays included) for
 * this batch to build.  Returns NULL on allocation failure — callers
 * then just leave the group on the classic A* path. */
derech_field *derech_field_cache_insert(derech_map *map, uint32_t goal_idx,
	uint32_t goalset_id, uint32_t profile_id)
{
	derech_field *f = calloc(1, sizeof(*f));
	uint32_t b;

	if (f == NULL) {
		return NULL;
	}
	f->goal_idx = goal_idx;
	f->goalset_id = goalset_id;
	if (goalset_id != DERECH_NO_GOALSET) {
		f->set_epoch = map->goalsets[goalset_id - 1].epoch;
	}
	f->profile_id = profile_id;
	f->dist_q = malloc((size_t)map->n * sizeof(*f->dist_q));
	f->next_dir = malloc(map->n);
	f->bytes = sizeof(*f) + (uint64_t)map->n * 5;
	if (f->dist_q == NULL || f->next_dir == NULL) {
		free(f->dist_q);
		free(f->next_dir);
		free(f);
		return NULL;
	}
	f->pinned = 1;
	b = field_hash(field_key(goal_idx, goalset_id), profile_id);
	f->hash_next = map->field_hash[b];
	map->field_hash[b] = f;
	field_lru_push_front(map, f);
	map->field_bytes += f->bytes;
	field_cache_trim(map); /* never evicts pinned entries */
	return f;
}

/* Unpin everything, drop failed builds, and trim to budget. */
void derech_field_cache_finish_batch(derech_map *map)
{
	derech_field *f = map->field_lru_head;

	while (f != NULL) {
		derech_field *next = f->lru_next;

		f->pinned = 0;
		if (!f->ok) {
			field_destroy(map, f);
		}
		f = next;
	}
	field_cache_trim(map);
}

/* ------------------------------------------------------------------ */
/* Component labels (planning phase only)                              */
/* ------------------------------------------------------------------ */

void derech_labels_flush(derech_map *map)
{
	for (uint32_t i = 0; i < map->label_class_count; i++) {
		free(map->label_classes[i].label);
	}
	map->label_class_count = 0;
}

/* Enterable for the blocking class, ignoring per-profile weights. */
static int class_enterable(const derech_map *map, uint64_t block_mask,
	uint64_t require_mask, uint32_t idx)
{
	uint64_t word;

	if (map->cost_q[idx] == DERECH_Q_BLOCKED) {
		return 0;
	}
	word = map->combo_words[map->combo_idx[idx]];
	if ((word & block_mask) != 0) {
		return 0;
	}
	return require_mask == 0 || (word & require_mask) != 0;
}

/* Flood-fill labels over the superset graph (8-connected, corners
 * allowed).  Uses ctx 0's path_scratch as the BFS queue — only legal
 * during the single-threaded planning phase. */
static derech_labels *labels_build(derech_map *map, uint64_t block_mask,
	uint64_t require_mask)
{
	derech_labels *lc = &map->label_classes[map->label_class_count];
	uint32_t *queue = map->ctxs[0].path_scratch;
	uint32_t next_label = 0;

	lc->block_mask = block_mask;
	lc->require_mask = require_mask;
	lc->label = calloc(map->n, sizeof(*lc->label));
	if (lc->label == NULL) {
		return NULL;
	}
	for (uint32_t seed = 0; seed < map->n; seed++) {
		uint64_t head = 0, tail = 0;

		if (lc->label[seed] != 0 ||
			!class_enterable(map, block_mask, require_mask, seed)) {
			continue;
		}
		next_label++;
		lc->label[seed] = next_label;
		queue[tail++] = seed;
		while (head < tail) {
			uint32_t cur = queue[head++];
			uint32_t x = cur % map->w;
			uint32_t y = cur / map->w;

			for (uint32_t d = 0; d < 8; d++) {
				int64_t nx = (int64_t)x + derech_dir_dx[d];
				int64_t ny = (int64_t)y + derech_dir_dy[d];
				uint32_t nb;

				if (nx < 0 || ny < 0 ||
					nx >= (int64_t)map->w ||
					ny >= (int64_t)map->h) {
					continue;
				}
				nb = (uint32_t)ny * map->w + (uint32_t)nx;
				if (lc->label[nb] != 0 ||
					!class_enterable(map, block_mask,
						require_mask, nb)) {
					continue;
				}
				lc->label[nb] = next_label;
				queue[tail++] = nb;
			}
		}
	}
	map->label_class_count++;
	return lc;
}

/* Labels for the profile's blocking class, building them on first use.
 * Returns NULL when the class cache is full or allocation fails — the
 * caller simply skips the quick check. */
const derech_labels *derech_labels_for(derech_map *map,
	const derech_profile *prof)
{
	for (uint32_t i = 0; i < map->label_class_count; i++) {
		if (map->label_classes[i].block_mask == prof->block_mask &&
			map->label_classes[i].require_mask ==
				prof->require_mask) {
			return &map->label_classes[i];
		}
	}
	if (map->label_class_count >= DERECH_MAX_LABEL_CLASSES) {
		return NULL;
	}
	return labels_build(map, prof->block_mask, prof->require_mask);
}

/* ------------------------------------------------------------------ */
/* Goal sets (registration + planning-phase maintenance)               */
/* ------------------------------------------------------------------ */

void derech_goalset_free(derech_goalset *gs)
{
	free(gs->tiles);
	free(gs->members);
	free(gs->seeds);
	memset(gs, 0, sizeof(*gs));
}

static void goalset_seed_tile(const derech_map *map, uint64_t *seeds,
	uint32_t idx, int adjacent)
{
	seeds[idx / 64] |= 1ULL << (idx % 64);
	if (!adjacent) {
		return;
	}
	{
		uint32_t x = idx % map->w;
		uint32_t y = idx / map->w;

		for (uint32_t d = 0; d < 8; d++) {
			int64_t nx = (int64_t)x + derech_dir_dx[d];
			int64_t ny = (int64_t)y + derech_dir_dy[d];
			uint32_t nb;

			if (nx < 0 || ny < 0 || nx >= (int64_t)map->w ||
				ny >= (int64_t)map->h) {
				continue;
			}
			nb = (uint32_t)ny * map->w + (uint32_t)nx;
			seeds[nb / 64] |= 1ULL << (nb % 64);
		}
	}
}

static int goalset_tile_matches(const derech_map *map,
	const derech_goalset *gs, uint32_t idx)
{
	uint64_t word = map->combo_words[map->combo_idx[idx]];

	if (gs->any_mask != 0 && (word & gs->any_mask) == 0) {
		return 0;
	}
	return (word & gs->all_mask) == gs->all_mask;
}

/* (Re)compute members + seeds from current map state.  Returns 0 on
 * allocation failure, 1 otherwise; *changed reports whether effective
 * membership differs from before. */
int derech_goalset_materialize(derech_map *map, derech_goalset *gs)
{
	uint32_t words = (map->n + 63) / 64;
	int changed = 0;

	if (gs->members == NULL) {
		gs->members = calloc(words, sizeof(uint64_t));
		gs->seeds = calloc(words, sizeof(uint64_t));
		if (gs->members == NULL || gs->seeds == NULL) {
			return 0;
		}
		changed = 1;
	}

	{
		uint32_t count = 0;

		if (gs->is_predicate) {
			for (uint32_t w = 0; w < words; w++) {
				uint64_t nw = 0;
				uint32_t base = w * 64;
				uint32_t top = base + 64 > map->n ?
					map->n - base : 64;

				for (uint32_t b = 0; b < top; b++) {
					if (goalset_tile_matches(map, gs,
						base + b)) {
						nw |= 1ULL << b;
					}
				}
				if (gs->members[w] != nw) {
					changed = 1;
					gs->members[w] = nw;
				}
			}
			for (uint32_t i = 0; i < map->n; i++) {
				count += (uint32_t)((gs->members[i / 64] >>
					(i % 64)) & 1);
			}
		} else {
			memset(gs->members, 0, (size_t)words *
				sizeof(uint64_t));
			for (uint32_t t = 0; t < gs->n_tiles; t++) {
				uint32_t idx = gs->tiles[t * 2 + 1] * map->w +
					gs->tiles[t * 2];

				gs->members[idx / 64] |= 1ULL << (idx % 64);
			}
			for (uint32_t i = 0; i < map->n; i++) {
				count += (uint32_t)((gs->members[i / 64] >>
					(i % 64)) & 1);
			}
		}
		gs->member_count = count;
	}

	memset(gs->seeds, 0, (size_t)((map->n + 63) / 64) * sizeof(uint64_t));
	for (uint32_t i = 0; i < map->n; i++) {
		if ((gs->members[i / 64] >> (i % 64)) & 1) {
			goalset_seed_tile(map, gs->seeds, i,
				(gs->flags & DERECH_GOALSET_ADJACENT) != 0);
		}
	}
	if (changed) {
		gs->epoch++;
	}
	return 1;
}

static int32_t goalset_register_common(derech_map *map, derech_goalset tmp)
{
	uint32_t slot;

	if (!derech_busy_acquire(&map->busy)) {
		free(tmp.tiles);
		return DERECH_E_BUSY;
	}
	for (slot = 0; slot < DERECH_MAX_GOALSETS; slot++) {
		if (!map->goalsets[slot].used) {
			break;
		}
	}
	if (slot == DERECH_MAX_GOALSETS) {
		derech_busy_release(&map->busy);
		free(tmp.tiles);
		return DERECH_E_TOO_MANY_GOALSETS;
	}
	map->goalsets[slot] = tmp;
	map->goalsets[slot].used = 1;
	if (!derech_goalset_materialize(map, &map->goalsets[slot])) {
		derech_goalset_free(&map->goalsets[slot]);
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	derech_busy_release(&map->busy);
	return (int32_t)slot + 1;
}

int32_t derech_goalset_register(derech_map *map, const uint32_t *xy_pairs,
	uint32_t n_tiles, uint32_t flags)
{
	derech_goalset tmp;

	if (map == NULL || xy_pairs == NULL || n_tiles == 0 ||
		(flags & ~(uint32_t)DERECH_GOALSET_ADJACENT) != 0) {
		return DERECH_E_INVALID_ARG;
	}
	for (uint32_t t = 0; t < n_tiles; t++) {
		if (xy_pairs[t * 2] >= map->w || xy_pairs[t * 2 + 1] >= map->h) {
			return DERECH_E_OOB;
		}
	}
	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = flags;
	tmp.n_tiles = n_tiles;
	tmp.tiles = malloc((size_t)n_tiles * 2 * sizeof(*tmp.tiles));
	if (tmp.tiles == NULL) {
		return DERECH_E_NOMEM;
	}
	memcpy(tmp.tiles, xy_pairs, (size_t)n_tiles * 2 * sizeof(*tmp.tiles));
	return goalset_register_common(map, tmp);
}

int32_t derech_goalset_register_tags(derech_map *map, uint64_t any_mask,
	uint64_t all_mask, uint32_t flags)
{
	derech_goalset tmp;

	if (map == NULL || (any_mask == 0 && all_mask == 0) ||
		(flags & ~(uint32_t)DERECH_GOALSET_ADJACENT) != 0) {
		return DERECH_E_INVALID_ARG;
	}
	memset(&tmp, 0, sizeof(tmp));
	tmp.is_predicate = 1;
	tmp.flags = flags;
	tmp.any_mask = any_mask;
	tmp.all_mask = all_mask;
	return goalset_register_common(map, tmp);
}

derech_status derech_goalset_unregister(derech_map *map, uint32_t id)
{
	derech_field *f;

	if (map == NULL || id == 0 || id > DERECH_MAX_GOALSETS ||
		!map->goalsets[id - 1].used) {
		return DERECH_E_BAD_GOALSET;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	f = map->field_lru_head;
	while (f != NULL) {
		derech_field *next = f->lru_next;

		if (f->goalset_id == id) {
			field_destroy(map, f);
		}
		f = next;
	}
	derech_goalset_free(&map->goalsets[id - 1]);
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

int64_t derech_goalset_count(derech_map *map, uint32_t id)
{
	derech_goalset *gs;
	int64_t count = 0;

	if (map == NULL || id == 0 || id > DERECH_MAX_GOALSETS ||
		!map->goalsets[id - 1].used) {
		return DERECH_E_BAD_GOALSET;
	}
	gs = &map->goalsets[id - 1];
	if (!gs->is_predicate) {
		return gs->n_tiles;
	}
	/* live scan: reflects all edits so far without mutating any state */
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	for (uint32_t i = 0; i < map->n; i++) {
		count += goalset_tile_matches(map, gs, i);
	}
	derech_busy_release(&map->busy);
	return count;
}

/* ------------------------------------------------------------------ */
/* Reconcile: apply the dirty log (planning phase only)                */
/* ------------------------------------------------------------------ */

/* Could this edit rect affect the field?  Only if the profile cares
 * about what changed AND the (1-dilated) rect touches the field's
 * reachable area — an edit can only alter distances by changing a tile
 * in, or adjacent to, the region the field covers. */
static int field_affected_by(const derech_map *map, const derech_field *f,
	const derech_dirty_rect *r)
{
	const derech_profile *prof = &map->profiles[f->profile_id];
	uint32_t x0, y0, x1, y1;

	if (!r->pass_changed &&
		(r->tag_bits & prof->relevant_mask) == 0) {
		return 0;
	}
	x0 = r->x > 0 ? r->x - 1 : 0;
	y0 = r->y > 0 ? r->y - 1 : 0;
	x1 = r->x + r->w + 1 < map->w ? r->x + r->w + 1 : map->w;
	y1 = r->y + r->h + 1 < map->h ? r->y + r->h + 1 : map->h;
	if ((uint64_t)(x1 - x0) * (y1 - y0) > map->n / 4) {
		return 1; /* big rect: skip the scan, assume affected */
	}
	for (uint32_t y = y0; y < y1; y++) {
		for (uint32_t x = x0; x < x1; x++) {
			if (f->dist_q[y * map->w + x] != DERECH_G_INFINITE) {
				return 1;
			}
		}
	}
	return 0;
}

void derech_reconcile(derech_map *map)
{
	derech_dirty *d = &map->dirty;
	uint64_t all_bits = 0;
	int any_pass = 0;

	if (d->count == 0 && !d->full) {
		return;
	}
	for (uint32_t r = 0; r < d->count; r++) {
		all_bits |= d->rects[r].tag_bits;
		any_pass |= d->rects[r].pass_changed;
	}
	if (d->full) {
		all_bits = ~0ULL;
		any_pass = 1;
	}

	/* predicate memberships (bumps epoch when membership changed) */
	for (uint32_t s = 0; s < DERECH_MAX_GOALSETS; s++) {
		derech_goalset *gs = &map->goalsets[s];

		if (gs->used && gs->is_predicate &&
			(all_bits & (gs->any_mask | gs->all_mask)) != 0) {
			/* allocation failure leaves the old membership in
			 * place; epoch-based invalidation still guards it */
			(void)derech_goalset_materialize(map, gs);
		}
	}

	/* fields: epoch mismatch (set membership moved) or a relevant edit
	 * touching the reachable area */
	{
		derech_field *f = map->field_lru_head;

		while (f != NULL) {
			derech_field *next = f->lru_next;
			int kill = 0;

			if (f->goalset_id != DERECH_NO_GOALSET &&
				map->goalsets[f->goalset_id - 1].epoch !=
					f->set_epoch) {
				kill = 1;
			} else if (d->full) {
				kill = 1;
			} else {
				for (uint32_t r = 0; r < d->count; r++) {
					if (field_affected_by(map, f,
						&d->rects[r])) {
						kill = 1;
						break;
					}
				}
			}
			if (kill) {
				field_destroy(map, f);
			}
			f = next;
		}
	}

	/* labels: any enterability-relevant change anywhere can restructure
	 * components globally, so affected classes flush outright */
	{
		uint32_t i = 0;

		while (i < map->label_class_count) {
			derech_labels *lc = &map->label_classes[i];

			if (any_pass || (all_bits &
				(lc->block_mask | lc->require_mask)) != 0) {
				free(lc->label);
				map->label_class_count--;
				*lc = map->label_classes[
					map->label_class_count];
				continue;
			}
			i++;
		}
	}

	d->count = 0;
	d->full = 0;
}
