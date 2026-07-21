/* Weighted A* kernel with quantized integer costs.
 *
 * Determinism: all costs are Q8 integers, the open list has a total order
 * (f asc, g desc, node index asc), neighbor iteration order is fixed, and
 * duplicates are handled by lazy deletion — so identical inputs produce
 * bitwise-identical paths on every platform.  Each search runs entirely
 * inside one derech_search_ctx, so batch results are also independent of
 * how requests are scheduled across worker threads.
 *
 * A context is reused across searches via generation stamps: a node's
 * g/parent slots are only meaningful when its stamp matches the current
 * generation, so resets are O(1).
 */

#include "derech_internal.h"

/* ------------------------------------------------------------------ */
/* Open list: binary min-heap, lazy deletion                           */
/* ------------------------------------------------------------------ */

static int heap_less(const derech_heap_entry *a, const derech_heap_entry *b)
{
	if (a->f != b->f) {
		return a->f < b->f;
	}
	if (a->g != b->g) {
		return a->g > b->g; /* deeper first: goal-biased tie-break */
	}
	return a->idx < b->idx;
}

static int heap_push(derech_search_ctx *ctx, uint64_t *len,
	derech_heap_entry e)
{
	derech_heap_entry *h;
	uint64_t i = *len;

	if (i == ctx->heap_cap) {
		uint64_t cap = ctx->heap_cap == 0 ? 4096 : ctx->heap_cap * 2;

		if (cap <= ctx->heap_cap || cap > SIZE_MAX / sizeof(*h)) {
			return 0;
		}
		h = realloc(ctx->heap, (size_t)cap * sizeof(*h));
		if (h == NULL) {
			return 0;
		}
		ctx->heap = h;
		ctx->heap_cap = cap;
	}
	h = ctx->heap;
	while (i > 0) {
		uint64_t up = (i - 1) / 2;

		if (!heap_less(&e, &h[up])) {
			break;
		}
		h[i] = h[up];
		i = up;
	}
	h[i] = e;
	(*len)++;
	return 1;
}

static derech_heap_entry heap_pop(derech_search_ctx *ctx, uint64_t *len)
{
	derech_heap_entry *h = ctx->heap;
	derech_heap_entry top = h[0];
	derech_heap_entry e = h[--(*len)];
	uint64_t i = 0;

	while (1) {
		uint64_t l = 2 * i + 1;
		uint64_t r = l + 1;
		uint64_t best = i;

		if (l < *len && heap_less(&h[l], &e)) {
			best = l;
		}
		if (r < *len && heap_less(&h[r], best == i ? &e : &h[l])) {
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

/* ------------------------------------------------------------------ */
/* Heuristic                                                           */
/* ------------------------------------------------------------------ */

static uint64_t heuristic_q(const derech_profile *prof, uint32_t x,
	uint32_t y, uint32_t gx, uint32_t gy)
{
	uint64_t dx = x > gx ? x - gx : gx - x;
	uint64_t dy = y > gy ? y - gy : gy - y;

	if (prof->connectivity == DERECH_CONN_4) {
		return (uint64_t)prof->h_d * (dx + dy);
	}
	/* octile: h_d2 <= 2*h_d is guaranteed at registration, so the
	 * subtracted term never exceeds the straight-line term */
	{
		uint64_t mn = dx < dy ? dx : dy;

		return (uint64_t)prof->h_d * (dx + dy) -
			(uint64_t)(2 * prof->h_d - prof->h_d2) * mn;
	}
}

/* ------------------------------------------------------------------ */
/* Node bookkeeping                                                    */
/* ------------------------------------------------------------------ */

static void touch(derech_search_ctx *ctx, uint32_t idx)
{
	if (ctx->stamp[idx] != ctx->stamp_gen) {
		ctx->stamp[idx] = ctx->stamp_gen;
		ctx->g[idx] = DERECH_G_INFINITE;
		ctx->parent[idx] = 0;
	}
}

/* Corner rule for diagonal direction d out of (x, y): both flanking
 * orthogonal tiles are in bounds whenever the diagonal target is. */
static int corner_forbidden(const derech_map *map,
	const derech_profile *prof, uint32_t idx, uint32_t d)
{
	uint32_t flank_a, flank_b;
	int ba, bb;

	if (prof->corner_rule == DERECH_CORNER_ALLOW) {
		return 0;
	}
	flank_a = derech_dir_dx[d] > 0 ? idx + 1 : idx - 1;
	flank_b = derech_dir_dy[d] > 0 ? idx + map->w : idx - map->w;
	ba = derech_tile_blocked(map, prof, flank_a);
	bb = derech_tile_blocked(map, prof, flank_b);
	if (prof->corner_rule == DERECH_CORNER_STRICT) {
		return ba || bb;
	}
	return ba && bb; /* DERECH_CORNER_LENIENT */
}

/* ------------------------------------------------------------------ */
/* Search                                                              */
/* ------------------------------------------------------------------ */

void derech_search(const derech_map *map, derech_search_ctx *ctx,
	const derech_profile *prof, uint32_t start_idx, uint32_t goal_idx,
	uint32_t eps_q, uint64_t max_expansions, uint64_t max_cost_q,
	derech_search_result *out)
{
	const uint32_t w = map->w;
	const uint32_t gx = goal_idx % w;
	const uint32_t gy = goal_idx / w;
	const uint32_t n_dirs = prof->connectivity == DERECH_CONN_4 ? 4 : 8;
	uint64_t heap_len = 0;
	uint64_t expansions = 0;
	uint64_t best_h;
	uint64_t best_g;
	uint32_t best_idx = start_idx;
	int cost_pruned = 0;
	derech_heap_entry e;

	out->error = DERECH_OK;
	if (derech_cancelled(map)) {
		out->error = DERECH_E_CANCELLED;
		return;
	}

	/* O(1) context reset; wrap resets the stamps for real */
	ctx->stamp_gen++;
	if (ctx->stamp_gen == 0) {
		memset(ctx->stamp, 0, (size_t)map->n * sizeof(*ctx->stamp));
		ctx->stamp_gen = 1;
	}

	touch(ctx, start_idx);
	ctx->g[start_idx] = 0;
	best_h = heuristic_q(prof, start_idx % w, start_idx / w, gx, gy);
	best_g = 0;

	e.f = ((uint64_t)eps_q * best_h) >> 8;
	e.g = 0;
	e.idx = start_idx;
	if (!heap_push(ctx, &heap_len, e)) {
		out->error = DERECH_E_NOMEM;
		return;
	}

	out->status = DERECH_PATH_UNREACHABLE;

	while (heap_len > 0) {
		uint32_t cur, x, y;
		uint64_t h_cur;

		e = heap_pop(ctx, &heap_len);
		cur = e.idx;
		if (e.g != ctx->g[cur]) {
			continue; /* stale duplicate */
		}
		if ((expansions & 255u) == 0 && derech_cancelled(map)) {
			out->error = DERECH_E_CANCELLED;
			return;
		}
		if (cur == goal_idx) {
			out->status = DERECH_PATH_FOUND;
			out->end_idx = goal_idx;
			out->expansions = derech_sat_u32(expansions);
			return;
		}
		if (expansions >= max_expansions) {
			out->status = DERECH_PATH_BUDGET_EXCEEDED;
			out->end_idx = best_idx;
			out->expansions = derech_sat_u32(expansions);
			return;
		}
		expansions++;

		x = cur % w;
		y = cur / w;
		h_cur = heuristic_q(prof, x, y, gx, gy);
		if (h_cur < best_h || (h_cur == best_h && e.g < best_g)) {
			best_h = h_cur;
			best_g = e.g;
			best_idx = cur;
		}

		for (uint32_t d = 0; d < n_dirs; d++) {
			int64_t nx = (int64_t)x + derech_dir_dx[d];
			int64_t ny = (int64_t)y + derech_dir_dy[d];
			uint32_t nb, step_q;
			uint64_t ng;
			derech_heap_entry ne;

			if (nx < 0 || ny < 0 || nx >= (int64_t)w ||
				ny >= (int64_t)map->h) {
				continue;
			}
			nb = (uint32_t)ny * w + (uint32_t)nx;
			if (derech_tile_blocked(map, prof, nb)) {
				continue;
			}
			if (d >= 4 && corner_forbidden(map, prof, cur, d)) {
				continue;
			}
			step_q = derech_perceived_step_q(map, prof, nb, d >= 4);
			ng = e.g + step_q;
			if (ng > max_cost_q) {
				cost_pruned = 1;
				continue;
			}
			touch(ctx, nb);
			if (ng >= ctx->g[nb]) {
				continue;
			}
			ctx->g[nb] = ng;
			ctx->parent[nb] = (uint8_t)(d + 1);
			ne.g = ng;
			ne.idx = nb;
			ne.f = ng + (((uint64_t)eps_q *
				heuristic_q(prof, (uint32_t)nx, (uint32_t)ny,
					gx, gy)) >> 8);
			if (!heap_push(ctx, &heap_len, ne)) {
				out->error = DERECH_E_NOMEM;
				return;
			}
		}
	}

	out->status = cost_pruned ? DERECH_PATH_BUDGET_EXCEEDED :
		DERECH_PATH_UNREACHABLE;
	out->end_idx = best_idx;
	out->expansions = derech_sat_u32(expansions);
}
