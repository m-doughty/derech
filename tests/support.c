#include <stdlib.h>

#include "support.h"

/* ------------------------------------------------------------------ */
/* Harness                                                             */
/* ------------------------------------------------------------------ */

int t_checks = 0;
int t_fails = 0;

void t_check_impl(int ok, const char *expr, const char *file, int line)
{
	t_checks++;
	if (!ok) {
		t_fails++;
		fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
	}
}

int t_done(const char *suite)
{
	printf("%s: %d checks, %d failures\n", suite, t_checks, t_fails);
	return t_fails == 0 ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* PRNG                                                                */
/* ------------------------------------------------------------------ */

uint64_t t_rng_next(t_rng *r)
{
	uint64_t z = (r->s += 0x9e3779b97f4a7c15ULL);

	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
	z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
	return z ^ (z >> 31);
}

uint32_t t_rng_below(t_rng *r, uint32_t bound)
{
	return (uint32_t)(t_rng_next(r) % bound);
}

/* ------------------------------------------------------------------ */
/* Reference cost model (mirrors the spec in derech.h)                 */
/* ------------------------------------------------------------------ */

static const int8_t R_DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
static const int8_t R_DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

static uint32_t r_round(double v, uint32_t cap)
{
	if (v >= (double)cap) {
		return cap;
	}
	if (v <= 0.0) {
		return 0;
	}
	return (uint32_t)(v + 0.5);
}

static uint32_t r_pass_q(float p)
{
	if (p <= 0.0f) {
		return 0;
	}
	return r_round(256.0 / (double)p, 1u << 24);
}

typedef struct r_entry {
	int blocked;
	uint32_t mult_q;
	uint32_t add_q;
} r_entry;

static r_entry r_fold(const derech_profile_desc *d, uint64_t word)
{
	r_entry e = { 0, 0, 0 };
	double mult = 1.0;
	double add = 0.0;

	if ((word & d->block_mask) != 0 ||
		(d->require_mask != 0 && (word & d->require_mask) == 0)) {
		e.blocked = 1;
		return e;
	}
	for (uint32_t b = 0; b < 64; b++) {
		if ((word >> b) & 1) {
			mult *= d->tag_mult[b] == 0.0f ? 1.0 :
				(double)d->tag_mult[b];
			add += (double)d->tag_add[b];
		}
	}
	e.mult_q = r_round(mult * 256.0, 1u << 28);
	e.add_q = r_round(add * 256.0, 1u << 28);
	return e;
}

static uint32_t r_diag_q(const derech_profile_desc *d)
{
	if (d->diagonal_mult == 0.0f) {
		return 362;
	}
	return r_round((double)d->diagonal_mult * 256.0, 1u << 13);
}

static uint64_t r_word_at(const ref_map *m, uint32_t idx)
{
	return m->tags == NULL ? 0 : m->tags[idx];
}

static int r_blocked(const ref_map *m, const derech_profile_desc *d,
	uint32_t idx)
{
	if (r_pass_q(m->pass[idx]) == 0) {
		return 1;
	}
	return r_fold(d, r_word_at(m, idx)).blocked;
}

/* True Q8 base cost of stepping onto idx. */
static uint64_t r_true_q(const ref_map *m, const derech_profile_desc *d,
	uint32_t idx, int diagonal)
{
	uint64_t q = r_pass_q(m->pass[idx]);

	if (diagonal) {
		q = (q * r_diag_q(d)) >> 8;
	}
	return q;
}

static uint64_t r_perceived_q(const ref_map *m, const derech_profile_desc *d,
	uint32_t idx, int diagonal)
{
	r_entry e = r_fold(d, r_word_at(m, idx));
	uint64_t q = r_true_q(m, d, idx, diagonal);

	q = ((q * e.mult_q) >> 8) + e.add_q;
	return q < 1 ? 1 : q;
}

static int r_corner_forbidden(const ref_map *m, const derech_profile_desc *d,
	uint32_t x, uint32_t y, int8_t dx, int8_t dy)
{
	uint32_t fa, fb;
	int ba, bb;

	if (d->corner_rule == DERECH_CORNER_ALLOW) {
		return 0;
	}
	fa = y * m->w + (uint32_t)((int64_t)x + dx);
	fb = (uint32_t)((int64_t)y + dy) * m->w + x;
	ba = r_blocked(m, d, fa);
	bb = r_blocked(m, d, fb);
	if (d->corner_rule == DERECH_CORNER_STRICT) {
		return ba || bb;
	}
	return ba && bb;
}

uint64_t *ref_solve_field(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy)
{
	uint32_t n = m->w * m->h;
	uint64_t *dist = malloc((size_t)n * sizeof(*dist));
	uint8_t *done = calloc(n, 1);
	uint32_t n_dirs = d->connectivity == DERECH_CONN_4 ? 4 : 8;

	if (dist == NULL || done == NULL) {
		free(dist);
		free(done);
		return NULL;
	}
	for (uint32_t i = 0; i < n; i++) {
		dist[i] = REF_INF;
	}
	dist[sy * m->w + sx] = 0;

	for (;;) {
		uint64_t best = REF_INF;
		uint32_t cur = 0;

		for (uint32_t i = 0; i < n; i++) {
			if (!done[i] && dist[i] < best) {
				best = dist[i];
				cur = i;
			}
		}
		if (best == REF_INF) {
			break;
		}
		done[cur] = 1;
		{
			uint32_t x = cur % m->w;
			uint32_t y = cur / m->w;

			for (uint32_t dd = 0; dd < n_dirs; dd++) {
				int64_t nx = (int64_t)x + R_DX[dd];
				int64_t ny = (int64_t)y + R_DY[dd];
				uint32_t nb;
				uint64_t nd;

				if (nx < 0 || ny < 0 || nx >= (int64_t)m->w ||
					ny >= (int64_t)m->h) {
					continue;
				}
				nb = (uint32_t)ny * m->w + (uint32_t)nx;
				if (r_blocked(m, d, nb)) {
					continue;
				}
				if (dd >= 4 && r_corner_forbidden(m, d, x, y,
					R_DX[dd], R_DY[dd])) {
					continue;
				}
				nd = dist[cur] +
					r_perceived_q(m, d, nb, dd >= 4);
				if (nd < dist[nb]) {
					dist[nb] = nd;
				}
			}
		}
	}
	free(done);
	return dist;
}

uint64_t ref_solve(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy)
{
	uint64_t *field = ref_solve_field(m, d, sx, sy);
	uint64_t v;

	if (field == NULL) {
		return REF_INF;
	}
	v = field[gy * m->w + gx];
	free(field);
	return v;
}

/* ------------------------------------------------------------------ */
/* Builders                                                            */
/* ------------------------------------------------------------------ */

derech_profile_desc t_neutral_desc(void)
{
	derech_profile_desc d;

	memset(&d, 0, sizeof(d));
	d.struct_size = (uint32_t)sizeof(d);
	return d;
}

derech_map *t_build_map(const ref_map *m)
{
	derech_map *map = derech_map_create(m->w, m->h, NULL);
	uint64_t n = (uint64_t)m->w * m->h;

	if (map == NULL) {
		return NULL;
	}
	if (derech_map_set_passability(map, m->pass, n) != DERECH_OK) {
		derech_map_destroy(map);
		return NULL;
	}
	if (m->tags != NULL &&
		derech_map_set_tags(map, m->tags, n) != DERECH_OK) {
		derech_map_destroy(map);
		return NULL;
	}
	return map;
}

derech_request t_req(uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy)
{
	derech_request q;

	memset(&q, 0, sizeof(q));
	q.struct_size = sizeof(q);
	q.start_x = sx;
	q.start_y = sy;
	q.goal_x = gx;
	q.goal_y = gy;
	q.epsilon = 1.0f;
	return q;
}

/* ------------------------------------------------------------------ */
/* Validator                                                           */
/* ------------------------------------------------------------------ */

void t_validate_result(const ref_map *m, const derech_profile_desc *d,
	uint32_t sx, uint32_t sy, uint32_t gx, uint32_t gy,
	const derech_results *res, uint32_t i)
{
	derech_path_status st = derech_result_status(res, i);
	uint32_t len = derech_result_length(res, i);
	const uint32_t *steps = derech_result_steps(res, i);
	const float *ticks = derech_result_step_ticks(res, i);
	uint32_t px = sx, py = sy;
	uint64_t true_sum_q = 0;
	uint64_t perceived_sum_q = 0;
	int all_ok = 1;

	T_CHECK(st != DERECH_PATH_NONE);
	if (len == 0) {
		T_CHECK(steps == NULL);
		T_CHECK(ticks == NULL);
		T_CHECK(derech_result_total_ticks(res, i) == 0.0f);
		T_CHECK(derech_result_total_perceived(res, i) == 0.0f);
		if (st == DERECH_PATH_FOUND) {
			T_CHECK(sx == gx && sy == gy);
		}
		return;
	}
	T_CHECK(steps != NULL);
	T_CHECK(ticks != NULL);
	if (steps == NULL || ticks == NULL) {
		return;
	}

	for (uint32_t s = 0; s < len; s++) {
		uint32_t x = steps[s * 2];
		uint32_t y = steps[s * 2 + 1];
		uint32_t adx, ady, idx;
		int diag;

		if (x >= m->w || y >= m->h) {
			all_ok = 0;
			break;
		}
		adx = x > px ? x - px : px - x;
		ady = y > py ? y - py : py - y;
		if (adx > 1 || ady > 1 || (adx == 0 && ady == 0)) {
			all_ok = 0;
			break;
		}
		diag = adx == 1 && ady == 1;
		if (diag && d->connectivity == DERECH_CONN_4) {
			all_ok = 0;
			break;
		}
		idx = y * m->w + x;
		if (r_blocked(m, d, idx)) {
			all_ok = 0;
			break;
		}
		if (diag && r_corner_forbidden(m, d, px, py,
			(int8_t)(x > px ? 1 : -1), (int8_t)(y > py ? 1 : -1))) {
			all_ok = 0;
			break;
		}
		{
			uint64_t tq = r_true_q(m, d, idx, diag);

			true_sum_q += tq;
			perceived_sum_q += r_perceived_q(m, d, idx, diag);
			if (ticks[s] != (float)((double)tq / 256.0)) {
				all_ok = 0;
				break;
			}
		}
		px = x;
		py = y;
	}
	T_CHECK(all_ok);
	if (!all_ok) {
		fprintf(stderr, "  (invalid step in result %" PRIu32
			", status %s, len %" PRIu32 ")\n", i,
			derech_path_status_str(st), len);
		return;
	}

	T_CHECK(derech_result_total_ticks(res, i) ==
		(float)((double)true_sum_q / 256.0));
	T_CHECK(derech_result_total_perceived(res, i) ==
		(float)((double)perceived_sum_q / 256.0));

	if (st == DERECH_PATH_FOUND) {
		T_CHECK(px == gx && py == gy);
	} else {
		/* a partial path must stop short of the goal */
		T_CHECK(!(px == gx && py == gy));
	}
}
