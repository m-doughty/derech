/* libFuzzer harness: interprets fuzz input as a program of API calls
 * against a small map — terrain edits (valid and invalid), profile
 * registrations (valid and invalid), and batches — then validates every
 * returned path structurally.  Violations abort(), so the fuzzer hunts
 * logic bugs as well as the memory bugs ASan/UBSan catch.
 *
 * Build with -DDERECH_FUZZ=ON (Clang only); run e.g.
 *   ./fuzz_derech -max_total_time=60 corpus/
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "derech.h"

typedef struct rd {
	const uint8_t *d;
	size_t n;
	size_t i;
} rd;

static uint8_t r8(rd *r)
{
	return r->i < r->n ? r->d[r->i++] : 0;
}

static uint16_t r16(rd *r)
{
	return (uint16_t)((uint16_t)r8(r) | ((uint16_t)r8(r) << 8));
}

static uint32_t r32(rd *r)
{
	return (uint32_t)r16(r) | ((uint32_t)r16(r) << 16);
}

static uint64_t r64(rd *r)
{
	return (uint64_t)r32(r) | ((uint64_t)r32(r) << 32);
}

/* Mostly-valid passability, sometimes hostile raw float bits. */
static float rpass(rd *r)
{
	uint8_t mode = r8(r);

	if (mode < 200) {
		return (float)(mode % 5) * 0.25f; /* 0, .25, .5, .75, 1 */
	}
	if (mode < 230) {
		return (float)r8(r) / 64.0f; /* often > 1: must reject */
	}
	{
		uint32_t bits = r32(r); /* NaN/inf/denormal territory */
		float f;

		memcpy(&f, &bits, sizeof(f));
		return f;
	}
}

static float rweight(rd *r)
{
	uint8_t mode = r8(r);

	if (mode < 180) {
		static const float w[6] = { 0.0f, 0.5f, 1.0f, 2.0f, 4.0f,
			10.0f };

		return w[mode % 6];
	}
	{
		uint32_t bits = r32(r);
		float f;

		memcpy(&f, &bits, sizeof(f));
		return f;
	}
}

#define FUZZ_ASSERT(cond) \
	do { \
		if (!(cond)) { \
			abort(); \
		} \
	} while (0)

static void validate_results(const derech_map *map,
	const derech_request *reqs, uint32_t n, const derech_results *res)
{
	uint32_t w = derech_map_width(map);
	uint32_t h = derech_map_height(map);

	FUZZ_ASSERT(derech_results_count(res) == n);
	for (uint32_t i = 0; i < n; i++) {
		derech_path_status st = derech_result_status(res, i);
		uint32_t len = derech_result_length(res, i);
		const uint32_t *steps = derech_result_steps(res, i);
		const float *ticks = derech_result_step_ticks(res, i);
		uint32_t px = reqs[i].start_x;
		uint32_t py = reqs[i].start_y;

		FUZZ_ASSERT(st >= DERECH_PATH_FOUND &&
			st <= DERECH_PATH_INVALID_ENDPOINT);
		if (len == 0) {
			FUZZ_ASSERT(steps == NULL && ticks == NULL);
			continue;
		}
		FUZZ_ASSERT(steps != NULL && ticks != NULL);
		FUZZ_ASSERT(st != DERECH_PATH_INVALID_ENDPOINT);
		for (uint32_t s = 0; s < len; s++) {
			uint32_t x = steps[s * 2];
			uint32_t y = steps[s * 2 + 1];
			uint32_t adx = x > px ? x - px : px - x;
			uint32_t ady = y > py ? y - py : py - y;

			FUZZ_ASSERT(x < w && y < h);
			FUZZ_ASSERT(adx <= 1 && ady <= 1);
			FUZZ_ASSERT(adx + ady > 0);
			FUZZ_ASSERT(ticks[s] >= 0.0f);
			px = x;
			py = y;
		}
		if (reqs[i].goalset == DERECH_NO_GOALSET) {
			if (st == DERECH_PATH_FOUND) {
				FUZZ_ASSERT(px == reqs[i].goal_x &&
					py == reqs[i].goal_y);
			} else {
				FUZZ_ASSERT(!(px == reqs[i].goal_x &&
					py == reqs[i].goal_y));
			}
		}
	}
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
	rd r = { data, size, 0 };
	derech_map_opts o;
	derech_map *map;
	uint32_t w = 1 + r8(&r) % 24;
	uint32_t h = 1 + r8(&r) % 24;
	uint32_t ops = 0;

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.n_threads = (r8(&r) % 8 == 0) ? 2 : 1; /* threads sometimes */
	o.field_cache_mb = 1;
	o.field_group_threshold = 1 + r8(&r) % 5;
	map = derech_map_create(w, h, &o);
	FUZZ_ASSERT(map != NULL);

	/* one guaranteed-valid profile so batches can always run */
	{
		derech_profile_desc d;

		memset(&d, 0, sizeof(d));
		d.struct_size = (uint32_t)sizeof(d);
		FUZZ_ASSERT(derech_profile_register(map, &d) == 0);
	}

	while (r.i < r.n && ops < 48) {
		ops++;
		switch (r8(&r) % 10) {
		case 0: { /* passability rect (bounds may be hostile) */
			uint32_t x = r8(&r) % (w + 2);
			uint32_t y = r8(&r) % (h + 2);
			uint32_t rw = 1 + r8(&r) % 8;
			uint32_t rh = 1 + r8(&r) % 8;
			float buf[64];

			for (uint32_t k = 0; k < 64; k++) {
				buf[k] = rpass(&r);
			}
			if (rw * rh <= 64) {
				(void)derech_map_set_passability_rect(map, x,
					y, rw, rh, buf);
			}
			break;
		}
		case 1: { /* tags rect */
			uint32_t x = r8(&r) % (w + 2);
			uint32_t y = r8(&r) % (h + 2);
			uint32_t rw = 1 + r8(&r) % 8;
			uint32_t rh = 1 + r8(&r) % 8;
			uint64_t buf[64];
			uint8_t wild = r8(&r);

			for (uint32_t k = 0; k < 64; k++) {
				buf[k] = wild % 4 == 0 ? r64(&r) :
					(uint64_t)(r8(&r) % 8);
			}
			if (rw * rh <= 64) {
				(void)derech_map_set_tags_rect(map, x, y, rw,
					rh, buf);
			}
			break;
		}
		case 2: /* single tiles */
			(void)derech_map_set_passability_at(map, r8(&r),
				r8(&r), rpass(&r));
			(void)derech_map_set_tags_at(map, r8(&r), r8(&r),
				r64(&r));
			break;
		case 3: { /* profile registration, often invalid */
			derech_profile_desc d;

			memset(&d, 0, sizeof(d));
			d.struct_size = r8(&r) % 4 == 0 ? r32(&r) :
				(uint32_t)sizeof(d);
			d.connectivity = r8(&r) % 3;
			d.corner_rule = r8(&r) % 4;
			d.diagonal_mult = rweight(&r);
			d.block_mask = r8(&r) % 3 == 0 ? r64(&r) : 0;
			d.require_mask = r8(&r) % 5 == 0 ? r64(&r) : 0;
			for (uint32_t k = 0; k < 4; k++) {
				d.tag_mult[r8(&r) % 64] = rweight(&r);
				d.tag_add[r8(&r) % 64] = rweight(&r);
			}
			(void)derech_profile_register(map, &d);
			break;
		}
		case 4: { /* batch */
			derech_request q[8];
			derech_results *res = NULL;
			uint32_t n = r8(&r) % 9;
			uint32_t nprof = derech_profile_count(map);
			derech_status rc;

			for (uint32_t k = 0; k < n; k++) {
				static const float eps[5] = { 0.0f, 1.0f,
					1.25f, 2.0f, 0.5f /* invalid */ };

				memset(&q[k], 0, sizeof(q[k]));
				q[k].start_x = r8(&r) % (w + 2);
				q[k].start_y = r8(&r) % (h + 2);
				q[k].goal_x = r8(&r) % (w + 2);
				q[k].goal_y = r8(&r) % (h + 2);
				q[k].profile_id = r8(&r) % (nprof + 1);
				q[k].flags = r8(&r) % 4; /* bit 1 invalid */
				q[k].max_expansions = r8(&r) % 3 == 0 ?
					r16(&r) : 0;
				q[k].max_perceived_cost = r8(&r) % 4 == 0 ?
					(float)(r8(&r)) : 0.0f;
				q[k].epsilon = eps[r8(&r) % 5];
				if (r8(&r) % 3 == 0) {
					q[k].goalset = r8(&r) % 9;
				}
			}
			rc = derech_find_paths(map, q, n, &res);
			FUZZ_ASSERT(rc == DERECH_OK ||
				rc == DERECH_E_INVALID_ARG ||
				rc == DERECH_E_BAD_PROFILE ||
				rc == DERECH_E_BAD_GOALSET ||
				rc == DERECH_E_NOMEM);
			if (rc == DERECH_OK) {
				validate_results(map, q, n, res);
				derech_results_destroy(res);
			} else {
				FUZZ_ASSERT(res == NULL);
			}
			break;
		}
		case 6: { /* explicit goal set (tiles sometimes OOB) */
			uint32_t xy[8];
			uint32_t nt = 1 + r8(&r) % 4;

			for (uint32_t k = 0; k < nt; k++) {
				xy[k * 2] = r8(&r) % (w + 2);
				xy[k * 2 + 1] = r8(&r) % (h + 2);
			}
			(void)derech_goalset_register(map, xy, nt,
				r8(&r) % 4); /* flag bit 1 is invalid */
			break;
		}
		case 7: { /* predicate goal set (masks sometimes empty) */
			uint64_t any = r8(&r) % 3 == 0 ? 0 :
				(uint64_t)(1ULL << (r8(&r) % 64));
			uint64_t all = r8(&r) % 3 == 0 ? 0 :
				(uint64_t)(1ULL << (r8(&r) % 64));

			(void)derech_goalset_register_tags(map, any, all,
				r8(&r) % 4);
			break;
		}
		case 8: /* unregister / count arbitrary ids */
			if (r8(&r) % 2 == 0) {
				(void)derech_goalset_unregister(map,
					r8(&r) % 70);
			} else {
				(void)derech_goalset_count(map, r8(&r) % 70);
			}
			break;
		case 5: { /* read-back, bounds included */
			float p;
			uint64_t t;

			(void)derech_map_get_passability_at(map, r8(&r),
				r8(&r), &p);
			(void)derech_map_get_tags_at(map, r8(&r), r8(&r),
				&t);
			break;
		}
		default: /* consistency probes */
			FUZZ_ASSERT(derech_map_width(map) == w);
			FUZZ_ASSERT(derech_map_height(map) == h);
			FUZZ_ASSERT(derech_map_thread_count(map) >= 1);
			break;
		}
	}

	derech_map_destroy(map);
	return 0;
}
