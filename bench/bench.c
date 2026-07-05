/* derech benchmark & counter-regression harness.
 *
 * Two jobs in one binary:
 *
 *   ./derech_bench          run all scenarios, print timings + counters
 *   ./derech_bench --check  same, but FAIL (exit 1) if any deterministic
 *                           counter deviates from the recorded baselines
 *
 * Wall-clock times are informational only — CI machines are far too
 * noisy to gate on.  The gate is the counters: found paths, total steps,
 * total expansions, and a Q8 checksum of perceived costs.  Those are
 * bitwise-deterministic BY DESIGN — independent of platform, compiler,
 * and thread count — so a single baseline table serves every CI runner,
 * and any drift means either a behavior change (update the table below,
 * deliberately) or a broken determinism guarantee (fix the library).
 *
 * Baselines were recorded from the scenarios exactly as written; any
 * change to seeds, sizes, or request mixes invalidates them. */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "derech.h"

/* ------------------------------------------------------------------ */

static uint64_t rng_s;

static uint64_t rng_next(void)
{
	uint64_t z = (rng_s += 0x9e3779b97f4a7c15ULL);

	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
	z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
	return z ^ (z >> 31);
}

static double now_ms(void)
{
	struct timespec ts;

	timespec_get(&ts, TIME_UTC);
	return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

typedef struct counters {
	uint64_t found;
	uint64_t steps;
	uint64_t expansions;
	uint64_t perceived_q; /* sum of per-request perceived totals, Q8 */
} counters;

static void tally(const derech_results *res, counters *c)
{
	for (uint32_t i = 0; i < derech_results_count(res); i++) {
		if (derech_result_status(res, i) == DERECH_PATH_FOUND) {
			c->found++;
		}
		c->steps += derech_result_length(res, i);
		c->expansions += derech_result_expansions(res, i);
		c->perceived_q += (uint64_t)((double)
			derech_result_total_perceived(res, i) * 256.0 + 0.5);
	}
}

/* ------------------------------------------------------------------ */

enum { W = 256, H = 256, NREQ = 500, GOALS = 10 };

static uint64_t *bench_tags; /* kept for read-modify-write reveals */

static derech_map *make_map(void)
{
	static const float levels[4] = { 1.0f, 0.5f, 0.25f, 0.125f };
	derech_map_opts o;
	derech_map *map;
	float *pass = malloc(sizeof(float) * W * H);
	uint64_t *tags = malloc(sizeof(uint64_t) * W * H);
	derech_profile_desc d;

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	map = derech_map_create(W, H, &o);
	if (map == NULL || pass == NULL || tags == NULL) {
		fprintf(stderr, "bench: map setup failed\n");
		exit(2);
	}
	rng_s = 0x5EED;
	for (uint32_t i = 0; i < W * H; i++) {
		pass[i] = rng_next() % 10 < 2 ? 0.0f :
			levels[rng_next() % 4];
		tags[i] = rng_next() % 8;
	}
	/* 12 "resource" tiles (bit 3) for the goal-set scenarios; the bit
	 * carries no cost weight, so earlier baselines are unaffected */
	for (uint32_t k = 0; k < 12; k++) {
		tags[(uint64_t)((k * 83 + 31) % H) * W +
			((k * 41 + 17) % W)] |= 8;
	}
	derech_map_set_passability(map, pass, W * H);
	derech_map_set_tags(map, tags, W * H);
	memset(&d, 0, sizeof(d));
	d.struct_size = (uint32_t)sizeof(d);
	d.tag_mult[0] = 4.0f;
	if (derech_profile_register(map, &d) != 0) {
		fprintf(stderr, "bench: profile setup failed\n");
		exit(2);
	}
	if (derech_goalset_register_tags(map, 8, 0, 0) != 1) {
		fprintf(stderr, "bench: goalset setup failed\n");
		exit(2);
	}
	free(pass);
	bench_tags = tags;
	return map;
}

static void fill_scatter(derech_request *q, uint32_t flags)
{
	rng_s = 0xACE5;
	for (uint32_t i = 0; i < NREQ; i++) {
		memset(&q[i], 0, sizeof(q[i]));
		q[i].start_x = (uint32_t)(rng_next() % W);
		q[i].start_y = (uint32_t)(rng_next() % H);
		q[i].goal_x = (uint32_t)(rng_next() % W);
		q[i].goal_y = (uint32_t)(rng_next() % H);
		q[i].flags = flags;
	}
}

static void fill_converge(derech_request *q, uint32_t flags)
{
	fill_scatter(q, flags);
	for (uint32_t i = 0; i < NREQ; i++) {
		uint32_t g = (uint32_t)(rng_next() % GOALS);

		q[i].goal_x = g * 24 + 7;
		q[i].goal_y = (q[i].goal_x * 7 + 3) % H;
	}
}

/* ------------------------------------------------------------------ */

typedef struct scenario_result {
	const char *name;
	double ms;
	counters c;
} scenario_result;

static void run_one(derech_map *map, const derech_request *q,
	const char *name, scenario_result *out)
{
	derech_results *res = NULL;
	double t0 = now_ms();

	if (derech_find_paths(map, q, NREQ, &res) != DERECH_OK) {
		fprintf(stderr, "bench: %s failed\n", name);
		exit(2);
	}
	out->ms = now_ms() - t0;
	out->name = name;
	memset(&out->c, 0, sizeof(out->c));
	tally(res, &out->c);
	derech_results_destroy(res);
}

enum { N_SCENARIOS = 7 };

static void run_all(scenario_result *r)
{
	derech_map *map = make_map();
	derech_request *q = malloc(sizeof(*q) * NREQ);

	if (q == NULL) {
		exit(2);
	}

	fill_scatter(q, DERECH_REQ_ALLOW_PARTIAL);
	run_one(map, q, "scatter-partial", &r[0]);

	fill_converge(q, 0);
	run_one(map, q, "converge-cold", &r[1]);
	run_one(map, q, "converge-cached", &r[2]);

	/* edit -> caches flush -> rebuild once more */
	derech_map_set_passability_at(map, 128, 128, 0.5f);
	run_one(map, q, "converge-after-edit", &r[3]);

	/* nearest-of-set: 500 NPCs to whichever resource tile is closest */
	fill_scatter(q, 0);
	for (uint32_t i = 0; i < NREQ; i++) {
		q[i].goalset = 1;
	}
	run_one(map, q, "goalset-nearest (cold)", &r[4]);

	/* a fog-reveal style write on a bit no profile weighs: the cached
	 * set field must survive (identical counters, cache-hot time) */
	{
		static uint64_t reveal[128 * 128];

		for (uint32_t y = 0; y < 128; y++) {
			for (uint32_t x = 0; x < 128; x++) {
				reveal[y * 128 + x] = bench_tags[
					(uint64_t)(64 + y) * W + 64 + x] |
					(1ULL << 5);
			}
		}
		derech_map_set_tags_rect(map, 64, 64, 128, 128, reveal);
	}
	run_one(map, q, "goalset-after-reveal", &r[5]);

	fill_converge(q, 0);
	/* every goal sealed behind walls: labels answer everything O(1) */
	for (uint32_t g = 0; g < GOALS; g++) {
		uint32_t gx = g * 24 + 7;
		uint32_t gy = (gx * 7 + 3) % H;

		for (uint32_t y = gy - 1; y <= gy + 1; y++) {
			for (uint32_t x = gx - 1; x <= gx + 1; x++) {
				if (x != gx || y != gy) {
					derech_map_set_passability_at(map,
						x, y, 0.0f);
				}
			}
		}
		derech_map_set_passability_at(map, gx, gy, 1.0f);
	}
	run_one(map, q, "sealed-goals-labels", &r[6]);

	free(q);
	derech_map_destroy(map);
}

/* ------------------------------------------------------------------ */
/* Recorded baselines — deterministic, platform- and thread-agnostic.
 * Regenerate by running the bench and pasting the printed values. */

static const counters BASELINE[N_SCENARIOS] = {
	{ 410, 82714, 12812237, 83469100 }, /* scatter-partial      */
	{ 454, 72374, 0, 73157000 },        /* converge-cold        */
	{ 454, 72374, 0, 73157000 },        /* converge-cached      */
	{ 454, 72374, 0, 73157000 },        /* converge-after-edit  */
	{ 498, 27610, 0, 29026816 },        /* goalset-nearest      */
	{ 498, 27610, 0, 29026816 },        /* goalset-after-reveal */
	{ 1, 1, 1, 1024 },                  /* sealed-goals-labels  */
};

int main(int argc, char **argv)
{
	int check = argc > 1 && strcmp(argv[1], "--check") == 0;
	scenario_result r[N_SCENARIOS];
	int failures = 0;

	run_all(r);

	printf("%-22s %10s %8s %10s %12s %16s\n", "scenario", "ms", "found",
		"steps", "expansions", "perceived_q");
	for (uint32_t i = 0; i < N_SCENARIOS; i++) {
		printf("%-22s %10.1f %8" PRIu64 " %10" PRIu64 " %12" PRIu64
			" %16" PRIu64 "\n", r[i].name, r[i].ms, r[i].c.found,
			r[i].c.steps, r[i].c.expansions, r[i].c.perceived_q);
		if (check && memcmp(&r[i].c, &BASELINE[i],
			sizeof(counters)) != 0) {
			fprintf(stderr, "COUNTER DRIFT in %s: baseline "
				"{%" PRIu64 ", %" PRIu64 ", %" PRIu64
				", %" PRIu64 "}\n", r[i].name,
				BASELINE[i].found, BASELINE[i].steps,
				BASELINE[i].expansions,
				BASELINE[i].perceived_q);
			failures++;
		}
	}
	if (check && failures > 0) {
		fprintf(stderr, "bench: %d scenario(s) drifted from "
			"baseline\n", failures);
		return 1;
	}
	if (check) {
		printf("bench: all counters match baselines\n");
	}
	return 0;
}
