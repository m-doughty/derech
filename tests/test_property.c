/* Property tests: random maps and profiles, cross-checked against the
 * independent reference Dijkstra.
 *
 *   - every returned path is structurally valid (t_validate_result)
 *   - epsilon = 1 results equal the reference optimum exactly
 *   - epsilon > 1 results stay within the inflation bound
 *   - reachability agrees with the reference in both directions
 *   - partial endpoints are genuinely reachable
 *
 * Seeds are fixed constants; failures print enough to reproduce. */

#include <math.h>
#include <stdlib.h>

#include "support.h"

enum { N_PROFILES = 6 };

static void build_profiles(derech_profile_desc *out)
{
	out[0] = t_neutral_desc();

	out[1] = t_neutral_desc(); /* avoider, mult + add */
	out[1].tag_mult[0] = 4.0f;
	out[1].tag_add[1] = 8.0f;

	out[2] = t_neutral_desc(); /* blocker */
	out[2].block_mask = 4;

	out[3] = t_neutral_desc(); /* 4-connected */
	out[3].connectivity = DERECH_CONN_4;

	out[4] = t_neutral_desc(); /* lenient corners, road preference */
	out[4].corner_rule = DERECH_CORNER_LENIENT;
	out[4].tag_mult[0] = 0.5f; /* mult < 1 exercises the h floor */

	out[5] = t_neutral_desc(); /* expensive diagonals: h_d2 clamp */
	out[5].diagonal_mult = 3.0f;
}

static void fill_terrain(t_rng *rng, uint32_t w, uint32_t h, float *pass,
	uint64_t *tags)
{
	static const float levels[4] = { 1.0f, 0.5f, 0.25f, 0.125f };
	uint32_t n = w * h;

	for (uint32_t i = 0; i < n; i++) {
		pass[i] = t_rng_below(rng, 10) < 2 ? 0.0f :
			levels[t_rng_below(rng, 4)];
		tags[i] = 0;
	}
	/* paint each region bit over a few random rects */
	for (uint32_t bit = 0; bit < 3; bit++) {
		uint32_t rects = 1 + t_rng_below(rng, 3);

		for (uint32_t r = 0; r < rects; r++) {
			uint32_t x0 = t_rng_below(rng, w);
			uint32_t y0 = t_rng_below(rng, h);
			uint32_t rw = 1 + t_rng_below(rng, w - x0);
			uint32_t rh = 1 + t_rng_below(rng, h - y0);

			for (uint32_t y = y0; y < y0 + rh; y++) {
				for (uint32_t x = x0; x < x0 + rw; x++) {
					tags[y * w + x] |= 1ULL << bit;
				}
			}
		}
	}
}

static uint64_t perceived_q(const derech_results *res, uint32_t i)
{
	return (uint64_t)llround(
		(double)derech_result_total_perceived(res, i) * 256.0);
}

/* min over bit-0-tagged tiles of the reference distance field */
static uint64_t ref_nearest_of_set(const ref_map *rm, const uint64_t *field)
{
	uint64_t best = REF_INF;

	for (uint32_t i = 0; i < rm->w * rm->h; i++) {
		if ((rm->tags[i] & 1ULL) != 0 && field[i] < best) {
			best = field[i];
		}
	}
	return best;
}

static void run_iteration(uint64_t seed, uint32_t w, uint32_t h)
{
	enum { NREQ = 12 };
	static const float epsilons[4] = { 1.0f, 1.0f, 1.25f, 2.0f };
	t_rng rng = { seed };
	uint32_t n = w * h;
	float *pass = malloc((size_t)n * sizeof(*pass));
	uint64_t *tags = malloc((size_t)n * sizeof(*tags));
	derech_profile_desc profiles[N_PROFILES];
	ref_map rm = { 0, 0, NULL, NULL };
	derech_map *m;
	derech_request q[NREQ];
	derech_results *res = NULL;
	int before_fails = t_fails;

	T_CHECK(pass != NULL && tags != NULL);
	if (pass == NULL || tags == NULL) {
		free(pass);
		free(tags);
		return;
	}
	fill_terrain(&rng, w, h, pass, tags);
	rm.w = w;
	rm.h = h;
	rm.pass = pass;
	rm.tags = tags;

	build_profiles(profiles);
	m = t_build_map(&rm);
	T_CHECK(m != NULL);
	for (uint32_t i = 0; i < N_PROFILES; i++) {
		T_CHECK(derech_profile_register(m, &profiles[i]) ==
			(int32_t)i);
	}

	for (uint32_t i = 0; i < NREQ; i++) {
		q[i] = t_req(t_rng_below(&rng, w), t_rng_below(&rng, h),
			t_rng_below(&rng, w), t_rng_below(&rng, h));
		q[i].profile_id = t_rng_below(&rng, N_PROFILES);
		q[i].flags = (t_rng_next(&rng) & 1) ?
			DERECH_REQ_ALLOW_PARTIAL : 0;
		q[i].epsilon = epsilons[t_rng_below(&rng, 4)];
	}
	/* force a same-(goal, profile) cluster so goal fields engage */
	for (uint32_t i = 1; i < 4; i++) {
		q[i].goal_x = q[0].goal_x;
		q[i].goal_y = q[0].goal_y;
		q[i].profile_id = q[0].profile_id;
	}
	/* and two goal-set queries against the bit-0 region tiles */
	T_CHECK(derech_goalset_register_tags(m, 1, 0, 0) == 1);
	for (uint32_t i = NREQ - 2; i < NREQ; i++) {
		q[i].goalset = 1;
		q[i].flags = 0; /* partials are invalid with sets */
	}

	T_CHECK(derech_find_paths(m, q, NREQ, &res) == DERECH_OK);
	T_CHECK(derech_results_count(res) == NREQ);

	for (uint32_t i = 0; i < NREQ; i++) {
		const derech_profile_desc *d = &profiles[q[i].profile_id];
		derech_path_status st = derech_result_status(res, i);
		uint64_t *field;
		uint64_t want;

		T_CHECK(st == DERECH_PATH_FOUND ||
			st == DERECH_PATH_UNREACHABLE);

		field = ref_solve_field(&rm, d, q[i].start_x, q[i].start_y);
		T_CHECK(field != NULL);
		if (field == NULL) {
			continue;
		}

		if (q[i].goalset != DERECH_NO_GOALSET) {
			/* nearest-of-set: exact regardless of epsilon */
			uint64_t best = ref_nearest_of_set(&rm, field);
			uint32_t sidx = q[i].start_y * w + q[i].start_x;

			/* a start ON a member is trivially arrived */
			if ((rm.tags[sidx] & 1ULL) != 0) {
				best = 0;
			}
			T_CHECK((st == DERECH_PATH_FOUND) ==
				(best != REF_INF));
			if (st == DERECH_PATH_FOUND) {
				T_CHECK(perceived_q(res, i) == best);
				if (derech_result_length(res, i) > 0) {
					const uint32_t *steps =
						derech_result_steps(res, i);
					uint32_t len =
						derech_result_length(res, i);

					t_validate_result(&rm, d,
						q[i].start_x, q[i].start_y,
						steps[(len - 1) * 2],
						steps[(len - 1) * 2 + 1],
						res, i);
					/* endpoint is a member */
					T_CHECK((rm.tags[steps[(len - 1) * 2 +
						1] * w + steps[(len - 1) *
						2]] & 1ULL) != 0);
				}
			}
			free(field);
			continue;
		}

		t_validate_result(&rm, d, q[i].start_x, q[i].start_y,
			q[i].goal_x, q[i].goal_y, res, i);
		want = field[q[i].goal_y * w + q[i].goal_x];

		/* completeness: reachability agreement both ways */
		T_CHECK((st == DERECH_PATH_FOUND) == (want != REF_INF));

		if (st == DERECH_PATH_FOUND) {
			uint64_t got = perceived_q(res, i);

			T_CHECK(got >= want);
			if (q[i].epsilon == 1.0f) {
				T_CHECK(got == want);
			} else {
				uint64_t eps_q = (uint64_t)llround(
					(double)q[i].epsilon * 256.0);

				T_CHECK(got <= (want * eps_q) >> 8);
			}
		} else if (derech_result_length(res, i) > 0) {
			/* partial endpoint must be genuinely reachable */
			const uint32_t *steps = derech_result_steps(res, i);
			uint32_t len = derech_result_length(res, i);
			uint32_t ex = steps[(len - 1) * 2];
			uint32_t ey = steps[(len - 1) * 2 + 1];

			T_CHECK(field[ey * w + ex] != REF_INF);
		}
		free(field);
	}

	if (t_fails != before_fails) {
		fprintf(stderr, "  (iteration seed=0x%" PRIx64 " %" PRIu32
			"x%" PRIu32 ")\n", seed, w, h);
	}
	derech_results_destroy(res);
	derech_map_destroy(m);
	free(pass);
	free(tags);
}

int main(void)
{
	for (uint64_t i = 0; i < 40; i++) {
		run_iteration(0xC0FFEE00ULL + i, 24, 24);
	}
	for (uint64_t i = 0; i < 6; i++) {
		run_iteration(0xBADC0DE00ULL + i, 48, 48);
	}
	return t_done("test_property");
}
