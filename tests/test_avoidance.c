#include <stdlib.h>
#include "support.h"

static void geometry(void)
{
	derech_avoidance a = { .struct_size = sizeof(derech_avoidance), .count = 1, .circles = {{10, 10, 4}} };
	T_CHECK(!derech_avoidance_allows_step(&a, 6, 10, 7, 10));
	T_CHECK(derech_avoidance_allows_step(&a, 10, 10, 11, 10));
	T_CHECK(!derech_avoidance_allows_step(&a, 12, 10, 11, 10));
	T_CHECK(!derech_avoidance_allows_step(&a, 10, 10, 10, 10));
	a.circles[0] = (derech_avoid_circle){1, 1, 1};
	T_CHECK(!derech_avoidance_allows_step(&a, 0, 1, 1, 0)); /* edge crosses, endpoints safe */
	a.count = 2; a.circles[0] = (derech_avoid_circle){10, 10, 4};
	a.circles[1] = (derech_avoid_circle){12, 10, 4};
	T_CHECK(derech_avoidance_allows_step(&a, 11, 10, 11, 11));
	T_CHECK(!derech_avoidance_allows_step(&a, 11, 11, 11, 10));
	a.count = 9;
	T_CHECK(!derech_avoidance_allows_step(&a, 1, 1, 2, 1));
	a.count = 1; a.circles[0].radius = UINT32_MAX;
	T_CHECK(!derech_avoidance_allows_step(&a, 1, 1, 2, 1));
	a.circles[0] = (derech_avoid_circle){INT32_MIN, INT32_MIN, 65535};
	T_CHECK(derech_avoidance_allows_step(&a, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX));
}
static void routes(void)
{
	float pass[32 * 24], reference[32 * 24];
	for (uint32_t i = 0; i < 32 * 24; i++) pass[i] = reference[i] = 1;
	ref_map rm = {32, 24, pass, NULL};
	derech_map *m = t_build_map(&rm);
	derech_profile_desc d = t_neutral_desc();
	d.connectivity = DERECH_CONN_4;
	T_CHECK(derech_profile_register(m, &d) == 0);
	uint32_t goals[] = {8, 10, 25, 10};
	int32_t gs = derech_goalset_register(m, goals, 2, 0);
	T_CHECK(gs > 0);
	derech_request q = t_req(2, 10, 0, 0); q.goalset = (uint32_t)gs;
	derech_results *r = NULL;
	T_CHECK(derech_find_paths(m, &q, 1, &r) == DERECH_OK);
	T_CHECK(derech_result_length(r, 0) == 6);
	derech_results_destroy(r);
	derech_avoidance av = {.struct_size = sizeof(derech_avoidance), .count = 2, .circles = {{8, 10, 3}, {18, 10, 3}}};
	for (uint32_t y = 0; y < 24; y++) for (uint32_t x = 0; x < 32; x++) {
		for (uint32_t c = 0; c < 2; c++) {
			int dx = (int)x - av.circles[c].x, dy = (int)y - av.circles[c].y;
			if (dx * dx + dy * dy < 9) reference[y * 32 + x] = 0;
		}
	}
	ref_map rr = {32, 24, reference, NULL};
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, NULL, &av, &r) == DERECH_OK);
	T_CHECK(derech_result_status(r, 0) == DERECH_PATH_FOUND);
	T_CHECK(derech_result_total_perceived_q8(r, 0) == ref_solve(&rr, &d, 2, 10, 25, 10));
	uint32_t len = derech_result_length(r, 0), x = 2, y = 10;
	const uint32_t *steps = derech_result_steps(r, 0);
	for (uint32_t i = 0; i < len; i++) {
		T_CHECK(derech_avoidance_allows_step(&av, x, y, steps[2*i], steps[2*i+1]));
		x = steps[2*i]; y = steps[2*i+1];
	}
	T_CHECK(x == 25 && y == 10);
	derech_results_destroy(r);
	q.max_expansions = 1;
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, NULL, &av, &r) == DERECH_OK);
	T_CHECK(derech_result_status(r, 0) == DERECH_PATH_BUDGET_EXCEEDED);
	T_CHECK(derech_result_length(r, 0) == 0);
	derech_results_destroy(r);
	q.max_expansions = 0;
	av.circles[1] = (derech_avoid_circle){25, 10, 3};
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, NULL, &av, &r) == DERECH_OK);
	T_CHECK(derech_result_status(r, 0) == DERECH_PATH_UNREACHABLE);
	derech_results_destroy(r);
	T_CHECK(derech_find_paths(m, &q, 1, &r) == DERECH_OK);
	T_CHECK(derech_result_length(r, 0) == 6); /* constrained requests cannot poison cache */
	derech_results_destroy(r);
	av.count = 1; av.circles[0] = (derech_avoid_circle){2, 10, 3};
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, NULL, &av, &r) == DERECH_OK);
	T_CHECK(derech_result_status(r, 0) == DERECH_PATH_FOUND); /* interior egress */
	derech_results_destroy(r);
	av.count = 9; r = NULL;
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, NULL, &av, &r) == DERECH_E_INVALID_ARG && r == NULL);
	av.count = 1;
	derech_cancel *cancel = NULL;
	T_CHECK(derech_cancel_create(&cancel) == DERECH_OK);
	derech_cancel_request(cancel);
	derech_find_opts opts = {sizeof(opts), 0, cancel};
	T_CHECK(derech_find_paths_avoiding(m, &q, 1, &opts, &av, &r) == DERECH_E_CANCELLED && r == NULL);
	derech_cancel_destroy(cancel);
	derech_map_destroy(m);
}
int main(void) { geometry(); routes(); return t_done("avoidance"); }
