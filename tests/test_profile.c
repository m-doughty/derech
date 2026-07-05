/* Profile registration and validation. */

#include <math.h>

#include "support.h"

static void test_basic_register(void)
{
	derech_map *m = derech_map_create(4, 4, NULL);
	derech_profile_desc d = t_neutral_desc();

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_count(m) == 0);
	T_CHECK(derech_profile_register(m, &d) == 0);
	T_CHECK(derech_profile_register(m, &d) == 1);
	T_CHECK(derech_profile_count(m) == 2);
	derech_map_destroy(m);
}

static void test_arg_validation(void)
{
	derech_map *m = derech_map_create(4, 4, NULL);
	derech_profile_desc d = t_neutral_desc();

	T_CHECK(m != NULL);
	T_CHECK(derech_profile_register(NULL, &d) == DERECH_E_INVALID_ARG);
	T_CHECK(derech_profile_register(m, NULL) == DERECH_E_INVALID_ARG);

	d.struct_size = 4;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d = t_neutral_desc();

	d.reserved0 = 1;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d = t_neutral_desc();
	d.reserved1 = 1;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d = t_neutral_desc();

	d.connectivity = 2;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d = t_neutral_desc();
	d.corner_rule = 3;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d = t_neutral_desc();

	d.diagonal_mult = 0.5f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.diagonal_mult = 17.0f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.diagonal_mult = nanf("");
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.diagonal_mult = 1.0f;
	T_CHECK(derech_profile_register(m, &d) == 0);
	d.diagonal_mult = 16.0f;
	T_CHECK(derech_profile_register(m, &d) == 1);
	d = t_neutral_desc();

	d.tag_mult[13] = -1.0f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_mult[13] = 65537.0f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_mult[13] = nanf("");
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_mult[13] = INFINITY;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_mult[13] = 0.001f;
	T_CHECK(derech_profile_register(m, &d) >= 0);
	d = t_neutral_desc();

	d.tag_add[63] = -0.5f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_add[63] = 65537.0f;
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_add[63] = nanf("");
	T_CHECK(derech_profile_register(m, &d) == DERECH_E_INVALID_ARG);
	d.tag_add[63] = 65536.0f;
	T_CHECK(derech_profile_register(m, &d) >= 0);

	derech_map_destroy(m);
}

static void test_profile_limit(void)
{
	derech_map *m = derech_map_create(2, 2, NULL);
	derech_profile_desc d = t_neutral_desc();
	int32_t id = -1;

	T_CHECK(m != NULL);
	for (uint32_t i = 0; i < DERECH_MAX_PROFILES; i++) {
		id = derech_profile_register(m, &d);
		T_CHECK(id == (int32_t)i);
		if (id < 0) {
			break;
		}
	}
	T_CHECK(derech_profile_register(m, &d) ==
		DERECH_E_TOO_MANY_PROFILES);
	T_CHECK(derech_profile_count(m) == DERECH_MAX_PROFILES);
	derech_map_destroy(m);
}

static void test_register_after_tags(void)
{
	/* profiles registered after tags exist must fold the existing
	 * combos — exercised by searching over them */
	derech_map *m = derech_map_create(3, 1, NULL);
	derech_profile_desc d = t_neutral_desc();
	derech_results *res = NULL;
	derech_request q;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_set_tags_at(m, 1, 0, 1) == DERECH_OK);
	d.block_mask = 1; /* the middle tile is a wall for this profile */
	T_CHECK(derech_profile_register(m, &d) == 0);

	q = t_req(0, 0, 2, 0);
	T_CHECK(derech_find_paths(m, &q, 1, &res) == DERECH_OK);
	T_CHECK(derech_result_status(res, 0) == DERECH_PATH_UNREACHABLE);
	derech_results_destroy(res);
	derech_map_destroy(m);
}

int main(void)
{
	test_basic_register();
	test_arg_validation();
	test_profile_limit();
	test_register_after_tags();
	return t_done("test_profile");
}
