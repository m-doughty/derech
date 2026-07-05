/* Map lifecycle, terrain setters, quantization, tag interning. */

#include <math.h>
#include <stdlib.h>

#include "support.h"

static void test_create_destroy(void)
{
	derech_map *m;

	T_CHECK(derech_map_create(0, 5, NULL) == NULL);
	T_CHECK(derech_map_create(5, 0, NULL) == NULL);
	T_CHECK(derech_map_create(2049, 5, NULL) == NULL);
	T_CHECK(derech_map_create(5, 2049, NULL) == NULL);

	m = derech_map_create(1, 1, NULL);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_width(m) == 1);
	T_CHECK(derech_map_height(m) == 1);
	T_CHECK(derech_map_generation(m) == 0);
	derech_map_destroy(m);

	m = derech_map_create(2048, 2048, NULL);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_width(m) == 2048);
	derech_map_destroy(m);

	derech_map_destroy(NULL); /* no-op */
}

static void test_create_opts(void)
{
	derech_map_opts o;
	derech_map *m;
	float p = -1.0f;
	uint64_t tags = 0;

	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 0.5f;
	o.default_tags = 0xDEADBEEFULL;

	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_get_passability_at(m, 3, 3, &p) == DERECH_OK);
	T_CHECK(p == 0.5f); /* q = 512, 256/512 exact */
	T_CHECK(derech_map_get_tags_at(m, 0, 0, &tags) == DERECH_OK);
	T_CHECK(tags == 0xDEADBEEFULL);
	derech_map_destroy(m);

	o.struct_size = 12345;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);
	o.struct_size = (uint32_t)sizeof(o);

	o.default_passability = 2.0f;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);
	o.default_passability = nanf("");
	T_CHECK(derech_map_create(4, 4, &o) == NULL);
	o.default_passability = -0.5f;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);

	/* all-blocked default fill is legal */
	o.default_passability = 0.0f;
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_get_passability_at(m, 1, 1, &p) == DERECH_OK);
	T_CHECK(p == 0.0f);
	derech_map_destroy(m);
}

static void test_set_passability_full(void)
{
	derech_map *m = derech_map_create(3, 2, NULL);
	float buf[6] = { 1.0f, 0.5f, 0.25f, 0.3f, 0.0f, 1.0f };
	float p = -1.0f;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_set_passability(NULL, buf, 6) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_set_passability(m, NULL, 6) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_set_passability(m, buf, 5) ==
		DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_generation(m) == 0);

	T_CHECK(derech_map_set_passability(m, buf, 6) == DERECH_OK);
	T_CHECK(derech_map_generation(m) == 1);

	T_CHECK(derech_map_get_passability_at(m, 0, 0, &p) == DERECH_OK);
	T_CHECK(p == 1.0f);
	T_CHECK(derech_map_get_passability_at(m, 1, 0, &p) == DERECH_OK);
	T_CHECK(p == 0.5f);
	T_CHECK(derech_map_get_passability_at(m, 2, 0, &p) == DERECH_OK);
	T_CHECK(p == 0.25f);
	/* 0.3 quantizes to q = round(256/0.3) = 853 */
	T_CHECK(derech_map_get_passability_at(m, 0, 1, &p) == DERECH_OK);
	T_CHECK(p == (float)(256.0 / 853.0));
	T_CHECK(fabsf(p - 0.3f) < 0.001f);
	T_CHECK(derech_map_get_passability_at(m, 1, 1, &p) == DERECH_OK);
	T_CHECK(p == 0.0f); /* blocked */

	derech_map_destroy(m);
}

static void test_set_passability_atomicity(void)
{
	derech_map *m = derech_map_create(2, 2, NULL);
	float good[4] = { 0.5f, 0.5f, 0.5f, 0.5f };
	float bad[4] = { 1.0f, 1.0f, 2.0f, 1.0f };
	float p = -1.0f;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_set_passability(m, good, 4) == DERECH_OK);
	T_CHECK(derech_map_generation(m) == 1);

	T_CHECK(derech_map_set_passability(m, bad, 4) ==
		DERECH_E_INVALID_ARG);
	bad[2] = nanf("");
	T_CHECK(derech_map_set_passability(m, bad, 4) ==
		DERECH_E_INVALID_ARG);
	bad[2] = -0.1f;
	T_CHECK(derech_map_set_passability(m, bad, 4) ==
		DERECH_E_INVALID_ARG);

	/* nothing moved: not even the tiles before the bad index */
	T_CHECK(derech_map_generation(m) == 1);
	T_CHECK(derech_map_get_passability_at(m, 0, 0, &p) == DERECH_OK);
	T_CHECK(p == 0.5f);

	derech_map_destroy(m);
}

static void test_set_passability_rect(void)
{
	derech_map *m = derech_map_create(5, 5, NULL);
	float rect[6] = { 0.5f, 0.5f, 0.5f, 0.25f, 0.25f, 0.25f };
	float p = -1.0f;

	T_CHECK(m != NULL);
	/* 3 wide, 2 tall at (1,1) */
	T_CHECK(derech_map_set_passability_rect(m, 1, 1, 3, 2, rect) ==
		DERECH_OK);
	T_CHECK(derech_map_get_passability_at(m, 1, 1, &p) == DERECH_OK);
	T_CHECK(p == 0.5f);
	T_CHECK(derech_map_get_passability_at(m, 3, 2, &p) == DERECH_OK);
	T_CHECK(p == 0.25f);
	/* outside untouched */
	T_CHECK(derech_map_get_passability_at(m, 0, 0, &p) == DERECH_OK);
	T_CHECK(p == 1.0f);
	T_CHECK(derech_map_get_passability_at(m, 4, 3, &p) == DERECH_OK);
	T_CHECK(p == 1.0f);

	T_CHECK(derech_map_set_passability_rect(m, 3, 0, 3, 1, rect) ==
		DERECH_E_OOB);
	T_CHECK(derech_map_set_passability_rect(m, 0, 4, 1, 2, rect) ==
		DERECH_E_OOB);
	T_CHECK(derech_map_set_passability_rect(m, 0, 0, 0, 1, rect) ==
		DERECH_E_INVALID_ARG);

	T_CHECK(derech_map_set_passability_at(m, 4, 4, 0.0f) == DERECH_OK);
	T_CHECK(derech_map_get_passability_at(m, 4, 4, &p) == DERECH_OK);
	T_CHECK(p == 0.0f);

	T_CHECK(derech_map_get_passability_at(m, 5, 0, &p) == DERECH_E_OOB);
	T_CHECK(derech_map_get_passability_at(m, 0, 5, &p) == DERECH_E_OOB);

	derech_map_destroy(m);
}

static void test_tags_roundtrip(void)
{
	derech_map *m = derech_map_create(3, 2, NULL);
	uint64_t buf[6] = { 0, 1, UINT64_MAX, 0x8000000000000000ULL, 42, 42 };
	uint64_t v = 1;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_get_tags_at(m, 0, 0, &v) == DERECH_OK);
	T_CHECK(v == 0);

	T_CHECK(derech_map_set_tags(m, NULL, 6) == DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_set_tags(m, buf, 7) == DERECH_E_INVALID_ARG);
	T_CHECK(derech_map_set_tags(m, buf, 6) == DERECH_OK);
	T_CHECK(derech_map_generation(m) == 1);

	T_CHECK(derech_map_get_tags_at(m, 1, 0, &v) == DERECH_OK);
	T_CHECK(v == 1);
	T_CHECK(derech_map_get_tags_at(m, 2, 0, &v) == DERECH_OK);
	T_CHECK(v == UINT64_MAX);
	T_CHECK(derech_map_get_tags_at(m, 0, 1, &v) == DERECH_OK);
	T_CHECK(v == 0x8000000000000000ULL);

	T_CHECK(derech_map_set_tags_at(m, 2, 1, 0xFFULL) == DERECH_OK);
	T_CHECK(derech_map_get_tags_at(m, 2, 1, &v) == DERECH_OK);
	T_CHECK(v == 0xFFULL);
	T_CHECK(derech_map_set_tags_at(m, 3, 0, 1) == DERECH_E_OOB);
	T_CHECK(derech_map_get_tags_at(m, 3, 0, &v) == DERECH_E_OOB);

	derech_map_destroy(m);
}

static void test_tags_exhaustion(void)
{
	derech_map *m = derech_map_create(300, 300, NULL);
	uint64_t n = 300ULL * 300ULL;
	uint64_t *buf = malloc((size_t)n * sizeof(*buf));
	derech_profile_desc d = t_neutral_desc();
	uint64_t v = 1;

	T_CHECK(m != NULL);
	T_CHECK(buf != NULL);
	/* a registered profile forces the table-extension path while the
	 * interner grows to its cap */
	T_CHECK(derech_profile_register(m, &d) == 0);

	for (uint64_t i = 0; i < n; i++) {
		buf[i] = i;
	}
	T_CHECK(derech_map_set_tags(m, buf, n) ==
		DERECH_E_TAG_COMBOS_EXHAUSTED);

	/* tile state untouched by the failed bulk set */
	T_CHECK(derech_map_get_tags_at(m, 150, 150, &v) == DERECH_OK);
	T_CHECK(v == 0);
	T_CHECK(derech_map_generation(m) == 0);

	/* already-interned words still usable; genuinely new ones are not */
	T_CHECK(derech_map_set_tags_at(m, 0, 0, 42) == DERECH_OK);
	T_CHECK(derech_map_get_tags_at(m, 0, 0, &v) == DERECH_OK);
	T_CHECK(v == 42);
	T_CHECK(derech_map_set_tags_at(m, 0, 0, 0xABCDEF0123456789ULL) ==
		DERECH_E_TAG_COMBOS_EXHAUSTED);

	free(buf);
	derech_map_destroy(m);
}

static void test_tags_rect(void)
{
	derech_map *m = derech_map_create(4, 4, NULL);
	uint64_t rect[4] = { 7, 7, 9, 9 };
	uint64_t v = 0;

	T_CHECK(m != NULL);
	T_CHECK(derech_map_set_tags_rect(m, 2, 2, 2, 2, rect) == DERECH_OK);
	T_CHECK(derech_map_get_tags_at(m, 2, 2, &v) == DERECH_OK);
	T_CHECK(v == 7);
	T_CHECK(derech_map_get_tags_at(m, 3, 3, &v) == DERECH_OK);
	T_CHECK(v == 9);
	T_CHECK(derech_map_get_tags_at(m, 1, 1, &v) == DERECH_OK);
	T_CHECK(v == 0);
	T_CHECK(derech_map_set_tags_rect(m, 3, 3, 2, 2, rect) ==
		DERECH_E_OOB);
	derech_map_destroy(m);
}

static void test_generation_counting(void)
{
	derech_map *m = derech_map_create(2, 2, NULL);
	float p4[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	uint64_t t4[4] = { 0, 0, 0, 0 };

	T_CHECK(m != NULL);
	T_CHECK(derech_map_generation(m) == 0);
	T_CHECK(derech_map_set_passability(m, p4, 4) == DERECH_OK);
	T_CHECK(derech_map_set_tags(m, t4, 4) == DERECH_OK);
	T_CHECK(derech_map_set_passability_at(m, 0, 0, 0.5f) == DERECH_OK);
	T_CHECK(derech_map_set_tags_at(m, 0, 0, 3) == DERECH_OK);
	T_CHECK(derech_map_generation(m) == 4);
	/* failures don't bump */
	T_CHECK(derech_map_set_passability_at(m, 9, 9, 0.5f) ==
		DERECH_E_OOB);
	T_CHECK(derech_map_generation(m) == 4);
	derech_map_destroy(m);
}

static void test_threads_opts(void)
{
	derech_map_opts o;
	derech_map *m;

	/* explicit thread counts are honored exactly */
	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.n_threads = 4;
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_thread_count(m) == 4);
	derech_map_destroy(m);

	o.n_threads = 1;
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_thread_count(m) == 1);
	derech_map_destroy(m);

	o.n_threads = DERECH_MAX_THREADS + 1;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);

	/* auto resolves to at least one, at most 16 */
	o.n_threads = 0;
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_thread_count(m) >= 1);
	T_CHECK(derech_map_thread_count(m) <= 16);
	derech_map_destroy(m);

	/* the 16-byte v0.1 opts layout (no n_threads) is still accepted */
	o.struct_size = 16;
	o.default_passability = 0.5f;
	o.default_tags = 7;
	o.n_threads = 99; /* must be ignored at this struct_size */
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_thread_count(m) >= 1);
	{
		float p = -1.0f;
		uint64_t tags = 0;

		T_CHECK(derech_map_get_passability_at(m, 0, 0, &p) ==
			DERECH_OK);
		T_CHECK(p == 0.5f);
		T_CHECK(derech_map_get_tags_at(m, 0, 0, &tags) == DERECH_OK);
		T_CHECK(tags == 7);
	}
	derech_map_destroy(m);

	/* the 24-byte v0.2 layout reads n_threads but no field options */
	memset(&o, 0, sizeof(o));
	o.struct_size = 24;
	o.default_passability = 1.0f;
	o.n_threads = 2;
	o.field_cache_mb = 99999; /* must be ignored at this struct_size */
	m = derech_map_create(4, 4, &o);
	T_CHECK(m != NULL);
	T_CHECK(derech_map_thread_count(m) == 2);
	derech_map_destroy(m);

	/* current layout validates the field options */
	memset(&o, 0, sizeof(o));
	o.struct_size = (uint32_t)sizeof(o);
	o.default_passability = 1.0f;
	o.field_cache_mb = 4097;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);
	o.field_cache_mb = 0;
	o.field_group_threshold = 65537;
	T_CHECK(derech_map_create(4, 4, &o) == NULL);
}

static void test_version(void)
{
	T_CHECK(derech_version() == ((0u << 16) | (4u << 8) | 0u));
	T_CHECK(strcmp(derech_version_str(), "0.4.0") == 0);
	T_CHECK(strcmp(derech_status_str(DERECH_OK), "ok") == 0);
	T_CHECK(strcmp(derech_status_str(-9999), "unknown status") == 0);
	T_CHECK(strcmp(derech_path_status_str(DERECH_PATH_FOUND),
		"found") == 0);
}

int main(void)
{
	test_create_destroy();
	test_create_opts();
	test_set_passability_full();
	test_set_passability_atomicity();
	test_set_passability_rect();
	test_tags_roundtrip();
	test_tags_exhaustion();
	test_tags_rect();
	test_generation_counting();
	test_threads_opts();
	test_version();
	return t_done("test_map");
}
