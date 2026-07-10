/* Real overlapping host calls: public readers, writers, and queries. */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "derech_thread.h"
#include "support.h"

enum { SIDE = 512, WRITES = 12 };

typedef struct writer_ctx {
	derech_map *map;
	const uint64_t *tags[2];
	const float *pass[2];
	derech_mutex mutex;
	derech_cond cond;
	int ready;
	int go;
	int done;
	uint32_t ok;
	uint32_t busy;
	derech_status error;
} writer_ctx;

static void writer_main(void *opaque)
{
	writer_ctx *ctx = opaque;
	const uint64_t n = (uint64_t)SIDE * SIDE;

	derech_mutex_lock(&ctx->mutex);
	ctx->ready = 1;
	derech_cond_signal(&ctx->cond);
	while (!ctx->go) {
		derech_cond_wait(&ctx->cond, &ctx->mutex);
	}
	derech_mutex_unlock(&ctx->mutex);

	for (uint32_t i = 0; i < WRITES && ctx->error == DERECH_OK; i++) {
		derech_status rc;
		uint32_t attempts = 0;
		int32_t profile_id;

		do {
			rc = derech_map_set_tags(ctx->map, ctx->tags[i & 1], n);
			if (rc == DERECH_E_BUSY) {
				ctx->busy++;
			}
		} while (rc == DERECH_E_BUSY && ++attempts < 1000000);
		if (rc != DERECH_OK) {
			ctx->error = rc;
			break;
		}
		ctx->ok++;

		attempts = 0;
		do {
			rc = derech_map_set_passability(ctx->map,
				ctx->pass[i & 1], n);
			if (rc == DERECH_E_BUSY) {
				ctx->busy++;
			}
		} while (rc == DERECH_E_BUSY && ++attempts < 1000000);
		if (rc != DERECH_OK) {
			ctx->error = rc;
			break;
		}
		ctx->ok++;

		attempts = 0;
		do {
			derech_profile_desc profile;

			memset(&profile, 0, sizeof(profile));
			profile.struct_size = (uint32_t)sizeof(profile);
			profile.block_mask = UINT64_C(1) << (i + 8);
			profile_id = derech_profile_register(ctx->map, &profile);
			if (profile_id == DERECH_E_BUSY) {
				ctx->busy++;
			}
		} while (profile_id == DERECH_E_BUSY &&
			++attempts < 1000000);
		if (profile_id != (int32_t)i + 1) {
			ctx->error = profile_id < 0 ? profile_id :
				DERECH_E_INVALID_ARG;
			break;
		}
		ctx->ok++;
	}

	derech_mutex_lock(&ctx->mutex);
	ctx->done = 1;
	derech_cond_signal(&ctx->cond);
	derech_mutex_unlock(&ctx->mutex);
}

static int writer_done(writer_ctx *ctx)
{
	int done;

	derech_mutex_lock(&ctx->mutex);
	done = ctx->done;
	derech_mutex_unlock(&ctx->mutex);
	return done;
}

static derech_request trivial_request(void)
{
	derech_request q;

	memset(&q, 0, sizeof(q));
	q.struct_size = (uint32_t)sizeof(q);
	q.start_x = 1;
	q.start_y = 1;
	q.goal_x = 1;
	q.goal_y = 1;
	q.epsilon = 1.0f;
	return q;
}

static void test_overlapping_public_calls(void)
{
	const size_t n = (size_t)SIDE * SIDE;
	uint64_t *tags0 = calloc(n, sizeof(*tags0));
	uint64_t *tags1 = malloc(n * sizeof(*tags1));
	float *pass0 = malloc(n * sizeof(*pass0));
	float *pass1 = malloc(n * sizeof(*pass1));
	derech_map_opts opts;
	derech_map *map;
	derech_profile_desc profile = t_neutral_desc();
	writer_ctx ctx;
	derech_thread thread;
	derech_request q = trivial_request();
	int created = 0;
	uint32_t reader_busy = 0;
	uint32_t reader_ok = 0;
	uint32_t last_profile_count = 1;
	uint64_t last_generation = 0;
	int32_t set_id;

	T_CHECK(tags0 != NULL);
	T_CHECK(tags1 != NULL);
	T_CHECK(pass0 != NULL);
	T_CHECK(pass1 != NULL);
	if (tags0 == NULL || tags1 == NULL || pass0 == NULL || pass1 == NULL) {
		free(tags0);
		free(tags1);
		free(pass0);
		free(pass1);
		return;
	}
	for (size_t i = 0; i < n; i++) {
		tags1[i] = 1;
		pass0[i] = 1.0f;
		pass1[i] = 0.5f;
	}

	memset(&opts, 0, sizeof(opts));
	opts.struct_size = (uint32_t)sizeof(opts);
	opts.default_passability = 1.0f;
	opts.n_threads = 1;
	map = derech_map_create(SIDE, SIDE, &opts);
	T_CHECK(map != NULL);
	if (map == NULL) {
		free(tags0);
		free(tags1);
		free(pass0);
		free(pass1);
		return;
	}
	T_CHECK(derech_profile_register(map, &profile) == 0);
	set_id = derech_goalset_register_tags(map, 1, 0, 0);
	T_CHECK(set_id == 1);

	memset(&ctx, 0, sizeof(ctx));
	ctx.map = map;
	ctx.tags[0] = tags0;
	ctx.tags[1] = tags1;
	ctx.pass[0] = pass0;
	ctx.pass[1] = pass1;
	derech_mutex_init(&ctx.mutex);
	derech_cond_init(&ctx.cond);
	created = derech_thread_create(&thread, writer_main, &ctx);
	T_CHECK(created != 0);
	if (!created) {
		derech_cond_destroy(&ctx.cond);
		derech_mutex_destroy(&ctx.mutex);
		derech_map_destroy(map);
		free(tags0);
		free(tags1);
		free(pass0);
		free(pass1);
		return;
	}

	derech_mutex_lock(&ctx.mutex);
	while (!ctx.ready) {
		derech_cond_wait(&ctx.cond, &ctx.mutex);
	}
	ctx.go = 1;
	derech_cond_signal(&ctx.cond);
	derech_mutex_unlock(&ctx.mutex);

	for (uint32_t iteration = 0; iteration < 500000; iteration++) {
		float pass = 0.0f;
		uint64_t tags = 9;
		derech_status rc;
		int64_t count;
		uint32_t profile_count;
		uint64_t generation;

		rc = derech_map_get_tags_at(map, SIDE / 2, SIDE / 2, &tags);
		if (rc == DERECH_E_BUSY) {
			reader_busy++;
		} else {
			T_CHECK(rc == DERECH_OK);
			T_CHECK(tags == 0 || tags == 1);
			reader_ok++;
		}
		rc = derech_map_get_passability_at(map, SIDE / 2, SIDE / 2,
			&pass);
		if (rc == DERECH_E_BUSY) {
			reader_busy++;
		} else {
			T_CHECK(rc == DERECH_OK);
			T_CHECK(pass == 1.0f || pass == 0.5f);
			reader_ok++;
		}
		count = derech_goalset_count(map, (uint32_t)set_id);
		if (count == DERECH_E_BUSY) {
			reader_busy++;
		} else {
			T_CHECK(count == 0 || count == (int64_t)n);
			reader_ok++;
		}
		generation = derech_map_generation(map);
		T_CHECK(generation >= last_generation);
		last_generation = generation;
		profile_count = derech_profile_count(map);
		T_CHECK(profile_count >= last_profile_count);
		T_CHECK(profile_count <= WRITES + 1);
		last_profile_count = profile_count;
		T_CHECK(derech_map_width(map) == SIDE);
		T_CHECK(derech_map_height(map) == SIDE);

		if ((iteration & 127u) == 0) {
			derech_results *results = NULL;

			rc = derech_find_paths(map, &q, 1, &results);
			if (rc == DERECH_E_BUSY) {
				reader_busy++;
			} else {
				T_CHECK(rc == DERECH_OK);
				T_CHECK(results != NULL);
				if (results != NULL) {
					T_CHECK(derech_result_status(results, 0) ==
						DERECH_PATH_FOUND);
				}
				derech_results_destroy(results);
				reader_ok++;
			}
		}
		if ((iteration & 63u) == 0 && writer_done(&ctx)) {
			break;
		}
	}

	derech_thread_join(thread);
	T_CHECK(ctx.error == DERECH_OK);
	T_CHECK(ctx.ok == WRITES * 3);
	T_CHECK(reader_busy > 0);
	T_CHECK(reader_ok > 0);
	T_CHECK(derech_profile_count(map) == WRITES + 1);
	T_CHECK(derech_map_generation(map) >= WRITES * 3);

	derech_cond_destroy(&ctx.cond);
	derech_mutex_destroy(&ctx.mutex);
	derech_map_destroy(map);
	free(tags0);
	free(tags1);
	free(pass0);
	free(pass1);
}

int main(void)
{
	test_overlapping_public_calls();
	return t_done("concurrency");
}
