/* Map lifecycle, terrain storage, and tag-word interning. */

#include <stdio.h>

#include "derech_internal.h"

const int8_t derech_dir_dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
const int8_t derech_dir_dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

uint32_t derech_version(void)
{
	return ((uint32_t)DERECH_VERSION_MAJOR << 16) |
		((uint32_t)DERECH_VERSION_MINOR << 8) |
		(uint32_t)DERECH_VERSION_PATCH;
}

const char *derech_version_str(void)
{
	return "0.4.0";
}

const char *derech_status_str(derech_status s)
{
	switch (s) {
	case DERECH_OK:
		return "ok";
	case DERECH_E_INVALID_ARG:
		return "invalid argument";
	case DERECH_E_OOB:
		return "coordinate out of bounds";
	case DERECH_E_NOMEM:
		return "out of memory";
	case DERECH_E_BUSY:
		return "another call is active on this map";
	case DERECH_E_BAD_PROFILE:
		return "unknown profile id";
	case DERECH_E_TAG_COMBOS_EXHAUSTED:
		return "too many distinct tag words on this map";
	case DERECH_E_TOO_MANY_PROFILES:
		return "too many profiles on this map";
	case DERECH_E_BAD_GOALSET:
		return "unknown goal-set id";
	case DERECH_E_TOO_MANY_GOALSETS:
		return "too many goal sets on this map";
	default:
		return "unknown status";
	}
}

const char *derech_path_status_str(derech_path_status s)
{
	switch (s) {
	case DERECH_PATH_NONE:
		return "none";
	case DERECH_PATH_FOUND:
		return "found";
	case DERECH_PATH_UNREACHABLE:
		return "unreachable";
	case DERECH_PATH_BUDGET_EXCEEDED:
		return "budget exceeded";
	case DERECH_PATH_INVALID_ENDPOINT:
		return "invalid endpoint";
	default:
		return "unknown path status";
	}
}

/* ------------------------------------------------------------------ */
/* Quantization & validation helpers                                   */
/* ------------------------------------------------------------------ */

/* Rejects NaN/inf/out-of-range via the negated comparison. */
static int passability_valid(float p)
{
	return p >= 0.0f && p <= 1.0f;
}

static uint32_t passability_to_q(float p)
{
	if (p <= 0.0f) {
		return DERECH_Q_BLOCKED;
	}
	return derech_q_round(256.0 / (double)p, DERECH_Q_TILE_MAX);
}

/* ------------------------------------------------------------------ */
/* Tag-word interning                                                  */
/* ------------------------------------------------------------------ */

static uint32_t combo_hash(uint64_t x)
{
	/* splitmix64 finalizer */
	x ^= x >> 30;
	x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27;
	x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	return (uint32_t)(x >> 32) ^ (uint32_t)x;
}

static void combo_table_insert(derech_combo_slot *table, uint32_t cap,
	uint64_t key, uint32_t idx)
{
	uint32_t mask = cap - 1;
	uint32_t i = combo_hash(key) & mask;

	while (table[i].val_plus1 != 0) {
		i = (i + 1) & mask;
	}
	table[i].key = key;
	table[i].val_plus1 = idx + 1;
}

static derech_status combo_table_grow(derech_map *map)
{
	uint32_t new_cap = map->combo_table_cap * 2;
	derech_combo_slot *table = calloc(new_cap, sizeof(*table));

	if (table == NULL) {
		return DERECH_E_NOMEM;
	}
	for (uint32_t i = 0; i < map->combo_count; i++) {
		combo_table_insert(table, new_cap, map->combo_words[i], i);
	}
	free(map->combo_table);
	map->combo_table = table;
	map->combo_table_cap = new_cap;
	return DERECH_OK;
}

static derech_status combo_words_grow(derech_map *map)
{
	uint32_t new_cap = map->combo_cap * 2;
	uint64_t *words;

	if (new_cap > DERECH_MAX_TAG_COMBOS) {
		new_cap = DERECH_MAX_TAG_COMBOS;
	}
	words = realloc(map->combo_words, (size_t)new_cap * sizeof(*words));
	if (words == NULL) {
		return DERECH_E_NOMEM;
	}
	map->combo_words = words;

	/* keep every profile table in step with the word capacity */
	for (uint32_t i = 0; i < map->profile_count; i++) {
		derech_pentry *t = realloc(map->profiles[i].table,
			(size_t)new_cap * sizeof(*t));

		if (t == NULL) {
			return DERECH_E_NOMEM;
		}
		map->profiles[i].table = t;
	}
	map->combo_cap = new_cap;
	return DERECH_OK;
}

/* Returns DERECH_OK and the combo index for `word`, interning it (and
 * extending every registered profile's folded table) if new. */
int derech_intern_tag_word(derech_map *map, uint64_t word, uint32_t *out_idx)
{
	uint32_t mask = map->combo_table_cap - 1;
	uint32_t i = combo_hash(word) & mask;
	uint32_t idx;
	derech_status rc;

	while (map->combo_table[i].val_plus1 != 0) {
		if (map->combo_table[i].key == word) {
			*out_idx = map->combo_table[i].val_plus1 - 1;
			return DERECH_OK;
		}
		i = (i + 1) & mask;
	}

	if (map->combo_count >= DERECH_MAX_TAG_COMBOS) {
		return DERECH_E_TAG_COMBOS_EXHAUSTED;
	}
	if (map->combo_count == map->combo_cap) {
		rc = combo_words_grow(map);
		if (rc != DERECH_OK) {
			return rc;
		}
	}
	/* grow at 70% load so probes stay short */
	if ((uint64_t)(map->combo_count + 1) * 10 >
		(uint64_t)map->combo_table_cap * 7) {
		rc = combo_table_grow(map);
		if (rc != DERECH_OK) {
			return rc;
		}
	}

	idx = map->combo_count;
	map->combo_words[idx] = word;
	map->combo_count = idx + 1;
	combo_table_insert(map->combo_table, map->combo_table_cap, word, idx);

	for (uint32_t p = 0; p < map->profile_count; p++) {
		derech_profile_fold_combo(&map->profiles[p], word,
			&map->profiles[p].table[idx]);
	}
	*out_idx = idx;
	return DERECH_OK;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/* Older derech_map_opts layouts remain accepted for ABI compatibility:
 * v0.1 had no n_threads (16 bytes), v0.2 no field options (24 bytes). */
#define DERECH_MAP_OPTS_SIZE_V1 16u
#define DERECH_MAP_OPTS_SIZE_V2 24u

static int ctx_init(derech_search_ctx *ctx, uint32_t n)
{
	ctx->g = malloc((size_t)n * sizeof(*ctx->g));
	ctx->stamp = calloc(n, sizeof(*ctx->stamp));
	ctx->parent = malloc((size_t)n * sizeof(*ctx->parent));
	ctx->path_scratch = malloc((size_t)n * sizeof(*ctx->path_scratch));
	ctx->heap_cap = 4096;
	ctx->heap = malloc((size_t)ctx->heap_cap * sizeof(*ctx->heap));
	return ctx->g != NULL && ctx->stamp != NULL && ctx->parent != NULL &&
		ctx->path_scratch != NULL && ctx->heap != NULL;
}

static void ctx_free(derech_search_ctx *ctx)
{
	free(ctx->g);
	free(ctx->stamp);
	free(ctx->parent);
	free(ctx->path_scratch);
	free(ctx->heap);
	free(ctx->out_steps);
	free(ctx->out_ticks);
}

derech_map *derech_map_create(uint32_t width, uint32_t height,
	const derech_map_opts *opts)
{
	derech_map *map;
	float fill_p = 1.0f;
	uint64_t fill_tags = 0;
	uint32_t n_threads = 0;
	uint32_t cache_mb = 0;
	uint32_t threshold = 0;
	uint32_t fill_q;
	uint32_t combo0;

	if (width == 0 || height == 0 || width > DERECH_MAX_DIM ||
		height > DERECH_MAX_DIM) {
		return NULL;
	}
	if (opts != NULL) {
		if (opts->struct_size != sizeof(*opts) &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V1 &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V2) {
			return NULL;
		}
		if (!passability_valid(opts->default_passability)) {
			return NULL;
		}
		fill_p = opts->default_passability;
		fill_tags = opts->default_tags;
		if (opts->struct_size >= DERECH_MAP_OPTS_SIZE_V2) {
			n_threads = opts->n_threads;
		}
		if (opts->struct_size == sizeof(*opts)) {
			cache_mb = opts->field_cache_mb;
			threshold = opts->field_group_threshold;
		}
	}
	if (n_threads > DERECH_MAX_THREADS || cache_mb > 4096 ||
		threshold > 65536) {
		return NULL;
	}
	if (n_threads == 0) {
		n_threads = derech_hw_threads();
		if (n_threads > 16) {
			n_threads = 16;
		}
	}

	map = calloc(1, sizeof(*map));
	if (map == NULL) {
		return NULL;
	}
	map->w = width;
	map->h = height;
	map->n = width * height;
	map->generation = 0;
	map->n_threads = n_threads;
	map->field_budget_bytes =
		(uint64_t)(cache_mb == 0 ? 64 : cache_mb) << 20;
	map->field_threshold = threshold == 0 ? 4 : threshold;

	map->combo_cap = 64;
	map->combo_table_cap = 256;

	map->cost_q = malloc((size_t)map->n * sizeof(*map->cost_q));
	map->combo_idx = calloc(map->n, sizeof(*map->combo_idx));
	map->combo_words = malloc((size_t)map->combo_cap *
		sizeof(*map->combo_words));
	map->combo_table = calloc(map->combo_table_cap,
		sizeof(*map->combo_table));
	map->ctxs = calloc(n_threads, sizeof(*map->ctxs));

	if (map->cost_q == NULL || map->combo_idx == NULL ||
		map->combo_words == NULL || map->combo_table == NULL ||
		map->ctxs == NULL) {
		derech_map_destroy(map);
		return NULL;
	}
	for (uint32_t i = 0; i < n_threads; i++) {
		if (!ctx_init(&map->ctxs[i], map->n)) {
			derech_map_destroy(map);
			return NULL;
		}
	}
	if (n_threads > 1) {
		map->pool = derech_pool_create(n_threads);
		if (map->pool == NULL) {
			derech_map_destroy(map);
			return NULL;
		}
	}

	if (derech_intern_tag_word(map, fill_tags, &combo0) != DERECH_OK) {
		derech_map_destroy(map);
		return NULL;
	}
	/* combo_idx is already zero-filled = combo0 */

	fill_q = passability_to_q(fill_p);
	for (uint32_t i = 0; i < map->n; i++) {
		map->cost_q[i] = fill_q;
	}
	return map;
}

void derech_map_destroy(derech_map *map)
{
	if (map == NULL) {
		return;
	}
	derech_pool_destroy(map->pool); /* joins workers before ctxs go */
	derech_field_cache_flush(map);
	derech_labels_flush(map);
	for (uint32_t i = 0; i < DERECH_MAX_GOALSETS; i++) {
		derech_goalset_free(&map->goalsets[i]);
	}
	if (map->ctxs != NULL) {
		for (uint32_t i = 0; i < map->n_threads; i++) {
			ctx_free(&map->ctxs[i]);
		}
	}
	free(map->ctxs);
	for (uint32_t i = 0; i < map->profile_count; i++) {
		free(map->profiles[i].table);
	}
	free(map->profiles);
	free(map->cost_q);
	free(map->combo_idx);
	free(map->combo_words);
	free(map->combo_table);
	free(map);
}

uint32_t derech_map_width(const derech_map *map)
{
	return map == NULL ? 0 : map->w;
}

uint32_t derech_map_height(const derech_map *map)
{
	return map == NULL ? 0 : map->h;
}

uint64_t derech_map_generation(const derech_map *map)
{
	return map == NULL ? 0 : map->generation;
}

uint32_t derech_profile_count(const derech_map *map)
{
	return map == NULL ? 0 : map->profile_count;
}

uint32_t derech_map_thread_count(const derech_map *map)
{
	return map == NULL ? 0 : map->n_threads;
}

/* ------------------------------------------------------------------ */
/* Terrain setters                                                     */
/* ------------------------------------------------------------------ */

/* Record an edit for the batch-start reconcile.  Rects that changed
 * nothing are never recorded, so idempotent rewrites stay free. */
void derech_dirty_add(derech_map *map, uint32_t x, uint32_t y, uint32_t w,
	uint32_t h, int pass_changed, uint64_t tag_bits)
{
	derech_dirty *d = &map->dirty;

	if (!pass_changed && tag_bits == 0) {
		return;
	}
	if (d->full) {
		return;
	}
	if ((uint64_t)w * h >= map->n || d->count == DERECH_MAX_DIRTY_RECTS) {
		d->full = 1;
		return;
	}
	d->rects[d->count].x = x;
	d->rects[d->count].y = y;
	d->rects[d->count].w = w;
	d->rects[d->count].h = h;
	d->rects[d->count].pass_changed = pass_changed ? 1 : 0;
	d->rects[d->count].tag_bits = tag_bits;
	d->count++;
}

/* Shared guard prologue: arg checks that don't depend on payload go
 * before the busy acquire so misuse never spuriously reports BUSY. */

derech_status derech_map_set_passability(derech_map *map, const float *p,
	uint64_t count)
{
	int changed = 0;

	if (map == NULL || p == NULL || count != map->n) {
		return DERECH_E_INVALID_ARG;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	for (uint32_t i = 0; i < map->n; i++) {
		if (!passability_valid(p[i])) {
			derech_busy_release(&map->busy);
			return DERECH_E_INVALID_ARG;
		}
	}
	for (uint32_t i = 0; i < map->n; i++) {
		uint32_t q = passability_to_q(p[i]);

		changed |= map->cost_q[i] != q;
		map->cost_q[i] = q;
	}
	derech_dirty_add(map, 0, 0, map->w, map->h, changed, 0);
	map->generation++;
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

static derech_status rect_valid(const derech_map *map, uint32_t x, uint32_t y,
	uint32_t w, uint32_t h)
{
	if (w == 0 || h == 0) {
		return DERECH_E_INVALID_ARG;
	}
	/* u64 math: x + w cannot overflow and cannot wrap past the map */
	if ((uint64_t)x + w > map->w || (uint64_t)y + h > map->h) {
		return DERECH_E_OOB;
	}
	return DERECH_OK;
}

derech_status derech_map_set_passability_rect(derech_map *map, uint32_t x,
	uint32_t y, uint32_t w, uint32_t h, const float *p)
{
	derech_status rc;

	if (map == NULL || p == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	rc = rect_valid(map, x, y, w, h);
	if (rc != DERECH_OK) {
		return rc;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	for (uint64_t i = 0; i < (uint64_t)w * h; i++) {
		if (!passability_valid(p[i])) {
			derech_busy_release(&map->busy);
			return DERECH_E_INVALID_ARG;
		}
	}
	{
		int changed = 0;

		for (uint32_t row = 0; row < h; row++) {
			uint32_t base = (y + row) * map->w + x;

			for (uint32_t col = 0; col < w; col++) {
				uint32_t q = passability_to_q(
					p[(uint64_t)row * w + col]);

				changed |= map->cost_q[base + col] != q;
				map->cost_q[base + col] = q;
			}
		}
		derech_dirty_add(map, x, y, w, h, changed, 0);
	}
	map->generation++;
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

derech_status derech_map_set_passability_at(derech_map *map, uint32_t x,
	uint32_t y, float p)
{
	return derech_map_set_passability_rect(map, x, y, 1, 1, &p);
}

derech_status derech_map_set_tags(derech_map *map, const uint64_t *tags,
	uint64_t count)
{
	uint16_t *scratch;
	derech_status rc = DERECH_OK;

	if (map == NULL || tags == NULL || count != map->n) {
		return DERECH_E_INVALID_ARG;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	scratch = malloc((size_t)map->n * sizeof(*scratch));
	if (scratch == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	for (uint32_t i = 0; i < map->n; i++) {
		uint32_t idx;

		rc = derech_intern_tag_word(map, tags[i], &idx);
		if (rc != DERECH_OK) {
			free(scratch);
			derech_busy_release(&map->busy);
			return rc;
		}
		scratch[i] = (uint16_t)idx;
	}
	{
		uint64_t bits = 0;

		for (uint32_t i = 0; i < map->n; i++) {
			bits |= map->combo_words[map->combo_idx[i]] ^
				map->combo_words[scratch[i]];
			map->combo_idx[i] = scratch[i];
		}
		derech_dirty_add(map, 0, 0, map->w, map->h, 0, bits);
	}
	free(scratch);
	map->generation++;
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

derech_status derech_map_set_tags_rect(derech_map *map, uint32_t x,
	uint32_t y, uint32_t w, uint32_t h, const uint64_t *tags)
{
	uint16_t *scratch;
	uint64_t count;
	derech_status rc;

	if (map == NULL || tags == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	rc = rect_valid(map, x, y, w, h);
	if (rc != DERECH_OK) {
		return rc;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	count = (uint64_t)w * h;
	scratch = malloc((size_t)count * sizeof(*scratch));
	if (scratch == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	for (uint64_t i = 0; i < count; i++) {
		uint32_t idx;

		rc = derech_intern_tag_word(map, tags[i], &idx);
		if (rc != DERECH_OK) {
			free(scratch);
			derech_busy_release(&map->busy);
			return rc;
		}
		scratch[i] = (uint16_t)idx;
	}
	{
		uint64_t bits = 0;

		for (uint32_t row = 0; row < h; row++) {
			uint32_t base = (y + row) * map->w + x;

			for (uint32_t col = 0; col < w; col++) {
				uint16_t nw = scratch[(size_t)row * w + col];

				bits |= map->combo_words[
					map->combo_idx[base + col]] ^
					map->combo_words[nw];
				map->combo_idx[base + col] = nw;
			}
		}
		derech_dirty_add(map, x, y, w, h, 0, bits);
	}
	free(scratch);
	map->generation++;
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

derech_status derech_map_set_tags_at(derech_map *map, uint32_t x, uint32_t y,
	uint64_t tags)
{
	return derech_map_set_tags_rect(map, x, y, 1, 1, &tags);
}

/* ------------------------------------------------------------------ */
/* Read-back                                                           */
/* ------------------------------------------------------------------ */

derech_status derech_map_get_passability_at(const derech_map *map, uint32_t x,
	uint32_t y, float *out)
{
	uint32_t q;

	if (map == NULL || out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	if (x >= map->w || y >= map->h) {
		return DERECH_E_OOB;
	}
	q = map->cost_q[(size_t)y * map->w + x];
	*out = q == DERECH_Q_BLOCKED ? 0.0f : (float)(256.0 / (double)q);
	return DERECH_OK;
}

derech_status derech_map_get_tags_at(const derech_map *map, uint32_t x,
	uint32_t y, uint64_t *out)
{
	if (map == NULL || out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	if (x >= map->w || y >= map->h) {
		return DERECH_E_OOB;
	}
	*out = map->combo_words[map->combo_idx[(size_t)y * map->w + x]];
	return DERECH_OK;
}
