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
	return DERECH_VERSION_STRING;
}

uint32_t derech_abi_version(void)
{
	return DERECH_ABI_VERSION;
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
	case DERECH_E_RESOURCE_LIMIT:
		return "configured resource limit exceeded";
	case DERECH_E_CANCELLED:
		return "cancelled";
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

/* Restore the logical interner after a validate-stage failure.  Capacity
 * growth may be retained, but words learned by the failed setter disappear
 * and every lookup table entry is rebuilt from the committed prefix. */
static void combo_rollback(derech_map *map, uint32_t committed_count)
{
	map->combo_count = committed_count;
	memset(map->combo_table, 0,
		(size_t)map->combo_table_cap * sizeof(*map->combo_table));
	for (uint32_t i = 0; i < committed_count; i++) {
		combo_table_insert(map->combo_table, map->combo_table_cap,
			map->combo_words[i], i);
	}
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/* Older derech_map_opts layouts remain accepted for ABI compatibility. */
#define DERECH_MAP_OPTS_SIZE_V1 16u
#define DERECH_MAP_OPTS_SIZE_V2_32 20u
#define DERECH_MAP_OPTS_SIZE_V2_64 24u
#define DERECH_MAP_OPTS_SIZE_V3_32 28u
#define DERECH_MAP_OPTS_SIZE_V3_64 32u
#define DERECH_DEFAULT_WORKER_MB 256u
#define DERECH_DEFAULT_SCRATCH_MB 16u
#define DERECH_DEFAULT_LABEL_MB 64u

typedef struct resolved_opts {
	float fill_p;
	uint64_t fill_tags;
	uint32_t n_threads;
	uint32_t cache_mb;
	uint32_t threshold;
	uint64_t worker_budget_bytes;
	uint64_t field_working_bytes;
	uint64_t scratch_retention_bytes;
	uint64_t label_budget_bytes;
} resolved_opts;

static uint64_t context_fixed_bytes(uint32_t n)
{
	return (uint64_t)n * (sizeof(uint64_t) + sizeof(uint32_t) +
		sizeof(uint8_t)) + 4096u * sizeof(derech_heap_entry);
}

static derech_status resolve_opts(uint32_t width, uint32_t height,
	const derech_map_opts *opts, resolved_opts *out)
{
	uint32_t requested_threads = 0;
	uint32_t worker_mb = 0;
	uint32_t field_working_mb = 0;
	uint32_t scratch_mb = 0;
	uint32_t label_mb = 0;
	uint64_t n;
	uint64_t one_field;
	int automatic = 1;

	memset(out, 0, sizeof(*out));
	out->fill_p = 1.0f;
	if (width == 0 || height == 0 || width > DERECH_MAX_DIM ||
		height > DERECH_MAX_DIM) {
		return DERECH_E_INVALID_ARG;
	}
	if (opts != NULL) {
		if (opts->struct_size != sizeof(*opts) &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V1 &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V2_32 &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V2_64 &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V3_32 &&
			opts->struct_size != DERECH_MAP_OPTS_SIZE_V3_64) {
			return DERECH_E_INVALID_ARG;
		}
		if (!passability_valid(opts->default_passability)) {
			return DERECH_E_INVALID_ARG;
		}
		out->fill_p = opts->default_passability;
		out->fill_tags = opts->default_tags;
		if (opts->struct_size >= DERECH_MAP_OPTS_SIZE_V2_32) {
			requested_threads = opts->n_threads;
		}
		if (opts->struct_size >= DERECH_MAP_OPTS_SIZE_V3_32) {
			out->cache_mb = opts->field_cache_mb;
			out->threshold = opts->field_group_threshold;
		}
		if (opts->struct_size == sizeof(*opts)) {
			if (opts->reserved0 != 0) {
				return DERECH_E_INVALID_ARG;
			}
			worker_mb = opts->worker_memory_mb;
			field_working_mb = opts->field_working_mb;
			scratch_mb = opts->scratch_retention_mb;
			label_mb = opts->label_cache_mb;
		}
	}
	if (requested_threads > DERECH_MAX_THREADS || out->cache_mb > 4096 ||
		out->threshold > 65536 || worker_mb > 4096 ||
		field_working_mb > 4096 || scratch_mb > 4096 || label_mb > 4096) {
		return DERECH_E_INVALID_ARG;
	}
	n = (uint64_t)width * height;
	automatic = requested_threads == 0;
	if (automatic) {
		requested_threads = derech_hw_threads();
		if (requested_threads > 16) {
			requested_threads = 16;
		}
	}
	out->worker_budget_bytes = worker_mb != 0 ? (uint64_t)worker_mb << 20 :
		(automatic ? (uint64_t)DERECH_DEFAULT_WORKER_MB << 20 : UINT64_MAX);
	while (automatic && requested_threads > 1 &&
		(uint64_t)requested_threads * context_fixed_bytes((uint32_t)n) >
			out->worker_budget_bytes) {
		requested_threads--;
	}
	if ((uint64_t)requested_threads * context_fixed_bytes((uint32_t)n) >
		out->worker_budget_bytes) {
		return DERECH_E_RESOURCE_LIMIT;
	}
	out->n_threads = requested_threads;
	out->cache_mb = out->cache_mb == 0 ? 64 : out->cache_mb;
	out->threshold = out->threshold == 0 ? 4 : out->threshold;
	one_field = sizeof(derech_field) + n * 9;
	out->field_working_bytes = field_working_mb == 0 ?
		((uint64_t)out->cache_mb << 20) + one_field :
		(uint64_t)field_working_mb << 20;
	out->scratch_retention_bytes = (uint64_t)(scratch_mb == 0 ?
		DERECH_DEFAULT_SCRATCH_MB : scratch_mb) << 20;
	out->label_budget_bytes = (uint64_t)(label_mb == 0 ?
		DERECH_DEFAULT_LABEL_MB : label_mb) << 20;
	if (out->label_budget_bytes < n * sizeof(uint32_t)) {
		out->label_budget_bytes = n * sizeof(uint32_t);
	}
	return DERECH_OK;
}

static int ctx_init(derech_search_ctx *ctx, uint32_t n)
{
	ctx->g = malloc((size_t)n * sizeof(*ctx->g));
	ctx->stamp = calloc(n, sizeof(*ctx->stamp));
	ctx->parent = malloc((size_t)n * sizeof(*ctx->parent));
	ctx->heap_cap = 4096;
	ctx->heap = malloc((size_t)ctx->heap_cap * sizeof(*ctx->heap));
	ctx->initialized = ctx->g != NULL && ctx->stamp != NULL &&
		ctx->parent != NULL && ctx->heap != NULL;
	return ctx->initialized;
}

static void ctx_free(derech_search_ctx *ctx)
{
	free(ctx->g);
	free(ctx->stamp);
	free(ctx->parent);
	free(ctx->heap);
	free(ctx->out_steps);
	free(ctx->out_ticks);
	memset(ctx, 0, sizeof(*ctx));
}

derech_status derech_contexts_ensure(derech_map *map, uint32_t count)
{
	uint32_t original_count = map->allocated_contexts;

	if (count > map->n_threads) {
		count = map->n_threads;
	}
	while (map->allocated_contexts < count) {
		uint32_t i = map->allocated_contexts;

		if (!ctx_init(&map->ctxs[i], map->n)) {
			ctx_free(&map->ctxs[i]);
			while (map->allocated_contexts > original_count) {
				map->allocated_contexts--;
				ctx_free(&map->ctxs[map->allocated_contexts]);
			}
			return DERECH_E_NOMEM;
		}
		map->allocated_contexts++;
	}
	return DERECH_OK;
}

void derech_contexts_trim(derech_map *map)
{
	for (uint32_t i = 0; i < map->allocated_contexts; i++) {
		derech_search_ctx *ctx = &map->ctxs[i];
		uint64_t retained = ctx->heap_cap * sizeof(*ctx->heap) +
			ctx->out_cap * (2 * sizeof(*ctx->out_steps) +
				sizeof(*ctx->out_ticks));

		if (retained <= map->scratch_retention_bytes) {
			continue;
		}
		if (ctx->heap_cap > 4096) {
			free(ctx->heap);
			ctx->heap = NULL;
			ctx->heap_cap = 0;
		}
		free(ctx->out_steps);
		free(ctx->out_ticks);
		ctx->out_steps = NULL;
		ctx->out_ticks = NULL;
		ctx->out_cap = 0;
		ctx->out_len = 0;
	}
}

derech_status derech_map_create_ex(uint32_t width, uint32_t height,
	const derech_map_opts *opts, derech_map **out)
{
	resolved_opts ro;
	derech_map *map;
	uint32_t fill_q;
	uint32_t combo0;
	derech_status rc;

	if (out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	*out = NULL;
	rc = resolve_opts(width, height, opts, &ro);
	if (rc != DERECH_OK) {
		return rc;
	}
	map = calloc(1, sizeof(*map));
	if (map == NULL) {
		return DERECH_E_NOMEM;
	}
	map->w = width;
	map->h = height;
	map->n = width * height;
	derech_atomic_store_u64(&map->generation, 0);
	derech_atomic_store_u32(&map->published_profile_count, 0);
	map->n_threads = ro.n_threads;
	map->worker_budget_bytes = ro.worker_budget_bytes;
	map->scratch_retention_bytes = ro.scratch_retention_bytes;
	map->field_budget_bytes = (uint64_t)ro.cache_mb << 20;
	map->field_working_bytes = ro.field_working_bytes;
	map->field_threshold = ro.threshold;
	map->label_budget_bytes = ro.label_budget_bytes;
	map->label_working_bytes = ro.label_budget_bytes +
		(uint64_t)map->n * sizeof(uint32_t);

	map->combo_cap = 64;
	map->combo_table_cap = 256;

	map->cost_q = malloc((size_t)map->n * sizeof(*map->cost_q));
	map->combo_idx = calloc(map->n, sizeof(*map->combo_idx));
	map->combo_words = malloc((size_t)map->combo_cap *
		sizeof(*map->combo_words));
	map->combo_table = calloc(map->combo_table_cap,
		sizeof(*map->combo_table));
	map->ctxs = calloc(ro.n_threads, sizeof(*map->ctxs));

	if (map->cost_q == NULL || map->combo_idx == NULL ||
		map->combo_words == NULL || map->combo_table == NULL ||
		map->ctxs == NULL) {
		derech_map_destroy(map);
		return DERECH_E_NOMEM;
	}
	if (ro.n_threads > 1) {
		map->pool = derech_pool_create(ro.n_threads);
		if (map->pool == NULL) {
			derech_map_destroy(map);
			return DERECH_E_NOMEM;
		}
	}

	if (derech_intern_tag_word(map, ro.fill_tags, &combo0) != DERECH_OK) {
		derech_map_destroy(map);
		return DERECH_E_NOMEM;
	}
	/* combo_idx is already zero-filled = combo0 */

	fill_q = passability_to_q(ro.fill_p);
	for (uint32_t i = 0; i < map->n; i++) {
		map->cost_q[i] = fill_q;
	}
	*out = map;
	return DERECH_OK;
}

derech_map *derech_map_create(uint32_t width, uint32_t height,
	const derech_map_opts *opts)
{
	derech_map *map = NULL;

	(void)derech_map_create_ex(width, height, opts, &map);
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
		for (uint32_t i = 0; i < map->allocated_contexts; i++) {
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
	return map == NULL ? 0 : derech_atomic_load_u64(&map->generation);
}

uint32_t derech_profile_count(const derech_map *map)
{
	return map == NULL ? 0 : derech_atomic_load_u32(
		(derech_atomic_u32 *)&map->published_profile_count);
}

uint32_t derech_map_thread_count(const derech_map *map)
{
	return map == NULL ? 0 : map->n_threads;
}

static void memory_stats_fill(const derech_map *map, derech_memory_stats *s)
{
	uint64_t words = (map->n + 63u) / 64u;

	memset(s, 0, sizeof(*s));
	s->struct_size = sizeof(*s);
	s->configured_threads = map->n_threads;
	s->allocated_contexts = map->allocated_contexts;
	s->terrain_bytes = sizeof(*map) +
		(uint64_t)map->n * (sizeof(*map->cost_q) + sizeof(*map->combo_idx)) +
		(uint64_t)map->combo_cap * sizeof(*map->combo_words) +
		(uint64_t)map->combo_table_cap * sizeof(*map->combo_table) +
		(uint64_t)map->n_threads * sizeof(*map->ctxs) +
		(uint64_t)map->profile_count * sizeof(*map->profiles);
	for (uint32_t p = 0; p < map->profile_count; p++) {
		s->terrain_bytes += (uint64_t)map->combo_cap *
			sizeof(*map->profiles[p].table);
	}
	for (uint32_t g = 0; g < DERECH_MAX_GOALSETS; g++) {
		const derech_goalset *gs = &map->goalsets[g];

		if (!gs->used) {
			continue;
		}
		s->terrain_bytes += (uint64_t)gs->n_tiles * 2 *
			sizeof(*gs->tiles);
		if (gs->members != NULL) {
			s->terrain_bytes += words * sizeof(*gs->members);
		}
		if (gs->seeds != NULL) {
			s->terrain_bytes += words * sizeof(*gs->seeds);
		}
	}
	for (uint32_t i = 0; i < map->allocated_contexts; i++) {
		const derech_search_ctx *ctx = &map->ctxs[i];
		uint64_t fixed = (uint64_t)map->n * (sizeof(*ctx->g) +
			sizeof(*ctx->stamp) + sizeof(*ctx->parent));
		uint64_t retained = ctx->heap_cap * sizeof(*ctx->heap) +
			ctx->out_cap * (2 * sizeof(*ctx->out_steps) +
				sizeof(*ctx->out_ticks));

		s->worker_bytes += fixed + retained;
		s->retained_scratch_bytes += retained;
	}
	s->field_bytes = map->field_bytes;
	s->field_peak_bytes = map->field_peak_bytes;
	s->field_cache_bytes = map->field_budget_bytes;
	s->field_working_bytes = map->field_working_bytes;
	s->label_bytes = map->label_bytes;
	s->label_cache_bytes = map->label_budget_bytes;
}

derech_status derech_map_memory_estimate(uint32_t width, uint32_t height,
	const derech_map_opts *opts, derech_memory_stats *out)
{
	resolved_opts ro;
	derech_memory_stats s;
	derech_status rc;
	uint32_t n;

	if (out == NULL || out->struct_size != sizeof(*out)) {
		return DERECH_E_INVALID_ARG;
	}
	rc = resolve_opts(width, height, opts, &ro);
	if (rc != DERECH_OK) {
		return rc;
	}
	n = width * height;
	memset(&s, 0, sizeof(s));
	s.struct_size = sizeof(s);
	s.configured_threads = ro.n_threads;
	s.terrain_bytes = sizeof(derech_map) +
		(uint64_t)n * (sizeof(uint32_t) + sizeof(uint16_t)) +
		64u * sizeof(uint64_t) + 256u * sizeof(derech_combo_slot) +
		(uint64_t)ro.n_threads * sizeof(derech_search_ctx);
	s.worker_bytes = (uint64_t)ro.n_threads * context_fixed_bytes(n);
	s.field_cache_bytes = (uint64_t)ro.cache_mb << 20;
	s.field_working_bytes = ro.field_working_bytes;
	s.label_cache_bytes = ro.label_budget_bytes;
	s.retained_scratch_bytes = (uint64_t)ro.n_threads *
		4096u * sizeof(derech_heap_entry);
	*out = s;
	return DERECH_OK;
}

derech_status derech_map_get_memory_stats(const derech_map *const_map,
	derech_memory_stats *out)
{
	derech_map *map = (derech_map *)const_map;
	derech_memory_stats s;

	if (map == NULL || out == NULL || out->struct_size != sizeof(*out)) {
		return DERECH_E_INVALID_ARG;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	memory_stats_fill(map, &s);
	derech_busy_release(&map->busy);
	*out = s;
	return DERECH_OK;
}

derech_status derech_cancel_create(derech_cancel **out)
{
	derech_cancel *cancel;

	if (out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	*out = NULL;
	cancel = calloc(1, sizeof(*cancel));
	if (cancel == NULL) {
		return DERECH_E_NOMEM;
	}
	derech_atomic_store_u32(&cancel->requested, 0);
	*out = cancel;
	return DERECH_OK;
}

void derech_cancel_request(derech_cancel *cancel)
{
	if (cancel != NULL) {
		derech_atomic_store_u32(&cancel->requested, 1);
	}
}

void derech_cancel_destroy(derech_cancel *cancel)
{
	free(cancel);
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
	derech_atomic_fetch_add_u64(&map->generation, 1);
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
	derech_atomic_fetch_add_u64(&map->generation, 1);
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
	uint32_t committed_combo_count;
	derech_status rc = DERECH_OK;

	if (map == NULL || tags == NULL || count != map->n) {
		return DERECH_E_INVALID_ARG;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}
	committed_combo_count = map->combo_count;
	scratch = malloc((size_t)map->n * sizeof(*scratch));
	if (scratch == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	for (uint32_t i = 0; i < map->n; i++) {
		uint32_t idx;

		rc = derech_intern_tag_word(map, tags[i], &idx);
		if (rc != DERECH_OK) {
			combo_rollback(map, committed_combo_count);
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
	derech_atomic_fetch_add_u64(&map->generation, 1);
	derech_busy_release(&map->busy);
	return DERECH_OK;
}

derech_status derech_map_set_tags_rect(derech_map *map, uint32_t x,
	uint32_t y, uint32_t w, uint32_t h, const uint64_t *tags)
{
	uint16_t *scratch;
	uint64_t count;
	uint32_t committed_combo_count;
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
	committed_combo_count = map->combo_count;
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
			combo_rollback(map, committed_combo_count);
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
	derech_atomic_fetch_add_u64(&map->generation, 1);
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
	derech_map *mutable_map = (derech_map *)map;
	uint32_t q;

	if (map == NULL || out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	if (x >= map->w || y >= map->h) {
		return DERECH_E_OOB;
	}
	if (!derech_busy_acquire(&mutable_map->busy)) {
		return DERECH_E_BUSY;
	}
	q = map->cost_q[(size_t)y * map->w + x];
	*out = q == DERECH_Q_BLOCKED ? 0.0f : (float)(256.0 / (double)q);
	derech_busy_release(&mutable_map->busy);
	return DERECH_OK;
}

derech_status derech_map_get_tags_at(const derech_map *map, uint32_t x,
	uint32_t y, uint64_t *out)
{
	derech_map *mutable_map = (derech_map *)map;

	if (map == NULL || out == NULL) {
		return DERECH_E_INVALID_ARG;
	}
	if (x >= map->w || y >= map->h) {
		return DERECH_E_OOB;
	}
	if (!derech_busy_acquire(&mutable_map->busy)) {
		return DERECH_E_BUSY;
	}
	*out = map->combo_words[map->combo_idx[(size_t)y * map->w + x]];
	derech_busy_release(&mutable_map->busy);
	return DERECH_OK;
}
