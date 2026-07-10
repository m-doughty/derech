/* Internal declarations shared by derech translation units.  Not installed;
 * white-box tests may include it. */

#ifndef DERECH_INTERNAL_H
#define DERECH_INTERNAL_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "derech.h"

/* ------------------------------------------------------------------ */
/* Internal allocation boundary                                       */
/* ------------------------------------------------------------------ */

void *derech_malloc(size_t size);
void *derech_calloc(size_t count, size_t size);
void *derech_realloc(void *ptr, size_t size);
void derech_free(void *ptr);

#ifdef DERECH_TESTING
/* Fail exactly one allocation: the one after `successful_allocations`
 * successful attempts following this call.  Passing 0 fails the next
 * allocation.  The attempt counter includes the injected failure. */
void derech_test_alloc_fail_after(uint64_t successful_allocations);
void derech_test_alloc_disable(void);
uint64_t derech_test_alloc_attempts(void);
#endif

/* DERECH_BUILD is private to the library target, so tests that include
 * this header keep using libc directly.  alloc.c suppresses the remap to
 * implement the boundary in terms of the real allocator. */
#if defined(DERECH_BUILD) && !defined(DERECH_ALLOC_IMPLEMENTATION)
#define malloc(size) derech_malloc(size)
#define calloc(count, size) derech_calloc((count), (size))
#define realloc(ptr, size) derech_realloc((ptr), (size))
#define free(ptr) derech_free(ptr)
#endif

/* ------------------------------------------------------------------ */
/* Atomics shim (MSVC's C17 mode lacks <stdatomic.h>)                  */
/* ------------------------------------------------------------------ */

#if defined(_MSC_VER) && !defined(__clang__)

#include <intrin.h>

typedef volatile long derech_busy_flag;
typedef volatile long derech_atomic_u32;
typedef volatile __int64 derech_atomic_u64;

static inline int derech_busy_acquire(derech_busy_flag *f)
{
	return _InterlockedCompareExchange(f, 1, 0) == 0;
}

static inline void derech_busy_release(derech_busy_flag *f)
{
	_InterlockedExchange(f, 0);
}

/* Returns the pre-increment value, like C11 atomic_fetch_add. */
static inline uint32_t derech_atomic_fetch_add_u32(derech_atomic_u32 *a,
	uint32_t v)
{
	return (uint32_t)_InterlockedExchangeAdd(a, (long)v);
}

static inline uint32_t derech_atomic_load_u32(derech_atomic_u32 *a)
{
	return (uint32_t)_InterlockedExchangeAdd(a, 0);
}

static inline void derech_atomic_store_u32(derech_atomic_u32 *a, uint32_t v)
{
	_InterlockedExchange(a, (long)v);
}

static inline uint64_t derech_atomic_load_u64(const derech_atomic_u64 *a)
{
	return (uint64_t)_InterlockedCompareExchange64(
		(volatile __int64 *)a, 0, 0);
}

static inline void derech_atomic_store_u64(derech_atomic_u64 *a, uint64_t v)
{
	_InterlockedExchange64(a, (__int64)v);
}

static inline uint64_t derech_atomic_fetch_add_u64(derech_atomic_u64 *a,
	uint64_t v)
{
	return (uint64_t)_InterlockedExchangeAdd64(a, (__int64)v);
}

#else

#include <stdatomic.h>

typedef atomic_int derech_busy_flag;
typedef atomic_uint derech_atomic_u32;
typedef atomic_uint_fast64_t derech_atomic_u64;

static inline int derech_busy_acquire(derech_busy_flag *f)
{
	int expected = 0;
	return atomic_compare_exchange_strong(f, &expected, 1);
}

static inline void derech_busy_release(derech_busy_flag *f)
{
	atomic_store(f, 0);
}

static inline uint32_t derech_atomic_fetch_add_u32(derech_atomic_u32 *a,
	uint32_t v)
{
	return atomic_fetch_add(a, v);
}

static inline uint32_t derech_atomic_load_u32(derech_atomic_u32 *a)
{
	return atomic_load(a);
}

static inline void derech_atomic_store_u32(derech_atomic_u32 *a, uint32_t v)
{
	atomic_store(a, v);
}

static inline uint64_t derech_atomic_load_u64(const derech_atomic_u64 *a)
{
	return atomic_load(a);
}

static inline void derech_atomic_store_u64(derech_atomic_u64 *a, uint64_t v)
{
	atomic_store(a, v);
}

static inline uint64_t derech_atomic_fetch_add_u64(derech_atomic_u64 *a,
	uint64_t v)
{
	return atomic_fetch_add(a, v);
}

#endif

/* ------------------------------------------------------------------ */
/* Cost constants with eight fractional bits                           */
/* ------------------------------------------------------------------ */

#define DERECH_Q_ONE 256u                  /* one tick                   */
#define DERECH_Q_TILE_MAX (1u << 24)       /* 65536 ticks per tile cap   */
#define DERECH_Q_BLOCKED 0u                /* cost_q sentinel            */
#define DERECH_Q_MULT_BLOCKED UINT32_MAX   /* profile entry sentinel     */
#define DERECH_Q_DIAG_SQRT2 362u           /* round(sqrt(2) * 256)       */
#define DERECH_Q_EPS_DEFAULT 320u          /* round(1.25 * 256)          */
#define DERECH_G_INFINITE UINT64_MAX

/* Round a non-negative double to uint32 with saturation at `cap`.
 * All float->fixed conversions go through this so quantization is
 * identical everywhere (and cross-platform deterministic). */
static inline uint32_t derech_q_round(double v, uint32_t cap)
{
	if (v >= (double)cap) {
		return cap;
	}
	if (v <= 0.0) {
		return 0;
	}
	return (uint32_t)(v + 0.5);
}

static inline uint64_t derech_q_round_u64(double v, uint64_t cap)
{
	if (v >= (double)cap) {
		return cap;
	}
	if (v <= 0.0) {
		return 0;
	}
	return (uint64_t)(v + 0.5);
}

static inline uint32_t derech_sat_u32(uint64_t v)
{
	return v > UINT32_MAX ? UINT32_MAX : (uint32_t)v;
}

/* ------------------------------------------------------------------ */
/* Internal structures                                                 */
/* ------------------------------------------------------------------ */

/* Per-(profile, tag-combination) folded weights. */
typedef struct derech_pentry {
	uint32_t mult_q; /* Q8 multiplier, DERECH_Q_MULT_BLOCKED = impassable */
	uint32_t add_q;  /* Q8 flat penalty                                   */
} derech_pentry;

typedef struct derech_profile {
	double mult[64]; /* validated, 0-slots already resolved to 1.0 */
	double add[64];
	uint64_t block_mask;
	uint64_t require_mask;
	uint64_t relevant_mask; /* tag bits that affect this profile at all:
	                           weighted, blocking, or required — edits to
	                           other bits never invalidate its fields   */
	uint8_t connectivity;
	uint8_t corner_rule;
	uint32_t diag_q;   /* Q8 diagonal multiplier                     */
	uint32_t h_d;      /* heuristic straight-step floor (Q8, >= 1)   */
	uint32_t h_d2;     /* heuristic diagonal floor, clamped <= 2*h_d */
	derech_pentry *table; /* one entry per interned combo, capacity
	                         tracks map->combo_cap                   */
} derech_profile;

typedef struct derech_heap_entry {
	uint64_t f;
	uint64_t g;
	uint32_t idx;
} derech_heap_entry;

typedef struct derech_combo_slot {
	uint64_t key;
	uint32_t val_plus1; /* 0 = empty */
} derech_combo_slot;

/* One search context per worker.  A context is only ever used by one
 * thread at a time; the out_* buffers hold reconstructed paths until the
 * (single-threaded) assembly phase copies them into the results arena. */
typedef struct derech_search_ctx {
	uint64_t *g;
	uint32_t *stamp;
	uint8_t *parent; /* 0 = none, else direction index + 1 */
	uint32_t stamp_gen;
	derech_heap_entry *heap;
	uint64_t heap_cap;
	int initialized;

	uint32_t *out_steps; /* interleaved x,y — 2 per step */
	float *out_ticks;    /* 1 per step */
	uint64_t out_len;
	uint64_t out_cap;
} derech_search_ctx;

/* Per-request staging filled by workers, consumed by assembly. */
typedef struct derech_stage_row {
	derech_path_status status;
	uint32_t expansions;
	uint32_t len;
	uint32_t worker;    /* which ctx's out buffers hold the steps */
	uint64_t local_off; /* step offset within that ctx            */
	uint64_t true_q;    /* summed base ticks, Q8                  */
	uint64_t perceived_q;
	derech_status error; /* required allocation/cancellation error */
} derech_stage_row;

typedef struct derech_pool derech_pool;

/* A goal set: explicit tile list or tag predicate, plus a lazily
 * maintained member bitmap (and, for ADJACENT sets, the dilated seed
 * bitmap actually used as goals).  epoch bumps whenever effective
 * membership changes, invalidating fields built over the set. */
typedef struct derech_goalset {
	int used;
	int is_predicate;
	uint32_t flags;          /* DERECH_GOALSET_*                        */
	uint64_t any_mask;       /* predicate sets                          */
	uint64_t all_mask;
	uint32_t *tiles;         /* explicit sets: interleaved x,y          */
	uint32_t n_tiles;
	uint64_t *members;       /* bitmap of n bits: matching tiles        */
	uint64_t *seeds;         /* goal bitmap: members, dilated if ADJ    */
	uint32_t member_count;
	uint64_t epoch;
} derech_goalset;

/* A goal field: exact perceived distance from every tile to one
 * (goal, profile) — or to the nearest seed of a (goalset, profile) —
 * plus the first move to take.  Built by reverse Dijkstra (edge v->u
 * costs perceived(u) — the enter-cost asymmetry), immutable once `ok`,
 * cached until invalidated by a relevant terrain edit. */
typedef struct derech_field {
	uint32_t goal_idx;       /* single-goal fields                     */
	uint32_t goalset_id;     /* DERECH_NO_GOALSET for single-goal      */
	uint64_t set_epoch;      /* goalset->epoch this field was built at */
	uint32_t profile_id;
	uint64_t *dist_q;   /* n; DERECH_G_INFINITE = cannot reach goal */
	uint8_t *next_dir;  /* n; direction index + 1 toward the goal   */
	uint64_t bytes;     /* cache accounting                         */
	int ok;             /* build completed successfully             */
	int pinned;         /* in use by the current batch              */
	derech_status error;
	int new_in_call;
	uint64_t touch_seq;
	struct derech_field *lru_prev;
	struct derech_field *lru_next;
	struct derech_field *hash_next;
} derech_field;

#define DERECH_FIELD_HASH_BUCKETS 256u

/* Accumulated terrain-edit tracking for targeted cache invalidation. */
typedef struct derech_dirty_rect {
	uint32_t x, y, w, h;
	uint64_t tag_bits;   /* OR of (old ^ new) tag words in the rect */
	uint8_t pass_changed;
} derech_dirty_rect;

#define DERECH_MAX_DIRTY_RECTS 16u

typedef struct derech_dirty {
	derech_dirty_rect rects[DERECH_MAX_DIRTY_RECTS];
	uint32_t count;
	int full; /* overflow or full-grid write: everything is dirty */
} derech_dirty;

/* Connected-component labels for one blocking class (block_mask,
 * require_mask), computed on the superset graph (8-connected, corners
 * allowed): differing labels soundly prove unreachability for any
 * profile of this class; equal labels prove nothing. */
typedef struct derech_labels {
	uint64_t block_mask;
	uint64_t require_mask;
	uint32_t *label; /* n; 0 = not enterable */
	uint64_t last_use;
	uint64_t touch_seq;
	int new_in_call;
} derech_labels;

#define DERECH_MAX_LABEL_CLASSES 16u

/* Batch work item. */
enum {
	DERECH_TASK_SOLVE = 0,       /* classic A* for one request      */
	DERECH_TASK_FIELD_BUILD = 1, /* reverse Dijkstra into a field   */
	DERECH_TASK_EXTRACT = 2      /* answer one request from a field */
};

typedef struct derech_task {
	uint8_t kind;
	uint32_t req_idx;     /* SOLVE, EXTRACT */
	derech_field *field;  /* FIELD_BUILD, EXTRACT */
} derech_task;

struct derech_map {
	uint32_t w, h;
	uint32_t n; /* w * h */
	derech_atomic_u64 generation;
	derech_atomic_u32 published_profile_count;
	derech_busy_flag busy;

	/* terrain */
	uint32_t *cost_q;    /* n; DERECH_Q_BLOCKED = impassable */
	uint16_t *combo_idx; /* n; index into combo_words        */

	/* tag-word interning */
	uint64_t *combo_words;
	uint32_t combo_count;
	uint32_t combo_cap;
	derech_combo_slot *combo_table;
	uint32_t combo_table_cap; /* power of two */

	/* profiles */
	derech_profile *profiles;
	uint32_t profile_count;

	/* batch execution: ctxs[0] belongs to the calling thread, the rest
	 * to pool workers.  pool is NULL when n_threads == 1. */
	uint32_t n_threads;
	uint32_t allocated_contexts;
	derech_search_ctx *ctxs;
	derech_pool *pool;
	uint64_t worker_budget_bytes;
	uint64_t scratch_retention_bytes;
	const struct derech_cancel *active_cancel;

	/* goal-field cache: hash by (goal-or-set, profile) + LRU by last
	 * use, mutated only during single-threaded planning; invalidated
	 * per-field from the dirty-region log at batch start */
	derech_field *field_hash[DERECH_FIELD_HASH_BUCKETS];
	derech_field *field_lru_head;
	derech_field *field_lru_tail;
	uint64_t field_bytes;
	uint64_t field_peak_bytes;
	uint64_t field_budget_bytes;
	uint64_t field_working_bytes;
	uint64_t field_touch_clock;
	uint32_t field_threshold;

	/* component labels per blocking class; flushed when an edit could
	 * change enterability for the class */
	derech_labels label_classes[DERECH_MAX_LABEL_CLASSES];
	uint32_t label_class_count;
	uint64_t label_bytes;
	uint64_t label_budget_bytes;
	uint64_t label_working_bytes;
	uint64_t label_clock;
	uint64_t label_touch_clock;

	/* goal sets + pending edits (reconciled at batch start) */
	derech_goalset goalsets[DERECH_MAX_GOALSETS];
	derech_dirty dirty;
};

/* Direction tables: 4 straight then 4 diagonal; parent stores index+1. */
extern const int8_t derech_dir_dx[8];
extern const int8_t derech_dir_dy[8];

/* map.c */
int derech_intern_tag_word(derech_map *map, uint64_t word, uint32_t *out_idx);
derech_status derech_contexts_ensure(derech_map *map, uint32_t count);
void derech_contexts_trim(derech_map *map);

/* profile.c */
void derech_profile_fold_combo(const derech_profile *p, uint64_t word,
	derech_pentry *out);

/* search.c */
typedef struct derech_search_result {
	derech_path_status status;
	uint32_t end_idx;    /* goal, or best partial endpoint */
	uint32_t expansions;
	derech_status error;
} derech_search_result;

void derech_search(const derech_map *map, derech_search_ctx *ctx,
	const derech_profile *prof, uint32_t start_idx, uint32_t goal_idx,
	uint32_t eps_q, uint32_t max_expansions, uint64_t max_cost_q,
	derech_search_result *out);

/* batch.c — solve one request into ctx + stage; thread-safe across
 * distinct ctxs (reads only immutable map state). */
void derech_solve_request(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_request *req, derech_stage_row *row);

/* batch.c — execute one task; thread-safe across distinct ctxs. */
void derech_run_task(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_task *task, const derech_request *reqs,
	derech_stage_row *stage);

/* batch.c — ensure room for `extra` more steps in ctx's out buffers. */
int derech_ctx_out_reserve(derech_search_ctx *ctx, uint64_t extra);

/* map.c — record an edit for the reconcile pass */
void derech_dirty_add(derech_map *map, uint32_t x, uint32_t y, uint32_t w,
	uint32_t h, int pass_changed, uint64_t tag_bits);

/* field.c */
void derech_field_build(const derech_map *map, derech_search_ctx *ctx,
	derech_field *field);
void derech_field_extract(const derech_map *map, derech_search_ctx *ctx,
	uint32_t worker, const derech_field *field, const derech_request *req,
	derech_stage_row *row);
derech_field *derech_field_cache_lookup(derech_map *map, uint32_t goal_idx,
	uint32_t goalset_id, uint32_t profile_id);
derech_field *derech_field_cache_insert(derech_map *map, uint32_t goal_idx,
	uint32_t goalset_id, uint32_t profile_id, derech_status *error);
void derech_field_cache_end_wave(derech_map *map);
void derech_field_cache_finish_batch(derech_map *map, int commit);
void derech_field_cache_flush(derech_map *map);
/* apply the dirty log: refresh predicate memberships, drop affected
 * fields and label classes, clear the log.  Planning phase only. */
derech_status derech_reconcile(derech_map *map);
derech_status derech_goalset_materialize(derech_map *map,
	derech_goalset *gs);
void derech_goalset_free(derech_goalset *gs);
const derech_labels *derech_labels_for(derech_map *map,
	const derech_profile *prof);
void derech_labels_flush(derech_map *map);
void derech_labels_finish_call(derech_map *map, int commit);

/* pool.c */
uint32_t derech_hw_threads(void);
derech_pool *derech_pool_create(uint32_t n_workers);
void derech_pool_destroy(derech_pool *pool);
/* Execute a task list: caller participates with ctxs[0]; blocks until
 * every task is done. */
void derech_pool_run(derech_pool *pool, derech_map *map,
	const derech_task *tasks, uint32_t n_tasks, const derech_request *reqs,
	derech_stage_row *stage, uint32_t n_participants);

static inline int derech_tile_blocked(const derech_map *map,
	const derech_profile *prof, uint32_t idx)
{
	return map->cost_q[idx] == DERECH_Q_BLOCKED ||
		prof->table[map->combo_idx[idx]].mult_q == DERECH_Q_MULT_BLOCKED;
}

/* True (base) Q8 cost of stepping onto `idx`, including the diagonal
 * multiplier when `diagonal` is nonzero.  Tile must not be base-blocked. */
static inline uint32_t derech_true_step_q(const derech_map *map,
	const derech_profile *prof, uint32_t idx, int diagonal)
{
	uint64_t q = map->cost_q[idx];
	if (diagonal) {
		q = (q * prof->diag_q) >> 8;
	}
	return derech_sat_u32(q);
}

/* Perceived Q8 cost of the same step.  Tile must not be blocked for the
 * profile. */
static inline uint32_t derech_perceived_step_q(const derech_map *map,
	const derech_profile *prof, uint32_t idx, int diagonal)
{
	const derech_pentry *e = &prof->table[map->combo_idx[idx]];
	uint64_t q = derech_true_step_q(map, prof, idx, diagonal);

	q = ((q * e->mult_q) >> 8) + e->add_q;
	if (q < 1) {
		q = 1;
	}
	return derech_sat_u32(q);
}

struct derech_cancel {
	derech_atomic_u32 requested;
};

static inline int derech_cancelled(const derech_map *map)
{
	return map->active_cancel != NULL &&
		derech_atomic_load_u32(
			(derech_atomic_u32 *)&map->active_cancel->requested) != 0;
}

#endif /* DERECH_INTERNAL_H */
