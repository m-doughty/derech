/*
 * derech — batch pathfinding over weighted tile grids.
 *
 * A zero-dependency C17 library for 2D tile worlds in which every tile has a
 * passability in [0, 1] (1 = crossed in one tick, 0.5 = two ticks, 0 =
 * impassable) and NPC archetypes ("profiles") perceive semantic tile tags
 * (enemy territory, wilderness, roads, water, ...) as cost multipliers,
 * flat penalties, or hard blocks.  Requests are submitted in batches.
 *
 * Design contract for FFI users (Python cffi, Raku NativeCall, LuaJIT, ...):
 *   - Opaque handles + fixed-layout POD structs.  No callbacks, no varargs,
 *     no thread-local error state, no globals.
 *   - All input buffers are fully copied during the call; derech never
 *     retains a caller pointer.
 *   - All result data is owned by the library and lives until the
 *     corresponding derech_results_destroy() call.
 *   - Every fallible function returns a derech_status (negative = error).
 *   - Structs carry a struct_size field for ABI evolution: set it to
 *     sizeof(the struct) as compiled against this header.
 *
 * Cost model
 * ----------
 * Costs use integer fixed point with eight fractional bits internally
 * (1/256 tick resolution); all search arithmetic is integer, so results are
 * bitwise-deterministic across platforms for identical inputs and library
 * versions.  Do not build with -ffast-math or equivalents.
 *
 *   tile entry cost  q(p)      = clamp(round(256 / p), 256 .. 1<<24)
 *                                (p = 0 stores a "blocked" sentinel;
 *                                 costs saturate at 65536 ticks per tile)
 *   diagonal step               = (q * diag_q) >> 8, where diag_q =
 *                                 round(diagonal_mult * 256); default
 *                                 sqrt(2) => 362
 *   perceived entry cost        = max(1, ((q_step * mult_q) >> 8) + add_q)
 *     where mult_q / add_q derive from the profile's per-tag weights folded
 *     over the tile's tag bits:  mult = product of tag_mult[b] over set
 *     bits, add = sum of tag_add[b] over set bits (flat penalties are NOT
 *     scaled by the diagonal multiplier — you enter the tile once).
 *   Folded mult_q and add_q are rounded to 1/256 and capped at 2^28.
 *   The final perceived entry cost is capped at
 *   DERECH_MAX_STEP_COST_Q8 before it is added to a cumulative cost.
 *
 * The search minimizes PERCEIVED cost (route choice), but reported step
 * durations are TRUE ticks (base cost including the diagonal multiplier,
 * excluding preference weights): danger changes where an NPC walks, not how
 * fast it walks.  Both totals are reported per result.
 *
 * Movement rules (per profile)
 * ----------------------------
 *   - 8-connected by default; 4-connected available.
 *   - Diagonal multiplier defaults to sqrt(2), must be >= 1.
 *   - Corner rule for a diagonal step (both flanking orthogonal tiles):
 *       STRICT  (default): forbidden if either flank is impassable
 *       LENIENT          : forbidden only if both flanks are impassable
 *       ALLOW            : never forbidden
 *     "Impassable" means blocked for this profile (p = 0, block_mask, or
 *     unmet require_mask) — merely *slow* flanks never forbid a diagonal.
 *   - Entry costs mean a blocked START tile is escapable by design: an NPC
 *     standing somewhere it could never enter can still path out.
 *
 * Search quality
 * --------------
 * Weighted A*: each request carries an inflation factor epsilon >= 1.
 * Returned paths cost at most epsilon times the optimal perceived cost.
 * epsilon = 0 selects the default (1.25); epsilon = 1 is exact.
 *
 * Threading contract
 * ------------------
 * Distinct maps are fully independent.  On one map, at most one stateful
 * call may be active at a time: a second mutation, path query, terrain
 * read-back, goal-set count, or memory-stat call fails fast with
 * DERECH_E_BUSY rather than racing.  Immutable dimensions/thread count and
 * the atomic generation/profile-count snapshots may be read concurrently.
 * Results objects are independent of the map and may be read from any thread.
 *
 * Batches are solved in parallel on an internal worker pool owned by the
 * map (derech_map_opts.n_threads; default: one worker per CPU core,
 * capped at 16).  Parallelism is an implementation detail with one hard
 * guarantee: the results object is bitwise-identical for any thread
 * count, because each request's search is fully self-contained and the
 * results arena is assembled in request order.  Each worker keeps a
 * search context with roughly 13 bytes of fixed state per map tile plus
 * reusable heap/output buffers.  Contexts are allocated only as a batch
 * needs them; automatic thread selection honors worker_memory_mb.
 *
 * Batch acceleration
 * ------------------
 * Requests sharing (goal, profile) are detected per batch: groups of at
 * least derech_map_opts.field_group_threshold (default 4) are answered
 * from one shared goal field (an exact reverse Dijkstra) instead of
 * per-request searches — the "many NPCs converge on one target" case
 * costs one search plus O(path length) per NPC.  Fields are cached
 * (field_cache_mb, default 64) and reused by LATER batches too, even
 * for single requests, until any terrain edit invalidates them.
 * Connected-component labels provide O(1) UNREACHABLE answers for
 * non-partial requests.  Consequences visible to callers:
 *   - field-answered requests report expansions == 0 and are EXACT
 *     (as if epsilon = 1), which always satisfies the requested bound;
 *   - impossible requests may report UNREACHABLE where a per-request
 *     search under a max_perceived_cost cap would have reported
 *     BUDGET_EXCEEDED — the field/labels know more, so you get the
 *     stronger answer;
 *   - results therefore depend on the batch history since the last
 *     terrain edit (via the cache), but never on thread count: replay
 *     the same batch sequence and you get identical bytes.
 * Requests needing closest-approach partials fall back to per-request
 * searches automatically when the field cannot answer them.
 *
 * Goal sets
 * ---------
 * A request may target a SET of tiles instead of one: the result is the
 * cheapest path to whichever member is nearest by perceived cost (the
 * reached member is simply the path's last step).  Sets are registered
 * once and referenced by id in derech_request.goalset (0 = ordinary
 * single-goal request):
 *   - explicit sets:  a fixed tile list (derech_goalset_register);
 *   - predicate sets: every tile matching tag masks
 *     (derech_goalset_register_tags) — membership tracks tag edits
 *     automatically, so "tiles tagged TREE and EXPLORED" stays correct
 *     as the host reveals terrain or fells trees.
 * DERECH_GOALSET_ADJACENT retargets a set to the tiles NEXT TO its
 * members — for goals that are themselves impassable (tree trunks, ore
 * veins): the path ends beside a member.  A start on (or beside, when
 * adjacent) a member returns FOUND with zero steps.
 * Set queries are answered from multi-source goal fields (exact, cached
 * like single-goal fields, expansions == 0) regardless of group size —
 * sets exist to be reused.  ALLOW_PARTIAL is invalid with a goal set:
 * with every member unreachable there is no gradient to descend, so
 * closest-approach is undefined.
 *
 * Cache invalidation is dirty-region based: terrain setters record what
 * they touched and which tag bits actually changed, and a cached field
 * is only dropped when the edit could affect it — it must intersect (or
 * abut) the field's reachable area AND change passability or a tag bit
 * its profile actually weighs (or, for set fields, change the set's
 * membership).  Bits no profile cares about — e.g. a host-maintained
 * EXPLORED bit — can be rewritten every tick without disturbing cached
 * fields.
 */

#ifndef DERECH_H
#define DERECH_H

#include <stddef.h>
#include <stdint.h>

#include <derech_version.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Export & version                                                    */
/* ------------------------------------------------------------------ */

#if defined(DERECH_STATIC)
#define DERECH_API
#elif defined(_WIN32)
#if defined(DERECH_BUILD)
#define DERECH_API __declspec(dllexport)
#else
#define DERECH_API __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define DERECH_API __attribute__((visibility("default")))
#else
#define DERECH_API
#endif

/* Returns (major << 16) | (minor << 8) | patch. */
DERECH_API uint32_t derech_version(void);
DERECH_API const char *derech_version_str(void);

/* Binary interface epoch.  Bindings should reject an unsupported epoch
 * before making any call that exchanges a public struct. */
DERECH_API uint32_t derech_abi_version(void);

/* ------------------------------------------------------------------ */
/* Limits & constants                                                  */
/* ------------------------------------------------------------------ */

#define DERECH_MAX_DIM 2048u         /* max map width/height            */
#define DERECH_MAX_TAG_COMBOS 65536u /* distinct tag words per map      */
#define DERECH_MAX_PROFILES 256u     /* registered profiles per map     */
#define DERECH_MAX_THREADS 64u       /* max explicit n_threads          */
#define DERECH_MAX_GOALSETS 64u      /* registered goal sets per map    */
#define DERECH_NO_GOALSET 0u         /* request.goalset: single goal    */
#define DERECH_DEFAULT_EPSILON 1.25f /* used when request.epsilon == 0  */
#define DERECH_MAX_STEP_COST_Q8 UINT32_MAX /* 2^24 ticks minus 1/256    */
#define DERECH_MAX_PATH_TICKS 70368744177664.0f /* 2^46 ticks             */

/* ------------------------------------------------------------------ */
/* Status codes                                                        */
/* ------------------------------------------------------------------ */

/* API-level status.  Negative values are errors.  Stored as int32_t in
 * signatures (enum object size is implementation-defined in C). */
typedef int32_t derech_status;

enum {
	DERECH_OK = 0,
	DERECH_E_INVALID_ARG = -1,         /* bad pointer, range, NaN, flag... */
	DERECH_E_OOB = -2,                 /* coordinate out of bounds         */
	DERECH_E_NOMEM = -3,               /* allocation failure               */
	DERECH_E_BUSY = -4,                /* concurrent call on this map      */
	DERECH_E_BAD_PROFILE = -5,         /* unknown profile id in a request  */
	DERECH_E_TAG_COMBOS_EXHAUSTED = -6,/* > DERECH_MAX_TAG_COMBOS distinct
	                                      tag words on one map            */
	DERECH_E_TOO_MANY_PROFILES = -7,   /* > DERECH_MAX_PROFILES            */
	DERECH_E_BAD_GOALSET = -8,         /* unknown goal-set id in a request */
	DERECH_E_TOO_MANY_GOALSETS = -9,   /* > DERECH_MAX_GOALSETS            */
	DERECH_E_RESOURCE_LIMIT = -10,     /* configured memory limit          */
	DERECH_E_CANCELLED = -11           /* cooperative cancellation         */
};

DERECH_API const char *derech_status_str(derech_status s);

/* Per-request outcome.  A search that did not reach the goal can still
 * carry a best-effort partial path if DERECH_REQ_ALLOW_PARTIAL was set:
 * the steps then lead to the reachable tile closest to the goal (by
 * heuristic distance, ties broken by lower cost).  A partial of length 0
 * means the start itself was the closest reachable tile. */
typedef int32_t derech_path_status;

enum {
	DERECH_PATH_NONE = -1,            /* accessor index out of range      */
	DERECH_PATH_FOUND = 0,
	DERECH_PATH_UNREACHABLE = 1,
	DERECH_PATH_BUDGET_EXCEEDED = 2,  /* max_expansions/max_perceived_cost */
	DERECH_PATH_INVALID_ENDPOINT = 3  /* start or goal out of bounds      */
};

DERECH_API const char *derech_path_status_str(derech_path_status s);

/* ------------------------------------------------------------------ */
/* Map                                                                 */
/* ------------------------------------------------------------------ */

typedef struct derech_map derech_map;

typedef struct derech_map_opts {
	uint32_t struct_size;       /* = sizeof(derech_map_opts)             */
	float default_passability;  /* initial fill; must be in [0,1]        */
	uint64_t default_tags;      /* initial tag word for every tile       */
	uint32_t n_threads;         /* total batch parallelism including the
	                               calling thread: 1 = fully serial (no
	                               threads spawned), 0 = auto (one per
	                               CPU core, capped at 16), explicit
	                               values up to DERECH_MAX_THREADS       */
	uint32_t field_cache_mb;    /* retained goal-field cache in MiB; 0 =
	                               default (64), max 4096                */
	uint32_t field_group_threshold; /* same-(goal, profile) requests per
	                               batch needed to build a shared field;
	                               0 = default (4), 1 = always          */
	uint32_t worker_memory_mb; /* hard fixed worker-state budget; 0 = 256 MiB
	                            for auto threads, unlimited for an
	                            explicit n_threads value                */
	uint32_t field_working_mb; /* hard resident-field working limit; 0 =
	                            retained cache plus one full field.
	                            Tighter values reduce effective cache
	                            retention to reserve replacement space */
	uint32_t scratch_retention_mb; /* retained heap/path-output capacity
	                                per worker; 0 = default (16 MiB)    */
	uint32_t label_cache_mb; /* connected-component label cache; 0 =
	                          default (64 MiB), always at least one map */
	uint32_t reserved0;      /* must be zero; occupies ABI tail padding */
} derech_map_opts;

/* Create a map of width x height tiles (each 1..DERECH_MAX_DIM).
 * opts may be NULL: passability 1.0, tags 0, auto threads, default
 * field cache.  Older derech_map_opts layouts are accepted via struct_size:
 * 16-byte v0.1, 20/24-byte v0.2, and 28/32-byte v0.3/v0.4 (the size varies
 * with the legacy platform ABI).  Returns NULL on invalid arguments or
 * allocation failure. */
DERECH_API derech_map *derech_map_create(uint32_t width, uint32_t height,
	const derech_map_opts *opts);

/* Status-returning constructor.  On failure *out is NULL. */
DERECH_API derech_status derech_map_create_ex(uint32_t width, uint32_t height,
	const derech_map_opts *opts, derech_map **out);

/* Destroy a map.  NULL is a no-op.  The caller must ensure no other call
 * on this map is in flight. */
DERECH_API void derech_map_destroy(derech_map *map);

DERECH_API uint32_t derech_map_width(const derech_map *map);
DERECH_API uint32_t derech_map_height(const derech_map *map);

/* Effective batch parallelism (resolved from n_threads / auto). */
DERECH_API uint32_t derech_map_thread_count(const derech_map *map);

typedef struct derech_memory_stats {
	uint32_t struct_size;          /* = sizeof(derech_memory_stats)       */
	uint32_t configured_threads;
	uint32_t allocated_contexts;
	uint32_t reserved0;
	uint64_t terrain_bytes;
	uint64_t worker_bytes;
	uint64_t field_bytes;
	uint64_t field_peak_bytes;
	uint64_t field_cache_bytes;   /* configured retention ceiling          */
	uint64_t field_working_bytes;
	uint64_t label_bytes;
	uint64_t label_cache_bytes;
	uint64_t retained_scratch_bytes;
} derech_memory_stats;

/* Estimate fixed terrain/worker/field limits without creating a map. */
DERECH_API derech_status derech_map_memory_estimate(uint32_t width,
	uint32_t height, const derech_map_opts *opts, derech_memory_stats *out);

/* Snapshot current allocations.  Returns DERECH_E_BUSY during another
 * stateful call on the map. */
DERECH_API derech_status derech_map_get_memory_stats(const derech_map *map,
	derech_memory_stats *out);

/* ------------------------------------------------------------------ */
/* Goal sets                                                           */
/* ------------------------------------------------------------------ */

enum {
	/* target the walkable tiles NEXT TO members instead of the members
	 * themselves (for goals that are impassable: trees, ore, walls) */
	DERECH_GOALSET_ADJACENT = 1u << 0
};

/* Register an explicit goal set from interleaved x,y pairs (n_tiles of
 * them, all in bounds, at least one).  The list is copied and immutable;
 * re-register to change it.  Returns an id (>= 1) for
 * derech_request.goalset, or a negative derech_status. */
DERECH_API int32_t derech_goalset_register(derech_map *map,
	const uint32_t *xy_pairs, uint32_t n_tiles, uint32_t flags);

/* Register a predicate goal set: every tile whose tag word contains at
 * least one bit of any_mask (if nonzero) and all bits of all_mask.  At
 * least one mask must be nonzero.  Membership follows tag edits
 * automatically. */
DERECH_API int32_t derech_goalset_register_tags(derech_map *map,
	uint64_t any_mask, uint64_t all_mask, uint32_t flags);

/* Drop a goal set (cached fields over it are discarded; its id may be
 * reused by later registrations). */
DERECH_API derech_status derech_goalset_unregister(derech_map *map,
	uint32_t id);

/* Current number of member tiles (before ADJACENT expansion), or a
 * negative derech_status. */
DERECH_API int64_t derech_goalset_count(derech_map *map, uint32_t id);

/* Monotonic edit counter; bumped once per successful mutating call.
 * Useful for host-side caching keyed on map state. */
DERECH_API uint64_t derech_map_generation(const derech_map *map);

/* Terrain setters are validate-then-commit: on error, tile state and the
 * generation counter are unchanged.  An allocation grown while validating a
 * tag update may be retained for reuse, but uncommitted tag words disappear.
 * Passability values must satisfy 0 <= p <= 1 (NaN/inf rejected).
 * Full-grid variants require count == width * height, row-major
 * (index = y * width + x).  Rect variants read a row-major w*h buffer. */
DERECH_API derech_status derech_map_set_passability(derech_map *map,
	const float *p, uint64_t count);
DERECH_API derech_status derech_map_set_passability_rect(derech_map *map,
	uint32_t x, uint32_t y, uint32_t w, uint32_t h, const float *p);
DERECH_API derech_status derech_map_set_passability_at(derech_map *map,
	uint32_t x, uint32_t y, float p);

DERECH_API derech_status derech_map_set_tags(derech_map *map,
	const uint64_t *tags, uint64_t count);
DERECH_API derech_status derech_map_set_tags_rect(derech_map *map,
	uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint64_t *tags);
DERECH_API derech_status derech_map_set_tags_at(derech_map *map,
	uint32_t x, uint32_t y, uint64_t tags);

/* Read-back (debugging / host verification).  Passability returns the
 * quantized effective value 256.0 / q — the value the search actually
 * uses — which may differ from the exact float that was set.  Blocked
 * tiles return 0. */
DERECH_API derech_status derech_map_get_passability_at(const derech_map *map,
	uint32_t x, uint32_t y, float *out);
DERECH_API derech_status derech_map_get_tags_at(const derech_map *map,
	uint32_t x, uint32_t y, uint64_t *out);

/* ------------------------------------------------------------------ */
/* Profiles                                                            */
/* ------------------------------------------------------------------ */

enum {
	DERECH_CONN_8 = 0, /* default */
	DERECH_CONN_4 = 1
};

enum {
	DERECH_CORNER_STRICT = 0, /* default: no cutting past any blocked flank */
	DERECH_CORNER_LENIENT = 1,/* forbidden only if both flanks blocked      */
	DERECH_CORNER_ALLOW = 2   /* never forbidden                            */
};

/* An NPC archetype.  Zero-initializing this struct (then setting
 * struct_size) yields a fully neutral profile: 8-connected, sqrt(2)
 * diagonals, strict corners, no blocks, all tags ignored.
 *
 * Field semantics:
 *   tag_mult[b]    perceived cost multiplier while tag bit b is set.
 *                  0 means "neutral" (treated as 1.0) so that zeroed
 *                  structs are safe; otherwise must be in (0, 65536].
 *                  Values < 1 express preference (roads), > 1 aversion.
 *   tag_add[b]     flat perceived penalty in ticks per entered tile with
 *                  bit b set; in [0, 65536].  Negative rewards are not
 *                  allowed (they would break the search); express
 *                  preference with tag_mult < 1 instead.
 *   block_mask     tiles with any of these bits are impassable for this
 *                  profile.
 *   require_mask   if nonzero, tiles LACKING all of these bits are
 *                  impassable (e.g. rail-bound or road-bound movers).
 *   diagonal_mult  0 selects sqrt(2); otherwise must be in [1, 16].
 */
typedef struct derech_profile_desc {
	uint32_t struct_size; /* = sizeof(derech_profile_desc) */
	uint8_t connectivity; /* DERECH_CONN_*                 */
	uint8_t corner_rule;  /* DERECH_CORNER_*               */
	uint8_t reserved0;    /* must be 0                     */
	uint8_t reserved1;    /* must be 0                     */
	float diagonal_mult;
	uint32_t reserved2;   /* must be 0; occupies ABI padding */
	uint64_t block_mask;
	uint64_t require_mask;
	float tag_mult[64];
	float tag_add[64];
	uint32_t reserved3;   /* must be 0                     */
	uint32_t reserved4;   /* must be 0                     */
} derech_profile_desc;

/* Register a profile; returns its id (>= 0) or a negative derech_status.
 * Profiles are immutable once registered and live as long as the map.
 * Historical 540-byte (32-bit) and 544-byte (64-bit) descriptors are
 * normalized to the current layout. */
DERECH_API int32_t derech_profile_register(derech_map *map,
	const derech_profile_desc *desc);

DERECH_API uint32_t derech_profile_count(const derech_map *map);

/* ------------------------------------------------------------------ */
/* Requests & results                                                  */
/* ------------------------------------------------------------------ */

enum {
	DERECH_REQ_ALLOW_PARTIAL = 1u << 0
};

typedef struct derech_request {
	uint32_t struct_size;         /* = sizeof(derech_request), fixed at 64 */
	uint32_t start_x, start_y;
	uint32_t goal_x, goal_y;   /* ignored when goalset != DERECH_NO_GOALSET */
	uint32_t profile_id;
	uint32_t flags;            /* DERECH_REQ_*                             */
	uint32_t max_expansions;   /* settled-node budget; 0 = whole map       */
	float max_perceived_cost;  /* ticks, <= DERECH_MAX_PATH_TICKS;
	                            0 = unlimited                              */
	float epsilon;             /* in [1, 256], or 0 for DEFAULT_EPSILON    */
	uint32_t goalset;          /* goal-set id, or DERECH_NO_GOALSET (0)
	                              for an ordinary single-goal request.
	                              ALLOW_PARTIAL is invalid with a set.    */
	uint32_t reserved[5];      /* must be zero; reserved within ABI 1     */
} derech_request;

typedef struct derech_results derech_results;
typedef struct derech_cancel derech_cancel;

typedef struct derech_find_opts {
	uint32_t struct_size;       /* = sizeof(derech_find_opts) */
	uint32_t reserved0;         /* must be zero               */
	const derech_cancel *cancel;
} derech_find_opts;

/* A token may be requested from another thread while find_paths_ex is
 * running.  The caller must keep it alive until that call returns. */
DERECH_API derech_status derech_cancel_create(derech_cancel **out);
DERECH_API void derech_cancel_request(derech_cancel *cancel);
DERECH_API void derech_cancel_destroy(derech_cancel *cancel);

/* Solve a batch of 0..n requests.  Request coordinates that are out of
 * bounds yield DERECH_PATH_INVALID_ENDPOINT for that request only; an
 * unknown profile_id, unknown flag bit, or invalid epsilon /
 * max_perceived_cost anywhere in the batch is a programming error and
 * fails the whole call (nothing is allocated).
 *
 * On DERECH_OK, *out receives a results object the caller must free with
 * derech_results_destroy().  reqs may be NULL when n_reqs == 0. */
DERECH_API derech_status derech_find_paths(derech_map *map,
	const derech_request *reqs, uint32_t n_reqs, derech_results **out);

DERECH_API derech_status derech_find_paths_ex(derech_map *map,
	const derech_request *reqs, uint32_t n_reqs,
	const derech_find_opts *opts, derech_results **out);

DERECH_API void derech_results_destroy(derech_results *results);

DERECH_API uint32_t derech_results_count(const derech_results *results);
DERECH_API derech_path_status derech_result_status(
	const derech_results *results, uint32_t i);

/* Number of steps (entered tiles); the start tile is not a step, the goal
 * (or partial endpoint) is the last step.  0 for trivial/failed paths. */
DERECH_API uint32_t derech_result_length(const derech_results *results,
	uint32_t i);

/* Interleaved x,y pairs, 2 * length uint32 values, ordered from the first
 * step to the endpoint.  NULL when length is 0.  Valid until
 * derech_results_destroy(). */
DERECH_API const uint32_t *derech_result_steps(const derech_results *results,
	uint32_t i);

/* TRUE tick duration of each step (base terrain cost incl. diagonal
 * multiplier, excluding preference weights).  length floats, NULL when
 * length is 0.  The authoritative total is
 * derech_result_total_ticks_q8(); summing these floats may differ by
 * rounding. */
DERECH_API const float *derech_result_step_ticks(
	const derech_results *results, uint32_t i);

DERECH_API float derech_result_total_ticks(const derech_results *results,
	uint32_t i);
DERECH_API float derech_result_total_perceived(const derech_results *results,
	uint32_t i);

/* Exact totals in fixed point with eight fractional bits.  These are
 * authoritative where a float cannot preserve 1/256-tick precision. */
DERECH_API uint64_t derech_result_total_ticks_q8(
	const derech_results *results, uint32_t i);
DERECH_API uint64_t derech_result_total_perceived_q8(
	const derech_results *results, uint32_t i);

/* Settled-node count of the search that produced result i (diagnostics /
 * budget tuning).  0 for requests resolved without searching. */
DERECH_API uint32_t derech_result_expansions(const derech_results *results,
	uint32_t i);

#ifdef __cplusplus
}
#endif

#endif /* DERECH_H */
