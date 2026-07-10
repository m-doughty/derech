# Changelog

## Unreleased

- No changes yet.

## 0.5.0 — 2026-07-09

- Cumulative true and perceived costs now use 64-bit fixed-point arithmetic
  with eight fractional bits across A*, goal fields, staging, and result
  assembly. Cached and uncached searches agree above the old 32-bit ceiling.
  New exact
  `derech_result_total_ticks_q8` and
  `derech_result_total_perceived_q8` accessors avoid float precision loss.
- The per-map concurrency contract is race-free and remains nonblocking:
  overlapping stateful calls fail with `DERECH_E_BUSY`, immutable metadata is
  lock-free, and mutable metadata uses guarded or atomic snapshots.
- Required allocation failures propagate as `DERECH_E_NOMEM`; failed batches
  do not commit partial cache or predicate-goal membership state, and failed
  tag setters roll back newly interned words. `derech_map_create_ex` adds a
  status-returning constructor while the original constructor remains as a
  compatibility wrapper. Allocation fault-injection tests cover search,
  fields, staging, assembly, and construction.
- Worker contexts are allocated on demand and no longer retain a full-map path
  scratch array. Worker, field-working, component-label, and retained-scratch
  memory have explicit limits. Field groups are processed in deterministic
  waves, optional single-goal fields can fall back to A*, and required goal-set
  fields report `DERECH_E_RESOURCE_LIMIT` when they cannot fit. New estimate
  and live-memory statistics APIs expose the effective limits and allocations.
- Explicit goal sets report their deduplicated member count. Duplicate input
  coordinates remain accepted and ignored.
- ABI epoch 1 freezes `derech_request` at 64 bytes with `struct_size` first and
  reserved zero-required fields. `DERECH_ABI_VERSION` and
  `derech_abi_version()` let bindings reject an unsupported native ABI.
  The current profile descriptor names former padding, while known 540-byte
  and 544-byte historical layouts remain accepted. Cooperative cancellation
  is available through `derech_cancel` and `derech_find_paths_ex`.
- CMake is the source of the `0.5.0` version header. Installations export the
  relocatable `derech::derech` target, CMake config/version files, pkg-config
  metadata, documentation, and shared-library soname epoch 1. CI builds C and
  C++ consumers from a relocated installation, checks static pkg-config use,
  and exercises shared loading on Linux, macOS, and Windows.
- Sanitizer CI now runs headless village and woodcutter self-tests. Nightly
  fuzzing publishes corpus growth on a dedicated pull-request branch instead
  of pushing directly to the primary branch.
- `ABI.md`, `INSTALL.md`, and `RELEASING.md` define the integration and release
  contracts. The gated tag workflow rebuilds and tests six static/shared
  platform packages, emits SHA-256 checksums and provenance attestations, and
  creates a draft release unless publication is explicitly approved.

## 0.4.0 — 2026-07-05

- **Goal sets**: requests may target a set of tiles via
  `derech_request.goalset` (0 = ordinary single goal; note the struct
  grew — pre-1.0 breaking change).  Explicit sets
  (`derech_goalset_register`) hold a fixed tile list; predicate sets
  (`derech_goalset_register_tags`) contain every tile matching tag
  masks and follow tag edits automatically.  `DERECH_GOALSET_ADJACENT`
  retargets to tiles beside members, for impassable goals (trees, ore).
  The result path ends at the reached member; a start on (or beside,
  when adjacent) a member is FOUND with zero steps.  Set queries are
  answered from multi-source goal fields — exact, cached, expansions
  == 0 — regardless of group size; ALLOW_PARTIAL is invalid with sets.
  New: `derech_goalset_unregister`, `derech_goalset_count`,
  `DERECH_E_BAD_GOALSET`, `DERECH_E_TOO_MANY_GOALSETS`.
- **Dirty-region cache invalidation**: terrain setters log what they
  touched and which tag bits actually flipped (no-op writes cost
  nothing).  A cached field is dropped only when an edit intersects or
  abuts its reachable area AND changes passability or a tag bit its
  profile weighs — or changes its goal set's membership.  Component
  labels flush only on enterability-relevant changes.  Host-maintained
  bits no profile weighs (fog-of-war EXPLORED, decals) can be rewritten
  constantly without disturbing cached routes: the bench's 128x128
  reveal write leaves the cached set field answering in ~0.1 ms with
  identical counters.
- `demo/woodcutters.c` — fog-of-war discovery demo exercising all of
  the above (frontier exploration on explicit sets, known-trees
  predicate set, fell-and-haul economy over cached fields).
- Bench: two new gated scenarios (goalset-nearest, goalset-after-reveal).
- Fuzzing: harness extended with goal-set ops; persistent corpus
  committed at `fuzz/corpus/`; nightly 45-minute campaign workflow that
  resumes from the corpus (cache + git), merges growth back, and fails
  loudly on crashes.  The CI smoke run now seeds from the corpus.

## 0.3.0 — 2026-07-04

Tooling (M5):

- `bench/` — benchmark + counter-regression harness
  (`-DDERECH_BUILD_BENCH=ON`, `derech_bench --check`).  Gates on
  deterministic counters (found/steps/expansions/perceived checksum),
  which are identical across platforms and thread counts by design; the
  CI job runs the gate on Linux, macOS, and Windows as a standing
  cross-platform determinism check.  Wall times are informational.
- `fuzz/` — libFuzzer harness (`-DDERECH_FUZZ=ON`, Clang):
  byte-driven API-call sequences (hostile floats, out-of-range rects,
  invalid profiles/requests included) with structural path-validity
  oracles, under ASan+UBSan.  Clean over 500k+ local runs; CI smoke-runs
  90 s per push.
- `demo/` — notcurses village demo (`-DDERECH_BUILD_DEMO=ON`): 100
  scheduled NPCs pathfinding through batched derech calls on a
  hand-laid map with slow zones and per-profile preferences; scrolling,
  NPC-follow with path display, day/night tint, live batch/field
  stats; `--selftest` verifies three headless days.

Library (M4):

- Goal fields: batch requests sharing (goal, profile) above
  `derech_map_opts.field_group_threshold` (default 4) are answered from
  one exact reverse Dijkstra field — one search plus O(path) per NPC.
  Fields honor the enter-cost asymmetry, corner rules, connectivity,
  and escapable blocked starts; field-answered requests report
  expansions == 0 and exact-optimal costs (always within any requested
  epsilon bound).
- LRU field cache (`field_cache_mb`, default 64 MiB, soft within a
  batch): later batches — even single requests — reuse cached fields
  until any terrain edit invalidates them.
- Component labels per (block_mask, require_mask) class: non-partial
  impossible requests get UNREACHABLE in O(1) instead of sweeping their
  component.  Requests needing closest-approach partials fall back to
  classic searches automatically.
- Statuses may now be more precise when answered from fields/labels
  (UNREACHABLE instead of BUDGET_EXCEEDED for goals that provably do
  not exist within any budget).
- Determinism contract restated: results depend on the batch sequence
  since the last terrain edit (the cache), never on thread count.
- Older opts layouts still accepted (16-byte v0.1, 24-byte v0.2).
- Reference batch (M2-class laptop, 512×512 harsh terrain, 1000
  requests, 14 workers): 1000 scattered goals ~920 ms; 1000 requests
  converging on 20 goals ~410 ms cold / **~5 ms cached, non-partial
  (zero expansions)**.

## 0.2.0 — 2026-07-04

- Parallel batch solving on a persistent internal worker pool.
  `derech_map_opts.n_threads`: total parallelism including the calling
  thread; `0` = auto (one per CPU core, capped at 16), `1` = fully
  serial (no threads spawned), explicit values up to 64.  New
  `derech_map_thread_count()` getter.
- Hard guarantee preserved: results objects are bitwise-identical for
  any thread count (each request solves in its own search context;
  the results arena is assembled in request order).  Covered by
  thread-count invariance and pool-reuse stress tests; ThreadSanitizer
  clean.
- The 16-byte v0.1 `derech_map_opts` layout is still accepted via its
  `struct_size`.
- Reference batch (M2-class laptop, 512×512 mixed terrain, 1000
  requests): 10.6 s serial → 1.0 s on 14 workers (10.6×).

## 0.1.0 — 2026-07-04

- Initial release: weighted tile grids (entry costs with eight fractional bits),
  64-bit tag words interned per map, NPC profiles (per-tag multipliers
  and flat penalties, block/require masks, 4/8-connectivity, diagonal
  multiplier, corner rules), batched weighted A* (per-request epsilon,
  default 1.25), partial paths, expansion/cost budgets, deterministic
  integer cost model, library-owned flat results.
- Tests: unit + golden scenarios + property tests cross-checked against
  an independent reference Dijkstra; ASan/UBSan clean; CI for macOS
  arm64, Linux x86_64/aarch64 (GCC + Clang), Windows x86_64 (MSVC).
