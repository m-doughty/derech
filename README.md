# derech

**derech** (דרך — *way, path*) is a zero-dependency C17 library for batch
pathfinding over weighted 2D tile grids, built for game hosts that drive it
through an FFI (Python cffi, Raku NativeCall, LuaJIT, C#, ...).

- Every tile has a **passability** in `[0, 1]`: `1` is crossed in one tick,
  `0.5` in two, `0.25` in four, `0` is impassable.
- Every tile carries a 64-bit **tag word** (enemy territory, wilderness,
  road, water, ... — the bits mean whatever your game says they mean).
- NPC archetypes are registered as **profiles** that weigh those tags:
  multipliers (`4.0` = "enemy land feels four times slower", `0.5` =
  "prefer roads"), flat per-tile penalties, hard block masks, and
  require masks (road-bound carts).
- Requests are submitted in **batches** and answered by weighted A* over
  quantized integer costs — **bitwise-deterministic** across platforms,
  with a per-request quality knob (`epsilon`: paths cost at most ε× the
  optimal perceived cost; default 1.25).
- Batches run in **parallel** on an internal worker pool
  (`derech_map_opts.n_threads`; default one worker per core, capped at
  16; `1` = fully serial). Determinism survives: results are
  bitwise-identical for any thread count. A 1000-request batch on
  512×512 mixed terrain goes ~10.6× faster on 14 workers than serial
  (M2-class laptop).
- Requests converging on shared goals are answered from **goal fields**
  (one exact reverse Dijkstra per `(goal, profile)` group, LRU-cached)
  and impossible requests from **component labels** in O(1). The "1000
  NPCs converge on 20 rally points" batch drops from ~900 ms to
  **~5 ms** once fields are cached — zero search expansions.
- Requests can target **goal sets** instead of single tiles: explicit
  tile lists, or **tag predicates** ("nearest tile tagged
  `TREE|EXPLORED`") whose membership tracks tag edits automatically —
  with an ADJACENT mode for impassable targets like tree trunks. Set
  queries are exact, multi-source-field-backed, and cached.
- Cache invalidation is **dirty-region based**: setters record what
  actually changed, and a cached field only dies when an edit touches
  its reachable area *and* changes something its profile weighs — so a
  host can rewrite a fog-of-war EXPLORED bit every tick without
  disturbing any cached route.
- The search minimizes **perceived** cost (route choice) but reports
  **true** tick durations per step — danger changes where an NPC walks,
  not how fast it walks.

## Building

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build          # 5 suites, ~7,000 checks
```

Static library by default; add `-DBUILD_SHARED_LIBS=ON` for a shared
build (FFI hosts usually want this). Supported first-class: macOS arm64,
Linux x86_64/aarch64 (GCC or Clang), Windows x86_64 (MSVC). Do **not**
build with `-ffast-math` — determinism depends on well-behaved float →
fixed-point conversion.

Extra verification targets:

```sh
# benchmark + counter regression gate (deterministic across platforms
# and thread counts; wall-clock is printed but never gated)
cmake -B build -DDERECH_BUILD_BENCH=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build && ./build/derech_bench --check

# libFuzzer harness (Clang only): API-sequence fuzzing with
# structural-validity oracles under ASan+UBSan.  fuzz/corpus/ is the
# committed coverage frontier — pass it so runs resume where previous
# ones stopped, and merge new findings back with -merge=1
cmake -B build-fuzz -DDERECH_FUZZ=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-fuzz
./build-fuzz/fuzz_derech -max_total_time=300 fuzz/corpus
```

CI runs the unit/property suites on five platform/compiler lanes, the
ASan/UBSan and TSan suites, a shared-lib build, the counter gate on
Linux + macOS + Windows (which doubles as a cross-platform determinism
check), and a corpus-seeded fuzz smoke run.  A separate nightly
workflow fuzzes for 45 minutes in fork mode, resuming from the
persistent corpus (Actions cache as the warm layer, the committed
`fuzz/corpus/` as the durable one) and committing corpus growth back —
which also counts as repo activity, keeping the schedule alive under
GitHub's 60-day inactivity rule for as long as fuzzing keeps finding
new coverage.

## Demos

`demo/village.c` is a notcurses ASCII toy world: 100 NPCs with daily
schedules (staggered work crews, tavern lunches, evening strolls,
bedtime) living in a hand-laid village — houses, bakery, smithy,
tavern, church, mill, market stalls, farm fields, a river with one
bridge, and marsh/tall-grass slow zones. Every route is a batched
derech call; crews head for their building's *door* first, so shared
goals light up the goal-field cache (watch the `field-answered`
counter during the morning rush). Townsfolk prefer roads and hate
marsh; farmers cut across the fields.

```sh
cmake -B build -DDERECH_BUILD_DEMO=ON     # notcurses-core via pkg-config
cmake --build build && ./build/derech_village
```

Without pkg-config'd notcurses, point at any build of it with
`-DDERECH_NOTCURSES_INCLUDE=<dir> -DDERECH_NOTCURSES_LIB=<dir>`.
Controls: arrows/WASD scroll, `f`/TAB follow a moving NPC (its
remaining path is drawn), SPACE pause, `1-3` speed, `q` quit.
`--selftest` runs three headless days and asserts everyone makes it
to bed; `--demo-seconds N` auto-quits after N seconds and prints the
session's batch stats.

`demo/woodcutters.c` (`./build/derech_woodcutters`) is the discovery
demo: a tribe camp under fog of war, twelve woodcutters who must FIND
the forest before they can fell it. Frontier exploration runs on
explicit goal sets, "nearest known tree" is a `TREE|EXPLORED`
tag-predicate set (ADJACENT — trunks are impassable), chopping clears
tags and opens stumps, log-hauling converges on the camp's cached
field, and the batched fog-reveal writes demonstrate dirty-region
invalidation leaving cached routes untouched. Same controls and
`--selftest`/`--demo-seconds` flags.

## Sixty-second tour

```c
#include <derech.h>

enum { TAG_WATER = 1u << 0, TAG_ENEMY = 1u << 1, TAG_ROAD = 1u << 2 };

derech_map *map = derech_map_create(512, 512, NULL);
derech_map_set_passability(map, terrain, 512 * 512); /* floats, row-major */
derech_map_set_tags(map, tag_words, 512 * 512);      /* uint64_t bitmasks */

derech_profile_desc villager = { .struct_size = sizeof(villager) };
villager.block_mask = TAG_WATER;      /* cannot swim            */
villager.tag_mult[1] = 4.0f;          /* dreads enemy territory */
villager.tag_mult[2] = 0.8f;          /* likes roads            */
int32_t villager_id = derech_profile_register(map, &villager);

derech_request reqs[2] = {
	{ .start_x = 3, .start_y = 4, .goal_x = 400, .goal_y = 220,
	  .profile_id = (uint32_t)villager_id,
	  .flags = DERECH_REQ_ALLOW_PARTIAL },
	{ .start_x = 9, .start_y = 9, .goal_x = 40, .goal_y = 41,
	  .profile_id = (uint32_t)villager_id, .epsilon = 1.0f },
};

derech_results *res;
if (derech_find_paths(map, reqs, 2, &res) == DERECH_OK) {
	for (uint32_t i = 0; i < derech_results_count(res); i++) {
		uint32_t len = derech_result_length(res, i);
		const uint32_t *xy = derech_result_steps(res, i); /* x,y pairs */
		const float *ticks = derech_result_step_ticks(res, i);
		/* walk the NPC: enter (xy[2k], xy[2k+1]), takes ticks[k] */
		(void)len; (void)xy; (void)ticks;
	}
	derech_results_destroy(res);
}
derech_map_destroy(map);
```

Everything you need to know precisely — quantization, movement rules,
corner cutting, partial paths, budgets, the threading contract — is
documented in [`include/derech.h`](include/derech.h). The header is the
reference manual.

## FFI notes

derech's API is designed for bindings: opaque handles, fixed-layout POD
structs with a `struct_size` field, no callbacks, no globals, no
thread-local error state. Inputs are copied during the call; results are
flat primitive arrays owned by the library until `derech_results_destroy`,
so hosts can view step buffers zero-copy.

Python (cffi):

```python
from cffi import FFI
ffi = FFI()
ffi.cdef("""
typedef struct derech_map derech_map;
typedef struct derech_results derech_results;
derech_map *derech_map_create(uint32_t, uint32_t, const void *);
int32_t derech_find_paths(derech_map *, const void *, uint32_t,
                          derech_results **);
uint32_t derech_result_length(const derech_results *, uint32_t);
const uint32_t *derech_result_steps(const derech_results *, uint32_t);
void derech_results_destroy(derech_results *);
""")  # ... etc.; declarations are cffi-compatible as written
lib = ffi.dlopen("libderech.so")
```

Raku (NativeCall):

```raku
use NativeCall;

class DerechMap     is repr('CPointer') {}
class DerechResults is repr('CPointer') {}

sub derech_map_create(uint32, uint32, Pointer --> DerechMap)
	is native('derech') {*}
sub derech_find_paths(DerechMap, Blob, uint32, Pointer[DerechResults]
	--> int32) is native('derech') {*}
sub derech_result_steps(DerechResults, uint32 --> CArray[uint32])
	is native('derech') {*}
```

## Status & roadmap

v0.3 — core engine, parallel batches, and batch structure: terrain +
tags + profiles, batched weighted A* on a persistent worker pool
(bitwise thread-count invariance, verified by tests and
ThreadSanitizer), goal-field grouping with an LRU field cache,
component-label unreachability, partial paths, budgets, exhaustive
tests (unit, golden-scenario, and property tests cross-checked against
an independent reference Dijkstra), ASan/UBSan/TSan-clean, 5-platform
CI.

Planned next:

1. **Hierarchical worlds** — multi-chunk maps (chunks up to 2048×2048)
   with a portal graph for very large worlds.

Non-goals: cooperative multi-agent avoidance (NPCs dodging each other is
the host's job — derech treats NPCs as independent points), navmeshes,
and 3D grids.

## License

MIT — see [LICENSE](LICENSE).
