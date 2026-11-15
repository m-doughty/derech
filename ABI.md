# derech ABI

derech 0.5 establishes binary ABI epoch 1. The ABI epoch is independent of
the source version: compatible 0.5 patch releases keep epoch 1, while an
incompatible public layout or calling-contract change requires a new epoch.

Bindings must check the ABI epoch before exchanging public structures. The
source version remains useful for feature gating and diagnostics, but a
binding should not reject a compatible patch release merely because its full
version differs from the headers used to build the binding:

```c
if (derech_abi_version() != DERECH_ABI_VERSION) {
    /* reject the loaded library */
}
```

## Epoch 1 Layout Rules

- `derech_request` is fixed at 64 bytes. `struct_size` is at offset 0 and
  must be `sizeof(derech_request)`; its five reserved words must be zero.
- Singular option and descriptor structures start with `struct_size`.
  Zero-initialize the full structure before assigning fields. The ABI-1
  `derech_profile_desc` is 552 bytes and names every padding/reserved word.
- The 16-byte v0.1, 20/24-byte v0.2, and 28/32-byte v0.3/v0.4
  `derech_map_opts` layouts remain accepted. The size variation reflects the
  legacy platform ABI; new code should always pass the current 48-byte layout.
- Historical profile descriptors are normalized from their known 540-byte
  (32-bit) or 544-byte (64-bit) layouts. Unknown shorter prefixes are rejected.
- Opaque maps, cancellation tokens, and results must only be manipulated by
  the library functions declared in `derech.h`.
- Result buffers belong to their `derech_results` object and remain valid
  until `derech_results_destroy`.
- `derech_avoidance` is 104 bytes: `struct_size`, `count`, then eight
  12-byte circles. A circle contains signed 32-bit x/y and an unsigned radius.
  `derech_find_paths_avoiding` adds an optional parallel descriptor array without
  changing `derech_request` or using its reserved words. The array stays
  immutable and alive until the call returns. Existing callers are unaffected.

Exact size and offset assertions live in `tests/test_abi.c`. The public shared
library export set is frozen in `cmake/derech.exports`; shared CI rejects
missing or additional exports.

## Work Diagnostics

`derech_work_stats` is 40 bytes: 32-bit `struct_size` and reserved storage,
then four 64-bit cumulative counters starting at offset 8. Zero-initialize it
and set `struct_size` before calling `derech_map_get_work_stats`. The getter
uses the map busy guard and returns `DERECH_E_BUSY` during another map call.
Counters describe coordinator-submitted work; workers never mutate them.
This additive API does not change existing public layouts or ABI epoch 1.
Consumers using it must install matching headers and a rebuilt library.

## Library Identity

`DERECH_REQ_DIRECT_SEARCH` is an additive request flag within the existing
64-byte layout. It is valid only for single-goal requests and skips field/cache
and component-label planning. It can be combined with `ALLOW_PARTIAL`.
Older libraries reject this unknown flag; consumers must rebuild and install
matching headers and libraries before using it. ABI epoch 1 is unchanged.

The installed shared library has ABI soname/install-name epoch `1` and file
version `0.5.0`. CMake consumers should link `derech::derech`. Static C and
C++ consumers must compile with `DERECH_STATIC`; the installed CMake target
and static `derech.pc` provide that definition automatically.

## Binding Checklist

1. Pin support to `DERECH_ABI_VERSION == 1`.
2. Assert native sizes and the key offsets used by the binding.
3. Check `derech_abi_version()` before the first struct-bearing call.
4. Use the exact-width integer types from the header.
5. Test a map/profile/request/result lifecycle against every bundled native
   artifact on its target platform.
6. Record the derech tag, commit, platform triple, and artifact checksum in
   any redistributed bundle manifest.
