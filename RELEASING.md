# Releasing derech

Releases are tag-driven and must start from a clean, reviewed commit. The
workflow never creates a tag. `v0.5.0` is intended to be the first formal tag;
do not invent tags for the imported 0.1-0.4 changelog history.

## Prepare

1. Set `project(derech VERSION ...)` in `CMakeLists.txt`. The public version
   header and pkg-config version are generated from it.
2. Update `CHANGELOG.md`, the current source version in `README.md`, and ABI
   documentation when applicable.
3. Run release/Werror, ASan+UBSan, TSan, shared-library, installed-consumer,
   pkg-config, benchmark-counter, demo-selftest, and fuzz smoke checks.
4. Reinstall and exercise each downstream binding. ABI-1 bindings must verify
   their native sizes and offsets at startup.
5. Review `cmake/derech.exports`. Any incompatible ABI change requires a new
   ABI epoch and `SOVERSION`.

`cmake/CheckReleaseVersion.cmake` rejects project/changelog/README/tag drift.

## Tag And Package

After the soak period, create one annotated tag matching the source version
and push it intentionally. A `v*` tag starts `.github/workflows/release.yml`.
The workflow:

- verifies that the tag matches the CMake version and checked-out commit;
- builds and tests static and shared packages on Linux x86_64, macOS arm64,
  and Windows x86_64;
- archives relocated install trees with a tag/commit/platform manifest;
- publishes SHA-256 checksum files and GitHub build-provenance attestations;
- creates or updates a **draft** GitHub release.

Draft assets must be smoke-tested before publication. Publishing is a
separate manual `workflow_dispatch` run with an existing tag and the explicit
`publish` input enabled. Configure GitHub's `release` environment with
required reviewers when repository policy requires an additional approval.

Never run release automation from an uncommitted worktree, and never use it
to create or move a tag.
