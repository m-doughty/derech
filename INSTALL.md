# Installing derech

derech requires a C17 compiler, CMake 3.16 or newer, and the platform thread
runtime. It has no third-party runtime dependency.

## Build And Test

Static is the default:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

For a shared FFI library:

```sh
cmake -S . -B build-shared -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-shared --config Release
ctest --test-dir build-shared -C Release --output-on-failure
```

`DERECH_BUILD_DEMO_SELFTESTS=ON` adds the two headless demo checks on macOS
and Linux. `DERECH_SANITIZE=ON` and `DERECH_TSAN=ON` are mutually exclusive
instrumented builds.

## Install

```sh
cmake --install build --prefix /your/prefix --config Release
```

The prefix contains the library, `derech.h`, generated
`derech_version.h`, CMake package files, `derech.pc`, and project
documentation. Both package formats are relocatable after installation.

## CMake Consumer

```cmake
find_package(derech 0.5 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE derech::derech)
```

Set `CMAKE_PREFIX_PATH` to the install prefix when it is not in CMake's normal
search path. The imported static target propagates `DERECH_STATIC` and the
thread dependency.

## pkg-config Consumer

Shared installation:

```sh
cc host.c $(pkg-config --cflags --libs derech)
```

Static installation:

```sh
cc host.c $(pkg-config --cflags --libs --static derech)
```

The static `.pc` metadata supplies `-DDERECH_STATIC` and the private thread
link flags. When linking a static archive manually, provide those flags
yourself.
