cmake_minimum_required(VERSION 3.16)

project(derech
	VERSION 0.5.0
	DESCRIPTION "Batch pathfinding over weighted tile grids"
	LANGUAGES C)

# Only build the test suite by default when derech is the top-level build;
# add_subdirectory() consumers opt in explicitly.  (Equivalent to
# PROJECT_IS_TOP_LEVEL, computed by hand so the CMake floor stays at 3.16.)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(DERECH_DEFAULT_BUILD_TESTS ON)
else()
	set(DERECH_DEFAULT_BUILD_TESTS OFF)
endif()
option(DERECH_BUILD_TESTS "Build the derech test suite"
	${DERECH_DEFAULT_BUILD_TESTS})
option(DERECH_BUILD_BENCH "Build the benchmark / counter-gate binary" OFF)
option(DERECH_FUZZ "Build the libFuzzer harness (requires Clang)" OFF)
option(DERECH_WERROR "Treat warnings as errors" OFF)
option(DERECH_SANITIZE "Build with AddressSanitizer + UBSanitizer" OFF)
option(DERECH_TSAN "Build with ThreadSanitizer" OFF)
option(DERECH_BUILD_DEMO_SELFTESTS "Build headless demo self-tests" OFF)

if(DERECH_SANITIZE AND DERECH_TSAN)
	message(FATAL_ERROR "DERECH_SANITIZE and DERECH_TSAN are mutually exclusive")
endif()
if(DERECH_FUZZ AND NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
	message(FATAL_ERROR "DERECH_FUZZ requires Clang (libFuzzer)")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(DERECH_ABI_VERSION 1)
set(DERECH_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
file(MAKE_DIRECTORY "${DERECH_GENERATED_INCLUDE_DIR}")
configure_file(cmake/derech_version.h.in
	"${DERECH_GENERATED_INCLUDE_DIR}/derech_version.h" @ONLY)

set(DERECH_SOURCES
	src/alloc.c
	src/map.c
	src/profile.c
	src/search.c
	src/batch.c
	src/pool.c
	src/field.c)
add_library(derech ${DERECH_SOURCES})
add_library(derech::derech ALIAS derech)

if(NOT WIN32)
	set(THREADS_PREFER_PTHREAD_FLAG ON)
	find_package(Threads REQUIRED)
	target_link_libraries(derech PUBLIC Threads::Threads)
endif()

target_include_directories(derech PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${DERECH_GENERATED_INCLUDE_DIR}>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(derech PROPERTIES
	C_STANDARD 17
	C_STANDARD_REQUIRED ON
	C_EXTENSIONS OFF
	C_VISIBILITY_PRESET hidden
	EXPORT_NAME derech
	VERSION ${PROJECT_VERSION}
	SOVERSION ${DERECH_ABI_VERSION})

target_compile_definitions(derech PRIVATE DERECH_BUILD)
if(NOT BUILD_SHARED_LIBS)
	target_compile_definitions(derech PUBLIC DERECH_STATIC)
endif()

if(MSVC)
	set(DERECH_C_WARNINGS /W4)
	if(DERECH_WERROR)
		list(APPEND DERECH_C_WARNINGS /WX)
	endif()
else()
	set(DERECH_C_WARNINGS -Wall -Wextra -Wshadow -Wvla)
	if(DERECH_WERROR)
		list(APPEND DERECH_C_WARNINGS -Werror)
	endif()
endif()
target_compile_options(derech PRIVATE ${DERECH_C_WARNINGS})

if(DERECH_SANITIZE)
	if(MSVC)
		message(FATAL_ERROR "DERECH_SANITIZE is not supported with MSVC")
	endif()
	# BUILD_INTERFACE keeps same-build-tree consumers instrumented while
	# preventing the flags from leaking into the exported/installed target.
	target_compile_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=address,undefined>
		$<BUILD_INTERFACE:-fno-sanitize-recover=all>)
	target_link_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=address,undefined>)
endif()

if(DERECH_TSAN)
	if(MSVC)
		message(FATAL_ERROR "DERECH_TSAN is not supported with MSVC")
	endif()
	target_compile_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=thread>)
	target_link_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=thread>)
endif()

install(TARGETS derech
	EXPORT derechTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES
	include/derech.h
	"${DERECH_GENERATED_INCLUDE_DIR}/derech_version.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(DERECH_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/derech")
configure_package_config_file(cmake/derechConfig.cmake.in
	"${CMAKE_CURRENT_BINARY_DIR}/derechConfig.cmake"
	INSTALL_DESTINATION "${DERECH_INSTALL_CMAKEDIR}")
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/derechConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMinorVersion)
install(EXPORT derechTargets
	FILE derechTargets.cmake
	NAMESPACE derech::
	DESTINATION "${DERECH_INSTALL_CMAKEDIR}")
install(FILES
	"${CMAKE_CURRENT_BINARY_DIR}/derechConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/derechConfigVersion.cmake"
	DESTINATION "${DERECH_INSTALL_CMAKEDIR}")

file(RELATIVE_PATH DERECH_PC_PREFIX_REL
	"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig"
	"${CMAKE_INSTALL_PREFIX}")
if(BUILD_SHARED_LIBS)
	set(DERECH_PC_PRIVATE_LIBS "")
	set(DERECH_PC_COMPILE_DEFINITIONS "")
else()
	set(DERECH_PC_COMPILE_DEFINITIONS " -DDERECH_STATIC")
	if(WIN32)
		set(DERECH_PC_PRIVATE_LIBS "")
	else()
		set(DERECH_PC_PRIVATE_LIBS " -pthread")
	endif()
endif()
configure_file(cmake/derech.pc.in "${CMAKE_CURRENT_BINARY_DIR}/derech.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/derech.pc"
	DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
install(FILES README.md CHANGELOG.md LICENSE ABI.md INSTALL.md RELEASING.md
	DESTINATION "${CMAKE_INSTALL_DOCDIR}")

if(DERECH_BUILD_TESTS)
	enable_testing()

	add_library(derech_test_support STATIC
		tests/support.c)
	target_link_libraries(derech_test_support PUBLIC derech)
	if(NOT WIN32)
		# tests use llround/nanf from libm — separate on glibc,
		# implicit inside libSystem on macOS
		target_link_libraries(derech_test_support PUBLIC m)
	endif()
	target_include_directories(derech_test_support PUBLIC tests src)
	set_target_properties(derech_test_support PROPERTIES
		C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
	target_compile_options(derech_test_support PRIVATE ${DERECH_C_WARNINGS})

	set(DERECH_TEST_NAMES map profile search batch field goalset property abi
		concurrency extreme resource)
	foreach(t IN LISTS DERECH_TEST_NAMES)
		add_executable(test_${t} tests/test_${t}.c)
		target_link_libraries(test_${t} PRIVATE derech_test_support)
		set_target_properties(test_${t} PROPERTIES
			C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
		target_compile_options(test_${t} PRIVATE ${DERECH_C_WARNINGS})
		add_test(NAME ${t} COMMAND test_${t})
	endforeach()

	# Fault injection is compiled into a private object-library copy so the
	# installed derech target remains production-identical when tests are on.
	add_library(derech_oom_core OBJECT ${DERECH_SOURCES})
	target_include_directories(derech_oom_core PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		${DERECH_GENERATED_INCLUDE_DIR})
	target_compile_definitions(derech_oom_core PRIVATE
		DERECH_BUILD DERECH_STATIC DERECH_TESTING)
	set_target_properties(derech_oom_core PROPERTIES
		C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
		C_VISIBILITY_PRESET hidden)
	target_compile_options(derech_oom_core PRIVATE ${DERECH_C_WARNINGS})
	if(NOT WIN32)
		target_link_libraries(derech_oom_core PUBLIC Threads::Threads)
	endif()

	add_executable(test_oom tests/test_oom.c tests/support.c)
	target_link_libraries(test_oom PRIVATE derech_oom_core)
	target_include_directories(test_oom PRIVATE tests src)
	target_compile_definitions(test_oom PRIVATE DERECH_STATIC DERECH_TESTING)
	set_target_properties(test_oom PROPERTIES
		C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
	target_compile_options(test_oom PRIVATE ${DERECH_C_WARNINGS})
	if(NOT WIN32)
		target_link_libraries(test_oom PRIVATE m)
	endif()
	if(DERECH_SANITIZE)
		target_compile_options(derech_oom_core PRIVATE
			-fsanitize=address,undefined -fno-sanitize-recover=all)
		target_compile_options(test_oom PRIVATE
			-fsanitize=address,undefined -fno-sanitize-recover=all)
		target_link_options(test_oom PRIVATE -fsanitize=address,undefined)
	elseif(DERECH_TSAN)
		target_compile_options(derech_oom_core PRIVATE -fsanitize=thread)
		target_compile_options(test_oom PRIVATE -fsanitize=thread)
		target_link_options(test_oom PRIVATE -fsanitize=thread)
	endif()
	add_test(NAME oom COMMAND test_oom)

	if(BUILD_SHARED_LIBS)
		set(DERECH_INSTALL_TEST_EXPECT_STATIC OFF)
	else()
		set(DERECH_INSTALL_TEST_EXPECT_STATIC ON)
	endif()
	# Sanitizer instrumentation no longer leaks through the exported target
	# (BUILD_INTERFACE-scoped), so a consumer of a sanitized static archive
	# must opt into matching instrumentation itself.  Hand the flags to the
	# installed-consumer test so it models that correctly.
	set(DERECH_CONSUMER_SANITIZE_FLAGS "")
	if(DERECH_SANITIZE)
		set(DERECH_CONSUMER_SANITIZE_FLAGS "-fsanitize=address,undefined")
	elseif(DERECH_TSAN)
		set(DERECH_CONSUMER_SANITIZE_FLAGS "-fsanitize=thread")
	endif()
	configure_file(cmake/VerifyInstall.cmake.in
		"${CMAKE_CURRENT_BINARY_DIR}/VerifyInstall.cmake" @ONLY)
	add_test(NAME install_consumer COMMAND ${CMAKE_COMMAND}
		-DDERECH_TEST_CONFIG=$<CONFIG>
		-P "${CMAKE_CURRENT_BINARY_DIR}/VerifyInstall.cmake")
	add_test(NAME version_consistency COMMAND ${CMAKE_COMMAND}
		-DDERECH_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
		-DDERECH_EXPECTED_VERSION=${PROJECT_VERSION}
		-P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckReleaseVersion.cmake")
	if(BUILD_SHARED_LIBS AND UNIX)
		add_test(NAME exported_symbols COMMAND ${CMAKE_COMMAND}
			-DDERECH_LIBRARY=$<TARGET_FILE:derech>
			-DDERECH_NM=${CMAKE_NM}
			-DDERECH_APPLE=${APPLE}
			-DDERECH_ALLOWLIST=${CMAKE_CURRENT_SOURCE_DIR}/cmake/derech.exports
			-P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckExports.cmake")
		if(APPLE)
			set(_derech_expected_file "libderech.${PROJECT_VERSION}.dylib")
			set(_derech_expected_soname
				"libderech.${DERECH_ABI_VERSION}.dylib")
		else()
			set(_derech_expected_file
				"${CMAKE_SHARED_LIBRARY_PREFIX}derech${CMAKE_SHARED_LIBRARY_SUFFIX}.${PROJECT_VERSION}")
			set(_derech_expected_soname
				"${CMAKE_SHARED_LIBRARY_PREFIX}derech${CMAKE_SHARED_LIBRARY_SUFFIX}.${DERECH_ABI_VERSION}")
		endif()
		add_test(NAME shared_library_identity COMMAND ${CMAKE_COMMAND}
			-DDERECH_FILE=$<TARGET_FILE:derech>
			-DDERECH_FILE_NAME=$<TARGET_FILE_NAME:derech>
			-DDERECH_EXPECTED_FILE_NAME=${_derech_expected_file}
			-DDERECH_SONAME_FILE=$<TARGET_SONAME_FILE:derech>
			-DDERECH_SONAME_FILE_NAME=$<TARGET_SONAME_FILE_NAME:derech>
			-DDERECH_EXPECTED_SONAME_FILE_NAME=${_derech_expected_soname}
			-P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckSharedIdentity.cmake")
	endif()
endif()

if(DERECH_BUILD_BENCH)
	enable_testing()
	add_executable(derech_bench bench/bench.c)
	target_link_libraries(derech_bench PRIVATE derech)
	set_target_properties(derech_bench PROPERTIES
		C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
	target_compile_options(derech_bench PRIVATE ${DERECH_C_WARNINGS})
	add_test(NAME bench_counters COMMAND derech_bench --check)
endif()

option(DERECH_BUILD_DEMO "Build the notcurses demos" OFF)

if(DERECH_BUILD_DEMO_SELFTESTS)
	enable_testing()
	foreach(demo village woodcutters)
		add_executable(derech_${demo}_selftest demo/${demo}.c)
		target_link_libraries(derech_${demo}_selftest PRIVATE derech)
		target_compile_definitions(derech_${demo}_selftest PRIVATE
			DEMO_SELFTEST_ONLY)
		set_target_properties(derech_${demo}_selftest PROPERTIES
			C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
		target_compile_options(derech_${demo}_selftest PRIVATE
			${DERECH_C_WARNINGS})
		add_test(NAME demo_${demo}_selftest
			COMMAND derech_${demo}_selftest --selftest)
	endforeach()
endif()

if(DERECH_BUILD_DEMO)
	find_package(PkgConfig QUIET)
	if(PKG_CONFIG_FOUND)
		pkg_check_modules(NOTCURSES QUIET IMPORTED_TARGET notcurses-core)
	endif()
	if(NOT NOTCURSES_FOUND AND DERECH_NOTCURSES_INCLUDE AND DERECH_NOTCURSES_LIB)
		if(NOT EXISTS ${DERECH_NOTCURSES_INCLUDE}/notcurses/version.h)
			# version.h is generated by notcurses' own build; a
			# source checkout lacks it, so synthesize one
			file(WRITE ${CMAKE_BINARY_DIR}/nc-compat/notcurses/version.h
				"#define NOTCURSES_VERSION_MAJOR 3\n"
				"#define NOTCURSES_VERSION_MINOR 0\n"
				"#define NOTCURSES_VERSION_PATCH 17\n"
				"#define NOTCURSES_VERSION_TWEAK 0\n"
				"#define NOTCURSES_VERSION \"3.0.17\"\n")
		endif()
		find_library(DERECH_NOTCURSES_CORE_LIB notcurses-core
			PATHS ${DERECH_NOTCURSES_LIB} NO_DEFAULT_PATH)
		if(NOT DERECH_NOTCURSES_CORE_LIB)
			message(FATAL_ERROR "libnotcurses-core not found in ${DERECH_NOTCURSES_LIB}")
		endif()
		if(APPLE)
			# vendored dylibs use @loader_path install names; point
			# each executable's reference at the absolute location
			execute_process(COMMAND otool -D ${DERECH_NOTCURSES_CORE_LIB}
				OUTPUT_VARIABLE _nc_id_out)
			string(REGEX REPLACE ".*\n(.+)\n?$" "\\1" _nc_id "${_nc_id_out}")
			string(STRIP "${_nc_id}" _nc_id)
			get_filename_component(_nc_real ${DERECH_NOTCURSES_CORE_LIB} REALPATH)
		endif()
	elseif(NOT NOTCURSES_FOUND)
		message(FATAL_ERROR "notcurses not found: install it (pkg-config"
			" notcurses-core) or pass -DDERECH_NOTCURSES_INCLUDE=..."
			" -DDERECH_NOTCURSES_LIB=...")
	endif()

	foreach(demo village woodcutters)
		add_executable(derech_${demo} demo/${demo}.c)
		target_link_libraries(derech_${demo} PRIVATE derech)
		set_target_properties(derech_${demo} PROPERTIES
			C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
		target_compile_options(derech_${demo} PRIVATE ${DERECH_C_WARNINGS})
		if(NOTCURSES_FOUND)
			target_link_libraries(derech_${demo} PRIVATE PkgConfig::NOTCURSES)
		else()
			target_include_directories(derech_${demo} PRIVATE
				${DERECH_NOTCURSES_INCLUDE})
			if(NOT EXISTS ${DERECH_NOTCURSES_INCLUDE}/notcurses/version.h)
				target_include_directories(derech_${demo} PRIVATE
					${CMAKE_BINARY_DIR}/nc-compat)
			endif()
			target_link_libraries(derech_${demo} PRIVATE
				${DERECH_NOTCURSES_CORE_LIB})
			if(APPLE)
				add_custom_command(TARGET derech_${demo} POST_BUILD
					COMMAND install_name_tool -change "${_nc_id}"
						"${_nc_real}" $<TARGET_FILE:derech_${demo}>)
			endif()
		endif()
	endforeach()
endif()

if(DERECH_FUZZ)
	add_executable(fuzz_derech fuzz/fuzz_derech.c)
	target_link_libraries(fuzz_derech PRIVATE derech)
	set_target_properties(fuzz_derech PROPERTIES
		C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
	target_compile_options(fuzz_derech PRIVATE
		${DERECH_C_WARNINGS} -fsanitize=fuzzer,address,undefined
		-fno-sanitize-recover=all)
	target_link_options(fuzz_derech PRIVATE
		-fsanitize=fuzzer,address,undefined)
	# the library itself needs matching instrumentation; BUILD_INTERFACE
	# keeps it out of any exported/installed target.
	target_compile_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=fuzzer-no-link,address,undefined>
		$<BUILD_INTERFACE:-fno-sanitize-recover=all>)
	target_link_options(derech PUBLIC
		$<BUILD_INTERFACE:-fsanitize=address,undefined>)
endif()
