cmake_minimum_required(VERSION 3.16)

project(derech_consumer LANGUAGES C CXX)

option(DERECH_EXPECT_STATIC "Require the installed static target contract" OFF)

# A sanitizer-instrumented static archive carries the sanitizer's module
# constructors, so a consumer must link the matching runtime.  The exported
# target intentionally does not force these flags (BUILD_INTERFACE-scoped);
# the install test passes them here so it can consume a sanitized build.
set(DERECH_CONSUMER_SANITIZE_FLAGS "" CACHE STRING
	"Sanitizer flags to build the install-consumer with")
if(DERECH_CONSUMER_SANITIZE_FLAGS)
	separate_arguments(_derech_san_flags UNIX_COMMAND
		"${DERECH_CONSUMER_SANITIZE_FLAGS}")
	add_compile_options(${_derech_san_flags})
	add_link_options(${_derech_san_flags})
endif()

find_package(derech 0.5 CONFIG REQUIRED)

add_executable(consumer_c consumer.c)
target_link_libraries(consumer_c PRIVATE derech::derech)
target_compile_definitions(consumer_c PRIVATE
	DERECH_EXPECT_STATIC=$<BOOL:${DERECH_EXPECT_STATIC}>)
set_target_properties(consumer_c PROPERTIES
	C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

add_executable(consumer_cpp consumer.cpp)
target_link_libraries(consumer_cpp PRIVATE derech::derech)
target_compile_definitions(consumer_cpp PRIVATE
	DERECH_EXPECT_STATIC=$<BOOL:${DERECH_EXPECT_STATIC}>)
set_target_properties(consumer_cpp PROPERTIES
	CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

if(WIN32)
	foreach(consumer consumer_c consumer_cpp)
		add_custom_command(TARGET ${consumer} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				$<TARGET_FILE:derech::derech>
				$<TARGET_FILE_DIR:${consumer}>)
	endforeach()
endif()

enable_testing()
add_test(NAME consumer_c COMMAND consumer_c)
add_test(NAME consumer_cpp COMMAND consumer_cpp)
