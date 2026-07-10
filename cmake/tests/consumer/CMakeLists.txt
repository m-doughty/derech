cmake_minimum_required(VERSION 3.16)

project(derech_consumer LANGUAGES C CXX)

option(DERECH_EXPECT_STATIC "Require the installed static target contract" OFF)

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
