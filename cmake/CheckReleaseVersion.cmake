if(NOT DEFINED DERECH_SOURCE_DIR)
	message(FATAL_ERROR "DERECH_SOURCE_DIR is required")
endif()

file(READ "${DERECH_SOURCE_DIR}/CMakeLists.txt" _cmake_lists)
string(REGEX MATCH
	"project\\(derech[ \t\r\n]+VERSION[ \t]+([0-9]+\\.[0-9]+\\.[0-9]+)"
	_project_version "${_cmake_lists}")
if(NOT _project_version)
	message(FATAL_ERROR "Could not read the CMake project version")
endif()
set(_project_version_value "${CMAKE_MATCH_1}")
if(DEFINED DERECH_EXPECTED_VERSION AND
	NOT "${_project_version_value}" STREQUAL "${DERECH_EXPECTED_VERSION}")
	message(FATAL_ERROR
		"CMake project version does not match ${DERECH_EXPECTED_VERSION}")
endif()
set(DERECH_EXPECTED_VERSION "${_project_version_value}")

file(READ "${DERECH_SOURCE_DIR}/CHANGELOG.md" _changelog)
string(REPLACE "\r\n" "\n" _changelog "${_changelog}")
string(FIND "${_changelog}"
	"# Changelog\n\n## Unreleased" _unreleased_heading)
if(NOT _unreleased_heading EQUAL 0)
	message(FATAL_ERROR "CHANGELOG.md does not begin with Unreleased")
endif()
string(FIND "${_changelog}"
	"\n## ${DERECH_EXPECTED_VERSION}" _changelog_version)
if(_changelog_version EQUAL -1)
	message(FATAL_ERROR
		"CHANGELOG.md does not contain ${DERECH_EXPECTED_VERSION}")
endif()

file(READ "${DERECH_SOURCE_DIR}/README.md" _readme)
string(FIND "${_readme}"
	"Current source version: **v${DERECH_EXPECTED_VERSION}**" _readme_version)
if(_readme_version EQUAL -1)
	message(FATAL_ERROR
		"README.md does not identify v${DERECH_EXPECTED_VERSION} as current source")
endif()

if(DEFINED DERECH_RELEASE_TAG AND
	NOT "${DERECH_RELEASE_TAG}" STREQUAL "" AND
	NOT "${DERECH_RELEASE_TAG}" STREQUAL "v${DERECH_EXPECTED_VERSION}")
	message(FATAL_ERROR
		"release tag ${DERECH_RELEASE_TAG} does not match v${DERECH_EXPECTED_VERSION}")
endif()
