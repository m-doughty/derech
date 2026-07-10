if(NOT DEFINED DERECH_LIBRARY OR NOT DEFINED DERECH_NM OR
	NOT DEFINED DERECH_ALLOWLIST)
	message(FATAL_ERROR
		"DERECH_LIBRARY, DERECH_NM, and DERECH_ALLOWLIST are required")
endif()

if(DERECH_APPLE)
	execute_process(
		COMMAND "${DERECH_NM}" -gUj "${DERECH_LIBRARY}"
		RESULT_VARIABLE _result
		OUTPUT_VARIABLE _nm_output
		ERROR_VARIABLE _nm_error)
else()
	execute_process(
		COMMAND "${DERECH_NM}" -D --defined-only "${DERECH_LIBRARY}"
		RESULT_VARIABLE _result
		OUTPUT_VARIABLE _nm_output
		ERROR_VARIABLE _nm_error)
endif()
if(NOT _result EQUAL 0)
	message(FATAL_ERROR "nm failed: ${_nm_error}")
endif()

string(REGEX MATCHALL "_?derech_[A-Za-z0-9_]+" _actual "${_nm_output}")
list(TRANSFORM _actual REPLACE "^_" "")
list(REMOVE_DUPLICATES _actual)
list(SORT _actual)

file(STRINGS "${DERECH_ALLOWLIST}" _expected REGEX "^derech_[A-Za-z0-9_]+$")
list(REMOVE_DUPLICATES _expected)
list(SORT _expected)

if(NOT "${_actual}" STREQUAL "${_expected}")
	message(FATAL_ERROR
		"exported symbols differ from the ABI allowlist\n"
		"expected: ${_expected}\nactual:   ${_actual}")
endif()
