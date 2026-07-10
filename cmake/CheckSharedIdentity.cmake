foreach(_required
	DERECH_FILE
	DERECH_FILE_NAME
	DERECH_EXPECTED_FILE_NAME
	DERECH_SONAME_FILE
	DERECH_SONAME_FILE_NAME
	DERECH_EXPECTED_SONAME_FILE_NAME)
	if(NOT DEFINED ${_required})
		message(FATAL_ERROR "${_required} is required")
	endif()
endforeach()

if(NOT EXISTS "${DERECH_FILE}")
	message(FATAL_ERROR "versioned shared library is missing: ${DERECH_FILE}")
endif()
if(NOT EXISTS "${DERECH_SONAME_FILE}")
	message(FATAL_ERROR "shared-library ABI link is missing: ${DERECH_SONAME_FILE}")
endif()
if(NOT DERECH_FILE_NAME STREQUAL DERECH_EXPECTED_FILE_NAME)
	message(FATAL_ERROR
		"shared-library filename ${DERECH_FILE_NAME} does not match "
		"${DERECH_EXPECTED_FILE_NAME}")
endif()
if(NOT DERECH_SONAME_FILE_NAME STREQUAL
	DERECH_EXPECTED_SONAME_FILE_NAME)
	message(FATAL_ERROR
		"shared-library ABI name ${DERECH_SONAME_FILE_NAME} does not match "
		"${DERECH_EXPECTED_SONAME_FILE_NAME}")
endif()
