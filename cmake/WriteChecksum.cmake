if(NOT DEFINED ARTIFACT OR NOT EXISTS "${ARTIFACT}")
	message(FATAL_ERROR "ARTIFACT must name an existing file")
endif()

file(SHA256 "${ARTIFACT}" _sha256)
get_filename_component(_name "${ARTIFACT}" NAME)
file(WRITE "${ARTIFACT}.sha256" "${_sha256}  ${_name}\n")
