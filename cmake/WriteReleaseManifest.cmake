foreach(_required OUTPUT TAG COMMIT PLATFORM LINKAGE VERSION ABI_VERSION)
	if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
		message(FATAL_ERROR "${_required} is required")
	endif()
endforeach()

file(WRITE "${OUTPUT}"
	"manifest_version=1\n"
	"project=derech\n"
	"version=${VERSION}\n"
	"abi_version=${ABI_VERSION}\n"
	"tag=${TAG}\n"
	"commit=${COMMIT}\n"
	"platform=${PLATFORM}\n"
	"linkage=${LINKAGE}\n")
