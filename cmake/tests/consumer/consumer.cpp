#include <cstddef>
#include <cstdint>
#include <cstring>

#include <derech.h>
#include <derech_version.h>

#if DERECH_EXPECT_STATIC && !defined(DERECH_STATIC)
#error "static package did not propagate DERECH_STATIC"
#elif !DERECH_EXPECT_STATIC && defined(DERECH_STATIC)
#error "shared package incorrectly propagated DERECH_STATIC"
#endif

static_assert(sizeof(derech_request) == 64, "unexpected request ABI");
static_assert(offsetof(derech_request, struct_size) == 0,
	"struct_size must remain the first request field");

int main()
{
	derech_map *map = nullptr;
	derech_map_opts map_opts = {};
	derech_profile_desc profile = {};
	derech_request request = {};
	derech_results *results = nullptr;
	derech_status status;

	if (DERECH_VERSION_MAJOR != 0 || DERECH_VERSION_MINOR != 5 ||
		DERECH_VERSION_PATCH != 0 || DERECH_ABI_VERSION != 1u) {
		return 1;
	}
	if (derech_version() != (5u << 8) ||
		std::strcmp(derech_version_str(), "0.5.0") != 0 ||
		derech_abi_version() != 1u) {
		return 2;
	}
	map_opts.struct_size = static_cast<std::uint32_t>(sizeof(map_opts));
	map_opts.default_passability = 1.0f;
	map_opts.n_threads = 1;
	status = derech_map_create_ex(4, 4, &map_opts, &map);
	if (status != DERECH_OK || map == nullptr) {
		return 3;
	}
	if (derech_map_width(map) != 4 || derech_map_height(map) != 4) {
		derech_map_destroy(map);
		return 4;
	}
	profile.struct_size = static_cast<std::uint32_t>(sizeof(profile));
	if (derech_profile_register(map, &profile) != 0) {
		derech_map_destroy(map);
		return 5;
	}
	request.struct_size = static_cast<std::uint32_t>(sizeof(request));
	request.goal_x = 3;
	request.goal_y = 3;
	request.epsilon = 1.0f;
	if (derech_find_paths(map, &request, 1, &results) != DERECH_OK ||
		results == nullptr) {
		derech_map_destroy(map);
		return 6;
	}
	if (derech_results_count(results) != 1 ||
		derech_result_status(results, 0) != DERECH_PATH_FOUND ||
		derech_result_length(results, 0) != 3 ||
		derech_result_total_ticks_q8(results, 0) != 1086u ||
		derech_result_total_perceived_q8(results, 0) != 1086u) {
		derech_results_destroy(results);
		derech_map_destroy(map);
		return 7;
	}
	derech_results_destroy(results);
	derech_map_destroy(map);
	return 0;
}
