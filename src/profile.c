/* Profile registration and per-combo weight folding. */

#include "derech_internal.h"

/* Fold a profile's per-tag weights over one interned tag word.  Bit order
 * is ascending (0..63) — tests/reference.c mirrors this exactly, so the
 * double-precision products round identically. */
void derech_profile_fold_combo(const derech_profile *p, uint64_t word,
	derech_pentry *out)
{
	double mult = 1.0;
	double add = 0.0;

	if ((word & p->block_mask) != 0 ||
		(p->require_mask != 0 && (word & p->require_mask) == 0)) {
		out->mult_q = DERECH_Q_MULT_BLOCKED;
		out->add_q = 0;
		return;
	}
	for (uint32_t b = 0; b < 64; b++) {
		if ((word >> b) & 1) {
			mult *= p->mult[b];
			add += p->add[b];
		}
	}
	/* cap far below the blocked sentinel; the search floors perceived
	 * step costs at 1 so a fully degenerate multiplier is still safe */
	out->mult_q = derech_q_round(mult * 256.0, 1u << 28);
	out->add_q = derech_q_round(add * 256.0, 1u << 28);
}

static int mult_valid(float v)
{
	/* 0 is the documented "neutral" placeholder */
	return v == 0.0f || (v > 0.0f && v <= 65536.0f);
}

static int add_valid(float v)
{
	return v >= 0.0f && v <= 65536.0f;
}

int32_t derech_profile_register(derech_map *map,
	const derech_profile_desc *desc)
{
	derech_profile prof;
	derech_profile *profiles;
	derech_pentry *table;
	double mult_floor = 1.0;
	uint64_t d2;
	int32_t id;

	if (map == NULL || desc == NULL ||
		desc->struct_size != sizeof(*desc)) {
		return DERECH_E_INVALID_ARG;
	}
	if (desc->connectivity > DERECH_CONN_4 ||
		desc->corner_rule > DERECH_CORNER_ALLOW ||
		desc->reserved0 != 0 || desc->reserved1 != 0) {
		return DERECH_E_INVALID_ARG;
	}
	if (desc->diagonal_mult != 0.0f && !(desc->diagonal_mult >= 1.0f &&
		desc->diagonal_mult <= 16.0f)) {
		return DERECH_E_INVALID_ARG;
	}
	for (uint32_t b = 0; b < 64; b++) {
		if (!mult_valid(desc->tag_mult[b]) ||
			!add_valid(desc->tag_add[b])) {
			return DERECH_E_INVALID_ARG;
		}
	}

	memset(&prof, 0, sizeof(prof));
	prof.block_mask = desc->block_mask;
	prof.require_mask = desc->require_mask;
	prof.relevant_mask = desc->block_mask | desc->require_mask;
	prof.connectivity = desc->connectivity;
	prof.corner_rule = desc->corner_rule;
	prof.diag_q = desc->diagonal_mult == 0.0f ? DERECH_Q_DIAG_SQRT2 :
		derech_q_round((double)desc->diagonal_mult * 256.0, 1u << 13);
	for (uint32_t b = 0; b < 64; b++) {
		prof.mult[b] = desc->tag_mult[b] == 0.0f ? 1.0 :
			(double)desc->tag_mult[b];
		prof.add[b] = (double)desc->tag_add[b];
		if (prof.mult[b] != 1.0 || prof.add[b] != 0.0) {
			prof.relevant_mask |= 1ULL << b;
		}
		if (prof.mult[b] < 1.0) {
			mult_floor *= prof.mult[b];
		}
	}

	/* Admissible heuristic floors: the cheapest possible perceived step
	 * is min-base (256) times the smallest multiplier any tag word could
	 * fold to; adds only ever increase cost.  Diagonal floor is clamped
	 * to 2 * straight so octile never overestimates when diagonals cost
	 * more than two straight steps. */
	prof.h_d = derech_q_round(256.0 * mult_floor, DERECH_Q_TILE_MAX);
	if (prof.h_d < 1) {
		prof.h_d = 1;
	}
	d2 = ((uint64_t)prof.diag_q * prof.h_d) >> 8;
	if (d2 < 1) {
		d2 = 1;
	}
	if (d2 > 2 * (uint64_t)prof.h_d) {
		d2 = 2 * (uint64_t)prof.h_d;
	}
	prof.h_d2 = (uint32_t)d2;

	if (map->profile_count >= DERECH_MAX_PROFILES) {
		return DERECH_E_TOO_MANY_PROFILES;
	}
	if (!derech_busy_acquire(&map->busy)) {
		return DERECH_E_BUSY;
	}

	profiles = realloc(map->profiles, ((size_t)map->profile_count + 1) *
		sizeof(*profiles));
	if (profiles == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	map->profiles = profiles;

	table = malloc((size_t)map->combo_cap * sizeof(*table));
	if (table == NULL) {
		derech_busy_release(&map->busy);
		return DERECH_E_NOMEM;
	}
	prof.table = table;
	for (uint32_t c = 0; c < map->combo_count; c++) {
		derech_profile_fold_combo(&prof, map->combo_words[c],
			&prof.table[c]);
	}

	id = (int32_t)map->profile_count;
	map->profiles[map->profile_count] = prof;
	map->profile_count++;
	derech_busy_release(&map->busy);
	return id;
}
