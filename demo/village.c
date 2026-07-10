#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

/* derech village demo — 100 NPCs living out daily schedules on a
 * weighted tile map, all pathfinding through batched derech calls.
 *
 * The village: houses, a bakery, smithy, tavern, church, mill, market
 * stalls, and farm fields; a river with one bridge; marsh and tall
 * grass as slower-passability areas.  Townsfolk prefer roads and hate
 * marsh; farmers cut across fields.  NPCs route to a building's DOOR
 * first (so crews converge on shared goals and derech's goal fields +
 * cache light up), then hop to their spot inside.
 *
 * Controls: arrows/WASD scroll · f follow a moving NPC (TAB cycles,
 * any scroll unfollows; the followed NPC's remaining path is shown) ·
 * SPACE pause · 1/2/3 speed · q quit.
 *
 * `./derech_village --selftest` runs three headless days and verifies
 * every NPC is in bed at 04:00, no request ever fails, and goal fields
 * actually engaged.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "derech.h"

#ifndef DEMO_SELFTEST_ONLY
#include <notcurses/notcurses.h>
#include <poll.h>
#endif

/* ------------------------------------------------------------------ */
/* World                                                               */
/* ------------------------------------------------------------------ */

enum { MAP_W = 128, MAP_H = 72, N_NPC = 100, DAY_MIN = 1440 };

enum {
	TAG_ROAD = 1u << 0,
	TAG_FIELD = 1u << 1,
	TAG_MARSH = 1u << 2
};

enum {
	T_GRASS = 0, T_ROAD, T_PLAZA, T_BRIDGE, T_WATER, T_MARSH, T_FIELD,
	T_TREE, T_WALL, T_FLOOR, T_DOOR
};

static uint8_t terr[MAP_H * MAP_W];

static uint64_t rng_s = 0x11A6E5EEDULL;

static uint64_t rng_next(void)
{
	uint64_t z = (rng_s += 0x9e3779b97f4a7c15ULL);

	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
	z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
	return z ^ (z >> 31);
}

static uint32_t rng_below(uint32_t bound)
{
	return (uint32_t)(rng_next() % bound);
}

static void t_rect(uint8_t t, uint32_t x, uint32_t y, uint32_t w,
	uint32_t h)
{
	for (uint32_t yy = y; yy < y + h && yy < MAP_H; yy++) {
		for (uint32_t xx = x; xx < x + w && xx < MAP_W; xx++) {
			terr[yy * MAP_W + xx] = t;
		}
	}
}

/* Buildings: perimeter walls, floor inside, one door tile. */
typedef struct building {
	uint32_t x, y, w, h;
	uint32_t door_x, door_y;
} building;

enum { MAX_BUILDINGS = 48 };
static building buildings[MAX_BUILDINGS];
static uint32_t n_buildings;

/* door_side: 0 bottom, 1 top, 2 left, 3 right */
static uint32_t add_building(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
	int door_side)
{
	building *b = &buildings[n_buildings];

	t_rect(T_WALL, x, y, w, h);
	t_rect(T_FLOOR, x + 1, y + 1, w - 2, h - 2);
	b->x = x;
	b->y = y;
	b->w = w;
	b->h = h;
	switch (door_side) {
	case 0: b->door_x = x + w / 2; b->door_y = y + h - 1; break;
	case 1: b->door_x = x + w / 2; b->door_y = y; break;
	case 2: b->door_x = x; b->door_y = y + h / 2; break;
	default: b->door_x = x + w - 1; b->door_y = y + h / 2; break;
	}
	terr[b->door_y * MAP_W + b->door_x] = T_DOOR;
	return n_buildings++;
}

/* interior spot k (deterministic, spread over the floor area) */
static void interior_spot(const building *b, uint32_t k, uint32_t *sx,
	uint32_t *sy)
{
	uint32_t iw = b->w - 2;
	uint32_t ih = b->h - 2;

	*sx = b->x + 1 + k % iw;
	*sy = b->y + 1 + (k / iw) % ih;
}

static int8_t river_off[MAP_H];

static void build_world(void)
{
	int8_t off = 0;

	memset(terr, T_GRASS, sizeof(terr));

	/* forest fringes */
	for (uint32_t y = 0; y < MAP_H; y++) {
		for (uint32_t x = 0; x < MAP_W; x++) {
			if ((y < 7 || x < 5 || y > 66) && rng_below(5) == 0) {
				terr[y * MAP_W + x] = T_TREE;
			}
		}
	}
	/* fields (tall grass) */
	t_rect(T_FIELD, 112, 8, 15, 20);  /* farm A, NE */
	t_rect(T_FIELD, 112, 40, 15, 11); /* farm B, E  */
	t_rect(T_FIELD, 6, 48, 13, 17);   /* farm C, SW */

	/* river with wobble, plus marshy banks */
	for (uint32_t y = 0; y < MAP_H; y++) {
		if (y % 3 == 0) {
			int8_t d = (int8_t)(rng_below(3)) - 1;

			off = (int8_t)(off + d);
			if (off < -3) off = -3;
			if (off > 3) off = 3;
		}
		river_off[y] = off;
		for (int32_t x = 99 + off; x < 103 + off; x++) {
			terr[y * MAP_W + x] = T_WATER;
		}
		for (int32_t x = 96 + off; x < 99 + off; x++) {
			if (rng_below(3) != 0) {
				terr[y * MAP_W + x] = T_MARSH;
			}
		}
		for (int32_t x = 103 + off; x < 106 + off; x++) {
			if (rng_below(3) != 0) {
				terr[y * MAP_W + x] = T_MARSH;
			}
		}
	}

	/* roads (turn river tiles under the main road into the bridge) */
	for (uint32_t x = 2; x < 127; x++) {
		for (uint32_t y = 35; y <= 36; y++) {
			terr[y * MAP_W + x] =
				terr[y * MAP_W + x] == T_WATER ? T_BRIDGE :
				T_ROAD;
		}
	}
	for (uint32_t y = 2; y < 70; y++) {
		terr[y * MAP_W + 48] = T_ROAD;
		terr[y * MAP_W + 49] = T_ROAD;
	}
	for (uint32_t x = 8; x < 96; x++) {
		terr[22 * MAP_W + x] = T_ROAD;
		terr[50 * MAP_W + x] = T_ROAD;
	}
	for (uint32_t y = 22; y <= 50; y++) {
		terr[y * MAP_W + 16] = T_ROAD;
		terr[y * MAP_W + 82] = T_ROAD;
	}

	/* plaza around the crossroads */
	for (uint32_t y = 30; y <= 41; y++) {
		for (uint32_t x = 42; x <= 58; x++) {
			uint8_t *t = &terr[y * MAP_W + x];

			if (*t == T_GRASS || *t == T_ROAD) {
				*t = T_PLAZA;
			}
		}
	}
}

/* ------------------------------------------------------------------ */
/* NPCs & schedules                                                    */
/* ------------------------------------------------------------------ */

enum { PROF_TOWNSFOLK = 0, PROF_FARMER = 1 };

typedef struct dest {
	uint32_t gx, gy; /* pathfinding goal (a door, or the spot itself) */
	uint32_t sx, sy; /* final spot (== goal when no interior hop)     */
} dest;

typedef struct npc {
	uint32_t x, y;
	uint32_t profile;
	dest home, work, lunch, evening;
	int has_lunch, evening_kind; /* 0 home, 1 tavern, 2 plaza */
	/* active path */
	uint32_t *steps; /* x,y pairs */
	float *ticks;
	uint32_t len, at;
	float rem;
	/* pending second leg after reaching a door */
	int leg2;
	dest target;
} npc;

static npc npcs[N_NPC];

/* batch stats */
static uint64_t st_batches, st_reqs, st_field_answered, st_errors;
static double st_ms_total;
static uint32_t moving_count;

static derech_map *dmap;

static dest door_dest(uint32_t b, uint32_t spot)
{
	dest d;

	d.gx = buildings[b].door_x;
	d.gy = buildings[b].door_y;
	interior_spot(&buildings[b], spot, &d.sx, &d.sy);
	return d;
}

static dest tile_dest(uint32_t x, uint32_t y)
{
	dest d = { x, y, x, y };

	return d;
}

/* field work spots: a walkable field tile near the given corner */
static dest field_dest(uint32_t x0, uint32_t y0, uint32_t w, uint32_t h)
{
	for (;;) {
		uint32_t x = x0 + rng_below(w);
		uint32_t y = y0 + rng_below(h);

		if (terr[y * MAP_W + x] == T_FIELD) {
			return tile_dest(x, y);
		}
	}
}

static uint32_t plaza_tiles[512][2];
static uint32_t n_plaza_tiles;

static dest plaza_dest(void)
{
	uint32_t k = rng_below(n_plaza_tiles);

	return tile_dest(plaza_tiles[k][0], plaza_tiles[k][1]);
}

static uint32_t bld_tavern;

static void setup_npcs(void)
{
	/* residential rows: {x list} x {y, door_side} */
	static const uint32_t hx[9] = { 8, 15, 22, 29, 36, 54, 61, 68, 75 };
	uint32_t homes[36];
	uint32_t nh = 0;
	uint32_t work_i = 0;
	uint32_t bld_bakery, bld_smithy, bld_church, bld_mill, bld_farmhouse;

	for (uint32_t i = 0; i < 9; i++) {
		homes[nh++] = add_building(hx[i], 16, 6, 5, 0); /* door S */
	}
	for (uint32_t i = 0; i < 5; i++) {
		/* second north row only west of the crossroads: the east
		 * side belongs to the tavern */
		homes[nh++] = add_building(hx[i], 24, 6, 5, 1); /* door N */
	}
	for (uint32_t i = 0; i < 9; i++) {
		homes[nh++] = add_building(hx[i], 43, 6, 5, 0);
	}
	for (uint32_t i = 0; i < 9; i++) {
		homes[nh++] = add_building(hx[i], 52, 6, 5, 1);
	}

	bld_bakery = add_building(34, 30, 8, 5, 3);    /* door E -> plaza */
	bld_tavern = add_building(60, 24, 10, 7, 2);   /* door W          */
	bld_smithy = add_building(86, 40, 8, 6, 2);    /* door W -> road  */
	bld_church = add_building(52, 6, 10, 9, 2);    /* door W -> road  */
	bld_mill = add_building(110, 24, 8, 6, 0);     /* door S -> road  */
	bld_farmhouse = add_building(112, 54, 7, 5, 1);

	/* market stall counters on the plaza */
	terr[32 * MAP_W + 44] = T_WALL;
	terr[39 * MAP_W + 44] = T_WALL;
	terr[32 * MAP_W + 55] = T_WALL;
	terr[39 * MAP_W + 55] = T_WALL;

	for (uint32_t y = 30; y <= 41; y++) {
		for (uint32_t x = 42; x <= 58; x++) {
			if (terr[y * MAP_W + x] == T_PLAZA &&
				n_plaza_tiles < 512) {
				plaza_tiles[n_plaza_tiles][0] = x;
				plaza_tiles[n_plaza_tiles][1] = y;
				n_plaza_tiles++;
			}
		}
	}

	for (uint32_t i = 0; i < N_NPC; i++) {
		npc *n = &npcs[i];
		uint32_t hb = homes[i / 4];

		memset(n, 0, sizeof(*n));
		n->home = door_dest(hb, 1 + (i % 4) * 3);
		n->x = n->home.sx;
		n->y = n->home.sy;

		/* jobs: 42 farmers (fields + mill), 58 townsfolk */
		if (work_i < 12) {
			n->work = field_dest(113, 9, 13, 18);
		} else if (work_i < 24) {
			n->work = field_dest(113, 41, 13, 9);
		} else if (work_i < 34) {
			n->work = field_dest(7, 49, 11, 15);
		} else if (work_i < 42) {
			n->work = door_dest(bld_mill, work_i - 34);
		} else if (work_i < 50) {
			n->work = door_dest(bld_bakery, work_i - 42);
		} else if (work_i < 58) {
			n->work = door_dest(bld_smithy, work_i - 50);
		} else if (work_i < 64) {
			n->work = door_dest(bld_tavern, work_i - 58);
		} else if (work_i < 68) {
			n->work = door_dest(bld_church, work_i - 64);
		} else if (work_i < 84) {
			/* market: spots beside the four stall counters */
			static const uint32_t stall[4][2] = { { 44, 32 },
				{ 44, 39 }, { 55, 32 }, { 55, 39 } };
			uint32_t s = (work_i - 68) / 4;
			uint32_t k = (work_i - 68) % 4;

			n->work = tile_dest(stall[s][0] + 1 + k % 2,
				stall[s][1] + (k / 2 == 0 ? 1 : -1) *
				(k % 2));
			if (terr[n->work.gy * MAP_W + n->work.gx] !=
				T_PLAZA) {
				n->work = tile_dest(stall[s][0] + 1,
					stall[s][1] + 1);
			}
		} else {
			n->work = plaza_dest(); /* traders & strollers */
		}
		n->profile = work_i < 42 ? PROF_FARMER : PROF_TOWNSFOLK;
		work_i++;

		n->has_lunch = i % 2 == 0;
		n->lunch = door_dest(bld_tavern, 2 + i % 30);
		n->evening_kind = i % 10 < 4 ? 1 : (i % 10 < 6 ? 2 : 0);
		n->evening = n->evening_kind == 1 ?
			door_dest(bld_tavern, 2 + (i * 7) % 30) :
			plaza_dest();
	}
	(void)bld_farmhouse;
}

/* schedule: returns 1 and fills *d when this NPC changes target at
 * day-minute m */
static int schedule_event(const npc *n, uint32_t i, uint32_t m, dest *d)
{
	/* building crews leave in lockstep (same door goal, same minute:
	 * they batch together and derech answers them from one field);
	 * field and plaza workers have individual goals, so they trickle */
	int crew = n->work.sx != n->work.gx || n->work.sy != n->work.gy;
	uint32_t leave = (n->profile == PROF_FARMER ? 300 : 420) +
		(crew ? (n->work.gx * 13 + n->work.gy * 7) % 40 :
			(i * 13) % 45);
	uint32_t lunch_out = 715 + (i % 4) * 6; /* four tavern waves */
	uint32_t lunch_back = 800 + (i % 4) * 6;
	uint32_t evening = n->evening_kind == 1 ? 1050 + (i % 5) * 7 :
		1050 + (i * 11) % 40;
	uint32_t home = n->evening_kind == 1 ? 1260 + i % 30 :
		(n->evening_kind == 2 ? 1200 + i % 30 : evening);

	if (m == leave) {
		*d = n->work;
		return 1;
	}
	if (n->has_lunch && m == lunch_out) {
		*d = n->lunch;
		return 1;
	}
	if (n->has_lunch && m == lunch_back) {
		*d = n->work;
		return 1;
	}
	if (n->evening_kind != 0 && m == evening) {
		*d = n->evening;
		return 1;
	}
	if (m == home) {
		*d = n->home;
		return 1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* derech glue                                                         */
/* ------------------------------------------------------------------ */

static double now_ms(void);

static void setup_derech(void)
{
	static float pass[MAP_H * MAP_W];
	static uint64_t tags[MAP_H * MAP_W];
	derech_profile_desc d;

	for (uint32_t i = 0; i < MAP_H * MAP_W; i++) {
		switch (terr[i]) {
		case T_GRASS: pass[i] = 0.8f; tags[i] = 0; break;
		case T_FIELD: pass[i] = 0.4f; tags[i] = TAG_FIELD; break;
		case T_MARSH: pass[i] = 0.2f; tags[i] = TAG_MARSH; break;
		case T_ROAD: case T_PLAZA: case T_BRIDGE:
			pass[i] = 1.0f; tags[i] = TAG_ROAD; break;
		case T_FLOOR: case T_DOOR:
			pass[i] = 1.0f; tags[i] = 0; break;
		default: pass[i] = 0.0f; tags[i] = 0; break; /* blocked */
		}
	}
	dmap = derech_map_create(MAP_W, MAP_H, NULL);
	if (dmap == NULL ||
		derech_map_set_passability(dmap, pass,
			MAP_H * MAP_W) != DERECH_OK ||
		derech_map_set_tags(dmap, tags, MAP_H * MAP_W) != DERECH_OK) {
		fprintf(stderr, "derech map setup failed\n");
		exit(1);
	}

	memset(&d, 0, sizeof(d));
	d.struct_size = (uint32_t)sizeof(d);
	d.tag_mult[0] = 0.8f;  /* townsfolk like roads    */
	d.tag_add[1] = 1.0f;   /* ...dislike fields       */
	d.tag_add[2] = 4.0f;   /* ...really dislike marsh */
	if (derech_profile_register(dmap, &d) != PROF_TOWNSFOLK) {
		exit(1);
	}
	memset(&d, 0, sizeof(d));
	d.struct_size = (uint32_t)sizeof(d);
	d.tag_mult[0] = 0.9f;
	d.tag_add[2] = 1.0f;
	if (derech_profile_register(dmap, &d) != PROF_FARMER) {
		exit(1);
	}
}

static void npc_clear_path(npc *n)
{
	free(n->steps);
	free(n->ticks);
	n->steps = NULL;
	n->ticks = NULL;
	n->len = n->at = 0;
	n->rem = 0.0f;
}

static void npc_adopt_path(npc *n, const derech_results *res, uint32_t i)
{
	uint32_t len = derech_result_length(res, i);

	npc_clear_path(n);
	if (len == 0) {
		return; /* already there */
	}
	n->steps = malloc((size_t)len * 2 * sizeof(*n->steps));
	n->ticks = malloc((size_t)len * sizeof(*n->ticks));
	if (n->steps == NULL || n->ticks == NULL) {
		npc_clear_path(n);
		st_errors++;
		return;
	}
	memcpy(n->steps, derech_result_steps(res, i),
		(size_t)len * 2 * sizeof(*n->steps));
	memcpy(n->ticks, derech_result_step_ticks(res, i),
		(size_t)len * sizeof(*n->ticks));
	n->len = len;
	n->at = 0;
	n->rem = n->ticks[0];
}

/* one simulated minute */
static void sim_tick(uint64_t tick)
{
	uint32_t m = (uint32_t)(tick % DAY_MIN);
	uint32_t queue[N_NPC];
	uint32_t nq = 0;
	dest d;

	/* schedule changes -> new leg-1 targets */
	for (uint32_t i = 0; i < N_NPC; i++) {
		npc *n = &npcs[i];

		if (schedule_event(n, i, m, &d)) {
			npc_clear_path(n);
			n->target = d;
			n->leg2 = d.sx != d.gx || d.sy != d.gy;
			if (n->x == d.gx && n->y == d.gy) {
				/* already at the door: only the hop */
				n->target.gx = d.sx;
				n->target.gy = d.sy;
				n->leg2 = 0;
			}
			queue[nq++] = i;
		}
	}

	/* movement */
	moving_count = 0;
	for (uint32_t i = 0; i < N_NPC; i++) {
		npc *n = &npcs[i];

		if (n->len == 0) {
			continue;
		}
		moving_count++;
		n->rem -= 1.0f;
		while (n->rem <= 0.0f && n->at < n->len) {
			n->x = n->steps[n->at * 2];
			n->y = n->steps[n->at * 2 + 1];
			n->at++;
			if (n->at < n->len) {
				n->rem += n->ticks[n->at];
			}
		}
		if (n->at >= n->len) {
			npc_clear_path(n);
			if (n->leg2) {
				n->leg2 = 0;
				n->target.gx = n->target.sx;
				n->target.gy = n->target.sy;
				queue[nq++] = i;
			}
		}
	}

	/* one batch for everything that changed this minute */
	if (nq > 0) {
		derech_request reqs[N_NPC * 2];
		derech_results *res = NULL;
		double t0 = now_ms();

		for (uint32_t k = 0; k < nq; k++) {
			npc *n = &npcs[queue[k]];

			memset(&reqs[k], 0, sizeof(reqs[k]));
			reqs[k].struct_size = sizeof(reqs[k]);
			reqs[k].start_x = n->x;
			reqs[k].start_y = n->y;
			reqs[k].goal_x = n->target.gx;
			reqs[k].goal_y = n->target.gy;
			reqs[k].profile_id = n->profile;
		}
		if (derech_find_paths(dmap, reqs, nq, &res) != DERECH_OK) {
			st_errors += nq;
			return;
		}
		st_ms_total += now_ms() - t0;
		st_batches++;
		st_reqs += nq;
		for (uint32_t k = 0; k < nq; k++) {
			npc *n = &npcs[queue[k]];

			if (derech_result_status(res, k) !=
				DERECH_PATH_FOUND) {
				st_errors++;
				continue;
			}
			if (derech_result_length(res, k) > 0 &&
				derech_result_expansions(res, k) == 0) {
				st_field_answered++;
			}
			npc_adopt_path(n, res, k);
			if (n->len == 0 && n->leg2) {
				/* arrived instantly; queue the hop next
				 * minute via a tiny self-request */
				n->leg2 = 0;
				n->target.gx = n->target.sx;
				n->target.gy = n->target.sy;
			}
		}
		derech_results_destroy(res);
	}
}

/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */

#include <time.h>

static double now_ms(void)
{
	struct timespec ts;

	timespec_get(&ts, TIME_UTC);
	return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

/* ------------------------------------------------------------------ */
/* Selftest                                                            */
/* ------------------------------------------------------------------ */

static int selftest(void)
{
	int ok = 1;

	for (uint64_t tick = 0; tick < 3ull * DAY_MIN; tick++) {
		sim_tick(tick);
		if (tick % DAY_MIN == 240 && tick > DAY_MIN) {
			uint32_t at_home = 0;

			for (uint32_t i = 0; i < N_NPC; i++) {
				if (npcs[i].x == npcs[i].home.sx &&
					npcs[i].y == npcs[i].home.sy) {
					at_home++;
				}
			}
			printf("day %" PRIu64 " 04:00 — %u/%u in bed\n",
				tick / DAY_MIN, at_home, N_NPC);
			if (at_home != N_NPC) {
				ok = 0;
			}
		}
	}
	printf("batches %" PRIu64 ", requests %" PRIu64 ", field-answered %"
		PRIu64 ", errors %" PRIu64 ", pathfind total %.1f ms\n",
		st_batches, st_reqs, st_field_answered, st_errors,
		st_ms_total);
	if (st_errors != 0 || st_field_answered == 0) {
		ok = 0;
	}
	printf("selftest: %s\n", ok ? "OK" : "FAILED");
	return ok ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* Rendering                                                           */
/* ------------------------------------------------------------------ */

#ifndef DEMO_SELFTEST_ONLY

typedef struct cell_style {
	char ch;
	uint8_t fr, fg, fb, br, bg, bb;
} cell_style;

static const cell_style STYLE[11] = {
	[T_GRASS] = { ',', 84, 122, 66, 26, 36, 24 },
	[T_ROAD] = { '.', 152, 142, 120, 62, 58, 50 },
	[T_PLAZA] = { '.', 168, 158, 134, 76, 70, 58 },
	[T_BRIDGE] = { '=', 170, 130, 80, 60, 48, 34 },
	[T_WATER] = { '~', 70, 130, 220, 16, 44, 92 },
	[T_MARSH] = { '~', 96, 140, 96, 30, 48, 34 },
	[T_FIELD] = { '"', 168, 152, 70, 44, 48, 22 },
	[T_TREE] = { 'T', 42, 130, 52, 20, 32, 20 },
	[T_WALL] = { '#', 205, 195, 175, 74, 64, 52 },
	[T_FLOOR] = { ' ', 0, 0, 0, 56, 48, 40 },
	[T_DOOR] = { '+', 224, 176, 96, 74, 64, 52 },
};

static uint8_t dim(uint8_t v, float f)
{
	return (uint8_t)((float)v * f);
}

/* daylight factor: full day 07-18h, dark night, dawn/dusk ramps */
static float daylight_factor(uint32_t m)
{
	float h = (float)m / 60.0f;

	if (h >= 7.0f && h <= 18.0f) {
		return 1.0f;
	}
	if (h > 18.0f && h < 21.0f) {
		return 1.0f - (h - 18.0f) / 3.0f * 0.55f;
	}
	if (h >= 5.0f && h < 7.0f) {
		return 0.45f + (h - 5.0f) / 2.0f * 0.55f;
	}
	return 0.45f;
}

static void draw(struct notcurses *nc, struct ncplane *std, uint64_t tick,
	int32_t vx, int32_t vy, int paused, int speed, int follow)
{
	uint32_t rows, cols;
	uint32_t m = (uint32_t)(tick % DAY_MIN);
	float dl = daylight_factor(m);
	char hud[256];

	ncplane_dim_yx(std, &rows, &cols);
	ncplane_erase(std);

	for (uint32_t ry = 0; ry + 2 < rows; ry++) {
		int32_t my = vy + (int32_t)ry;

		if (my < 0 || my >= MAP_H) {
			continue;
		}
		for (uint32_t rx = 0; rx < cols; rx++) {
			int32_t mx = vx + (int32_t)rx;
			const cell_style *s;

			if (mx < 0 || mx >= MAP_W) {
				continue;
			}
			s = &STYLE[terr[my * MAP_W + mx]];
			ncplane_set_fg_rgb8(std, dim(s->fr, dl),
				dim(s->fg, dl), dim(s->fb, dl));
			ncplane_set_bg_rgb8(std, dim(s->br, dl),
				dim(s->bg, dl), dim(s->bb, dl));
			ncplane_putchar_yx(std, (int)ry + 2, (int)rx, s->ch);
		}
	}

	/* the followed NPC's remaining route */
	if (follow >= 0 && npcs[follow].len > 0) {
		const npc *n = &npcs[follow];

		for (uint32_t k = n->at; k < n->len; k++) {
			int32_t sx = (int32_t)n->steps[k * 2] - vx;
			int32_t sy = (int32_t)n->steps[k * 2 + 1] - vy;

			if (sx >= 0 && sx < (int32_t)cols && sy >= 0 &&
				sy + 2 < (int32_t)rows) {
				ncplane_set_fg_rgb8(std, 255, 255, 200);
				ncplane_set_bg_rgb8(std, 90, 70, 20);
				ncplane_putchar_yx(std, sy + 2, sx, 'o');
			}
		}
	}

	for (uint32_t i = 0; i < N_NPC; i++) {
		int32_t sx = (int32_t)npcs[i].x - vx;
		int32_t sy = (int32_t)npcs[i].y - vy;

		if (sx < 0 || sx >= (int32_t)cols || sy < 0 ||
			sy + 2 >= (int32_t)rows) {
			continue;
		}
		if ((int)i == follow) {
			ncplane_set_fg_rgb8(std, 255, 90, 90);
		} else if (npcs[i].profile == PROF_FARMER) {
			ncplane_set_fg_rgb8(std, dim(140, dl),
				dim(235, dl), dim(140, dl));
		} else {
			ncplane_set_fg_rgb8(std, dim(255, dl),
				dim(220, dl), dim(90, dl));
		}
		ncplane_set_bg_rgb8(std, dim(40, dl), dim(40, dl),
			dim(34, dl));
		ncplane_putchar_yx(std, sy + 2, sx, '@');
	}

	ncplane_set_fg_rgb8(std, 230, 230, 230);
	ncplane_set_bg_rgb8(std, 20, 20, 24);
	snprintf(hud, sizeof(hud),
		" derech village | day %" PRIu64 " %02u:%02u %s x%d | "
		"moving %u/%u%s%s",
		tick / DAY_MIN + 1, m / 60, m % 60,
		paused ? "PAUSED" : "     ", speed, moving_count, N_NPC,
		follow >= 0 ? " | following #" : "",
		follow >= 0 ? (npcs[follow].profile == PROF_FARMER ?
			"farmer" : "townsfolk") : "");
	ncplane_putstr_yx(std, 0, 0, hud);
	snprintf(hud, sizeof(hud),
		" batches %" PRIu64 " reqs %" PRIu64 " field-answered %"
		PRIu64 " err %" PRIu64 " avg %.2fms | arrows scroll  f/TAB "
		"follow  SPACE pause  1-3 speed  q quit",
		st_batches, st_reqs, st_field_answered, st_errors,
		st_batches > 0 ? st_ms_total / (double)st_batches : 0.0);
	ncplane_putstr_yx(std, 1, 0, hud);

	notcurses_render(nc);
}

static int pick_moving_npc(int after)
{
	for (int k = 1; k <= N_NPC; k++) {
		int i = (after + k) % N_NPC;

		if (npcs[i].len > 0) {
			return i;
		}
	}
	return -1;
}

static int run_ui(int demo_seconds)
{
	struct notcurses_options opts;
	struct notcurses *nc;
	struct ncplane *std;
	uint64_t tick = 290; /* 04:50 — the farmers stir within moments */
	double t_start = now_ms();
	double acc = 0.0;
	int32_t vx = 48 - 40, vy = 34 - 20;
	int paused = 0, speed = 1, follow = -1;
	static const double tpf[4] = { 0.0, 0.5, 2.0, 8.0 };

	memset(&opts, 0, sizeof(opts));
	opts.flags = NCOPTION_SUPPRESS_BANNERS;
	nc = notcurses_core_init(&opts, NULL);
	if (nc == NULL) {
		fprintf(stderr, "notcurses_init failed (need a tty)\n");
		return 1;
	}
	std = notcurses_stdplane(nc);

	for (;;) {
		struct timespec pace = { 0, 50 * 1000 * 1000 };
		struct pollfd pfd;
		ncinput ni;
		uint32_t key;
		int quit = 0;

		/* Input: gate on the input fd's readiness, then drain with
		 * short-timeout gets.  Never call notcurses_get with a NULL
		 * timespec — terminal chatter (e.g. kitty focus events on
		 * tab-away) can signal readiness without a dequeueable user
		 * event, and a NULL get then blocks the whole UI. */
		pfd.fd = notcurses_inputready_fd(nc);
		pfd.events = POLLIN;
		pfd.revents = 0;
		if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN) != 0) {
			for (;;) {
				struct timespec tiny = { 0, 1 * 1000 * 1000 };

				key = notcurses_get(nc, &tiny, &ni);
				if (key == 0) {
					break;
				}
				if (key == (uint32_t)-1) {
					quit = 1;
					break;
				}
				/* kitty-protocol terminals report releases
				 * and repeats distinctly; act on presses */
				if (ni.evtype == NCTYPE_RELEASE) {
					continue;
				}
				if (key == 'q' || key == 'Q') {
					quit = 1;
					break;
				}
				switch (key) {
				case NCKEY_LEFT: case 'a':
					vx -= 4; follow = -1; break;
				case NCKEY_RIGHT: case 'd':
					vx += 4; follow = -1; break;
				case NCKEY_UP: case 'w':
					vy -= 2; follow = -1; break;
				case NCKEY_DOWN: case 's':
					vy += 2; follow = -1; break;
				case ' ': paused = !paused; break;
				case '1': speed = 1; break;
				case '2': speed = 2; break;
				case '3': speed = 3; break;
				case 'f': case NCKEY_TAB:
					follow = pick_moving_npc(follow < 0 ?
						0 : follow);
					break;
				default: break;
				}
			}
		}
		if (quit || (demo_seconds > 0 &&
			now_ms() - t_start > demo_seconds * 1000.0)) {
			break;
		}

		if (!paused) {
			acc += tpf[speed];
			while (acc >= 1.0) {
				sim_tick(tick++);
				acc -= 1.0;
			}
		}
		if (follow >= 0) {
			uint32_t rows, cols;

			ncplane_dim_yx(std, &rows, &cols);
			vx = (int32_t)npcs[follow].x - (int32_t)cols / 2;
			vy = (int32_t)npcs[follow].y -
				(int32_t)(rows - 2) / 2;
		}
		{
			uint32_t rows, cols;

			ncplane_dim_yx(std, &rows, &cols);
			if (vx > MAP_W - (int32_t)cols) {
				vx = MAP_W - (int32_t)cols;
			}
			if (vy > MAP_H - (int32_t)rows + 2) {
				vy = MAP_H - (int32_t)rows + 2;
			}
			if (vx < 0) {
				vx = 0;
			}
			if (vy < 0) {
				vy = 0;
			}
		}
		draw(nc, std, tick, vx, vy, paused, speed, follow);
		nanosleep(&pace, NULL);
	}
	notcurses_stop(nc);
	printf("village session: day %" PRIu64 " %02u:%02u | batches %"
		PRIu64 ", requests %" PRIu64 ", field-answered %" PRIu64
		", errors %" PRIu64 ", pathfind %.1f ms total\n",
		tick / DAY_MIN + 1, (uint32_t)(tick % DAY_MIN) / 60,
		(uint32_t)(tick % DAY_MIN) % 60, st_batches, st_reqs,
		st_field_answered, st_errors, st_ms_total);
	return 0;
}

#endif /* !DEMO_SELFTEST_ONLY */

int main(int argc, char **argv)
{
#if !defined(DEMO_SELFTEST_ONLY) && !defined(_WIN32)
	/* a vendored ncurses may have a baked-in terminfo path from its
	 * build machine, and a terminal-specific $TERMINFO (kitty's, say)
	 * only covers its own entry.  ncurses searches $TERMINFO first and
	 * $TERMINFO_DIRS after it, so providing system fallback dirs here
	 * is always safe and rescues both cases (e.g. running under tmux
	 * inside kitty). */
	setenv("TERMINFO_DIRS",
		"/usr/share/terminfo:/etc/terminfo:/lib/terminfo:"
		"/usr/lib/terminfo", 0);
#endif

	build_world();
	setup_npcs();
	setup_derech();

	if (argc > 1 && strcmp(argv[1], "--selftest") == 0) {
		return selftest();
	}
#ifdef DEMO_SELFTEST_ONLY
	fprintf(stderr, "built selftest-only; run with --selftest\n");
	return 1;
#else
	{
		int demo_seconds = 0;

		if (argc > 2 && strcmp(argv[1], "--demo-seconds") == 0) {
			demo_seconds = atoi(argv[2]);
		}
		return run_ui(demo_seconds);
	}
#endif
}
