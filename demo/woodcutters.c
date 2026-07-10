#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif

/* derech woodcutters demo — discovery, goal sets, and the dirty-region
 * cache in one loop.
 *
 * A tribe camp sits in the south-west under fog of war.  Twelve
 * woodcutters must FIND the forest before they can work it:
 *
 *   - the demo keeps the tribe's knowledge as an EXPLORED tag bit,
 *     revealed in batched writes as NPCs walk (dirty-region
 *     invalidation keeps those writes from disturbing cached fields);
 *   - "known trees" is a derech predicate goal set — tiles tagged
 *     TREE|EXPLORED, ADJACENT since trunks are impassable — so tribal
 *     knowledge IS the query, and it updates as trees are discovered
 *     and felled;
 *   - with no known trees, an NPC samples frontier tiles (explored
 *     tiles bordering fog) and paths to the nearest via an explicit
 *     goal set — walking there widens the fog;
 *   - chopping clears the TREE tag and opens the stump; hauling the
 *     log home converges every carrier on the camp's cached field.
 *
 * Pathfinding is deliberately omniscient (routes may cross fog);
 * knowledge gates only WHAT they target — the usual game compromise.
 *
 * Controls: arrows/WASD scroll · f/TAB follow · SPACE pause · 1/2/3
 * speed · q quit.  --selftest runs headless and asserts the economy
 * works; --demo-seconds N auto-quits with a stats summary. */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "derech.h"

#ifndef DEMO_SELFTEST_ONLY
#include <notcurses/notcurses.h>
#include <poll.h>
#endif

enum { MAP_W = 128, MAP_H = 72, N_NPC = 12 };

enum {
	TAG_TREE = 1u << 0,
	TAG_EXPLORED = 1u << 1
};

enum {
	T_GRASS = 0, T_BRUSH, T_ROCK, T_TREE, T_STUMP, T_CAMP
};

enum { CAMP_X = 14, CAMP_Y = 62 };

static uint8_t terr[MAP_H * MAP_W];
static uint64_t tags[MAP_H * MAP_W];   /* demo-side source of truth */
static uint8_t explored[MAP_H * MAP_W];
static uint8_t claimed[MAP_H * MAP_W];
static uint8_t reveal_pending[MAP_H * MAP_W];
static int have_pending_reveals;

static derech_map *dmap;
static int32_t set_known_trees;

static uint64_t rng_s = 0xF0EE57ULL;

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

/* ------------------------------------------------------------------ */
/* World                                                               */
/* ------------------------------------------------------------------ */

static float terr_pass(uint8_t t)
{
	switch (t) {
	case T_GRASS: return 0.8f;
	case T_BRUSH: return 0.4f;
	case T_STUMP: return 0.7f;
	case T_CAMP: return 1.0f;
	default: return 0.0f; /* rock, standing tree */
	}
}

static void plant_tree(uint32_t x, uint32_t y)
{
	uint32_t i = y * MAP_W + x;

	if (terr[i] == T_GRASS || terr[i] == T_BRUSH) {
		terr[i] = T_TREE;
		tags[i] |= TAG_TREE;
	}
}

static void build_world(void)
{
	memset(terr, T_GRASS, sizeof(terr));
	memset(tags, 0, sizeof(tags));

	/* brush blobs and rocks */
	for (uint32_t k = 0; k < 40; k++) {
		uint32_t cx = rng_below(MAP_W);
		uint32_t cy = rng_below(MAP_H);
		uint32_t r = 2 + rng_below(4);

		for (uint32_t y = cy > r ? cy - r : 0;
			y <= cy + r && y < MAP_H; y++) {
			for (uint32_t x = cx > r ? cx - r : 0;
				x <= cx + r && x < MAP_W; x++) {
				if (rng_below(3) != 0) {
					terr[y * MAP_W + x] = T_BRUSH;
				}
			}
		}
	}
	for (uint32_t i = 0; i < MAP_H * MAP_W; i++) {
		if (rng_below(40) == 0) {
			terr[i] = T_ROCK;
		}
	}

	/* the big north-east forest */
	for (uint32_t y = 4; y < 36; y++) {
		for (uint32_t x = 76; x < 124; x++) {
			if (rng_below(10) < 5) {
				plant_tree(x, y);
			}
		}
	}
	/* two outlying groves */
	for (uint32_t k = 0; k < 18; k++) {
		plant_tree(96 + rng_below(20), 52 + rng_below(14));
	}
	for (uint32_t k = 0; k < 14; k++) {
		plant_tree(6 + rng_below(18), 6 + rng_below(10));
	}

	/* camp clearing */
	for (uint32_t y = CAMP_Y - 4; y <= CAMP_Y + 4; y++) {
		for (uint32_t x = CAMP_X - 6; x <= CAMP_X + 6; x++) {
			terr[y * MAP_W + x] = T_CAMP;
			tags[y * MAP_W + x] &= ~(uint64_t)TAG_TREE;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Knowledge: fog, reveals, frontier                                   */
/* ------------------------------------------------------------------ */

static void reveal_disc(uint32_t cx, uint32_t cy, uint32_t r)
{
	for (int64_t y = (int64_t)cy - r; y <= (int64_t)cy + r; y++) {
		for (int64_t x = (int64_t)cx - r; x <= (int64_t)cx + r; x++) {
			uint32_t i;

			if (x < 0 || y < 0 || x >= MAP_W || y >= MAP_H) {
				continue;
			}
			if ((x - cx) * (x - cx) + (y - cy) * (y - cy) >
				(int64_t)r * r) {
				continue;
			}
			i = (uint32_t)y * MAP_W + (uint32_t)x;
			if (!explored[i]) {
				explored[i] = 1;
				reveal_pending[i] = 1;
				have_pending_reveals = 1;
			}
		}
	}
}

/* Push pending EXPLORED bits into derech as one bounding-rect write —
 * the batched-reveal pattern: tag edits derech's profiles don't weigh,
 * so cached fields survive them (only known-tree membership reacts). */
static uint64_t st_reveal_writes;

static void flush_reveals(void)
{
	uint32_t x0 = MAP_W, y0 = MAP_H, x1 = 0, y1 = 0;
	static uint64_t buf[MAP_H * MAP_W];

	if (!have_pending_reveals) {
		return;
	}
	for (uint32_t y = 0; y < MAP_H; y++) {
		for (uint32_t x = 0; x < MAP_W; x++) {
			if (reveal_pending[y * MAP_W + x]) {
				if (x < x0) x0 = x;
				if (y < y0) y0 = y;
				if (x > x1) x1 = x;
				if (y > y1) y1 = y;
				tags[y * MAP_W + x] |= TAG_EXPLORED;
			}
		}
	}
	for (uint32_t y = y0; y <= y1; y++) {
		for (uint32_t x = x0; x <= x1; x++) {
			buf[(y - y0) * (x1 - x0 + 1) + (x - x0)] =
				tags[y * MAP_W + x];
		}
	}
	if (derech_map_set_tags_rect(dmap, x0, y0, x1 - x0 + 1, y1 - y0 + 1,
		buf) == DERECH_OK) {
		st_reveal_writes++;
	}
	memset(reveal_pending, 0, sizeof(reveal_pending));
	have_pending_reveals = 0;
}

static int is_frontier(uint32_t i)
{
	uint32_t x = i % MAP_W;
	uint32_t y = i / MAP_W;

	if (!explored[i] || terr_pass(terr[i]) <= 0.0f) {
		return 0;
	}
	for (int64_t dy = -1; dy <= 1; dy++) {
		for (int64_t dx = -1; dx <= 1; dx++) {
			int64_t nx = (int64_t)x + dx;
			int64_t ny = (int64_t)y + dy;

			if (nx < 0 || ny < 0 || nx >= MAP_W || ny >= MAP_H) {
				continue;
			}
			if (!explored[ny * MAP_W + nx]) {
				return 1;
			}
		}
	}
	return 0;
}

/* Sample up to `want` frontier tiles, biased by per-NPC randomness. */
static uint32_t sample_frontier(uint32_t *xy, uint32_t want)
{
	uint32_t found = 0;

	for (uint32_t tries = 0; tries < 4000 && found < want; tries++) {
		uint32_t i = rng_below(MAP_H * MAP_W);

		if (is_frontier(i)) {
			xy[found * 2] = i % MAP_W;
			xy[found * 2 + 1] = i / MAP_W;
			found++;
		}
	}
	return found;
}

/* ------------------------------------------------------------------ */
/* Woodcutters                                                         */
/* ------------------------------------------------------------------ */

enum { ST_IDLE = 0, ST_TO_TREE, ST_EXPLORE, ST_CHOP, ST_TO_CAMP };

typedef struct npc {
	uint32_t x, y;
	int state;
	uint32_t chop_left;
	uint32_t tree_x, tree_y; /* the claimed trunk while chopping */
	uint32_t *steps;
	float *ticks;
	uint32_t len, at;
	float rem;
} npc;

static npc npcs[N_NPC];

static uint64_t st_batches, st_reqs, st_field_answered, st_errors;
static uint64_t st_logs, st_chopped;
static double st_ms_total;

static double now_ms(void)
{
	struct timespec ts;

	timespec_get(&ts, TIME_UTC);
	return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
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

static void setup_derech(void)
{
	static float pass[MAP_H * MAP_W];
	derech_profile_desc d;

	for (uint32_t i = 0; i < MAP_H * MAP_W; i++) {
		pass[i] = terr_pass(terr[i]);
	}
	dmap = derech_map_create(MAP_W, MAP_H, NULL);
	if (dmap == NULL ||
		derech_map_set_passability(dmap, pass,
			MAP_H * MAP_W) != DERECH_OK ||
		derech_map_set_tags(dmap, tags, MAP_H * MAP_W) != DERECH_OK) {
		fprintf(stderr, "derech setup failed\n");
		exit(1);
	}
	memset(&d, 0, sizeof(d));
	d.struct_size = (uint32_t)sizeof(d);
	if (derech_profile_register(dmap, &d) != 0) {
		exit(1);
	}
	set_known_trees = derech_goalset_register_tags(dmap,
		0, TAG_TREE | TAG_EXPLORED, DERECH_GOALSET_ADJACENT);
	if (set_known_trees < 1) {
		exit(1);
	}
}

/* Chop finished: fell the tree — tag gone, stump walkable.  Both edits
 * are dirty-region tracked; the known-trees set updates itself. */
static void fell_tree(uint32_t x, uint32_t y)
{
	uint32_t i = y * MAP_W + x;

	terr[i] = T_STUMP;
	tags[i] &= ~(uint64_t)TAG_TREE;
	claimed[i] = 0;
	(void)derech_map_set_tags_at(dmap, x, y, tags[i]);
	(void)derech_map_set_passability_at(dmap, x, y,
		terr_pass(T_STUMP));
	st_chopped++;
}

/* Find an unclaimed standing tree next to (x, y) and claim it. */
static int claim_adjacent_tree(uint32_t x, uint32_t y, uint32_t *tx,
	uint32_t *ty)
{
	for (int64_t dy = -1; dy <= 1; dy++) {
		for (int64_t dx = -1; dx <= 1; dx++) {
			int64_t nx = (int64_t)x + dx;
			int64_t ny = (int64_t)y + dy;
			uint32_t i;

			if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 ||
				nx >= MAP_W || ny >= MAP_H) {
				continue;
			}
			i = (uint32_t)ny * MAP_W + (uint32_t)nx;
			if (terr[i] == T_TREE && explored[i] && !claimed[i]) {
				claimed[i] = 1;
				*tx = (uint32_t)nx;
				*ty = (uint32_t)ny;
				return 1;
			}
		}
	}
	return 0;
}

typedef struct pend {
	uint32_t npc;
	int next_state;
	int32_t frontier_set; /* explicit set to unregister after use */
} pend;

static void sim_tick(uint64_t tick)
{
	pend queue[N_NPC];
	derech_request reqs[N_NPC];
	uint32_t nq = 0;

	/* movement + arrivals */
	for (uint32_t i = 0; i < N_NPC; i++) {
		npc *n = &npcs[i];

		if (n->len > 0) {
			n->rem -= 1.0f;
			while (n->rem <= 0.0f && n->at < n->len) {
				n->x = n->steps[n->at * 2];
				n->y = n->steps[n->at * 2 + 1];
				n->at++;
				if (n->at < n->len) {
					n->rem += n->ticks[n->at];
				}
			}
			reveal_disc(n->x, n->y, 5);
			if (n->at < n->len) {
				continue;
			}
			npc_clear_path(n);
			/* arrived */
			if (n->state == ST_TO_TREE) {
				if (claim_adjacent_tree(n->x, n->y,
					&n->tree_x, &n->tree_y)) {
					n->state = ST_CHOP;
					n->chop_left = 25;
				} else {
					n->state = ST_IDLE; /* contested */
				}
			} else if (n->state == ST_EXPLORE) {
				n->state = ST_IDLE;
			} else if (n->state == ST_TO_CAMP) {
				st_logs++;
				n->state = ST_IDLE;
			}
			continue;
		}

		if (n->state == ST_CHOP) {
			if (--n->chop_left == 0) {
				fell_tree(n->tree_x, n->tree_y);
				n->state = ST_TO_CAMP;
				queue[nq].npc = i;
				queue[nq].next_state = ST_TO_CAMP;
				queue[nq].frontier_set = 0;
				memset(&reqs[nq], 0, sizeof(reqs[nq]));
				reqs[nq].struct_size = sizeof(reqs[nq]);
				reqs[nq].start_x = n->x;
				reqs[nq].start_y = n->y;
				reqs[nq].goal_x = CAMP_X;
				reqs[nq].goal_y = CAMP_Y;
				nq++;
			}
			continue;
		}

		if (n->state == ST_IDLE) {
			/* try the known-trees set first */
			queue[nq].npc = i;
			queue[nq].next_state = ST_TO_TREE;
			queue[nq].frontier_set = 0;
			memset(&reqs[nq], 0, sizeof(reqs[nq]));
			reqs[nq].struct_size = sizeof(reqs[nq]);
			reqs[nq].start_x = n->x;
			reqs[nq].start_y = n->y;
			reqs[nq].goalset = (uint32_t)set_known_trees;
			nq++;
		}
	}

	/* reveals flush every 12 ticks — batched, as a host would */
	if (tick % 12 == 0) {
		flush_reveals();
	}

	if (nq == 0) {
		return;
	}
	{
		derech_results *res = NULL;
		double t0 = now_ms();

		if (derech_find_paths(dmap, reqs, nq, &res) != DERECH_OK) {
			st_errors += nq;
			return;
		}
		st_ms_total += now_ms() - t0;
		st_batches++;
		st_reqs += nq;

		for (uint32_t k = 0; k < nq; k++) {
			npc *n = &npcs[queue[k].npc];
			derech_path_status st = derech_result_status(res, k);
			uint32_t len = derech_result_length(res, k);

			if (st == DERECH_PATH_FOUND && len > 0 &&
				derech_result_expansions(res, k) == 0) {
				st_field_answered++;
			}
			if (st != DERECH_PATH_FOUND) {
				if (queue[k].next_state == ST_TO_TREE) {
					/* no known reachable tree: explore */
					n->state = ST_EXPLORE;
				} else {
					st_errors++;
					n->state = ST_IDLE;
				}
				continue;
			}
			n->state = queue[k].next_state;
			if (len == 0) {
				/* already there (e.g. beside a tree) */
				if (n->state == ST_TO_TREE) {
					if (claim_adjacent_tree(n->x, n->y,
						&n->tree_x, &n->tree_y)) {
						n->state = ST_CHOP;
						n->chop_left = 25;
					} else {
						n->state = ST_IDLE;
					}
				} else if (n->state == ST_TO_CAMP) {
					st_logs++;
					n->state = ST_IDLE;
				} else {
					n->state = ST_IDLE;
				}
				continue;
			}
			npc_clear_path(n);
			n->steps = malloc((size_t)len * 2 *
				sizeof(*n->steps));
			n->ticks = malloc((size_t)len * sizeof(*n->ticks));
			if (n->steps == NULL || n->ticks == NULL) {
				npc_clear_path(n);
				st_errors++;
				n->state = ST_IDLE;
				continue;
			}
			memcpy(n->steps, derech_result_steps(res, k),
				(size_t)len * 2 * sizeof(*n->steps));
			memcpy(n->ticks, derech_result_step_ticks(res, k),
				(size_t)len * sizeof(*n->ticks));
			n->len = len;
			n->at = 0;
			n->rem = n->ticks[0];
		}
		derech_results_destroy(res);
	}

	/* explorers pick frontier targets via one-shot explicit sets */
	for (uint32_t i = 0; i < N_NPC; i++) {
		npc *n = &npcs[i];
		uint32_t xy[12];
		uint32_t found;
		int32_t sid;
		derech_request q;
		derech_results *res = NULL;

		if (n->state != ST_EXPLORE || n->len > 0) {
			continue;
		}
		found = sample_frontier(xy, 6);
		if (found == 0) {
			n->state = ST_IDLE; /* world fully explored */
			continue;
		}
		sid = derech_goalset_register(dmap, xy, found, 0);
		if (sid < 1) {
			n->state = ST_IDLE;
			continue;
		}
		memset(&q, 0, sizeof(q));
		q.struct_size = sizeof(q);
		q.start_x = n->x;
		q.start_y = n->y;
		q.goalset = (uint32_t)sid;
		if (derech_find_paths(dmap, &q, 1, &res) == DERECH_OK) {
			st_batches++;
			st_reqs++;
			if (derech_result_status(res, 0) ==
				DERECH_PATH_FOUND &&
				derech_result_length(res, 0) > 0) {
				uint32_t len = derech_result_length(res, 0);

				n->steps = malloc((size_t)len * 2 *
					sizeof(*n->steps));
				n->ticks = malloc((size_t)len *
					sizeof(*n->ticks));
				if (n->steps != NULL && n->ticks != NULL) {
					memcpy(n->steps,
						derech_result_steps(res, 0),
						(size_t)len * 2 *
						sizeof(*n->steps));
					memcpy(n->ticks,
						derech_result_step_ticks(res,
							0), (size_t)len *
						sizeof(*n->ticks));
					n->len = len;
					n->at = 0;
					n->rem = n->ticks[0];
				} else {
					npc_clear_path(n);
					n->state = ST_IDLE;
				}
			} else {
				n->state = ST_IDLE;
			}
			derech_results_destroy(res);
		}
		(void)derech_goalset_unregister(dmap, (uint32_t)sid);
	}
}

static void setup_npcs(void)
{
	for (uint32_t i = 0; i < N_NPC; i++) {
		npcs[i].x = CAMP_X - 3 + i % 7;
		npcs[i].y = CAMP_Y - 2 + i / 7;
		npcs[i].state = ST_IDLE;
	}
	reveal_disc(CAMP_X, CAMP_Y, 12);
	flush_reveals();
}

/* ------------------------------------------------------------------ */
/* Selftest                                                            */
/* ------------------------------------------------------------------ */

static uint32_t explored_pct(void)
{
	uint32_t n = 0;

	for (uint32_t i = 0; i < MAP_H * MAP_W; i++) {
		n += explored[i];
	}
	return n * 100 / (MAP_H * MAP_W);
}

static int selftest(void)
{
	int ok = 1;
	uint32_t pct0 = explored_pct();
	int64_t known;

	for (uint64_t tick = 0; tick < 8000; tick++) {
		sim_tick(tick);
	}
	known = derech_goalset_count(dmap, (uint32_t)set_known_trees);
	printf("after 8000 ticks: %" PRIu64 " logs stockpiled, %" PRIu64
		" trees felled, %u%%->%u%% explored, %" PRId64
		" known standing trees\n", st_logs, st_chopped, pct0,
		explored_pct(), known);
	printf("batches %" PRIu64 ", requests %" PRIu64 ", field-answered %"
		PRIu64 ", reveal writes %" PRIu64 ", errors %" PRIu64
		", pathfind total %.1f ms\n", st_batches, st_reqs,
		st_field_answered, st_reveal_writes, st_errors, st_ms_total);
	if (st_errors != 0 || st_logs < 20 || st_chopped < 20 ||
		explored_pct() <= pct0 || st_field_answered == 0 ||
		known < 0) {
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

static const cell_style STYLE[6] = {
	[T_GRASS] = { ',', 84, 122, 66, 26, 36, 24 },
	[T_BRUSH] = { '"', 120, 128, 60, 34, 38, 20 },
	[T_ROCK] = { 'o', 130, 130, 130, 40, 40, 42 },
	[T_TREE] = { 'T', 40, 140, 50, 18, 40, 22 },
	[T_STUMP] = { '.', 150, 120, 80, 30, 34, 24 },
	[T_CAMP] = { '.', 180, 160, 120, 70, 60, 46 },
};

static void draw(struct notcurses *nc, struct ncplane *std, uint64_t tick,
	int32_t vx, int32_t vy, int paused, int speed, int follow)
{
	uint32_t rows, cols;
	char hud[256];
	int64_t known = derech_goalset_count(dmap,
		(uint32_t)set_known_trees);

	ncplane_dim_yx(std, &rows, &cols);
	ncplane_erase(std);

	for (uint32_t ry = 0; ry + 2 < rows; ry++) {
		int32_t my = vy + (int32_t)ry;

		if (my < 0 || my >= MAP_H) {
			continue;
		}
		for (uint32_t rx = 0; rx < cols; rx++) {
			int32_t mx = vx + (int32_t)rx;
			uint32_t i;

			if (mx < 0 || mx >= MAP_W) {
				continue;
			}
			i = (uint32_t)my * MAP_W + (uint32_t)mx;
			if (!explored[i]) {
				ncplane_set_fg_rgb8(std, 28, 28, 34);
				ncplane_set_bg_rgb8(std, 12, 12, 16);
				ncplane_putchar_yx(std, (int)ry + 2,
					(int)rx, ' ');
				continue;
			}
			{
				const cell_style *s = &STYLE[terr[i]];

				ncplane_set_fg_rgb8(std, s->fr, s->fg,
					s->fb);
				ncplane_set_bg_rgb8(std, s->br, s->bg,
					s->bb);
				ncplane_putchar_yx(std, (int)ry + 2,
					(int)rx, claimed[i] ? 't' : s->ch);
			}
		}
	}

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
		char ch = npcs[i].state == ST_CHOP ? '*' :
			(npcs[i].state == ST_TO_CAMP ? '$' : '@');

		if (sx < 0 || sx >= (int32_t)cols || sy < 0 ||
			sy + 2 >= (int32_t)rows) {
			continue;
		}
		if ((int)i == follow) {
			ncplane_set_fg_rgb8(std, 255, 90, 90);
		} else {
			ncplane_set_fg_rgb8(std, 255, 220, 90);
		}
		ncplane_set_bg_rgb8(std, 40, 40, 34);
		ncplane_putchar_yx(std, sy + 2, sx, ch);
	}

	ncplane_set_fg_rgb8(std, 230, 230, 230);
	ncplane_set_bg_rgb8(std, 20, 20, 24);
	snprintf(hud, sizeof(hud),
		" derech woodcutters | tick %" PRIu64 " %s x%d | logs %"
		PRIu64 " | felled %" PRIu64 " | known trees %" PRId64
		" | explored %u%%", tick, paused ? "PAUSED" : "", speed,
		st_logs, st_chopped, known, explored_pct());
	ncplane_putstr_yx(std, 0, 0, hud);
	snprintf(hud, sizeof(hud),
		" batches %" PRIu64 " reqs %" PRIu64 " field-answered %"
		PRIu64 " reveals %" PRIu64 " err %" PRIu64
		" | arrows scroll  f follow  SPACE pause  1-3 speed  q quit",
		st_batches, st_reqs, st_field_answered, st_reveal_writes,
		st_errors);
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
	uint64_t tick = 0;
	double t_start = now_ms();
	double acc = 0.0;
	int32_t vx = 0, vy = MAP_H - 40;
	int paused = 0, speed = 2, follow = -1;
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
	printf("woodcutters session: tick %" PRIu64 " | logs %" PRIu64
		" felled %" PRIu64 " explored %u%% | batches %" PRIu64
		", requests %" PRIu64 ", field-answered %" PRIu64
		", reveals %" PRIu64 ", errors %" PRIu64 ", pathfind %.1f ms"
		" total\n", tick, st_logs, st_chopped, explored_pct(),
		st_batches, st_reqs, st_field_answered, st_reveal_writes,
		st_errors, st_ms_total);
	return 0;
}

#endif /* !DEMO_SELFTEST_ONLY */

int main(int argc, char **argv)
{
#if !defined(DEMO_SELFTEST_ONLY) && !defined(_WIN32)
	if (getenv("TERMINFO") == NULL || getenv("TERMINFO_DIRS") == NULL) {
		setenv("TERMINFO_DIRS",
			"/usr/share/terminfo:/etc/terminfo:/lib/terminfo:"
			"/usr/lib/terminfo", 0);
	}
#endif

	build_world();
	setup_derech();
	setup_npcs();

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
