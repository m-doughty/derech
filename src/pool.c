/* Persistent worker pool for batch solving.
 *
 * Shape: n_workers total participants — the thread calling
 * derech_pool_run() plus (n_workers - 1) spawned workers.  Work is
 * claimed request-by-request from an atomic cursor, so heterogeneous
 * searches load-balance dynamically.  Each participant only ever touches
 * its own derech_search_ctx and the staging rows of the requests it
 * claimed; the map is read-only during a batch (enforced by the busy
 * guard in derech_find_paths), so no request-level locking is needed.
 *
 * Handshake invariants (the subtle part):
 *   - Workers copy the job descriptor and increment n_active UNDER THE
 *     LOCK before draining, so a batch cannot be considered complete
 *     while any participant might still read it.
 *   - derech_pool_run() returns only when n_active == 0 and every
 *     request is done, and it zeroes job.n_reqs before returning — a
 *     worker that wakes late therefore copies an empty job and never
 *     touches the cursor, so it can neither dereference a dangling
 *     request pointer nor steal an index from a later batch.
 */

#include "derech_internal.h"
#include "derech_thread.h"

typedef struct derech_worker_arg {
	derech_pool *pool;
	uint32_t index; /* ctx index; 0 is reserved for the caller */
} derech_worker_arg;

typedef struct derech_batch_job {
	derech_map *map;
	const derech_task *tasks;
	uint32_t n_tasks;
	uint32_t n_participants;
	const derech_request *reqs;
	derech_stage_row *stage;
} derech_batch_job;

struct derech_pool {
	derech_mutex m;
	derech_cond work_cv;
	derech_cond done_cv;

	derech_thread *threads;
	derech_worker_arg *args;
	uint32_t n_spawned;
	int m_initialized;
	int work_cv_initialized;
	int done_cv_initialized;
	int shutdown;      /* guarded by m */
	uint64_t seq;      /* guarded by m; bumped per dispatched batch */
	derech_batch_job job; /* guarded by m for writes; workers copy   */
	uint32_t n_active; /* guarded by m; participants mid-drain       */

	derech_atomic_u32 cursor;
	derech_atomic_u32 n_done;
};

uint32_t derech_hw_threads(void)
{
	return derech_hw_threads_raw();
}

static void pool_drain(derech_pool *p, const derech_batch_job *job,
	uint32_t worker)
{
	derech_search_ctx *ctx = &job->map->ctxs[worker];

	for (;;) {
		uint32_t k = derech_atomic_fetch_add_u32(&p->cursor, 1);

		if (k >= job->n_tasks) {
			break;
		}
		derech_run_task(job->map, ctx, worker, &job->tasks[k],
			job->reqs, job->stage);
		derech_atomic_fetch_add_u32(&p->n_done, 1);
	}
}

static void pool_worker(void *argp)
{
	derech_worker_arg *wa = argp;
	derech_pool *p = wa->pool;
	uint64_t seen = 0;

	derech_mutex_lock(&p->m);
	for (;;) {
		derech_batch_job job;

		while (!p->shutdown && p->seq == seen) {
			derech_cond_wait(&p->work_cv, &p->m);
		}
		if (p->shutdown) {
			break;
		}
		seen = p->seq;
		job = p->job;
		if (job.n_tasks == 0 || wa->index >= job.n_participants) {
			continue; /* batch already completed without us */
		}
		p->n_active++;
		derech_mutex_unlock(&p->m);

		pool_drain(p, &job, wa->index);

		derech_mutex_lock(&p->m);
		p->n_active--;
		if (p->n_active == 0) {
			derech_cond_signal(&p->done_cv);
		}
	}
	derech_mutex_unlock(&p->m);
}

void derech_pool_run(derech_pool *p, derech_map *map,
	const derech_task *tasks, uint32_t n_tasks, const derech_request *reqs,
	derech_stage_row *stage, uint32_t n_participants)
{
	derech_batch_job job;

	job.map = map;
	job.tasks = tasks;
	job.n_tasks = n_tasks;
	job.n_participants = n_participants;
	job.reqs = reqs;
	job.stage = stage;

	derech_mutex_lock(&p->m);
	p->job = job;
	derech_atomic_store_u32(&p->cursor, 0);
	derech_atomic_store_u32(&p->n_done, 0);
	p->seq++;
	p->n_active++; /* the caller participates */
	derech_cond_broadcast(&p->work_cv);
	derech_mutex_unlock(&p->m);

	pool_drain(p, &job, 0);

	derech_mutex_lock(&p->m);
	p->n_active--;
	while (p->n_active != 0 ||
		derech_atomic_load_u32(&p->n_done) < n_tasks) {
		derech_cond_wait(&p->done_cv, &p->m);
	}
	p->job.n_tasks = 0; /* late wakers must see an empty batch */
	derech_mutex_unlock(&p->m);
}

derech_pool *derech_pool_create(uint32_t n_workers)
{
	derech_pool *p;

	if (n_workers < 2) {
		return NULL; /* a pool below two participants is a bug */
	}
	p = calloc(1, sizeof(*p));
	if (p == NULL) {
		return NULL;
	}
	if (!derech_mutex_init(&p->m)) {
		free(p);
		return NULL;
	}
	p->m_initialized = 1;
	if (!derech_cond_init(&p->work_cv)) {
		derech_pool_destroy(p);
		return NULL;
	}
	p->work_cv_initialized = 1;
	if (!derech_cond_init(&p->done_cv)) {
		derech_pool_destroy(p);
		return NULL;
	}
	p->done_cv_initialized = 1;
	p->threads = calloc(n_workers - 1, sizeof(*p->threads));
	p->args = calloc(n_workers - 1, sizeof(*p->args));
	if (p->threads == NULL || p->args == NULL) {
		derech_pool_destroy(p);
		return NULL;
	}
	for (uint32_t i = 0; i < n_workers - 1; i++) {
		p->args[i].pool = p;
		p->args[i].index = i + 1;
		if (!derech_thread_create(&p->threads[i], pool_worker,
			&p->args[i])) {
			derech_pool_destroy(p);
			return NULL;
		}
		p->n_spawned++;
	}
	return p;
}

void derech_pool_destroy(derech_pool *p)
{
	if (p == NULL) {
		return;
	}
	if (p->n_spawned > 0) {
		derech_mutex_lock(&p->m);
		p->shutdown = 1;
		derech_cond_broadcast(&p->work_cv);
		derech_mutex_unlock(&p->m);
		for (uint32_t i = 0; i < p->n_spawned; i++) {
			derech_thread_join(p->threads[i]);
		}
	}
	if (p->done_cv_initialized) {
		derech_cond_destroy(&p->done_cv);
	}
	if (p->work_cv_initialized) {
		derech_cond_destroy(&p->work_cv);
	}
	if (p->m_initialized) {
		derech_mutex_destroy(&p->m);
	}
	free(p->threads);
	free(p->args);
	free(p);
}
