/* Portable threading shim: threads, mutex, condvar, core count.
 * POSIX (pthreads) and Win32 (SRWLOCK + CONDITION_VARIABLE).  Included
 * only by pool.c — everything is static inline, so no link artifacts. */

#ifndef DERECH_THREAD_H
#define DERECH_THREAD_H

#include <stdint.h>
#include <stdlib.h>

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <process.h>

typedef HANDLE derech_thread;
typedef SRWLOCK derech_mutex;
typedef CONDITION_VARIABLE derech_cond;

static inline int derech_mutex_init(derech_mutex *m)
{
	InitializeSRWLock(m);
	return 1;
}

static inline void derech_mutex_destroy(derech_mutex *m)
{
	(void)m;
}

static inline void derech_mutex_lock(derech_mutex *m)
{
	AcquireSRWLockExclusive(m);
}

static inline void derech_mutex_unlock(derech_mutex *m)
{
	ReleaseSRWLockExclusive(m);
}

static inline int derech_cond_init(derech_cond *c)
{
	InitializeConditionVariable(c);
	return 1;
}

static inline void derech_cond_destroy(derech_cond *c)
{
	(void)c;
}

static inline void derech_cond_wait(derech_cond *c, derech_mutex *m)
{
	SleepConditionVariableSRW(c, m, INFINITE, 0);
}

static inline void derech_cond_signal(derech_cond *c)
{
	WakeConditionVariable(c);
}

static inline void derech_cond_broadcast(derech_cond *c)
{
	WakeAllConditionVariable(c);
}

struct derech_thread_boot {
	void (*fn)(void *);
	void *arg;
};

static inline unsigned __stdcall derech_thread_tramp_(void *p)
{
	struct derech_thread_boot b = *(struct derech_thread_boot *)p;

	free(p);
	b.fn(b.arg);
	return 0;
}

static inline int derech_thread_create(derech_thread *t, void (*fn)(void *),
	void *arg)
{
	struct derech_thread_boot *b = malloc(sizeof(*b));
	uintptr_t h;

	if (b == NULL) {
		return 0;
	}
	b->fn = fn;
	b->arg = arg;
	h = _beginthreadex(NULL, 0, derech_thread_tramp_, b, 0, NULL);
	if (h == 0) {
		free(b);
		return 0;
	}
	*t = (HANDLE)h;
	return 1;
}

static inline void derech_thread_join(derech_thread t)
{
	WaitForSingleObject(t, INFINITE);
	CloseHandle(t);
}

static inline uint32_t derech_hw_threads_raw(void)
{
	SYSTEM_INFO si;

	GetSystemInfo(&si);
	return si.dwNumberOfProcessors > 0 ?
		(uint32_t)si.dwNumberOfProcessors : 1;
}

#else /* POSIX */

#include <pthread.h>
#include <unistd.h>

typedef pthread_t derech_thread;
typedef pthread_mutex_t derech_mutex;
typedef pthread_cond_t derech_cond;

static inline int derech_mutex_init(derech_mutex *m)
{
	return pthread_mutex_init(m, NULL) == 0;
}

static inline void derech_mutex_destroy(derech_mutex *m)
{
	pthread_mutex_destroy(m);
}

static inline void derech_mutex_lock(derech_mutex *m)
{
	pthread_mutex_lock(m);
}

static inline void derech_mutex_unlock(derech_mutex *m)
{
	pthread_mutex_unlock(m);
}

static inline int derech_cond_init(derech_cond *c)
{
	return pthread_cond_init(c, NULL) == 0;
}

static inline void derech_cond_destroy(derech_cond *c)
{
	pthread_cond_destroy(c);
}

static inline void derech_cond_wait(derech_cond *c, derech_mutex *m)
{
	pthread_cond_wait(c, m);
}

static inline void derech_cond_signal(derech_cond *c)
{
	pthread_cond_signal(c);
}

static inline void derech_cond_broadcast(derech_cond *c)
{
	pthread_cond_broadcast(c);
}

struct derech_thread_boot {
	void (*fn)(void *);
	void *arg;
};

static inline void *derech_thread_tramp_(void *p)
{
	struct derech_thread_boot b = *(struct derech_thread_boot *)p;

	free(p);
	b.fn(b.arg);
	return NULL;
}

static inline int derech_thread_create(derech_thread *t, void (*fn)(void *),
	void *arg)
{
	struct derech_thread_boot *b = malloc(sizeof(*b));

	if (b == NULL) {
		return 0;
	}
	b->fn = fn;
	b->arg = arg;
	if (pthread_create(t, NULL, derech_thread_tramp_, b) != 0) {
		free(b);
		return 0;
	}
	return 1;
}

static inline void derech_thread_join(derech_thread t)
{
	pthread_join(t, NULL);
}

static inline uint32_t derech_hw_threads_raw(void)
{
	long n = sysconf(_SC_NPROCESSORS_ONLN);

	return n > 0 ? (uint32_t)n : 1;
}

#endif

#endif /* DERECH_THREAD_H */
