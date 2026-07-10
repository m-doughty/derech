/* Internal allocator boundary and deterministic test-only fault injection. */

#define DERECH_ALLOC_IMPLEMENTATION
#include "derech_internal.h"

#ifdef DERECH_TESTING

/* fail_at is a zero-based attempt index.  An injected failure is one-shot
 * because the attempt counter advances. */
static derech_atomic_u64 alloc_attempts;
static derech_atomic_u64 alloc_fail_at;
static derech_atomic_u32 alloc_failure_enabled;

static int allocation_should_fail(void)
{
	uint64_t attempt = derech_atomic_fetch_add_u64(&alloc_attempts, 1);

	return derech_atomic_load_u32(&alloc_failure_enabled) != 0 &&
		attempt == derech_atomic_load_u64(&alloc_fail_at);
}

void derech_test_alloc_fail_after(uint64_t successful_allocations)
{
	derech_atomic_store_u64(&alloc_attempts, 0);
	derech_atomic_store_u64(&alloc_fail_at, successful_allocations);
	derech_atomic_store_u32(&alloc_failure_enabled, 1);
}

void derech_test_alloc_disable(void)
{
	derech_atomic_store_u32(&alloc_failure_enabled, 0);
	derech_atomic_store_u64(&alloc_attempts, 0);
}

uint64_t derech_test_alloc_attempts(void)
{
	return derech_atomic_load_u64(&alloc_attempts);
}

#else

static int allocation_should_fail(void)
{
	return 0;
}

#endif

void *derech_malloc(size_t size)
{
	return allocation_should_fail() ? NULL : malloc(size);
}

void *derech_calloc(size_t count, size_t size)
{
	return allocation_should_fail() ? NULL : calloc(count, size);
}

void *derech_realloc(void *ptr, size_t size)
{
	return allocation_should_fail() ? NULL : realloc(ptr, size);
}

void derech_free(void *ptr)
{
	free(ptr);
}
