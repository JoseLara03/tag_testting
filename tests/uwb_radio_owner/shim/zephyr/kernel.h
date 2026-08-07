/*
 * Minimal host shim for the slice of <zephyr/kernel.h> that
 * src/uwb_radio_owner.c uses, backed by pthreads.
 *
 * Why this exists: uwb_radio_owner is concurrency logic, so a test that does
 * not actually run two threads against it proves nothing. Zephyr's native_sim
 * and unit_testing boards both build on arch/posix, which refuses to configure
 * on this host ("The POSIX architecture only works on Linux"), so the module is
 * compiled unmodified against these definitions instead and linked with
 * winpthread from the NCS toolchain's mingw64 gcc.
 *
 * What it models faithfully:
 *   - k_mutex as a real mutual-exclusion lock.
 *   - k_condvar_wait's atomic "unlock, block, relock" — this comes straight
 *     from pthread_cond_wait/pthread_cond_timedwait, which is the one property
 *     the whole handover rests on. Get it wrong and the test is worthless.
 *   - k_condvar_broadcast waking every waiter.
 *   - A relative timeout on k_condvar_wait, returning non-zero on expiry, so
 *     the timeout-vs-yield race is reachable.
 *
 * What it does NOT model:
 *   - Zephyr's priority-based scheduling: no priority inheritance, no
 *     preemption points, no cooperative threads. The host scheduler decides.
 *   - Tick quantisation: timeouts are milliseconds against CLOCK_REALTIME, not
 *     ticks, and host timer granularity is coarse (~1-15 ms on Windows).
 *   - k_mutex recursion counts, ISR context checks, or K_NO_WAIT semantics
 *     beyond "expires immediately".
 * None of those are what the module's correctness depends on: every state
 * transition in uwb_radio_owner.c is made under `lock`, so the argument is a
 * mutual-exclusion argument, not a scheduling one.
 */
#ifndef UWB_RADIO_OWNER_TEST_ZEPHYR_KERNEL_H_
#define UWB_RADIO_OWNER_TEST_ZEPHYR_KERNEL_H_

#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- timeouts ---- */

typedef struct { int64_t ms; } k_timeout_t;   /* ms < 0 means "wait forever" */

#define K_FOREVER     ((k_timeout_t){ -1 })
#define K_NO_WAIT     ((k_timeout_t){ 0 })
#define K_MSEC(v)     ((k_timeout_t){ (int64_t)(v) })
#define K_SECONDS(v)  K_MSEC((int64_t)(v) * 1000)

/* ---- mutex ---- */

struct k_mutex   { pthread_mutex_t m; };
struct k_condvar { pthread_cond_t  c; };

#define K_MUTEX_DEFINE(name)    struct k_mutex   name = { PTHREAD_MUTEX_INITIALIZER }
#define K_CONDVAR_DEFINE(name)  struct k_condvar name = { PTHREAD_COND_INITIALIZER }

static inline int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
    (void)timeout;   /* uwb_radio_owner only ever locks with K_FOREVER */
    return pthread_mutex_lock(&mutex->m) == 0 ? 0 : -EINVAL;
}

static inline int k_mutex_unlock(struct k_mutex *mutex)
{
    return pthread_mutex_unlock(&mutex->m) == 0 ? 0 : -EINVAL;
}

/* ---- condition variable ---- */

static inline int k_condvar_broadcast(struct k_condvar *condvar)
{
    return pthread_cond_broadcast(&condvar->c) == 0 ? 0 : -EINVAL;
}

/*
 * Returns 0 when woken by a broadcast, -EAGAIN when `timeout` expired first.
 * The mutex is released while blocked and re-acquired before returning, in
 * both cases — pthread guarantees exactly that.
 */
static inline int k_condvar_wait(struct k_condvar *condvar,
                                 struct k_mutex *mutex,
                                 k_timeout_t timeout)
{
    if (timeout.ms < 0) {
        return pthread_cond_wait(&condvar->c, &mutex->m) == 0 ? 0 : -EINVAL;
    }

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    int64_t nsec = (int64_t)ts.tv_nsec + (timeout.ms % 1000) * 1000000;
    ts.tv_sec  += (time_t)(timeout.ms / 1000 + nsec / 1000000000);
    ts.tv_nsec  = (long)(nsec % 1000000000);

    int rc = pthread_cond_timedwait(&condvar->c, &mutex->m, &ts);

    return (rc == ETIMEDOUT) ? -EAGAIN : (rc == 0 ? 0 : -EINVAL);
}

#endif /* UWB_RADIO_OWNER_TEST_ZEPHYR_KERNEL_H_ */
