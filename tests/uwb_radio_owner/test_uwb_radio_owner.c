/*
 * Host test for src/uwb_radio_owner.c — the real module, unmodified, driven by
 * two threads through the pthread shim in shim/zephyr/kernel.h.
 *
 * This module shipped with two genuine handover races on its first attempt and
 * had nothing executable guarding it. The three properties below are the ones
 * those races violated.
 *
 * Build (NCS toolchain gcc, winpthread):
 *   gcc -Wall -Wextra -std=c99 -Isrc -Itests/uwb_radio_owner/shim \
 *       -o uro.exe tests/uwb_radio_owner/test_uwb_radio_owner.c \
 *       src/uwb_radio_owner.c -lpthread
 */

#include "uwb_radio_owner.h"

#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>

/* ---- CHECK, in the style of the other host tests, but thread-safe ---- */

static int fails;
static pthread_mutex_t fail_lock = PTHREAD_MUTEX_INITIALIZER;

#define CHECK(c)                                                          \
    do {                                                                  \
        if (!(c)) {                                                       \
            pthread_mutex_lock(&fail_lock);                               \
            printf("FAIL line %d: %s\n", __LINE__, #c);                   \
            fails++;                                                      \
            pthread_mutex_unlock(&fail_lock);                             \
        }                                                                 \
    } while (0)

/* ---- timing helpers ---- */

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

static void sleep_ms(unsigned ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* Busy-wait: host sleep granularity is far too coarse to place a yield at a
 * chosen offset inside a 20 ms window. */
static void spin_us(uint64_t us)
{
    uint64_t t0 = now_us();
    while (now_us() - t0 < us) {
        /* spin */
    }
}

/* =======================================================================
 * (a) request() blocks until yield(); release() unblocks the parked yield()
 * ======================================================================= */

static volatile int a_req_done;      /* 0 = still blocked, 1 = true, 2 = false */
static volatile int a_yield_done;

static void *a_claimant(void *arg)
{
    (void)arg;

    bool ok = uwb_radio_request(K_SECONDS(5));

    a_req_done = ok ? 1 : 2;
    CHECK(ok);

    /* Hold the radio a while: the runner must stay parked inside yield() for
     * the whole time, not return the moment it handed over. */
    sleep_ms(200);
    CHECK(a_yield_done == 0);

    uwb_radio_release();
    return NULL;
}

static void test_request_blocks_until_yield(void)
{
    a_req_done = 0;
    a_yield_done = 0;

    pthread_t th;
    pthread_create(&th, NULL, a_claimant, NULL);

    /* No yield yet, so the claim must still be outstanding. */
    sleep_ms(200);
    CHECK(a_req_done == 0);
    CHECK(uwb_radio_request_pending());

    bool handed = uwb_radio_yield();   /* parks until the claimant releases */
    a_yield_done = 1;

    CHECK(handed);
    CHECK(a_req_done == 1);

    pthread_join(th, NULL);
    CHECK(!uwb_radio_request_pending());
}

/* =======================================================================
 * (b) a stray release() must not let a later yield() return early
 * ======================================================================= */

static volatile int b_req_done;
static volatile int b_yield_done;

static void *b_claimant(void *arg)
{
    (void)arg;

    bool ok = uwb_radio_request(K_SECONDS(5));

    b_req_done = ok ? 1 : 2;
    CHECK(ok);

    sleep_ms(250);
    /* If a stray release had armed the handover, the runner would already have
     * come back out of yield() while we still hold the radio -- two threads on
     * the DW3000. */
    CHECK(b_yield_done == 0);

    uwb_radio_release();
    return NULL;
}

static void test_stray_release_is_inert(void)
{
    b_req_done = 0;
    b_yield_done = 0;

    /* Stray #1: released with the module idle and nothing ever claimed. */
    uwb_radio_release();
    CHECK(!uwb_radio_request_pending());

    pthread_t th;
    pthread_create(&th, NULL, b_claimant, NULL);

    uint64_t t0 = now_us();
    while (!uwb_radio_request_pending() && now_us() - t0 < 1000000u) {
        sleep_ms(1);
    }
    CHECK(uwb_radio_request_pending());

    /* Stray #2: released with a claim pending but no handover outstanding.
     * It must neither complete the claim nor arm the next yield. */
    uwb_radio_release();
    sleep_ms(150);
    CHECK(b_req_done == 0);
    CHECK(uwb_radio_request_pending());

    bool handed = uwb_radio_yield();
    b_yield_done = 1;

    CHECK(handed);
    CHECK(b_req_done == 1);

    pthread_join(th, NULL);
    CHECK(!uwb_radio_request_pending());
}

/* =======================================================================
 * (c) the timeout-vs-yield race always resolves to exactly one owner
 *
 * The claimant asks with a fixed 20 ms budget; the runner reaches its yield
 * point at an offset swept across and past that budget, so the two collide at
 * every relative alignment. The invariant is that request() and yield() always
 * agree: either the handover happened for both, or for neither. A yield() that
 * committed while the claimant walked away would strand the runner in a
 * K_FOREVER wait; a request() that returned true against a runner that never
 * yielded would put two threads on the radio.
 * ======================================================================= */

/* The sweep runs well past the claim budget on purpose: host condvar timeouts
 * are coarse and fire late, so a sweep that only just reaches C_CLAIM_MS can
 * land entirely on the "handed over" side. */
#define C_CLAIM_MS      20
#define C_ITERS         25
#define C_STEP_US       4000u

static volatile int c_req_result;    /* -1 = not finished, 0 = false, 1 = true */
static volatile int c_yield_result;

static void *c_claimant(void *arg)
{
    (void)arg;

    bool ok = uwb_radio_request(K_MSEC(C_CLAIM_MS));

    if (ok) {
        uwb_radio_release();
    }
    c_req_result = ok ? 1 : 0;
    return NULL;
}

static void test_timeout_vs_yield_race(void)
{
    int n_handed = 0, n_missed = 0;

    for (int i = 0; i < C_ITERS; i++) {
        c_req_result = -1;
        c_yield_result = -1;

        pthread_t th;
        pthread_create(&th, NULL, c_claimant, NULL);

        /* Anchor the sweep to the moment the claimant is actually inside
         * request(), so offset 0 really means "yield immediately". */
        uint64_t t0 = now_us();
        while (!uwb_radio_request_pending() && now_us() - t0 < 1000000u) {
            /* spin */
        }

        spin_us((uint64_t)i * C_STEP_US);

        bool handed = uwb_radio_yield();
        c_yield_result = handed ? 1 : 0;

        pthread_join(th, NULL);

        /* Exactly one owner: the two sides never disagree about whether the
         * radio changed hands. */
        CHECK(c_req_result == c_yield_result);

        if (handed) {
            n_handed++;
        } else {
            n_missed++;
        }

        /* And the module is idle again, ready for the next claim. */
        CHECK(!uwb_radio_request_pending());
    }

    /* The sweep must have produced both outcomes, or it never reached the
     * race and the invariant above was tested against one case only. */
    CHECK(n_handed > 0);
    CHECK(n_missed > 0);
    printf("race sweep: %d handed over, %d timed out\n", n_handed, n_missed);
}

/* =======================================================================
 * Watchdog
 *
 * The characteristic failure of this module is not a wrong answer, it is a
 * thread parked forever -- a yield() that committed to a handover nobody is
 * coming back from. Without this, such a regression makes the suite hang
 * instead of fail, which in CI is indistinguishable from a slow machine.
 * ======================================================================= */

#define WATCHDOG_S  30

static void *watchdog(void *arg)
{
    (void)arg;
    sleep_ms(WATCHDOG_S * 1000);
    printf("FAIL watchdog: still running after %d s -- a thread is parked "
           "forever (a handover that was never returned)\n", WATCHDOG_S);
    fflush(stdout);
    _exit(1);
    return NULL;
}

int main(void)
{
    pthread_t wd;
    pthread_create(&wd, NULL, watchdog, NULL);
    pthread_detach(wd);

    test_request_blocks_until_yield();
    test_stray_release_is_inert();
    test_timeout_vs_yield_race();

    printf("uwb_radio_owner: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
