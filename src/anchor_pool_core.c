#include "anchor_pool_core.h"
#include <string.h>

#define EMA_ALPHA  0.3f
#define EMA_DECAY  0.5f

void anchor_pool_init(struct anchor_pool *p)
{
    memset(p, 0, sizeof(*p));
}

void anchor_pool_begin_round(struct anchor_pool *p)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        p->entries[i].seen = false;
    }
}

static int find_valid(struct anchor_pool *p, uint8_t id)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (p->entries[i].valid && p->entries[i].id == id) {
            return i;
        }
    }
    return -1;
}

static int find_free(struct anchor_pool *p)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (!p->entries[i].valid) {
            return i;
        }
    }
    return -1;
}

static int find_worst(struct anchor_pool *p)
{
    int worst = 0;
    for (int i = 1; i < ANCHOR_POOL_MAX; i++) {
        if (p->entries[i].ema_score < p->entries[worst].ema_score) {
            worst = i;
        }
    }
    return worst;
}

void anchor_pool_observe_score(struct anchor_pool *p, uint8_t id, float score,
                               uint32_t now_ms)
{
    int i = find_valid(p, id);
    if (i >= 0) {
        p->entries[i].ema_score = EMA_ALPHA * score +
                                  (1.0f - EMA_ALPHA) * p->entries[i].ema_score;
        p->entries[i].seen = true;
        p->entries[i].last_seen_ms = now_ms;
        return;
    }

    i = find_free(p);
    if (i < 0) {
        i = find_worst(p);
    }
    p->entries[i].id           = id;
    p->entries[i].ema_score    = score;
    p->entries[i].valid        = true;
    p->entries[i].seen         = true;
    p->entries[i].last_seen_ms = now_ms;
    /* A fresh (or evicted-and-reused) slot has no learned position until an
     * ANNOUNCE says otherwise. */
    p->entries[i].have_pos     = false;
}

void anchor_pool_observe_pos(struct anchor_pool *p, uint8_t id,
                             float x, float y, float z, uint32_t now_ms)
{
    int i = find_valid(p, id);
    if (i < 0) {
        i = find_free(p);
        if (i < 0) {
            /* No ranging value yet and no room -- do not evict a
             * CIR-ranked entry to make space for one. */
            return;
        }
        p->entries[i].id        = id;
        p->entries[i].ema_score = 0.0f;
        p->entries[i].valid     = true;
    }
    p->entries[i].x           = x;
    p->entries[i].y           = y;
    p->entries[i].z           = z;
    p->entries[i].have_pos    = true;
    p->entries[i].seen        = true;
    p->entries[i].last_seen_ms = now_ms;
}

void anchor_pool_decay_missed(struct anchor_pool *p)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (p->entries[i].valid && !p->entries[i].seen) {
            p->entries[i].ema_score *= EMA_DECAY;
        }
    }
}

void anchor_pool_expire_stale(struct anchor_pool *p, uint32_t now_ms)
{
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (!p->entries[i].valid) {
            continue;
        }
        /* Unsigned subtraction: correct across a uint32_t wrap, same pattern
         * as uwb_net.c's lease_age(). */
        if (now_ms - p->entries[i].last_seen_ms > ANCHOR_ENTRY_STALE_MS) {
            memset(&p->entries[i], 0, sizeof(p->entries[i]));
        }
    }
}

uint8_t anchor_pool_select(const struct anchor_pool *p, uint8_t *out, uint8_t max_out)
{
    uint8_t order[ANCHOR_POOL_MAX];
    uint8_t count = 0;

    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (p->entries[i].valid) {
            order[count++] = (uint8_t)i;
        }
    }

    /* Insertion sort descending by ema_score (pool is small, O(n^2) is fine). */
    for (int i = 1; i < count; i++) {
        uint8_t key = order[i];
        int j = i - 1;
        while (j >= 0 &&
               p->entries[order[j]].ema_score < p->entries[key].ema_score) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }

    uint8_t n = (count < max_out) ? count : max_out;
    for (uint8_t i = 0; i < n; i++) {
        out[i] = p->entries[order[i]].id;
    }
    return n;
}

uint8_t anchor_pool_live_count(const struct anchor_pool *p)
{
    uint8_t count = 0;
    for (int i = 0; i < ANCHOR_POOL_MAX; i++) {
        if (p->entries[i].valid) {
            count++;
        }
    }
    return count;
}
