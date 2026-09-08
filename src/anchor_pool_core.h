#ifndef ANCHOR_POOL_CORE_H
#define ANCHOR_POOL_CORE_H

#include <stdint.h>
#include <stdbool.h>

/* Pure anchor-pool bookkeeping shared by discovery (CIR-ranked) and passive
 * ANNOUNCE learning (geometry). No Zephyr, no radio -- host-tested in
 * tests/anchor_pool/. See spec/2026-09-07-network-scaling-design.md §3.C. */

#define ANCHOR_POOL_MAX   16
#define ANCHOR_SELECT_MAX 4   /* stays 4: the slot budget and POS_MAX_ANCHORS
                                * both assume it. A bigger pool means a better
                                * 4, not a bigger sweep. */

/* An entry not refreshed (by discovery or ANNOUNCE) within this long is
 * dropped. Without this the pool never invalidates an entry, which is what
 * let the tag report anchors it could no longer reach while emitting no fix
 * at all (CLAUDE.md, sweep-gate entry) -- a bigger pool makes that easier to
 * hit, not harder, so eviction on staleness is required at this size. */
#define ANCHOR_ENTRY_STALE_MS 300000u

struct anchor_pool_entry {
    uint8_t  id;
    float    ema_score;         /* CIR-power + quality composite, higher is better */
    float    x, y, z;           /* announce-learned position; z may be NaN */
    bool     have_pos;
    bool     valid;
    bool     seen;              /* refreshed this round; cleared by anchor_pool_begin_round() */
    uint32_t last_seen_ms;
};

struct anchor_pool {
    struct anchor_pool_entry entries[ANCHOR_POOL_MAX];
};

void anchor_pool_init(struct anchor_pool *p);

/* Clear the transient "seen this round" flag ahead of a discovery round. */
void anchor_pool_begin_round(struct anchor_pool *p);

/* Discovery response: EMA-update an existing entry's score, or insert a new
 * one. Pool full -> evicts the lowest-scoring valid entry. */
void anchor_pool_observe_score(struct anchor_pool *p, uint8_t id, float score,
                               uint32_t now_ms);

/* ANNOUNCE: seed/refresh an anchor's learned position. Does not evict an
 * existing (CIR-ranked) entry to make room -- a geometry-only entry has no
 * ranging value yet, so it is only inserted when a free slot exists. z may
 * be NaN (anchor did not report a height); passed through unchanged. */
void anchor_pool_observe_pos(struct anchor_pool *p, uint8_t id,
                             float x, float y, float z, uint32_t now_ms);

/* Decay the score of every valid entry not marked seen this round. */
void anchor_pool_decay_missed(struct anchor_pool *p);

/* Drop entries not seen (by either observe call) within ANCHOR_ENTRY_STALE_MS. */
void anchor_pool_expire_stale(struct anchor_pool *p, uint32_t now_ms);

/* Top-N valid entries by ema_score descending, N = min(live count, max_out).
 * Never returns more anchors than are actually live in the pool. Returns the
 * count written to out. */
uint8_t anchor_pool_select(const struct anchor_pool *p, uint8_t *out, uint8_t max_out);

uint8_t anchor_pool_live_count(const struct anchor_pool *p);

#endif /* ANCHOR_POOL_CORE_H */
