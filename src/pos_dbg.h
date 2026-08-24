#ifndef POS_DBG_H
#define POS_DBG_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * TEMPORARY raw-range debug log over BLE NUS.
 *
 * ####################################################################
 * #  This module exists for ONE purpose: capturing the data needed   #
 * #  to tune the EKF (R, sigma_a, gate threshold) offline, after     #
 * #  which it is DELETED. It is not general telemetry. Do not build  #
 * #  anything on top of it. Removal checklist is in                  #
 * #  spec/2026-08-22-position-filtering-design.md, and removal must  #
 * #  happen before CLAUDE.md Open Work item 7 (dropping NUS).        #
 * ####################################################################
 *
 * BLE is the only channel: CONFIG_SERIAL=n, and RTT needs a tether while the
 * entire experiment is *walking* the tag. That imposes a 20-byte hard limit
 * (default ATT MTU 23; bt_nus_send does not fragment and >20 bytes fails
 * -EMSGSIZE silently), so records are binary and start with a non-ASCII magic
 * byte to demultiplex them from the text lines ("P:", "BATT:", cal verdicts)
 * sharing the characteristic.
 *
 * The encoders below are pure and host-tested; the emit path is glue.
 */

#define POS_DBG_MAGIC_SWEEP  0xA5u
#define POS_DBG_MAGIC_SET    0xA6u
#define POS_DBG_MAGIC_MARK   0xA7u

#define POS_DBG_SWEEP_LEN  20u
#define POS_DBG_SET_LEN    13u
#define POS_DBG_MARK_LEN    4u

/* Largest record; also the NUS payload ceiling. */
#define POS_DBG_REC_MAX  POS_DBG_SWEEP_LEN

/* Anchor slots per SET record. Two records cover POS_MAX_ANCHORS. */
#define POS_DBG_SET_SLOTS  2u

/* Empty anchor slot in a SET record. */
#define POS_DBG_AID_NONE  0xFFu

/* SWEEP `res` field when the solve failed. */
#define POS_DBG_RES_NONE  0xFFFFu

/* SWEEP `dt_ms` field when the gap exceeded the 16-bit range.
 *
 * Also emitted for the FIRST record of a session, where there is no previous
 * timestamp to difference against. The host disambiguates the two by `seq`:
 * the first record of a session carries seq 0. */
#define POS_DBG_DT_OVF  0xFFFFu

/*
 * SWEEP record, 20 bytes, magic 0xA5, one per sweep.
 *
 *   0  1  0xA5
 *   1  1  seq          ++ per sweep; gaps identify dropped notifications
 *   2  2  dt_ms        uint16 LE, ms since the previous SWEEP record
 *   4  1  epoch        matches the SET record that defines the slots
 *   5  1  flags        b0-3 respond mask, b4-5 tier, b6 moving, b7 solved
 *   6  2  r0           int16 LE, cm -- SIGNED: close-in ranges can go negative
 *   8  2  r1
 *  10  2  r2
 *  12  2  r3
 *  14  4  q0..q3       quality; zero unless enabled
 *  18  2  res          uint16 LE, residual in cm; POS_DBG_RES_NONE if unsolved
 *
 * dt_ms is generated on the tag and is authoritative: host arrival timestamps
 * carry BLE connection-interval jitter and must not be used as the filter's dt.
 *
 * `r_cm` and `q` are POS_MAX_ANCHORS-element arrays indexed by slot; entries
 * whose mask bit is clear are ignored by the host and should be written zero.
 * `q` may be NULL, which encodes all-zero quality.
 *
 * Returns bytes written (POS_DBG_SWEEP_LEN), or 0 if buf is NULL or cap is too
 * small.
 */
size_t pos_dbg_enc_sweep(uint8_t *buf, size_t cap,
                         uint8_t seq, uint16_t dt_ms, uint8_t epoch,
                         uint8_t mask, uint8_t tier, bool moving, bool solved,
                         const int16_t *r_cm, const uint8_t *q,
                         uint16_t res_cm);

/*
 * SET record, 13 bytes, magic 0xA6. Emitted at log start and whenever the
 * selected anchor set changes; two records (base 0 and base 2) cover four
 * anchors. Splitting identity and coordinates out of the per-sweep record is
 * what keeps that record inside 20 bytes.
 *
 * Anchor coordinates arrive over the air in the E1 response and can be
 * misconfigured, so the host must record them rather than assume them.
 *
 *   0  1  0xA6
 *   1  1  epoch        ++ on every selected-set change
 *   2  1  base         0 or 2, which slot pair this record carries
 *   3  1  aid[base+0]  POS_DBG_AID_NONE for an empty slot
 *   4  2  x[base+0]    int16 LE, cm
 *   6  2  y[base+0]    int16 LE, cm
 *   8  1  aid[base+1]
 *   9  2  x[base+1]
 *  11  2  y[base+1]
 *
 * `aid`, `x_cm`, `y_cm` are POS_DBG_SET_SLOTS-element arrays holding just this
 * record's pair (already offset by `base` -- the encoder does not index them
 * by base). Returns bytes written, or 0 on a bad argument.
 */
size_t pos_dbg_enc_set(uint8_t *buf, size_t cap, uint8_t epoch, uint8_t base,
                       const uint8_t *aid, const int16_t *x_cm,
                       const int16_t *y_cm);

/*
 * MARK record, 4 bytes, magic 0xA7. Inserted by `dbg mark` when the operator
 * passes a surveyed point, so a capture can be aligned to ground truth without
 * time synchronisation.
 *
 *   0  1  0xA7
 *   1  1  seq          the SWEEP seq this mark follows
 *   2  1  mark_id      ++ per mark
 *   3  1  0            reserved
 */
size_t pos_dbg_enc_mark(uint8_t *buf, size_t cap, uint8_t seq, uint8_t mark_id);

/* Saturating float metres -> int16 centimetres, for the encoders' callers.
 * Rejects NaN/Inf (returns 0) -- ranges and coordinates arrive over the air
 * and an unchecked cast would be undefined behaviour. */
int16_t pos_dbg_m_to_cm(float m);

/* ---- emit path (Zephyr glue) ---------------------------------------------- */

/* Whether records are being emitted. Off at boot; never persisted. */
bool pos_dbg_enabled(void);

/* Quality bytes require dwt_readdiagnostics() after each RX. Four extra
 * diagnostic reads inside a 24 ms slot already holding a ~22 ms sweep can
 * overrun the slot and cause exactly the inter-tag collisions T_SLOT_MS was
 * sized to prevent -- so this is OFF by default and single-tag bench only. It
 * is not needed to tune the EKF, only for the later per-range weighting work. */
bool pos_dbg_quality_enabled(void);

/* Handle the `dbg ...` NUS command family. Returns true if consumed.
 * Commands: `dbg on|off`, `dbg q on|off`, `dbg mark`. */
bool pos_dbg_on_cmd(const char *cmd);

/*
 * Emit one SWEEP record for a completed sweep, plus any SET records the
 * selected-set change requires. No-op unless enabled. Computes dt_ms and seq
 * internally. Safe to call from the runner thread; never blocks (a full queue
 * drops the record, which the host detects as a seq gap).
 *
 * `aid`, `ax_cm`, `ay_cm`, `r_cm`, `q` are POS_MAX_ANCHORS-element slot arrays;
 * `mask` says which slots responded.
 *
 * CALLER CONTRACT -- `aid`/`ax_cm`/`ay_cm` describe the SELECTED anchor set,
 * not the set that answered this sweep. Change detection compares them across
 * all slots and deliberately ignores `mask`, because the SET record exists to
 * name the slots, and a slot's identity does not stop existing just because
 * that anchor missed one sweep. Populating coordinates only for anchors that
 * responded -- the natural thing to do, since the coordinates arrive in the E1
 * response -- would zero a missing anchor's entry, read as a set change, and
 * flap the epoch every time any anchor dropped a single sweep: three records
 * instead of one on the way out and three more on the way back, on a queue
 * that is already the tightest resource during a capture. Pass a stable
 * snapshot of the selected set, carrying the last known coordinates for a
 * slot that did not answer.
 *
 * Not synchronised. `pos_dbg_on_cmd()` runs on the BT RX thread and mutates
 * the same session state this reads from the runner thread. A `dbg on` landing
 * mid-sweep can therefore produce one record with a half-reset comparison
 * array (a spurious SET burst) or a MARK naming a stale seq. Accepted rather
 * than locked: this is temporary bench instrumentation, the damage is one
 * cosmetically wrong record, and a mutex here would be the only lock in the
 * module.
 */
void pos_dbg_sweep(uint32_t now_ms, const uint8_t *aid, const int16_t *ax_cm,
                   const int16_t *ay_cm, const int16_t *r_cm, const uint8_t *q,
                   uint8_t mask, uint8_t tier, bool moving, bool solved,
                   uint16_t res_cm);

#endif /* POS_DBG_H */
