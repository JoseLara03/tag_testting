#ifndef POS_CFG_H
#define POS_CFG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Positioning geometry configuration: anchor mounting height and assumed worn
 * tag height, which together give the `dz` the 3D range model needs.
 *
 * v1 is a single tag-side pair of constants applied to every anchor, because
 * all anchors in the deployment are ceiling-mounted at one height and because
 * per-anchor z would need two more bytes in the E1 ranging response -- i.e.
 * anchor firmware, which is not in this repo. If anchors ever end up at
 * differing heights this becomes wrong and per-anchor z goes into the frame.
 *
 * Persisted in NVS as storage id 4 (1 = cal, 2 = NFC name, 3 = alert state).
 *
 * See spec/2026-08-22-position-filtering-design.md.
 */

/* Fallbacks used when NVS holds no valid record. These are ESTIMATES, not
 * measurements -- see the design's open questions. They are deliberately not
 * zero: dz = 0 is exactly the planar-model bug this module exists to fix, so a
 * plausible default is strictly better than an obviously-wrong one. */
#define POS_CFG_DEF_ANCHOR_H_CM  270
#define POS_CFG_DEF_TAG_H_CM     110

/* Accepted range for either height, centimetres. Guards the NUS setter and a
 * corrupt NVS record. */
#define POS_CFG_H_MIN_CM   0
#define POS_CFG_H_MAX_CM   1500

struct pos_cfg {
    int16_t anchor_h_cm;
    int16_t tag_h_cm;
};

/* Load from NVS, falling back to the defaults above. Safe to call before any
 * position work; returns 0 if a stored record was loaded, -ENOENT if the
 * defaults were applied, or another negative errno on a storage error (the
 * defaults are applied in that case too). */
int pos_cfg_init(void);

void pos_cfg_get(struct pos_cfg *out);

/* Validate against POS_CFG_H_MIN_CM/MAX_CM, apply, and persist to NVS.
 * Returns 0 on success, -EINVAL on an out-of-range value, or the storage
 * error. On -EINVAL nothing is changed. */
int pos_cfg_set(int16_t anchor_h_cm, int16_t tag_h_cm);

/* (anchor_h - tag_h) in metres -- the `dz` for struct pos_meas. May be
 * negative if the tag is worn above the anchor plane; the range model squares
 * it, so the sign does not matter, but the value is returned signed rather
 * than clamped so a mis-set configuration is visible in diagnostics. */
float pos_cfg_dz_m(void);

/* Handle the `pos ...` NUS command family. Returns true if `cmd` was consumed.
 * Commands: `pos z <anchor_cm> <tag_cm>` sets and persists, `pos z` reports the
 * current pair. Replies go out via ble_log/twr_log, 20-byte limit per line. */
bool pos_cfg_on_cmd(const char *cmd);

#endif /* POS_CFG_H */
