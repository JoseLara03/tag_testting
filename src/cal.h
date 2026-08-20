#ifndef CAL_H
#define CAL_H

#include <stdint.h>
#include <stdbool.h>
#include "cal_math.h"   /* struct cal_record */

/* Shared NVS id for the calibration record. Both cal.c (read) and cal_run.c
 * (write, cal-image-only) use this -- kept here so the two files can never
 * drift onto different ids. */
#define CAL_NVS_ID  1

/* Mount NVS, load the stored calibration for the current PHY. Returns true if
 * a valid record was loaded. Read-only: never writes NVS. */
bool cal_init(void);

/* Active antenna delays. Meaningful only when cal_is_valid() is true. */
void cal_get_ant_dly(uint16_t *tx, uint16_t *rx);

/* True if a valid calibration is currently loaded/active. */
bool cal_is_valid(void);

/* NUS command parser: recognizes only `cal status`. Every other `cal ...`
 * line gets a "not available in this build" error -- production can read a
 * calibration but never start, clear, or overwrite one. The calibration
 * image additionally registers cal_run_on_rx() (src/cal_run.h) ahead of this
 * handler in tag_cmd.c's dispatch for every other `cal ...` command. */
void cal_on_rx(const uint8_t *data, uint16_t len);

/* ---- cal-image-only writers: called only by src/cal_run.c ----
 *
 * `active`/`active_valid` are owned here so cal_is_valid()/cal_get_ant_dly()
 * always see a consistent value; cal_run.c calls these instead of touching
 * the state directly. Harmless to declare unconditionally -- declared here,
 * defined in cal.c (present in every build), and the only caller is
 * cal_run.c, which no production build compiles. */
void cal_internal_activate(const struct cal_record *r);
void cal_internal_invalidate(void);

#endif /* CAL_H */
