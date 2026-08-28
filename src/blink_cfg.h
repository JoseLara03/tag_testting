#ifndef BLINK_CFG_H_
#define BLINK_CFG_H_

#include <stdbool.h>

/* TDoA transmit mode, persisted in NVS (id 5).
 *
 * DEFAULT OFF, and that default is load-bearing: a board that is flashed and
 * not told otherwise must keep running the TWR sweep and reporting 0xEA POS
 * frames, because the anchor-side antenna-delay bring-up depends on exactly
 * those fixes. This switch only chooses what the tag emits in the cadence
 * slot uwb_net.c already schedules -- see UWB_ACT_SEND_BLINK in uwb_net.h.
 *
 * Same persistence shape as pos_cfg.c (magic/version/pad/crc32), which
 * CLAUDE.md names as the pattern to follow. */
int  blink_cfg_init(void);
bool blink_cfg_enabled(void);
int  blink_cfg_set(bool enabled);

/* NUS command handler for the whole `blink` keyword. Returns true if the
 * command was claimed (answered), false if it was not a `blink` command.
 * Runs on the BT RX thread. */
bool blink_cfg_on_cmd(const char *cmd);

#endif /* BLINK_CFG_H_ */
