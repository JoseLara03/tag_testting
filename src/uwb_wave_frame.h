#ifndef UWB_WAVE_FRAME_H_
#define UWB_WAVE_FRAME_H_

/*
 * Shared WAVE (calibration/positioning poll-response) byte patterns and field
 * offsets, used by uwb_ss_initiator.c (production ranging/calibration) and
 * cal_diag.c (cal-image-only diagnostics). Previously two independent
 * hand-rolled static arrays inside uwb_ss_initiator.c; kept in one place now
 * for the same reason uwb_frame_802_15_4z.c is kept byte-identical to the
 * anchor's copy -- a forked wire format is a bug waiting to happen.
 *
 * The arrays stay per-translation-unit (the sequence number, and for the
 * positioning poll the anchor id, are written into the buffer at runtime, so
 * it cannot be const); only the byte pattern and offsets are shared.
 */

#define ALL_MSG_COMMON_LEN   10
#define ALL_MSG_SN_IDX        2

/* Non-addressed calibration poll/response (10-byte header only). Wire-
 * compatible with a stock Qorvo ss_twr_responder; no anchor id byte, so an
 * ANCLA anchor's anchor_respond_wave_poll() silently drops it (its length
 * check requires an 11th byte). */
#define UWB_WAVE_POLL_INIT  { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0 }
#define UWB_WAVE_RESP_INIT  { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 }
#define UWB_WAVE_RESP_POLL_RX_TS_IDX  10
#define UWB_WAVE_RESP_RESP_TX_TS_IDX  14

/* Addressed positioning poll/response: anchor id at byte 10 identifies the
 * peer and lets an ANCLA anchor self-report (x, y). */
#define UWB_WAVE_POS_POLL_INIT  { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0 }
#define UWB_WAVE_POS_RESP_INIT  { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1 }
#define UWB_WAVE_POS_ANCHOR_ID_IDX     10
#define UWB_WAVE_POS_POLL_RX_TS_IDX    11
#define UWB_WAVE_POS_RESP_TX_TS_IDX    15
#define UWB_WAVE_POS_ANCHOR_X_IDX      19
#define UWB_WAVE_POS_ANCHOR_Y_IDX      23

#endif /* UWB_WAVE_FRAME_H_ */
