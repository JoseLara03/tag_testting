#ifndef UWB_FRAME_802_15_4Z_H
#define UWB_FRAME_802_15_4Z_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Message-type tags (byte 9), continuing the existing 0xE0/0xE1 series ---- */
#define UWB_FRAME_TYPE_DISC   0xE2
#define UWB_FRAME_TYPE_MPOL   0xE3
#define UWB_FRAME_TYPE_RESP   0xE4

#define UWB_FRAME_PANID       0xCADE  /* written as literal bytes 0xCA,0xDE */
#define UWB_FRAME_ADDR_BCAST  0xFFFF
#define UWB_FRAME_MAX_ANCHORS 4

#define UWB_FRAME_HDR_LEN     10  /* bytes 0-9 common header */
#define UWB_FRAME_LEN_DISC    14
#define UWB_FRAME_LEN_RESP    20
#define UWB_FRAME_MAX_LEN     31  /* 4-anchor multi-poll, excl. FCS */

struct uwb_anchor_slot {
    uint16_t addr;      /* anchor short address */
    uint16_t delay_us;  /* staggered response delay, microseconds */
};

/* ---- Builders: return bytes written (excl. FCS), or negative errno ---- */
int uwb_frame_discovery_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr, uint32_t tx_ts);
int uwb_frame_multipoll_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr,
                              const struct uwb_anchor_slot *slots,
                              uint8_t num_slots, uint32_t tx_ts);
int uwb_frame_response_build(uint8_t *buf, size_t buf_len,
                             uint16_t src_addr, uint16_t dest_addr,
                             uint32_t tx_ts, int32_t cir_power,
                             uint16_t cir_quality);

/* ---- Parsers: return 0 on success, negative errno on bad frame ---- */
int uwb_frame_parse_discovery_response(const uint8_t *buf, size_t len,
                                       uint16_t *src_addr,
                                       int32_t *cir_power,
                                       uint16_t *cir_quality);
int uwb_frame_parse_multipoll(const uint8_t *buf, size_t len,
                              struct uwb_anchor_slot *slots_out,
                              uint8_t *num_slots, uint32_t *tx_ts);

/* ---- Validators ---- */
bool uwb_frame_is_valid(const uint8_t *buf, size_t len);
bool uwb_frame_is_discovery(const uint8_t *buf, size_t len);
bool uwb_frame_is_multipoll(const uint8_t *buf, size_t len);
bool uwb_frame_is_response(const uint8_t *buf, size_t len);

/* ---- Utilities ---- */
uint16_t uwb_frame_get_src_addr(const uint8_t *buf);
uint16_t uwb_frame_get_dest_addr(const uint8_t *buf);
uint8_t  uwb_frame_get_seq_num(const uint8_t *buf);
void     uwb_frame_set_seq_num(uint8_t *buf, uint8_t seq);

#endif /* UWB_FRAME_802_15_4Z_H */
