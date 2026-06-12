#include "uwb_frame_802_15_4z.h"
#include <errno.h>

/* NOTE: the DISC/MPOL/RESP frames use 2-byte little-endian short addresses at
 * bytes 5-6 (dest) and 7-8 (src). This is intentionally distinct from the legacy
 * SS-TWR E0/E1 frames in uwb_ss_initiator.c, whose bytes 5-8 are a 4-char ASCII
 * tag ('WAVE'/'VEWA'); the two frame families are not wire-compatible by design. */

/* ---- Field offsets ---- */
#define OFF_FC0        0
#define OFF_FC1        1
#define OFF_SEQ        2
#define OFF_PAN        3   /* bytes 3-4 */
#define OFF_DEST       5   /* bytes 5-6 */
#define OFF_SRC        7   /* bytes 7-8 */
#define OFF_TYPE       9
#define OFF_DISC_TS    10  /* bytes 10-13 */
#define OFF_RESP_TS    10  /* bytes 10-13 */
#define OFF_RESP_CIRP  14  /* bytes 14-17 */
#define OFF_RESP_CIRQ  18  /* bytes 18-19 */
#define OFF_MPOL_NUM   10
#define OFF_MPOL_SLOTS 11

/* ---- Little-endian field helpers ---- */
static void put_u16(uint8_t *p, uint16_t v) __attribute__((unused));
static void put_u32(uint8_t *p, uint32_t v) __attribute__((unused));
static uint16_t get_u16(const uint8_t *p) __attribute__((unused));
static uint32_t get_u32(const uint8_t *p) __attribute__((unused));
static void write_hdr(uint8_t *buf, uint16_t dest, uint16_t src, uint8_t type) __attribute__((unused));

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;  p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Write the common 10-byte header. PANID is written as literal bytes 0xCA,0xDE
 * (matching existing tag frames), NOT little-endian-encoded. */
static void write_hdr(uint8_t *buf, uint16_t dest, uint16_t src, uint8_t type)
{
    buf[OFF_FC0] = 0x41;
    buf[OFF_FC1] = 0x88;
    buf[OFF_SEQ] = 0;           /* caller sets via uwb_frame_set_seq_num */
    buf[OFF_PAN] = 0xCA;
    buf[OFF_PAN + 1] = 0xDE;
    put_u16(&buf[OFF_DEST], dest);
    put_u16(&buf[OFF_SRC], src);
    buf[OFF_TYPE] = type;
}

/* ---- Builders (STUBS) ---- */
int uwb_frame_discovery_build(uint8_t *buf, size_t buf_len,
                              uint16_t src_addr, uint32_t tx_ts)
{
    if (!buf) {
        return -EINVAL;
    }
    if (buf_len < UWB_FRAME_LEN_DISC) {
        return -EMSGSIZE;
    }
    write_hdr(buf, UWB_FRAME_ADDR_BCAST, src_addr, UWB_FRAME_TYPE_DISC);
    put_u32(&buf[OFF_DISC_TS], tx_ts);
    return UWB_FRAME_LEN_DISC;
}

int uwb_frame_multipoll_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                              const struct uwb_anchor_slot *slots,
                              uint8_t num_slots, uint32_t tx_ts)
{ (void)buf; (void)buf_len; (void)src_addr; (void)slots; (void)num_slots; (void)tx_ts; return -EINVAL; }

int uwb_frame_response_build(uint8_t *buf, size_t buf_len, uint16_t src_addr,
                             uint16_t dest_addr, uint32_t tx_ts,
                             int32_t cir_power, uint16_t cir_quality)
{ (void)buf; (void)buf_len; (void)src_addr; (void)dest_addr; (void)tx_ts;
  (void)cir_power; (void)cir_quality; return -EINVAL; }

/* ---- Parsers (STUBS) ---- */
int uwb_frame_parse_discovery_response(const uint8_t *buf, size_t len,
                                       uint16_t *src_addr, int32_t *cir_power,
                                       uint16_t *cir_quality)
{ (void)buf; (void)len; (void)src_addr; (void)cir_power; (void)cir_quality; return -EINVAL; }

int uwb_frame_parse_multipoll(const uint8_t *buf, size_t len,
                              struct uwb_anchor_slot *slots_out,
                              uint8_t *num_slots, uint32_t *tx_ts)
{ (void)buf; (void)len; (void)slots_out; (void)num_slots; (void)tx_ts; return -EINVAL; }

/* ---- Validators ---- */
bool uwb_frame_is_valid(const uint8_t *buf, size_t len)
{
    if (!buf) {
        return false;
    }
    if (len < UWB_FRAME_HDR_LEN || len > UWB_FRAME_MAX_LEN) {
        return false;
    }
    if (buf[OFF_FC0] != 0x41 || buf[OFF_FC1] != 0x88) {
        return false;
    }
    if (buf[OFF_PAN] != 0xCA || buf[OFF_PAN + 1] != 0xDE) {
        return false;
    }
    return true;
}

bool uwb_frame_is_discovery(const uint8_t *buf, size_t len)
{
    return uwb_frame_is_valid(buf, len) &&
           buf[OFF_TYPE] == UWB_FRAME_TYPE_DISC &&
           len >= UWB_FRAME_LEN_DISC;
}

bool uwb_frame_is_multipoll(const uint8_t *buf, size_t len)  { (void)buf; (void)len; return false; }
bool uwb_frame_is_response(const uint8_t *buf, size_t len)   { (void)buf; (void)len; return false; }

/* ---- Utilities ---- */
uint16_t uwb_frame_get_src_addr(const uint8_t *buf)  { return get_u16(&buf[OFF_SRC]); }
uint16_t uwb_frame_get_dest_addr(const uint8_t *buf) { return get_u16(&buf[OFF_DEST]); }
uint8_t  uwb_frame_get_seq_num(const uint8_t *buf)   { return buf[OFF_SEQ]; }
void     uwb_frame_set_seq_num(uint8_t *buf, uint8_t seq) { buf[OFF_SEQ] = seq; }
