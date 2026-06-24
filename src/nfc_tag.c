#include <zephyr/kernel.h>
#include <string.h>
#include <errno.h>
#include <nfc_t4t_lib.h>
#include <nfc/ndef/msg_parser.h>
#include <nfc/ndef/record.h>
#include "storage.h"
#include "nfc_tag.h"

#define NDEF_BUF_SIZE   256
#define NFC_NAME_NVS_ID 2
#define NFC_NAME_MAX    32

static uint8_t ndef_buf[NDEF_BUF_SIZE];
static char    current_name[NFC_NAME_MAX + 1];

/*
 * Encode current_name into ndef_buf as a raw NDEF T4T file:
 *   [NLEN_H][NLEN_L] [0xD1][0x01][payload_len]['T'][0x02]['e']['n'][name...]
 *
 * NLEN is big-endian length of the NDEF message (everything after the 2-byte NLEN field).
 * 0xD1 = MB|ME|SR|TNF_WELL_KNOWN. Payload = status(1) + lang(2) + name.
 * Max name 32 chars → payload 35 bytes → SR (1-byte length) is sufficient.
 */
static int encode_name(void)
{
    uint32_t name_len    = (uint32_t)strlen(current_name);
    uint32_t payload_len = 1U + 2U + name_len; /* status + "en" + name */
    uint32_t msg_len     = 4U + payload_len;    /* header + type_len + payload_len_byte + type + payload */

    if (2U + msg_len > NDEF_BUF_SIZE) {
        return -ENOMEM;
    }

    uint8_t *p = ndef_buf;

    /* NLEN (big-endian) */
    *p++ = (uint8_t)(msg_len >> 8);
    *p++ = (uint8_t)(msg_len & 0xFFU);

    /* NDEF record header: MB=1 ME=1 CF=0 SR=1 IL=0 TNF=Well-Known */
    *p++ = 0xD1U;
    /* Type length */
    *p++ = 0x01U;
    /* Payload length (1 byte, SR=1) */
    *p++ = (uint8_t)payload_len;
    /* Type: 'T' */
    *p++ = 'T';
    /* Status byte: UTF-8 (bit7=0), lang-code length = 2 */
    *p++ = 0x02U;
    /* Language code */
    *p++ = 'e';
    *p++ = 'n';
    /* Name */
    memcpy(p, current_name, name_len);

    return 0;
}

static void on_nfc_write(const uint8_t *file_buf, uint32_t nlen)
{
    /* file_buf = ndef_buf; first 2 bytes are NLEN, then the NDEF message */
    uint8_t result_buf[NFC_NDEF_PARSER_REQUIRED_MEM(1)] __aligned(4);
    uint32_t result_len  = sizeof(result_buf);

    if (nlen == 0U) {
        return;
    }

    const uint8_t *msg_data = file_buf + 2; /* skip 2-byte NLEN field */
    uint32_t msg_len        = nlen;

    if (nfc_ndef_msg_parse(result_buf, &result_len, msg_data, &msg_len) != 0) {
        return;
    }

    const struct nfc_ndef_msg_desc *msg =
        (const struct nfc_ndef_msg_desc *)result_buf;

    if (msg->record_count == 0) {
        return;
    }

    const struct nfc_ndef_record_desc *rec = msg->record[0];
    const struct nfc_ndef_bin_payload_desc *pay =
        (const struct nfc_ndef_bin_payload_desc *)rec->payload_descriptor;

    if (pay == NULL || pay->payload == NULL) {
        return;
    }

    /* Accept only NDEF Text records: TNF=Well-Known, type='T' */
    if (rec->tnf != TNF_WELL_KNOWN || rec->type_length != 1 ||
        rec->type == NULL || rec->type[0] != 'T' || pay->payload_length < 1) {
        return;
    }

    /* Payload: [status][lang...][text...] — bits5:0 of status = lang code length */
    uint8_t  lang_len   = pay->payload[0] & 0x3FU;
    uint32_t text_offset = 1U + lang_len;

    if (text_offset >= pay->payload_length) {
        return;
    }

    uint32_t text_len = pay->payload_length - text_offset;

    if (text_len > NFC_NAME_MAX) {
        text_len = NFC_NAME_MAX;
    }

    /* Extract name before encode_name() overwrites ndef_buf */
    memcpy(current_name, pay->payload + text_offset, text_len);
    current_name[text_len] = '\0';

    storage_write(NFC_NAME_NVS_ID, current_name, text_len + 1U);
    (void)encode_name();
}

static void nfc_callback(void *context, nfc_t4t_event_t event,
                         const uint8_t *data, size_t data_length, uint32_t flags)
{
    ARG_UNUSED(context);
    ARG_UNUSED(flags);

    if (event == NFC_T4T_EVENT_NDEF_UPDATED && data_length > 0) {
        on_nfc_write(data, (uint32_t)data_length);
    }
}

int nfc_tag_init(void)
{
    /* Load persisted name or fall back to factory default */
    int rc = storage_read(NFC_NAME_NVS_ID, current_name, sizeof(current_name));

    if (rc <= 0) {
        strncpy(current_name, "User name", NFC_NAME_MAX);
    }
    current_name[NFC_NAME_MAX] = '\0'; /* always guarantee null-termination */

    int err = encode_name();

    if (err) {
        return err;
    }

    err = nfc_t4t_setup(nfc_callback, NULL);
    if (err) {
        return err;
    }

    err = nfc_t4t_ndef_rwpayload_set(ndef_buf, sizeof(ndef_buf));
    if (err) {
        return err;
    }

    return nfc_t4t_emulation_start();
}
