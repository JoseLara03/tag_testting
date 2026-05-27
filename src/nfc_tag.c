#include <zephyr/kernel.h>
#include <string.h>
#include <nfc_t4t_lib.h>
#include <nfc/ndef/uri_msg.h>
#include <nfc/t4t/ndef_file.h>
#include <nfc/ndef/msg_parser.h>
#include <nfc/ndef/record.h>
#include "ble_log.h"
#include "nfc_tag.h"

#define NDEF_BUF_SIZE 256

static uint8_t ndef_buf[NDEF_BUF_SIZE];

static const char *uri_prefix(uint8_t id)
{
    switch (id) {
    case 0x01: return "http://www.";
    case 0x02: return "https://www.";
    case 0x03: return "http:";
    case 0x04: return "https:";
    case 0x05: return "tel:";
    case 0x06: return "mailto:";
    default:   return "";
    }
}

static void parse_and_forward(const uint8_t *file_buf, uint32_t nlen)
{
    /* result_buf must be 4-byte aligned per nfc_ndef_msg_parse requirement */
    uint8_t result_buf[NFC_NDEF_PARSER_REQUIRED_MEM(1)] __aligned(4);
    uint32_t result_len = sizeof(result_buf);
    const uint8_t *msg_data = file_buf + NFC_NDEF_FILE_NLEN_FIELD_SIZE;
    uint32_t msg_len = nlen;
    char out[256];

    if (nfc_ndef_msg_parse(result_buf, &result_len, msg_data, &msg_len) != 0) {
        /* Parse failed — hex dump first 16 bytes */
        int pos = snprintf(out, sizeof(out), "NFC_RAW:");
        for (uint32_t i = 0; i < msg_len && i < 16 && pos < (int)sizeof(out) - 4; i++) {
            pos += snprintf(out + pos, sizeof(out) - pos, " %02X", msg_data[i]);
        }
        snprintf(out + pos, sizeof(out) - pos, "\n");
        ble_log_send(out);
        return;
    }

    const struct nfc_ndef_msg_desc *msg = (const struct nfc_ndef_msg_desc *)result_buf;

    if (msg->record_count == 0) {
        return;
    }

    const struct nfc_ndef_record_desc *rec = msg->record[0];
    const struct nfc_ndef_bin_payload_desc *pay =
        (const struct nfc_ndef_bin_payload_desc *)rec->payload_descriptor;

    if (pay == NULL || pay->payload == NULL) {
        return;
    }

    if (rec->tnf == TNF_WELL_KNOWN && rec->type_length == 1 &&
        rec->type != NULL && rec->type[0] == 'U' && pay->payload_length >= 1) {
        /* URI record: payload[0] = prefix ID, payload[1..] = URI string */
        snprintf(out, sizeof(out), "NFC: %s%.*s\n",
                 uri_prefix(pay->payload[0]),
                 (int)(pay->payload_length - 1),
                 (const char *)(pay->payload + 1));

    } else if (rec->tnf == TNF_WELL_KNOWN && rec->type_length == 1 &&
               rec->type != NULL && rec->type[0] == 'T' && pay->payload_length >= 1) {
        /* Text record: payload[0] = status byte, bits[5:0] = lang code length */
        uint8_t lang_len = pay->payload[0] & 0x3F;
        uint32_t text_offset = 1U + lang_len;

        if (text_offset < pay->payload_length) {
            snprintf(out, sizeof(out), "NFC: %.*s\n",
                     (int)(pay->payload_length - text_offset),
                     (const char *)(pay->payload + text_offset));
        } else {
            snprintf(out, sizeof(out), "NFC: (empty)\n");
        }
    } else {
        /* Unknown record type — hex dump payload */
        int pos = snprintf(out, sizeof(out), "NFC_RAW:");
        for (uint32_t i = 0; i < pay->payload_length && i < 16 &&
             pos < (int)sizeof(out) - 4; i++) {
            pos += snprintf(out + pos, sizeof(out) - pos, " %02X", pay->payload[i]);
        }
        snprintf(out + pos, sizeof(out) - pos, "\n");
    }

    ble_log_send(out);
}

static void nfc_callback(void *context, nfc_t4t_event_t event,
                         const uint8_t *data, size_t data_length, uint32_t flags)
{
    ARG_UNUSED(context);
    ARG_UNUSED(flags);

    if (event == NFC_T4T_EVENT_NDEF_UPDATED && data_length > 0) {
        parse_and_forward(data, (uint32_t)data_length);
    }
}

int nfc_tag_init(void)
{
    uint32_t msg_size = nfc_t4t_ndef_file_msg_size_get(NDEF_BUF_SIZE);
    uint8_t *msg_ptr  = nfc_t4t_ndef_file_msg_get(ndef_buf);

    /* Encode https://google.mx URI into NDEF content area (after NLEN) */
    int err = nfc_ndef_uri_msg_encode(NFC_URI_HTTPS,
                                      (const uint8_t *)"//google.mx",
                                      (uint16_t)strlen("//google.mx"),
                                      msg_ptr, &msg_size);
    if (err) {
        return err;
    }

    /* Write big-endian NLEN into ndef_buf[0..1] */
    err = nfc_t4t_ndef_file_encode(ndef_buf, &msg_size);
    if (err) {
        return err;
    }

    err = nfc_t4t_setup(nfc_callback, NULL);
    if (err) {
        return err;
    }

    /* Read-write mode: phones can both read and overwrite the tag */
    err = nfc_t4t_ndef_rwpayload_set(ndef_buf, sizeof(ndef_buf));
    if (err) {
        return err;
    }

    return nfc_t4t_emulation_start();
}
