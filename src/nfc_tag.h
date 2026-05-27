#ifndef NFC_TAG_H
#define NFC_TAG_H

/* Initialize NFC T4T emulation.
 * Default payload: https://google.mx (URI record).
 * On phone write: parses first NDEF record and sends via ble_log_send().
 * Writes arriving before BLE client connects are silently dropped.
 * Returns 0 on success, negative errno on failure. */
int nfc_tag_init(void);

#endif /* NFC_TAG_H */
