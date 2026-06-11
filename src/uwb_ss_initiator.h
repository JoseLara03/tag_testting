#ifndef UWB_SS_INITIATOR_H_
#define UWB_SS_INITIATOR_H_

/* Start the interrupt-driven SS-TWR initiator (spawns ranging + BLE threads).
 * Call after uwb_init() has configured the DW3000 PHY. */
void uwb_ss_initiator_start(void);

#endif /* UWB_SS_INITIATOR_H_ */
