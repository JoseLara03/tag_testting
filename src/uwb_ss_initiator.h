#ifndef UWB_SS_INITIATOR_H_
#define UWB_SS_INITIATOR_H_

#include <stdbool.h>

/* Start the interrupt-driven SS-TWR initiator (spawns ranging + BLE threads).
 * Call after uwb_init() has configured the DW3000 PHY. */
void uwb_ss_initiator_start(void);

/* Set ranging cadence: true -> fast (1 s), false -> slow (5 s).
 * Safe to call from ISR context. Only the still->moving transition wakes
 * the ranging thread immediately; repeated calls while already moving do
 * not give extra semaphore tokens. */
void uwb_set_moving(bool moving);

#endif /* UWB_SS_INITIATOR_H_ */
