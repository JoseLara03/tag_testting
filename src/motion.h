#ifndef MOTION_H_
#define MOTION_H_

/* Configure the LIS2HH12 activity interrupt and the INT1 GPIO.
 * On motion the ranging cadence is set fast; after ~5 s of stillness, slow.
 * Returns 0 on success, negative errno on failure. */
int motion_init(void);

#endif /* MOTION_H_ */
