#ifndef BATT_H_
#define BATT_H_

/* Read battery state of charge as a percentage (0-100).
 * Returns 0 on success and writes *soc; negative errno on failure
 * (e.g. -ENODEV when the gauge is not ready / no battery attached). */
int batt_read_soc(int *soc);

#endif /* BATT_H_ */
