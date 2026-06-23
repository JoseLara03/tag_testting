#ifndef BATT_H_
#define BATT_H_

/* Read battery state of charge as a percentage (0-100).
 * Returns 0 on success and writes *soc; negative errno on failure
 * (e.g. -ENODEV when the gauge is not ready / no battery attached). */
int batt_read_soc(int *soc);

/* Read average battery current in milliamps (discharge as a positive
 * magnitude). Returns 0 on success; negative errno on failure. */
int batt_read_current(int *ma);

#endif /* BATT_H_ */
