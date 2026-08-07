#ifndef BATT_CURVE_H_
#define BATT_CURVE_H_

/* Map a resting LiPo terminal voltage (millivolts) to a state-of-charge
 * percentage (0-100), by linear interpolation over a standard lithium-polymer
 * discharge curve: 4200 mV = full, 3500 mV = empty.
 *
 * Pure function, no device access — see tests/batt_curve/.
 *
 * Accuracy: about half the cell's usable capacity sits between 3700 and
 * 3850 mV, so in that plateau ~30 mV of error (load sag, ADC resolution) is
 * worth ~10 percentage points. Faithful near the extremes, coarse in the
 * middle. Inherent to voltage-only estimation.
 *
 * Voltages outside the curve saturate: >= 4200 returns 100, <= 3500 returns 0.
 * Only meaningful for a resting cell — a charging cell reads high. */
int lipo_mv_to_pct(int mv);

#endif /* BATT_CURVE_H_ */
