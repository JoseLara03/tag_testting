# LiPo State-of-Charge by voltage approximation (nPM1304) — Design

**Date:** 2026-08-07
**Status:** Design approved, pre-implementation
**Depends on:** the nPM1304 charger node added in `boards/Innovaforce/nRF52833_tag/nRF52833_tag.dts`
(`npm1304_charger`), the `nordic,npm1304-charger` sensor driver, `src/batt.c`, and the
battery display in `src/tag_ui.c` (`led_show_battery()`).
**Scope:** Give `batt_read_soc()` a real implementation — currently `-ENOSYS` — by mapping the
battery voltage the nPM1304 already measures onto a percentage through a LiPo discharge curve,
and report charger presence instead of a bogus percentage while VBUS is connected. **Out of
scope:** the nRF Fuel Gauge library, battery profiling, temperature compensation, and IR-drop
compensation (see §6).

---

## 1. Motivation

The migration from the bq274xx to the nPM1304 removed the only source of state-of-charge in the
system: the `nordic,npm1304-charger` driver measures voltage, current and charger status, but
does **not** expose `SENSOR_CHAN_GAUGE_STATE_OF_CHARGE`. `batt_read_soc()` was left returning
`-ENOSYS`, so `led_show_battery()` always falls into its error branch and the tag shows dim blue
on every button press.

The official path to a real percentage is the nRF Fuel Gauge library plus a battery model. That
model does not exist for this cell: nrfxlib ships models only for **primary** cells
(`nrfxlib/nrf_fuel_gauge/include/battery_models/primary_cell/` — alkaline and CR2032), and the two
`.inc` files in `nrf/samples/pmic/native/npm13xx_fuel_gauge/src/` characterise Nordic's own EK
cells, not ours. Generating one requires an nPM1300/nPM1304-EK and hours of charge/discharge
cycling with the Battery Profiler.

This design deliberately takes the cheap approximation instead, with its error budget stated up
front (§3).

**Success criteria:** a button press reports a plausible percentage that tracks the resting
voltage measured with a multimeter within the accuracy stated in §3, reports charging as a
distinct state rather than a false ~100%, and always logs the current draw in mA.

---

## 2. The cell

Confirmed with the user, not inherited from the old bq274xx node:

| Parameter | Value |
|---|---|
| Chemistry | Standard lithium-polymer (LiCoO2) |
| Full charge | 4.2 V |
| Nominal | 3.7 V |
| Effectively empty | ~3.5 V |

Note this **corrects** the `terminate-voltage = <3000>` that the removed `bq274xx@55` node
declared. 0% is anchored at 3500 mV, not 3000 mV. The `term-microvolt = <4200000>` already set on
the `npm1304_charger` node is correct for this chemistry and is not changed.

---

## 3. The curve and its error budget

Resting open-circuit-voltage table for a standard LiPo, anchored to §2, with linear interpolation
between points and saturation outside the range:

```
mV:   4200 4150 4110 4080 4020 3980 3950 3910 3870 3850 3840
%:     100   95   90   85   80   75   70   65   60   55   50

mV:   3820 3800 3790 3770 3750 3730 3710 3690 3610 3500
%:      45   40   35   30   25   20   15   10    5    0
```

**Known limitation, accepted by design:** roughly half the cell's usable capacity sits between
3700 and 3850 mV. That is 150 mV spanning ~50 percentage points, so ±30 mV of error — from UWB
burst load sag, or the nPM1304 ADC's ~4.9 mV/LSB — is about **±10% SoC** in the middle of the
range. The reading is faithful near the extremes and a coarse estimate in the plateau. This is
inherent to voltage-only estimation and cannot be tightened without profiling the cell.

---

## 4. Components

### 4.1 `src/batt.c`

Three additions. Only `batt_read_soc()` is public; its signature is unchanged.

- **`lipo_mv_to_pct(int mv)`** — the table above plus linear interpolation. A pure function: no
  device access, no state. This is the only piece with real logic, and keeping it pure is what
  makes §5 possible.
- **`vbus_present()`** — wraps
  `sensor_attr_get(fg, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT, &v)`,
  returning `v.val1 != 0`. Verified against the driver: this attribute performs its own I²C read
  of `VBUS_OFFSET_STATUS` and does not require a preceding `sensor_sample_fetch()`
  (`npm13xx_charger.c:435-444`). **If the query itself fails, assume no charger** — degrading to a
  possibly-inflated percentage beats refusing to report anything.
- **`batt_read_soc(int *soc)`** — returns `-EBUSY` when VBUS is present, because a charging cell's
  terminal voltage says nothing about its charge. Otherwise `sensor_sample_fetch()` +
  `sensor_channel_get(SENSOR_CHAN_GAUGE_VOLTAGE)`, converted to mV and passed through
  `lipo_mv_to_pct()`. Existing `-EINVAL` / `-ENODEV` / `-EIO` paths are kept.

`batt_read_current()` is not modified.

### 4.2 `src/batt.h`

Replace the `-ENOSYS` comment on `batt_read_soc()` with the real contract, documenting `-EBUSY`
as "charging, percentage not meaningful" and stating the §3 accuracy limitation so callers do not
over-trust the number.

### 4.3 `src/tag_ui.c`

`led_show_battery()` currently logs the current only in the success path, so it vanished exactly
when the SoC was unavailable. Restructure so that:

1. **The mA reading is always logged**, in every branch — it comes straight from the nPM1304 and
   depends on no model, so there is no reason to gate it behind the SoC.
2. A new branch ahead of the generic error handler catches `-EBUSY`: log `"BATT: charging"` and
   set the LED to **cyan** (`led_set(0, 8, 8)`), distinct from both the dim blue "no data" and the
   four level colours.
3. Every other branch, the four SoC colour thresholds, and the gesture logic in `ui_fn()` are
   unchanged.

---

## 5. Testing

`lipo_mv_to_pct()` is a pure function, so it is tested directly on the host before any target
build — no PMIC, no board:

- Exact table points return their exact percentage.
- Midpoints between entries interpolate linearly.
- Monotonicity: percentage never decreases as voltage rises across the full 3400–4300 mV sweep.
- Saturation: ≥4200 mV → 100, ≤3500 mV → 0, and out-of-range inputs clamp rather than extrapolate.

The device-touching parts (`vbus_present()`, the fetch path) are thin wrappers over the driver and
are covered by the build plus on-hardware checks rather than unit tests.

Then: build for `nRF52833_tag` and report flash/RAM. Expected cost is small — the table is ~170
bytes plus the interpolation code; no new library is linked.

**Hardware validation is the user's:** compare the reported percentage against the resting
terminal voltage measured with a multimeter, and confirm the cyan/charging branch triggers when
USB is plugged in.

---

## 6. Explicitly not included

- **nRF Fuel Gauge library and battery model.** No `CONFIG_NRF_FUEL_GAUGE`, no `.inc`. Revisit
  when an EK is available to profile the real cell; that path also unlocks time-to-empty and
  time-to-full.
- **Temperature.** Not used by this method. This matters because the charger node sets
  `thermistor-ohms = <0>`, which makes `SENSOR_CHAN_GAUGE_TEMP` return `-ENOTSUP`
  (`npm13xx_charger.c:262-265`) — a real blocker for the fuel-gauge path, but a non-issue here.
  Worth knowing that Nordic's own sample ignores that return value.
- **IR-drop compensation.** Estimating open-circuit voltage as `V + I×R_internal` would steady the
  reading during UWB bursts, but `R_internal` for this cell is unmeasured, and a wrong value makes
  the estimate worse rather than better.
- **The PMIC rails and charger settings.** BUCK1/BUCK2/LDO1/LDO2 and the charger parameters stay
  exactly as validated with a multimeter during bring-up.
