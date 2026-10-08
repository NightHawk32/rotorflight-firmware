# Blackbox tuning: altitude hold, position hold, hard deck

This page explains how to record blackbox logs for tuning the state estimator,
Altitude Hold, Position Hold and the Hard Deck, and what each debug field means.

A blackbox log records **one** `debug_mode` (8 values). Each debug mode below
contains everything needed for one tuning job, so every test flight uses exactly
one debug mode. The log header also records all related settings (see
[§5](#5-settings-recorded-in-the-log-header)), so a log can be analysed without a
separate CLI dump.

Related docs: [MicroLink-AltHold-PosHold.md](MicroLink-AltHold-PosHold.md),
[HardDeck.md](HardDeck.md).

---

## 1. One-time blackbox setup

```
set blackbox_log_setpoint = ON
set blackbox_log_command = ON
set blackbox_log_attitude = ON
set blackbox_log_gyro = ON
set blackbox_log_acc = ON
set blackbox_log_alt = ON
set blackbox_log_gps = ON
set blackbox_log_mixer = ON
set blackbox_log_governor = ON
save
```

**Logging rate.** The estimator runs at 100 Hz and the MicroLink at 50 Hz. A log
rate of about **500 Hz** is enough for everything on this page:

```
set blackbox_rate_denom = <PID loop rate / 500>      # e.g. 8 at a 4 kHz PID loop
```

The PID loop rate is in the log header as `pid_rate_hz`. Use a higher rate only
when you tune the normal rate PIDs in the same flight.

Why each field group matters:

| Field group | Used for |
| --- | --- |
| attitude | Heading for the flow rotation, tilt for flow scaling and the collective tilt factor |
| gyro | Checking whether rotation leaks into the optical flow |
| acc | Checking the IMU vertical acceleration that drives the prediction |
| setpoint, command | Collective and cyclic demand, stick input |
| alt | The general altitude output (`getAltitude()`) |
| GPS | GPS position, altitude, ground speed and course (the estimator's GPS input) |
| mixer, governor | Collective actually applied, headspeed (baro downwash depends on it) |

---

## 2. What to send back with a log

1. The log file (`.bbl` / `.bfl`, or `.txt` from the SD card).
2. Which test from [§3](#3-test-flights) it is, and which `debug_mode` was set.
3. A short timeline: when each switch was flipped and what you did (e.g.
   "0:40 ALTHOLD on, 0:55 collective up for 3 s, 1:20 POSHOLD on, pushed
   right").
4. Conditions: wind, surface under the model (grass, concrete, carpet),
   indoor or outdoor, height flown.
5. After changing hardware or mounting: a photo or description of where the
   MicroLink and the FC/baro sit.

`diff all` is only needed when something outside the settings in §5 is
suspected.

---

## 3. Test flights

Do them in this order. Each one builds on the previous one.

| # | Test | `debug_mode` | What to fly |
| --- | --- | --- | --- |
| 1 | Flow signs (bench, props off) | `OPTICAL_FLOW` | Hold the model level ~1 m over a textured floor. Slide it **forward** 1 m, back, then **right** 1 m, back. Then, without moving it, pitch it forward/back and roll it left/right by ~20°. |
| 2 | LIDAR chain | `RANGEFINDER` | Hover at 0.5, 1, 2, 4 m, each for ~10 s. Fly slowly over an edge (table, step, kerb). |
| 3 | Altitude estimator | `POS_EST_Z` | Hover, slow climbs/descents of 5–10 m, several sharp collective punches up and down, fast cyclic (flips/rolls) if you fly them. Outdoors with a GPS fix, and once without GPS if possible. |
| 3b | Terrain | `POS_EST_TERRAIN` | Hover 20 s over one spot at ~1.5 m, then fly slowly over a step (table, kerb, bank) and back, then a 20 m traverse over flat ground at ~2 m/s. |
| 4 | Altitude Hold | `ALTHOLD` | Hover, ALTHOLD on, hands off collective for 20 s. Then collective up ~1 s, release, wait 10 s; same down. Repeat once at a different height. |
| 5 | Horizontal estimator | `POS_EST_XY` | Manual hover (no POSHOLD) for 20 s, then translate 5 m forward and back, 5 m right and back, and a slow 360° pirouette. |
| 6 | Position Hold | `POSHOLD` | ALTHOLD + POSHOLD on, hands off for 30 s. Push cyclic in each direction for ~1 s and release. Yaw 90° and repeat one push. |
| 7 | Hard deck | `HARDDECK` | Follow [HardDeck.md §5](HardDeck.md#5-testing-procedure). |

For tests 4, 6 and 7, fly one log with default gains first, then change one thing at a time.

---

## 4. Debug field reference

Unless stated otherwise:
- altitudes and positions are in **cm**, velocities in **cm/s**;
- the altitude frame is relative to the **arm point**;
- horizontal values are earth frame **East/North**;
- angles are in **centidegrees**.

### `OPTICAL_FLOW`

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Flow X (forward) | Module units, "cm/s at 1 m", body frame after `optical_flow_align`. Updated at 50 Hz |
| 1 | Flow Y (left) | Same |
| 2 | Flow quality 0–255 | Not fused at 50 or below |
| 3 | Height used for scaling | AGL, cm |
| 4 | Flow scale ×100 | height / cos²(tilt) |
| 5 | Velocity forward | cm/s, last fused sample |
| 6 | Velocity right | cm/s, last fused sample |
| 7 | Status of latest sample | 0 fused, 1 disabled by `position_xy_source`, 2 no sensor / timeout, 3 no valid AGL, 4 quality too low, 5 tilt above 45° |

Test 1 expectation: the estimator only fuses flow while **armed**, so on a
disarmed bench fields 3-7 do not update. Use the raw fields: sliding
**forward** must make field 0 positive, and sliding **right** must make field 1
negative (Y points left). The configurator's Position & Hold tab shows the same
check as an arrow. Pitching or rolling in place moves fields 0 and 1 in step
with gyro pitch and roll: the module does not compensate for rotation, the
estimator does (`position_flow_gyro_comp`). Check that part **armed with props
off**: fields 5/6 (velocity after compensation) must stay near zero while
rotating in place. If they move about as much as without compensation but with
the opposite sign, set `position_flow_gyro_comp = -100`; if they still move
with the same sign, the module axes do not match the body frame: fix
`optical_flow_align`. The configurator's **Optical flow orientation** check
(Position & Hold tab) does both measurements disarmed: two slides give the
orientation, rocking in place gives `position_flow_gyro_comp` from the gyro
rates in `MSP2_GET_POSITION_STATUS` (payload version 3).

### `RANGEFINDER`

| Field | Value | Notes |
| --- | --- | --- |
| 1 | Raw distance | cm, median filtered, from `sensors/rangefinder.c` |
| 2 | Tilt-compensated altitude | cm |
| 3 | SNR | MicroLink: 255 − signal strength (lower is better) |
| 4 | AGL altitude | cm, updated once per sample |
| 5 | AGL vario | cm/s |
| 6 | AGL reliability | 0–1000 ‰; valid at ≥ 330 |
| 7 | AGL valid | 0/1 (reliability and sensor health) |

### `POS_EST_Z` (vertical Kalman filter, 100 Hz)

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Fused altitude | Filter state |
| 1 | Fused vertical velocity | Filter state |
| 2 | Baro measurement | Raw baro sample minus arm offset, **before** bias removal, updated once per baro sample. Filter view of baro = field 2 − field 5 |
| 3 | GPS altitude measurement | Last fused GPS altitude minus arm offset |
| 4 | Rangefinder measurement | Last fused AGL, aligned to the arm frame. The filter compares it with field 0 − terrain (`POS_EST_TERRAIN` field 4) |
| 5 | Baro bias | Filter state (downwash error) |
| 6 | Altitude 1σ | √P, cm |
| 7 | Flags ×1000 + disturbance ×100 | Decode: `flags = v / 1000`, `disturbance = (v % 1000) / 100`. Flags: 1 anchor fresh (bias can be learned), 2 inverted thrust bias active, 4 last baro sample rejected by the 5σ gate, 8 baro available |

The IMU acceleration input is not logged separately. It can be reconstructed from
`acc` and `attitude`.

### `POS_EST_TERRAIN` (rangefinder into the Z filter, 100 Hz)

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Rangefinder raw distance | cm, median filtered |
| 1 | AGL altitude | cm, tilt compensated |
| 2 | Rangefinder measurement | As fused: AGL − alignment offset (field 7) |
| 3 | Rangefinder innovation | Measurement − (altitude − terrain), cm. Large values while hovering mean the lidar and the IMU/baro disagree |
| 4 | Terrain offset | Filter state: ground height under the model relative to the arm point, cm |
| 5 | Terrain 1σ | √P, cm. Grows while moving, shrinks with each lidar sample |
| 6 | Horizontal speed used | cm/s, drives the terrain random walk (0 = terrain frozen) |
| 7 | Alignment offset | `rfAltOffset`, cm, set on the first valid sample after arming |

Test 3b expectation: over one spot fields 4 and 5 stay flat. Over a step, field 4
follows the step within about a second while `POS_EST_Z` field 0 stays level.
If the fused altitude follows the step instead, raise `position_est_q_terrain`;
if field 4 wanders on flat ground, lower it.

### `ALTHOLD`

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Target altitude | In the frame of field 1 |
| 1 | Current altitude used | AGL when flag 2 is set, otherwise the fused altitude |
| 2 | Current vario used | Same source as field 1 |
| 3 | P term ×10 | Collective units (0–1000) ×10 |
| 4 | I term ×10 | Same |
| 5 | D term ×10 | Same (damping on vario) |
| 6 | Output collective | 0–1000. When not engaged: the pass-through collective |
| 7 | Flags | 1 engaged, 2 using AGL, 4 altitude source valid, 8 pilot moving target with the stick, 16 mode on but yielding to rescue/failsafe |

The output is `(althold_hover_collective + P + I + D) × cos²(tilt)`, clamped to
0–1000. The velocity command is
`clamp(althold_alt_p_gain/10 × (target − current), ±althold_max_climb_rate)`.

### `POSHOLD`

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Position error East | Hold target − estimate |
| 1 | Position error North | |
| 2 | Estimated velocity East | |
| 3 | Estimated velocity North | |
| 4 | Wind trim (I term) East | Centidegrees of tilt |
| 5 | Wind trim (I term) North | Centidegrees of tilt |
| 6 | Roll command | Centidegrees, body frame, + = right |
| 7 | Pitch command | Centidegrees, body frame, + = forward |

All fields read 0 while Position Hold is not engaged (mode off, ALTHOLD off, or
no valid XY estimate). The velocity command is
`clamp(poshold_pos_p_gain/100 × error, ±poshold_max_horiz_speed)`.

### `POS_EST_XY` (horizontal Kalman filters, 100 Hz)

| Field | Value | Notes |
| --- | --- | --- |
| 0 | Position East | Relative to the arm point |
| 1 | Position North | |
| 2 | Velocity East | |
| 3 | Velocity North | |
| 4 | Flow velocity East | Last fused flow sample |
| 5 | Flow velocity North | |
| 6 | Position 1σ | cm, average of both axes |
| 7 | Flags | Bits 0–4: 1 valid, 2 GPS fused < 500 ms ago, 4 flow fused < 500 ms ago, 8 dead-reckoning clamp active, 16 GPS origin set. **Bits 8+ (`v >> 8`)**: flow status as in `OPTICAL_FLOW` field 7 |

GPS measurements come from the blackbox GPS frames: position relative to home,
ground speed in cm/s, course in decidegrees.

### `HARDDECK`

See [HardDeck.md §3.4](HardDeck.md#34-mode-and-debug).

### `ALTITUDE` (existing mode)

Fields 0/1 are the general altitude and vario (the Kalman filter, or AGL in
`LIDAR_ONLY`). Fields 2–7 are raw baro and GPS, as on master.

---

## 5. Settings recorded in the log header

| Header line | Values |
| --- | --- |
| `althold_gain` | `althold_alt_p_gain`, `_i_gain`, `_d_gain` |
| `althold_limits` | `althold_max_climb_rate`, `althold_stick_deadband`, `althold_hover_collective` |
| `poshold_gain` | `poshold_pos_p_gain`, `poshold_vel_p_gain`, `poshold_vel_i_gain` |
| `poshold_limits` | `poshold_max_horiz_speed`, `poshold_max_tilt_angle`, `poshold_stick_deadband` |
| `harddeck_altitude` | `harddeck_altitude`, `_arm_margin`, `_recovery_margin`, `_release_altitude` |
| `harddeck_predict` | `harddeck_recovery_accel`, `_reaction_time`, `_sigma_factor`, `_use_agl` |
| `rescue_mode` | `rescue_mode`, `rescue_flip_mode` |
| `rescue_gain` | `rescue_level_gain`, `rescue_flip_gain` |
| `rescue_time` | `rescue_pull_up_time`, `_climb_time`, `_flip_time`, `_exit_time` |
| `rescue_collective` | `rescue_pull_up_collective`, `_climb_collective`, `_hover_collective`, `_max_collective` |
| `rescue_alt` | `rescue_hover_altitude`, `rescue_alt_p_gain`, `_i_gain`, `_d_gain` |
| `rescue_setpoint` | `rescue_max_setpoint_rate`, `rescue_max_setpoint_accel` |
| `position_source` | `position_alt_source`, `position_xy_source` (enum index) |
| `position_lpf` | `position_baro_alt_lpf`, `_baro_offset_lpf`, `_gps_alt_lpf`, `_gps_offset_lpf`, `_vario_lpf` |
| `position_gps_min_sats` | `position_gps_min_sats` |
| `position_est_q` | `position_est_q_accel_xy`, `_q_accel_z`, `_q_baro_bias`, `_q_terrain` |
| `position_est_r` | `position_est_r_baro_alt`, `_r_lidar_alt`, `_r_gps_pos`, `_r_gps_vel`, `_r_flow_vel`, `_r_gps_vvel` |
| `position_baro_downwash_comp` | `position_baro_downwash_comp` |
| `position_flow_gyro_comp` | `position_flow_gyro_comp` |
| `rangefinder_hardware`, `optical_flow_hardware` | Enum index |
| `optical_flow_align` | Enum index (`CW0`, `CW90`, `CW180`, `CW270`, then the `FLIP` variants) |
| `gps_provider` | Enum index (`NMEA`, `UBLOX`, `MSP`, `FBUS`, `CRSF`) |
| `pid_rate_hz` | PID loop rate |

The existing header lines (PIDs, filters, `debug_mode`, …) are unchanged.

---

## 6. Limitations

- `flightModeFlags` in the blackbox slow frames covers only the first 32 mode
  **switch** boxes, so the POSHOLD and HARD DECK switches are not in it. The
  slow frames therefore also carry `activeFlightModes`: the modes actually
  active (`flightModeFlags_e`: bit 4 ALTHOLD, 5 RESCUE, 7 POSHOLD, 8 HARD
  DECK, 0 FAILSAFE). A switched-on mode that is not engaged (POSHOLD without
  ALTHOLD or a valid XY estimate) reads 0 here. Within the debug modes,
  `ALTHOLD` field 7, `POSHOLD` all non-zero and `HARDDECK` field 0 show the
  same.
- Blackbox Explorer does not know the new debug modes. It shows them as
  `debug[0]` … `debug[7]`; use the tables above.
- GPS Doppler vertical velocity, the N/E velocity and the accuracy estimates
  (u-blox) are fused but not logged. The `gps_provider` header line tells
  whether they existed at all (`UBLOX` only; `CRSF` has none of them).
