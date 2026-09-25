# Hard Deck

The hard deck is a **training aid**. While the `HARD DECK` mode is switched on,
the helicopter cannot be flown below a set altitude. You fly normally (acro,
angle, upright or inverted). If the helicopter is about to go through the deck,
the flight controller takes over and does the following:

1. It levels to the nearest horizon and pushes away from the ground. When
   inverted, it uses negative collective, so the descent is stopped without
   waiting for a flip.
2. If inverted, it rolls over to upright.
3. It climbs back to *deck + recovery margin* and brakes any horizontal drift.
4. It holds that altitude and position (position hold with GPS and/or optical
   flow) until you take control back.

This page covers the controller, the altitude estimation that feeds it, the
settings, and how to test it.

> **Status:** experimental. The controller has been tested in closed-loop
> simulation (`src/test/unit/harddeck_unittest.cc`), but it has not been flown
> yet. Follow the [test procedure](#5-testing-procedure) and start with a
> generous deck altitude.

---

## 1. How it decides to take over

The hard deck does not wait for the helicopter to reach the deck. It
continuously predicts the **lowest altitude the helicopter would reach if a
recovery started now**:

```
predicted = altitude
          - sigma_factor * sigma                    (estimate uncertainty)
          - v * t_react                             (descent during reaction)
          - v^2 / (2 * recovery_accel)              (braking distance)
          - g/4 * t_flip^2        (inverted only)   (sag during the half-roll)

t_react = reaction_time + (angle to nearest level) / rescue_max_setpoint_rate
t_flip  = 180 deg / rescue_max_setpoint_rate
```

The recovery starts as soon as `predicted < harddeck_altitude`. As a result:

* **Faster descents trigger higher.** With the defaults, a slow sink onto the
  deck triggers 1–3 m above it. An 8 m/s inverted dive triggers about 10 m
  above it.
* **Knife-edge is the worst case.** No vertical thrust is available, so the
  time to rotate to the nearest level attitude is added to the reaction time.
* **Poor altitude estimates trigger higher.** `sigma` is the 1-sigma
  uncertainty of the fused altitude estimate. When GPS is weak or the baro is
  disturbed, the margin grows by itself.

### Arming the deck

The deck only arms after the helicopter has been above
`harddeck_altitude + harddeck_arm_margin` (including the uncertainty margin)
for one second. You can therefore switch the mode on while still on the ground:
it does nothing until you have climbed through the arm altitude.

> **To land:** switch the hard deck off. While it is on, the helicopter will
> not descend below the deck.

### Terrain

With a working rangefinder and `harddeck_use_agl = ON`, the lower of the fused
altitude (relative to the arm point) and the rangefinder AGL reading is used.
Flying low over a hill or a tree line then triggers the deck as well.

---

## 2. Altitude estimation: baro in the downwash, GPS and IMU

On a helicopter the barometer usually sits inside the rotor downwash. Its
reading has a pressure error that depends on rotor thrust, and the error
changes character when the thrust direction reverses (in inverted flight the
fuselage is on the inflow side of the disk). A plain baro altitude can
therefore be off by metres during aerobatics, which is exactly when the hard
deck matters.

The vertical estimator in `flight/position.c` is a 3-state Kalman filter:

| State | Driven by |
| --- | --- |
| altitude | IMU vertical acceleration (prediction) |
| vertical velocity | IMU vertical acceleration (prediction) |
| **baro bias** | random walk, faster during rotor transients |

These measurements are fused into it:

| Sensor | Measures | Notes |
| --- | --- | --- |
| Baro | altitude **+ bias** | Noise is raised during collective transients and high cyclic rates. Spikes beyond 5 sigma are rejected. |
| GPS altitude | altitude | Raw altitude, fused once per GPS message, noise scaled by DOP² |
| GPS Doppler velocity (u-blox) | vertical velocity | Not affected by downwash and much less noisy than differentiated GPS altitude |
| Rangefinder | altitude | Only when valid and allowed by `position_alt_source` |

What this does:

* **IMU** provides the fast response, and short-term motion is taken from it.
* **Baro** provides smooth, precise short-term altitude. Its **downwash offset
  is estimated continuously** against GPS (and/or the rangefinder) instead of
  pulling the altitude around.
* **GPS altitude and Doppler velocity** anchor the long-term altitude and make
  the baro bias observable.
* **Two learned biases** are kept, one for positive and one for negative
  collective. They are swapped when the thrust direction reverses, so the
  inverted-flight offset is not learned again after every flip.
* In **rotor transients** (a collective punch, a fast flip) the baro is
  temporarily trusted less and the IMU and GPS carry the estimate through.
* Without GPS or rangefinder (baro only), the bias is frozen and the baro acts
  as the altitude reference, as before.

In a host simulation of 10 minutes of alternating upright and inverted flight
with collective punch-outs (baro bias of +1.5 m upright and −2.5 m inverted,
GPS with 1.5 m noise), the RMS altitude error was **16 cm** (worst case
56 cm). The previous 2-state filter with cross-calibration had an RMS error of
**2.2 m** (worst case 4 m).

The estimator also removes an earlier problem: the same low-pass-filtered GPS
altitude was fused on every 100 Hz tick, which made the filter's variance far
too optimistic. GPS is now fused once per new message.

---

## 3. Settings

### 3.1 Hard deck (per PID profile)

| CLI name | Default | Range | Meaning |
| --- | --- | --- | --- |
| `harddeck_altitude` | 100 | 10–10000 | Deck altitude above the arm point, **dm** (100 = 10 m) |
| `harddeck_arm_margin` | 20 | 0–1000 | Must climb this far above the deck (dm) before the deck arms |
| `harddeck_recovery_margin` | 30 | 0–1000 | Recovery climbs to and holds deck + this (dm); also the lowest hold altitude |
| `harddeck_release_altitude` | 0 | 0–10000 | If > 0: climbing the hold to deck + this (dm) hands control back with the deck still armed. 0 = only the switch releases |
| `harddeck_recovery_accel` | 50 | 5–500 | Assumed recovery vertical acceleration, dm/s² (50 = 5 m/s²). Lower = triggers earlier |
| `harddeck_reaction_time` | 300 | 0–2000 | Reaction latency in the prediction, ms |
| `harddeck_sigma_factor` | 20 | 0–50 | Uncertainty margin in tenths of sigma (20 = 2 sigma) |
| `harddeck_use_agl` | ON | OFF/ON | Also respect the rangefinder AGL reading |

### 3.2 Shared with rescue and position hold

The recovery reuses settings you may already have tuned for Rotorflight rescue
and for altitude/position hold:

| Used for | Settings |
| --- | --- |
| Level, pull-up and flip | `rescue_flip_gain`, `rescue_level_gain`, `rescue_pull_up_collective`, `rescue_pull_up_time`, `rescue_flip_time`, `rescue_exit_time`, `rescue_max_setpoint_rate`, `rescue_max_setpoint_accel` |
| Climb and hold collective | `rescue_hover_collective` (starting value of the hover integrator, and the scale of the vertical velocity loop gain), ceiling `max(rescue_max_collective, rescue_pull_up_collective)` |
| Hold: climb rate and stick | `althold_max_climb_rate`, `althold_stick_deadband` |
| Hold: position | `poshold_pos_p_gain`, `poshold_vel_p_gain`, `poshold_vel_i_gain`, `poshold_max_horiz_speed`, `poshold_max_tilt_angle`, `poshold_stick_deadband` |

**Set `rescue_hover_collective` correctly for your helicopter.** The vertical
loop gain is normalised by it, so the loop behaves the same across
helicopters.

The `rescue_mode` value is not needed: the hard deck works whether the rescue
switch is configured or not.

### 3.3 Estimator (master settings)

| CLI name | Default | Meaning |
| --- | --- | --- |
| `position_baro_downwash_comp` | 30 | Strength of baro downwash handling (noise inflation, per-thrust-direction bias). 0 = off |
| `position_est_q_baro_bias` | 400 | Baro bias random walk, cm²/s |
| `position_est_r_gps_vvel` | 400 | GPS Doppler vertical velocity noise, (cm/s)² at DOP 1 |

### 3.4 Mode and debug

* Mode box: **`HARD DECK`** (permanent id 59). Map it to a switch in the Modes
  tab or with `aux`.
* OSD flight mode shows **`DECK`**, and CRSF telemetry shows **`HARDDECK`**,
  while the hard deck is in control.
* `set debug_mode = HARDDECK`:

| Field | Value |
| --- | --- |
| 0 | state (0 off, 1 wait, 2 watch, 3 pull-up, 4 flip, 5 climb, 6 hold, 7 exit) |
| 1 | fused altitude, cm |
| 2 | altitude 1-sigma, cm |
| 3 | predicted minimum altitude, cm |
| 4 | recovery / hold target, cm |
| 5 | estimated baro bias, cm |
| 6 | baro disturbance ×100 |
| 7 | hard-deck collective output |

---

## 4. Handing control back

| Action | Result |
| --- | --- |
| Switch off (any state) | Control blends back to the sticks over `rescue_exit_time` |
| Switch back on during that blend | Deck stays armed and a new recovery starts at once if needed |
| Collective up in hold, past `harddeck_release_altitude` (if set) | Blends back to the pilot. The deck stays armed and protects again |
| Collective down in hold | Hold target goes down, but never below deck + recovery margin |
| Cyclic in hold | Moves the hold position (needs a valid GPS or optical-flow position) |
| Yaw during climb and hold | Stays with the pilot |

Without a valid horizontal position estimate, the climb and hold phases stay
level instead of holding position.

If the altitude estimate is lost (no baro, GPS or rangefinder), the deck
cannot trigger. During a recovery, the collective falls back to the trimmed
hover collective.

---

## 5. Testing procedure

1. **Bench, props off:** `set debug_mode = HARDDECK`. Check that field 1
   follows the altitude when you lift the model, and that field 2 (sigma) is
   well under 1 m with GPS fix.
2. **Tune rescue first.** The recovery uses the rescue attitude gains and
   collectives. Check them with the normal rescue switch at a safe altitude.
3. **First flight:** use a generous deck (e.g. `harddeck_altitude = 300`,
   30 m). Climb above 32 m and check that debug field 0 goes to 2 (watch).
   Then descend slowly, upright, and let it catch you. It should climb to 33 m
   and hold.
4. Try faster upright descents, then a gentle inverted descent. Watch field 3
   (predicted minimum) and field 0 (state) in the log.
5. Lower the deck in steps, and only after the behaviour is fully predictable.
6. Check blackbox for the baro bias (field 5) after inverted segments. It
   should settle to a stable value per flight attitude.
