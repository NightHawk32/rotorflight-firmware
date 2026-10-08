# Test plan: MicroLink, altitude hold, position hold, hard deck

This plan covers everything the `feature/microlink-alt-pos-hold` branch adds:
the MicroLink LIDAR + optical-flow driver, the Kalman state estimator, Altitude
Hold, Position Hold, the Hard Deck, the blackbox tuning data and the
configurator **Position & Hold** tab.

Work through it in order. Each phase builds on the one before it. Do not fly a
phase until every check in the earlier phases has passed.

- **Phases 0–3** are on the desk or bench, with no flying.
- **Phases 4–9** are flight tests. Each flight has a blackbox `debug_mode`; send
  the log back for analysis (see
  [Blackbox-Tuning-AltHold-PosHold.md §2](Blackbox-Tuning-AltHold-PosHold.md#2-what-to-send-back-with-a-log)).
- Pass criteria marked *(initial target)* are first guesses. Adjust them once
  the first logs show what this helicopter actually achieves.

> **Safety.** Everything here is experimental and has not been flown. Bench
> tests with motor power: remove the main and tail blades, or unplug the motor.
> In flight, always have a switch ready that turns off every new mode, and fly
> high enough to recover by hand.

---

## Phase 0: Preparation

| # | Step | Pass |
| --- | --- | --- |
| 0.1 | On the **old** firmware, save `diff all` and `aux` from the CLI to a file | Files saved |
| 0.2 | Build and flash the branch firmware (e.g. `make TARGET=STM32F7X2`) | Boots, configurator connects |
| 0.3 | Run the configurator from the `feature/microlink-alt-pos-hold` branch (`pnpm install`, `pnpm start`) | Connects without an "unsupported firmware" warning |
| 0.4 | CLI `aux`: compare with the list from 0.1 | **Every line is identical.** The switch assignments survived the flash (this checks the box-numbering fix) |
| 0.5 | CLI `diff all`: the PID profiles and `position_*` settings are back to defaults, as expected after this flash | Only those groups differ from 0.1 |
| 0.6 | Paste back the PID-profile and `position_*` lines from 0.1, `save` | `diff all` matches 0.1 again, apart from new settings |
| 0.7 | CLI `get debug_mode`. If you had one set before, it still has the same name | Unchanged (this checks the debug-mode numbering fix) |

---

## Phase 1: Configurator and configuration (desk, FC on USB)

| # | Step | Pass |
| --- | --- | --- |
| 1.1 | Look at the tab list | **Position & Hold** is shown |
| 1.2 | Configuration tab → Ports: open the function list of the UART wired to the MicroLink | **MicroLink (LIDAR + flow)** is offered. Select it, save and reboot |
| 1.3 | Position & Hold → Sensors and estimator: set Rangefinder = `MICROLINK`, Optical flow = `MICROLINK`, Save and Reboot | Reconnects. Both settings persist. CLI `get rangefinder_hardware` / `optical_flow_hardware` agree |
| 1.4 | Set Rangefinder = `MICROLINK` and Optical flow = `NONE` (don't save) | A warning explains that both are needed. Revert |
| 1.5 | CLI `feature RANGEFINDER`, `save` | Rangefinder task runs (phase 2) |
| 1.6 | Change a hold setting, e.g. Deck altitude 10 → 12 m, Save | **Save** (no reboot) is offered. CLI `get harddeck_altitude` = 120 |
| 1.7 | Switch to another PID profile (e.g. with an adjustment switch or the Profiles tab), return to Position & Hold | "Applies to PID profile N" shows the new number and the values change with the profile |
| 1.8 | Change a value, then click Revert | The value returns to its saved value |
| 1.9 | Modes tab | `ALTHOLD`, `POSHOLD` and `HARD DECK` are listed. Assign each to a switch, save |
| 1.10 | Modes tab, move each new switch | The mode lights up while its switch is on |
| 1.11 | Blackbox tab → Debug mode list | Contains `ALTHOLD`, `POSHOLD`, `HARDDECK`, `OPTICAL_FLOW`, `POS_EST_Z`, `POS_EST_XY`, `POLAR_RATE`, `GYRO_CALIBRATION`, and no `USER1`–`USER4` |
| 1.12 | Set blackbox fields and rate as in [the tuning doc §1](Blackbox-Tuning-AltHold-PosHold.md#1-one-time-blackbox-setup) | Saved |
| 1.13 | *(optional)* Disconnect, connect in **Virtual** mode | The Position & Hold tab opens with bench-like values and settings can be edited |

---

## Phase 2: Sensors on the bench (disarmed)

Use the Position & Hold tab, **Live: sensors**.

| # | Step | Pass |
| --- | --- | --- |
| 2.1 | Power up with the MicroLink connected, model on the floor | Rangefinder and Optical flow show **Detected** / **Healthy** |
| 2.2 | Lift the model level to 0.5 m, 1 m, 2 m above the floor | Raw distance and height above ground follow within a few cm. *Height valid* = Yes. Reliability goes to 100 % within about 1 s |
| 2.3 | Tilt the model 20° | Height above ground stays about the vertical height (tilt compensated), raw distance increases |
| 2.4 | Point the sensor at the sky or cover it | Raw distance shows "Out of range". Reliability decays and *Height valid* = No within about 0.5 s |
| 2.5 | Unplug the MicroLink UART connector while powered | Optical flow shows **No data** within 0.5 s. Height becomes invalid. Plug back in: recovers without reboot |
| 2.6 | Hold ~1 m over a textured floor in good light | Flow quality > 50 (meter past the mark) |
| 2.7 | Same over a plain white sheet or in the dark | Quality drops. Note the values: this is where position hold will stop using flow |
| 2.8 | **Flow direction check**: hold level ~1 m up. Slide forward, then right | Arrow points **up** for forward and **right** for right. If not, the sensor is rotated or mirrored: run the **Optical flow orientation** check and apply its `optical_flow_align` before anything else |
| 2.9 | Hold still, pitch forward/back and roll left/right 20° | Note whether raw flow X/Y move in step with the rotation (no gyro compensation in the module). Report the result |
| 2.10 | Leave the model still for 5 min (GPS outdoors if possible) | Live: altitude estimate: fused altitude stays within ±0.5 m *(initial target)*. Uncertainty settles (< 1 m with GPS fix) |
| 2.11 | Lift by 1 m and hold | Fused altitude follows within 1–2 s. Baro, GPS and LIDAR traces agree |

---

## Phase 3: Bench armed, motors safe (blades off / motor unplugged)

The horizontal estimator and the controllers only run while armed. Use a
props-off setup that lets you arm.

| # | Step | Pass |
| --- | --- | --- |
| 3.1 | Arm on the floor, lift to ~1 m, carry the model 2 m forward and back | **Live: horizontal**: *Optical flow in use* = Yes. The trail moves about 2 m in the direction you carried the model and comes back near the start *(initial target: within 0.5 m)* |
| 3.2 | Turn to a different heading and repeat | The trail moves in the new direction (heading rotation correct) |
| 3.3 | Outdoors with GPS fix: arm, walk 10 m and back | *GPS in use* = Yes, *GPS origin set* = Yes. The trail follows the walk. *Drift limit* never shows |
| 3.4 | Indoors, `position_xy_source = FLOW_ONLY`: carry the model > 10 m away | *Drift limit (10 m) reached* shows. Position stays at the 10 m circle |
| 3.5 | Switch ALTHOLD on at ~1 m | Live: controllers → Altitude hold **Engaged**, source LIDAR, target latched at the current height |
| 3.6 | Move the collective stick outside the deadband | *Collective moving target* = Yes, target changes at up to the max climb rate |
| 3.7 | With ALTHOLD on, switch the **rescue** switch on | Altitude hold shows **Paused: rescue in control**. Collective output follows rescue, not altitude hold (this checks the priority fix). Rescue off: altitude hold re-engages with a new target |
| 3.8 | ALTHOLD + POSHOLD on, with *Estimate valid* = Yes | Position hold **Engaged**, red cross at the current position. Move the model: roll/pitch commands point back towards the cross |
| 3.9 | POSHOLD on with ALTHOLD off | Position hold stays **Off** |
| 3.10 | HARD DECK on, model on the floor | State **Waiting to climb above deck** |
| 3.11 | Disarm | All three controllers go **Off** |
| 3.12 | Record a 30 s blackbox log with `debug_mode = POS_EST_Z` | Log header contains `althold_gain`, `poshold_gain`, `harddeck_altitude`, `position_est_r`, `pid_rate_hz` (open the log as text or in Blackbox Explorer → header) |

---

## Phase 4: Baseline flights (manual)

No new modes switched on. These logs show the raw quality of the estimates.

| # | Flight | `debug_mode` | Pass |
| --- | --- | --- | --- |
| 4.1 | Tuning test 2: hover at 0.5 / 1 / 2 / 4 m, pass over an edge | `RANGEFINDER` | AGL valid throughout the hover heights. Over the edge the height steps cleanly, no spikes |
| 4.2 | Tuning test 3: hover, climbs, collective punches, flips if flown | `POS_EST_Z` | Fused altitude has no jumps on collective punches. Baro bias settles per flight direction. Uncertainty < 1 m with GPS *(initial target)* |
| 4.3 | Tuning test 5: hover, translate, pirouette | `POS_EST_XY` | Flow status mostly "fused" below 4 m. Position returns near the start after translations |

**Send all three logs before continuing.** Estimator settings may be changed
after these.

---

## Phase 5: Altitude hold

| # | Flight | `debug_mode` | Pass |
| --- | --- | --- | --- |
| 5.1 | Hover at 1.5 m (LIDAR range), ALTHOLD on, hands off collective 20 s | `ALTHOLD` | Holds ±0.3 m *(initial target)*, no oscillation |
| 5.2 | Collective up ~1 s, release; then down | `ALTHOLD` | Climbs/descends at ≤ max climb rate, stops and holds at the new height |
| 5.3 | Hover at 15 m (beyond LIDAR range), ALTHOLD on | `ALTHOLD` | Holds on the fused altitude ±1 m *(initial target)*. Source shown as "Fused altitude" |
| 5.4 | Climb and descend through the LIDAR range limit with ALTHOLD on | `ALTHOLD` | No collective jump when the source switches |
| 5.5 | ALTHOLD on, then trigger rescue | `ALTHOLD` | Rescue flies normally. After rescue, altitude hold resumes at the current height |

Tuning: first `althold_hover_collective` (the output settles at the hover
value), then the gains. Known point: the inner-loop gain is weak by default,
so expect the I term to do most of the work (see the changes doc).

---

## Phase 6: Position hold

| # | Flight | `debug_mode` | Pass |
| --- | --- | --- | --- |
| 6.1 | Indoors or calm, 1–2 m, `FLOW_ONLY`: ALTHOLD + POSHOLD on, hands off 30 s | `POSHOLD` | Stays within 0.5 m *(initial target)*, no oscillation |
| 6.2 | Push cyclic in each direction ~1 s, release | `POSHOLD` | Moves in the pushed direction, stops and holds at the new spot |
| 6.3 | Yaw 90°, repeat one push | `POSHOLD` | Still moves in the pushed direction (stick is heading-relative) |
| 6.4 | Outdoors with light wind, `AUTO`, 2 m | `POSHOLD` | Holds. Wind trim (fields 4/5) builds up against the wind |
| 6.5 | Fly from textured ground over a plain or dark surface with POSHOLD on | `POS_EST_XY` | Flow status changes to "quality too low". Position hold drops out (or GPS takes over) without a jerk |
| 6.6 | Climb above LIDAR range with POSHOLD on, `FLOW_ONLY` | `POSHOLD` | Position hold drops out cleanly when the estimate becomes invalid |

---

## Phase 7: Hard deck

Follow [HardDeck.md §5](HardDeck.md#5-testing-procedure). Summary:

| # | Flight | `debug_mode` | Pass |
| --- | --- | --- | --- |
| 7.1 | Deck 30 m. Climb above 32 m | `HARDDECK` | State goes WAIT → WATCH |
| 7.2 | Slow upright descent towards the deck | `HARDDECK` | Takes over above the deck, climbs to 33 m and holds. OSD shows `DECK` |
| 7.3 | Switch HARD DECK off during the hold | `HARDDECK` | Control blends back over the rescue exit time |
| 7.4 | Faster upright descent | `HARDDECK` | Triggers higher than in 7.2. Never goes below 30 m |
| 7.5 | Gentle inverted descent | `HARDDECK` | Pull-up with negative collective, flip, climb, hold. Never below the deck |
| 7.6 | Lower the deck in steps | `HARDDECK` | Same behaviour at each step |

---

## Phase 8: Failure and edge cases

Fly these only after phases 5–7 pass.

| # | Case | Pass |
| --- | --- | --- |
| 8.1 | GPS fix lost with POSHOLD on in `AUTO` over textured ground | Continues on flow, no jump |
| 8.2 | Radio failsafe with ALTHOLD/POSHOLD on | Normal failsafe behaviour; altitude hold shows paused |
| 8.3 | Arm/disarm several times in one power cycle | Each flight starts with the arm point as origin: position near 0, altitude near 0 |
| 8.4 | Battery swap without reboot, fly again | Same as 8.3 |
| 8.5 | Switch PID profile in flight with ALTHOLD on | Profile gains apply, no jump beyond the gain change |

---

## Phase 9: Regression (existing features)

| # | Check | Pass |
| --- | --- | --- |
| 9.1 | Normal acro and angle flight with none of the new modes | Flies exactly as on the previous firmware |
| 9.2 | Rescue with its own switch | Unchanged |
| 9.3 | OSD and telemetry altitude | Plausible. It now comes from the fused estimate (or LIDAR with `LIDAR_ONLY`) |
| 9.4 | GPS rescue (if used) | Unchanged |
| 9.5 | CPU load (CLI `status` or Setup tab) | No significant increase compared with the old firmware |

---

## Result sheet

Copy this per test session:

```
Date / location / weather:
Firmware commit:            Configurator commit:
Helicopter / FC / MicroLink mounting:

Phase  Test  Result (pass/fail/notes)                         Log file
-----  ----  -----------------------------------------------  --------
```
