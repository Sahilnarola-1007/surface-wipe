# Surface Wipe — Design Document

## Purpose

This document explains the design rationale for the surface wiping demo: why the
brain/executor split, how contact detection works, the trajectory generator design,
and the failure modes considered.

---

## Brain/Executor Architecture

### Why Two Nodes?

The wipe_node (brain) and admittance_node (executor) are separate because:

1. **Tight force loop:** The PI controller must run inside the node that owns the
   Kortex API connection. Any ROS2 topic latency between PI and velocity command
   would degrade force tracking. The admittance_node already has the connection,
   so the PI runs there.

2. **Clean separation of concerns:** The brain decides WHAT to do (state machine,
   trajectory). The executor decides HOW to do it (force control, safety, velocity
   commands). The brain never touches the arm directly.

3. **Independent lifecycle:** The admittance controller works standalone as a 6-DOF
   compliance controller. Wiping is an optional overlay. If the wipe_node crashes,
   the stale-setpoint timeout (0.5s) reverts to normal admittance automatically.

### Communication: WipeSetpoint Topic

The brain publishes at 50 Hz. The executor reads the latest setpoint each control
cycle (~13 Hz). The 4:1 oversampling ensures the executor always has fresh data.
The setpoint is a complete state descriptor — no memory of previous messages needed.

---

## State Machine

```
IDLE ──(~/start service)──► APPROACH
                               │
                    fz < -contact_threshold
                               │
                               ▼
                             WIPE
                               │
                    trajectory complete
                               │
                               ▼
                            RETRACT
                               │
                    t > retract_duration
                               │
                               ▼
                             IDLE
```

### State Transitions

**IDLE → APPROACH:** Triggered by `~/start` service call. Guard prevents double-start.

**APPROACH → WIPE:** Contact detection via `wrench_corrected` Fz. The wipe_node reads
the same corrected wrench that the admittance_node computes — gravity-compensated,
filtered, and dead-zoned. Contact fires when `fz < -contact_threshold_` (default
-2 N). The timestamp of contact becomes t=0 for the wipe trajectory.

**WIPE → RETRACT:** The WipeTrajectory reports `isComplete(t)`. All passes done.

**RETRACT → IDLE:** Fixed-duration timer. After `retract_duration` seconds of upward
motion, the state machine returns to IDLE and the admittance controller resumes normal
compliance.

---

## Contact Detection

### Sign Convention

The MAE sensor reads **negative Fz** when the tool presses into a surface (the contact
reaction pushes the tool upward in the sensor frame). This was verified on hardware:
pressing down into a table produces negative `wrench_corrected` Fz.

The contact check is therefore:
```cpp
if (fz < -contact_threshold_)   // e.g. fz < -2.0 N
```

This is not an arbitrary choice — it follows directly from the sensor mounting and the
Rz(90°) rotation applied in `mae_sensor_node.py`.

### Threshold Selection

The 2 N threshold sits well above the dead zone (0.3 N) and EMA filter noise floor,
but below the desired wiping force (5 N). This prevents false triggers from sensor
drift while detecting genuine surface contact promptly.

---

## Trajectory Generator

### WipeTrajectory Design

Generates a zigzag pattern: alternating X-direction strokes with Y-step advances.

```
Pass 1: ─────────►  (x_len in +X)
                  │  (row_spacing in +Y)
Pass 2: ◄─────────  (x_len in -X)
                  │  (row_spacing in +Y)
Pass 3: ─────────►  (x_len in +X)
```

The generator is time-parameterized: given elapsed time `t` since contact, it returns
`(vx, vy)` velocities. The admittance_node applies these directly as linear velocity
commands while the PI controller handles Z-axis force independently.

### Timing

Each pass takes `x_len / wipe_speed` seconds. Each Y-step takes `row_spacing / wipe_speed`
seconds. Total duration: `num_passes * x_len/speed + (num_passes-1) * row_spacing/speed`.

With defaults (x_len=0.10m, spacing=0.05m, passes=3, speed=0.02m/s):
total = 3×5.0 + 2×2.5 = 20.0 seconds.

---

## Force Control Integration

### Bumpless Transfer

When transitioning from APPROACH to WIPE, the PI integral is reset to zero:
```cpp
if (z_fc && !prev_z_force_control_) {
    force_pi_->reset();   // rising edge: bumpless transfer
}
```

Without this, the integral would be non-zero from previous control activity, causing
an immediate velocity spike at the moment force control activates.

### Decoupled Axes

During WIPE mode:
- **Z-axis:** PI force controller (`vz = force_pi_->update(F_desired, -fz)`)
- **X/Y axes:** Trajectory-commanded velocity (from WipeTrajectory)
- **Angular:** Quaternion orientation hold (unchanged from admittance mode)

Position regulation (Kp · position_error) is disabled on X/Y/Z during wiping — the
spring-back behavior would fight the wipe trajectory and force controller.

---

## Safety Considerations

### Stale Setpoint Timeout

If the wipe_node crashes or stops publishing, the admittance_node detects stale setpoints
(age > 0.5s) and reverts to IDLE mode (normal admittance). The arm does NOT continue
pressing or wiping unsupervised.

### Force Overshoot Envelope

Measured worst case: -7 N against -5 N target (1.4× overshoot at direction reversals).
This is within safe limits for the Gen3's joint torque capacity and the test surface.
For brittle or sensitive surfaces, reduce Ki or add a direct integral clamp.

### Velocity Limits

All velocities (admittance, trajectory, force controller output) pass through the same
safety clamp: 0.15 m/s linear, 0.5 rad/s angular. The force controller's v_max is set
to this same value.

---

## Known Limitations

1. **No surface inclination handling** — assumes surface normal is base-frame Z. Tilted
   surfaces produce coupled lateral forces during contact.

2. **Fixed trajectory geometry** — zigzag only. Circular, spiral, or adaptive patterns
   would require extending WipeTrajectory.

3. **No force spike protection** — the PI has anti-windup but no explicit force limit.
   If the surface has a sudden step, force can spike before the PI reacts (one loop
   cycle = 77ms at 13 Hz).

4. **Single contact threshold** — same threshold for all surface types. Compliant surfaces
   (foam, fabric) may need lower thresholds.

---

## Revision History

| Date | Change | Author |
|------|--------|--------|
| June 2026 | Initial design: state machine, trajectory, brain/executor split | Sahil Narola |
| June 2026 | Hardware validation: 5N wiping, Kp=0.001 Ki=0.01, 13Hz loop | Sahil Narola |
