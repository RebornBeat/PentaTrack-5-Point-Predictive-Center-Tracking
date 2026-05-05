# Physics Models — Cross-Project Sensor and Latency Considerations

**Project:** PentaTrack (cross-project documentation)
**Domain:** Physics-of-sensing tradeoffs and parameter selection for PentaTrack's downstream consumers
**Implementation Language:** Rust

---

## 1. Purpose

PentaTrack is implemented in Rust and designed to operate as a downstream consumer of OMNI-SENSE's `DetectionEvent` stream. This document characterizes the physics-of-sensing differences across PentaTrack's consuming projects and specifies how PentaTrack's parameters should be chosen for each context.

PentaTrack's core algorithm is sensor-agnostic. It receives `DetectionEvent` structs carrying position, velocity (optional), covariance, modality, frame, and timestamp. The parameter choices that make tracking perform well in a given context — recursion depth, weight strategy, velocity method, drift thresholds — are governed by the physics of the sensor stack feeding the events.

---

## 2. Signal-Medium Tradeoffs

| Medium | Propagation Speed | Strengths | Weaknesses |
|---|---|---|---|
| Light (LiDAR, optical) | ~3×10⁸ m/s | High spatial resolution; precise ToF | Atmospheric attenuation; scan-cycle latency for scanning architectures |
| Radio (radar) | ~3×10⁸ m/s | Through-obscuration; direct Doppler velocity | Lower angular resolution at small antenna sizes; multi-path |
| Sound (acoustic) | ~343 m/s | Through-obscuration; material discrimination | Slow; insufficient for fast events; ambient-noise sensitivity |
| Inertial (IMU) | N/A | High temporal resolution; no environmental dependence | Drift; no absolute reference; no information about external objects |

These tradeoffs directly influence which PentaTrack parameters produce good tracking:

- **Scanning LiDAR** has scan-cycle latency (50–100 ms). PentaTrack's history window for velocity estimation should be longer than one scan cycle to smooth over gaps.
- **CW Doppler radar** provides direct velocity readout. PentaTrack's `velocity_method` can be set to passthrough (the velocity comes from the sensor, not from position-history differentiation).
- **Acoustic** is suitable for events and slow targets only. PentaTrack's recursion depth should be shallow (depth 1) and velocity estimation should use a heavily-smoothed method.

---

## 3. Latency Budgets and Parameter Selection by Project

### 3.1 AEGIS-MESH (Residential distributed sensing)

**Target velocities:** 1–2 m/s typical (human walking); up to 5 m/s (running).
**Engagement distances:** 1–20 m (room scale).
**Latency budget:** Hundreds of milliseconds for awareness-class alerting.

**PentaTrack parameters:**
```rust
TrackingConfig {
    mode: TrackingMode::Dual,
    recursion_depth: 2,
    enable_velocity_weighting: true,
    weight_strategy: WeightStrategy::Cosine,
    velocity_method: VelocityMethod::Ema { alpha: 0.3 },
    history_window: 30,
    enable_drift_analysis: true,
    enable_adaptive_drift: true,
    drift_speed_thresholds: [0.1, 0.5, 2.0], // slow residential speeds
    enable_object_type: true,
    // object types from ResidentialObjectClass
}
```

**Rationale:** Residential motion is slow. Low drift speed thresholds. Drift analysis is essential because radar and LiDAR won't agree exactly on position — drift analysis accommodates the gap. Object-type awareness uses residential taxonomy (human walking, pet, vehicle in driveway).

### 3.2 SENTINEL-WEAR (Body-frame wearable sensing)

**Target velocities:** 1–30 m/s typical (pedestrians to vehicles). Body-frame: relative velocity depends on wearer motion.
**Engagement distances:** 1–15 m (body-frame scale).
**Latency budget:** 100–500 ms for routine awareness.

**PentaTrack parameters (routine awareness):**
```rust
TrackingConfig {
    mode: TrackingMode::Dual,
    recursion_depth: 2,
    enable_velocity_weighting: true,
    weight_strategy: WeightStrategy::Cosine,
    velocity_method: VelocityMethod::Ema { alpha: 0.4 }, // slightly faster response
    history_window: 20,
    enable_drift_analysis: true,
    enable_adaptive_drift: true,
    drift_speed_thresholds: [0.25, 1.0, 5.0],
    enable_object_type: true,
    // object types from BodyFrameObjectClass
}
```

**PentaTrack parameters (extreme velocity research track):**
```rust
TrackingConfig {
    mode: TrackingMode::Discrete,
    recursion_depth: 1,               // No time for deep prediction
    enable_velocity_weighting: true,
    weight_strategy: WeightStrategy::Cosine,
    velocity_method: VelocityMethod::LastDelta, // CW Doppler provides direct velocity
    min_weight: 0.05,                // Broader uncertainty floor
    enable_drift_analysis: true,
    enable_adaptive_drift: true,
    drift_speed_thresholds: [50.0, 300.0, 1000.0], // ballistic speed regimes
    enable_object_type: false,       // No time for object-type classification
    history_window: 5,               // Very short history for ultra-fast targets
}
```

### 3.3 HALO-AD (Long-range distributed coverage)

**Target velocities:** 10–1000+ m/s.
**Engagement distances:** 100 m–10+ km.
**Latency budget:** Seconds for typical; sub-second for hypersonic.

**PentaTrack parameters:**
```rust
TrackingConfig {
    mode: TrackingMode::Dual,
    recursion_depth: 3,
    enable_diagonals: true,          // Threats maneuver in 3D
    enable_velocity_weighting: true,
    weight_strategy: WeightStrategy::Softmax { temperature: 0.5 }, // sharper bias
    velocity_method: VelocityMethod::Lsq { window: 15 }, // stable, accepts latency
    history_window: 30,
    enable_drift_analysis: true,
    enable_adaptive_drift: true,
    drift_speed_thresholds: [5.0, 50.0, 500.0], // long-range speed regimes
    enable_object_type: true,
    // object types from HALO-AD threat taxonomy
    enable_homing: false,            // Tracking only; no homing in HALO-AD
}
```

### 3.4 TALON-MESH (Counter-UAS multi-effector coordination)

**Target velocities:** 10–200 m/s (small UAS range).
**Engagement distances:** 10 m–1 km.
**Latency budget:** Seconds.

**PentaTrack parameters:**
```rust
TrackingConfig {
    mode: TrackingMode::Dual,
    recursion_depth: 3,
    enable_diagonals: true,          // UAS maneuver in arbitrary directions
    enable_velocity_weighting: true,
    weight_strategy: WeightStrategy::Cosine,
    velocity_method: VelocityMethod::Ema { alpha: 0.3 },
    history_window: 20,
    enable_drift_analysis: true,
    enable_adaptive_drift: true,
    drift_speed_thresholds: [2.0, 20.0, 100.0], // UAS speed regimes
    enable_object_type: true,
    // object types from UasClass taxonomy
    enable_homing: false,            // Effector coordination is in talon-decision, not here
}
```

---

## 4. Velocity Estimation: Method Selection

The velocity estimate feeding PentaTrack's weighting comes from different sources depending on the sensor modality:

| Source | Method | Notes |
|---|---|---|
| CW Doppler radar | Passthrough | Direct radial velocity from OMNI-SENSE; no history needed |
| FMCW radar (Doppler bin) | LastDelta or Ema | Doppler bin gives direct velocity; Ema smooths chirp-to-chirp noise |
| Scanning LiDAR | Lsq or Wma | Position-only; differentiation adds noise; LSQ is most stable |
| Event camera | Ema | Trajectory gradient on spike stream; EMA smooths noise |
| IMU + body frame | Ema | Dead-reckoning velocity; EMA appropriate for smooth motion |
| Multi-modal fused | Ema or Lsq | The fuser produces a velocity estimate; use that directly |

---

## 5. Coordinate Frames and PentaTrack

PentaTrack operates in a single coordinate frame per tracked object. The frame is determined by the `DetectionEvent::frame_id` field. Applications must ensure all detections for the same logical target use the same frame (or transform them to a common frame before feeding PentaTrack).

In practice, the OMNI-SENSE frame tree handles this: the consuming project configures the "output frame" for its perception pipeline, and all detections are transformed to that frame before producing `DetectionEvent` values. PentaTrack then operates entirely in that frame.

| Project | PentaTrack frame |
|---|---|
| AEGIS-MESH | Home frame (floor-plan coordinates) |
| SENTINEL-WEAR | Stabilized body frame (torso-relative) |
| HALO-AD | Zone-local world frame |
| TALON-MESH | Airspace world frame |

---

## 6. The Civilian Transfer Quick Reference

| Application | Velocity regime | PentaTrack depth | Key extensions |
|---|---|---|---|
| Residential awareness (AEGIS-MESH) | 0–5 m/s | 2 | Drift, object-type |
| Wearable awareness (SENTINEL-WEAR) | 0–30 m/s | 2 | Drift, object-type, body-frame |
| Sports tracking | 0–35 m/s | 2 | Drift, adaptive-drift, diagonals |
| Multi-cobot pick-and-place | 0–3 m/s | 1 | Drift, homing |
| Multi-drone agricultural | 0–15 m/s | 2 | Drift, diagonals, homing |
| Counter-UAS (TALON-MESH) | 10–200 m/s | 3 | Drift, adaptive-drift, diagonals, object-type |
| Long-range air defense (HALO-AD) | 10–1000 m/s | 3 | Drift, adaptive-drift, diagonals |
| Ballistic sensing (SENTINEL-WEAR EV) | 300–1200 m/s | 1 | Drift, adaptive-drift (minimal) |
