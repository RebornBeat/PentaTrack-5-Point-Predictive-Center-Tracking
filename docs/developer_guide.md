# PentaTrack & Omni-Sense: Complete Developer Documentation

**Version:** 1.0.0-production
**Audience:** Systems Engineers, Robotics Integrators, and Backend Developers.

---

## Part I: Architecture Overview

### 1. The Three-Layer Pattern
This system is built on a strict **Separation of Concerns**. It is not a monolithic application. You, the developer, are responsible for the "Integration Layer" (The Bus).

| Layer | Component | Responsibility | Output |
| :--- | :--- | :--- | :--- |
| **1. Sensing** | **Omni-Sense** | Hardware abstraction, raw data capture, physics calculations. | `DetectionEvent` (Raw/Relative Position) |
| **2. Integration** | **SystemBus** | Coordinate transformation, Data translation, Lifecycle management. | `WorldDetection` (Clean World Position) |
| **3. Prediction** | **PentaTrack** | Velocity estimation, Prediction tree generation, Drift learning. | `PredictionTelemetry` (Future States) |

### 2. The Data Flow

```
[Sensor Hardware]
      ↓
[Omni-Sense Driver] -> Generates DetectionEvent (Frame: "Body" or "Sensor")
      ↓
[SYSTEM BUS] -> Transforms "Body" -> "World"
             -> Translates DetectionEvent -> WorldDetection
      ↓
[PentaTracker] -> Maintains History (HashMap)
               -> Calculates Velocity
               -> Builds Prediction Tree
               -> Learns Physics Profiles
      ↓
[Consumer] -> Autopilot / GUI / Database
```

---

## Part II: The PentaTrack Engine (Core Library)

PentaTrack is a **World-Space Prediction Engine**. It is "dumb" to hardware. It does not know about cameras, LiDAR, or radars. It only understands objects moving in a 3D world.

### 1. The Universal Input Contract
Your integration code must construct and provide `WorldDetection` structs.

**File:** `pentatrack/src/io.rs`

```rust
pub struct WorldDetection {
    /// Unique ID for this object.
    /// Must persist across frames. If this ID changes, history is lost.
    pub track_id: String,

    /// Absolute World Position (x, y, z) in meters.
    /// CRITICAL: Must be relative to a fixed world origin.
    pub position: Vector3<f32>,

    /// Monotonic timestamp in nanoseconds.
    /// Must always increase.
    pub timestamp_ns: u64,

    /// Optional: Object classification (e.g., "Sedan", "Human").
    /// Used to load initial physics constraints.
    pub object_class: Option<String>,

    /// Optional: Bounding box size (for proportional delta scaling).
    pub dimensions: Option<Vector3<f32>>,
}
```

### 2. Configuration & Profiles

PentaTrack behavior is controlled by `TrackingConfig`.

```rust
let config = TrackingConfig {
    // GEOMETRY
    mode: TrackingMode::Dual,         // Discrete + Proportional predictions
    recursion_depth: 2,               // Lookahead levels (1=Fast, 3=Complex)
    enable_diagonals: true,           // 9-point compass (vs 5-point)

    // VELOCITY
    enable_velocity_weighting: true,  // Bias predictions toward movement direction
    velocity_method: VelocityMethod::Ema { alpha: 0.3 },

    // LEARNING
    enable_drift_analysis: true,      // Measure prediction error
    enable_object_type: true,         // Apply physics constraints

    ..Default::default()
};
```

**Cold Start Behavior:**
1.  If `object_class` is provided ("sedan"), PentaTrack loads a hardcoded baseline profile.
2.  If `None`, it loads a "Generic" profile (wide constraints).
3.  **Live Learning:** Regardless of the start profile, the `ProfileLearner` observes actual movement. After ~200 frames, it generates a **Live Profile** specific to that object, overriding the baseline. This allows tracking of "modified" vehicles or unpredictable drones.

### 3. The Live Loop API

The engine has two entry points. You must implement the loop that calls them.

```rust
let mut tracker = PentaTracker::new(config);

loop {
    // 1. GET DATA (From your SystemBus)
    let input = get_clean_world_detection();

    // 2. UPDATE TRACKER
    // Returns the prediction for this frame
    let prediction = tracker.update(input);

    // 3. CONSUME PREDICTION
    if let Some(intercept) = &prediction.intercept {
        println!("Intercept Point: {:?}", intercept.weighted_position);
    }

    // 4. MAINTAIN (Run on a timer or every loop iteration)
    // Removes tracks that haven't been seen in >1 second
    let now_ns = system_time_now_ns();
    tracker.maintain(now_ns);
}
```

---

## Part III: The SystemBus (Integration Layer)

This is the "Glue" code. It connects Omni-Sense to PentaTrack. **This is the most critical part of the integration.**

### 1. The SystemBus Structure

The Bus holds an instance of the `FrameTree` (to know where sensors are) and the `PentaTracker`.

```rust
use omni_sense_core::{DetectionEvent, FrameId, Timestamp};
use omni_sense_frames::FrameTree;
use pentatrack::{PentaTracker, PentaPrediction, WorldDetection, TrackingConfig};
use nalgebra::Vector3;
use std::collections::HashMap;

pub struct SystemBus {
    /// The Prediction Engine
    tracker: PentaTracker,

    /// The Coordinate Transformer (Knows where sensors are in the world)
    frame_tree: FrameTree,

    /// Optional: State for persistence
    learned_profiles: HashMap<String, DriftProfile>,
}

impl SystemBus {
    pub fn new(config: TrackingConfig) -> Self {
        Self {
            tracker: PentaTracker::new(config),
            frame_tree: FrameTree::new(), // Populate this with your sensor positions
            learned_profiles: HashMap::new(),
        }
    }

    // ... Implementation details below ...
}
```

### 2. The Core Logic: `process_detection`

This function is called every time a sensor produces data. It handles coordinate transformation and translation.

```rust
impl SystemBus {
    /// Called whenever Omni-Sense produces a new detection
    pub fn process_detection(&mut self, event: DetectionEvent) {

        // -------------------------------------------------------
        // PHASE 1: COORDINATE TRANSFORMATION (The Math)
        // -------------------------------------------------------
        // We must convert the sensor-relative position to a World-relative position.

        let target_world_frame = FrameId::world(); // "world"
        let sensor_frame = &event.frame;

        // Look up the transform: Sensor -> World
        // If the sensor is static (security camera), this transform is constant.
        // If the sensor is moving (drone), this transform changes every frame.
        let world_position = match self.frame_tree.lookup(&sensor_frame, &target_world_frame) {
            Some(transform) => {
                // Apply the rotation and translation
                transform.apply(&event.position)
            },
            None => {
                // Fallback: Assume the event is already in World Coordinates
                // WARNING: If this assumption is wrong, tracking will fail for moving sensors.
                event.position
            }
        };

        // -------------------------------------------------------
        // PHASE 2: DATA TRANSLATION (The Glue)
        // -------------------------------------------------------
        // Convert Omni-Sense specific types to PentaTrack Universal Types.

        let input = WorldDetection {
            track_id: event.source_sensor.0.clone(), // Assuming sensor ID acts as tracking ID
            position: world_position,
            timestamp_ns: event.timestamp.monotonic_ns,

            // Extract classification from Vision/LiDAR hints
            object_class: event.hints.eo_classification
                .as_ref()
                .map(|h| h.object_class.clone()),

            // Extract dimensions from LiDAR hints if available
            dimensions: event.hints.lidar_geometry
                .as_ref()
                .map(|h| Vector3::from(h.bounding_box_m)),
        };

        // -------------------------------------------------------
        // PHASE 3: EXECUTION
        // -------------------------------------------------------
        let prediction = self.tracker.update(input);

        // -------------------------------------------------------
        // PHASE 4: DISPATCH
        // -------------------------------------------------------
        self.dispatch_prediction(prediction);
    }

    /// Called by a timer thread (e.g., 100Hz) to clean up lost tracks
    pub fn on_tick(&mut self, current_time_ns: u64) {
        self.tracker.maintain(current_time_ns);
    }

    fn dispatch_prediction(&self, pred: PentaPrediction) {
        // IMPLEMENT THIS: Send to WebSocket, ROS Topic, or Autopilot
        // Example:
        // if let Some(intercept) = &pred.intercept {
        //    autopilot_interface.send_target(intercept.weighted_position);
        // }
    }
}
```

---

## Part IV: Scenario Implementation Guides

### Scenario A: Static Security Camera (Fixed Sensor)

**Setup:** A camera mounted on a wall tracking a person.
**Complexity:** Low. Coordinate transformation is constant.

1.  **Omni-Sense:** Camera driver initializes. `frame_tree.add_transform(Transform::identity("camera", "world"))`.
2.  **Detection:** Camera sees a person. `DetectionEvent { position: [2.0, 0.0], frame: "camera" }`.
3.  **SystemBus:**
    *   Looks up transform `camera -> world`. Finds translation offset (e.g., camera is at `[10, 20]`).
    *   Calculates `World Pos = [10, 20] + [2, 0] = [12, 20]`.
    *   Sends `[12, 20]` to PentaTrack.
4.  **PentaTrack:** Updates velocity. Generates prediction tree for the person's path.

### Scenario B: Drone Pursuit (Moving Sensor)

**Setup:** A drone (Chaser) tracking another drone (Target).
**Complexity:** High. The "Sensor Position" changes every frame.

1.  **Prerequisite:** Your drone must have a Localization Source (GPS/SLAM) that updates the `FrameTree` constantly.
2.  **The Loop:**
    *   **Localization Thread:** Updates `frame_tree`. `Chaser is now at [100, 200]`.
    *   **Sensor Thread:** Camera sees Target. `DetectionEvent { position: [0, 5], frame: "chaser_body" }`. (Target is 5m ahead).
    *   **SystemBus:**
        *   Looks up `chaser_body -> world`. Finds `Chaser` is at `[100, 200]`.
        *   Calculates `World Pos = [100, 200] + [0, 5] = [100, 205]`.
        *   Sends `[100, 205]` to PentaTrack.
3.  **Homing Logic:**
    *   You call `tracker.update_with_homing(target, chaser_position, chaser_velocity)`.
    *   PentaTrack compares Target's predicted path against Chaser's position.
    *   Returns **Intercept Point**: The coordinate the Chaser should fly to.

---

## Part V: Persistence & Learning

By default, PentaTrack keeps learned profiles in RAM. To save them:

```rust
// On Shutdown or periodically:
fn save_state(&self, path: &std::path::Path) {
    // 1. Export profiles from the tracker
    let profiles = self.tracker.export_learned_profiles();

    // 2. Serialize to JSON
    let json = serde_json::to_string_pretty(&profiles).unwrap();

    // 3. Write to disk
    std::fs::write(path.join("profiles.json"), json).unwrap();
}

// On Startup:
fn load_state(&mut self, path: &std::path::Path) {
    let data = std::fs::read_to_string(path.join("profiles.json")).unwrap();
    let profiles: HashMap<String, DriftProfile> = serde_json::from_str(&data).unwrap();

    // Inject into tracker
    self.tracker.load_profiles(profiles);
}
```

---

## Part VI: Troubleshooting & Tuning

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| **Predictions lag behind the object.** | `recursion_depth` is too high for CPU, or `velocity_method` is too slow (LSQ). | Lower `recursion_depth` to 1. Switch to `Ema` velocity method. |
| **Stationary objects appear to move.** | Sensor movement was not subtracted. | Check `SystemBus`. Ensure `frame_tree` is updated with sensor GPS/SLAM data. |
| **Track ID jumps constantly.** | Upstream detector (YOLO/etc) is changing IDs. | Integrate a Tracker (like SORT/DeepSORT) *before* PentaTrack, or fix detection logic. |
| **Prediction tree is too sparse/wide.** | `softmax_temperature` is too high (uniform weights). | Lower temperature to sharpen direction bias. |
| **Anomaly flags trigger constantly.** | `DriftProfile` constraints are too tight. | Enable `Live Learning`. The system will auto-expand the profile to match reality. |

---

## Part VII: Quick Start Checklist

1.  **Prepare Inputs:** Ensure your localization system (GPS/SLAM) is running.
2.  **Initialize Bus:** Create `SystemBus` with `FrameTree`.
3.  **Register Sensors:** Add sensors to `FrameTree` with initial positions.
4.  **Run Loop:**
    *   Update `FrameTree` with current sensor positions.
    *   Read Sensor Data -> `process_detection()`.
    *   Read Prediction -> Control/GUI.
    *   Call `on_tick()` for maintenance.
5.  **Persist:** Save learned profiles on shutdown.
