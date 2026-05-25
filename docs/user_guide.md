# PentaTrack: Complete Operator & User Guide

**Version:** 1.0.0-production
**Audience:** System Operators, Mission Controllers, and Robotics Tuning Engineers.

---

## Part I: Understanding the Interface

### 1. The Prediction Tree (The "Ghost" Objects)
When you look at a PentaTrack visualization (3D View or HUD), you will see more than just the object. You see a "cloud" of positions. This is the **Prediction Tree**.

*   **The Root (Neutral / C0):**
    *   This is the solid, main object. It represents the **Current Position**.
    *   In "Dual Mode", you might see two roots: one for "Discrete" (exact position) and one for "Proportional" (center of mass).
*   **The Children (C+x, C-x, C+z, etc.):**
    *   These are the **Predicted Positions**.
    *   Think of them as "Ghost Objects" or "Possible Futures."
    *   They branch out from the root.
        *   **C+x (Right):** Where the object would be if it moved one unit to the right.
        *   **C+x+y (Forward-Right):** A second-level prediction (depth 2) showing a diagonal movement.
*   **Color Coding (Confidence):**
    *   The color of the Ghost Objects indicates **Confidence**.
    *   **Red/White (High Confidence):** The system believes the object *will* move this way. This usually aligns with the object's current velocity.
    *   **Blue/Transparent (Low Confidence):** The system thinks this movement is unlikely (e.g., reversing direction instantly).
*   **User Action:** If you see the Ghost Objects "lagging" or "jittering" wildly, the prediction depth may be too high, or the velocity signal is noisy.

### 2. The Drift Leader Indicator
PentaTrack constantly compares its *past predictions* against the *current reality*. It reports a **Drift Leader**.

*   **What it is:** The prediction center (e.g., C+x) that had the smallest error compared to where the object actually ended up.
*   **Visual:** Often displayed as a "Badge" or "Arrow" on the object (e.g., "Leader: C+x").
*   **Interpretation:**
    *   **Leader: C+x:** "The object is consistently moving to the Right. My Right prediction was most accurate."
    *   **Leader: C0 (Neutral):** "The object is stationary. Predicting movement was wrong."

### 3. The Intercept Point (Homing)
If you are running a pursuit scenario (Drone vs. Drone, Chaser vs. Target), PentaTrack outputs a special coordinate: **The Intercept Point**.

*   **Visual:** A distinct marker (e.g., a Green Sphere) floating in front of the Target.
*   **Meaning:** "Fly HERE to catch the target."
*   **Behavior:**
    *   When the chaser is far away, the Intercept Point floats far ahead of the target (leading it).
    *   As the chaser gets closer, the Intercept Point moves closer to the target's actual body (converging).
*   **User Action:** Your autopilot should always aim at the Intercept Point, not the Target itself. Aiming at the target results in chasing (following behind). Aiming at the Intercept Point results in capture.

---

## Part II: Interpreting System Status

### 1. Anomaly Flags
PentaTrack enforces physics. If an object violates its assigned "Profile," it flags an **Anomaly**.

**Common Flags & Meanings:**

| Flag Name | Meaning | Likely Real-World Cause | Action |
| :--- | :--- | :--- | :--- |
| **LateralDriftAnomaly** | Object moved sideways faster than physics allow. | Car drifting/skidding, or Sensor misalignment. | Check ground conditions (ice?). |
| **ApproachAccelAnomaly** | Object accelerated toward you too fast. | Head-on collision imminent, or Object misclassified (it's a missile, not a car). | Raise alert priority. |
| **VerticalDriftAnomaly** | Ground object moved vertically. | Object is airborne (ramp, crash), or GPS glitch. | Check altitude sensor. |

**If you see constant Anomalies:**
This usually means the **Profile** is wrong. You have told PentaTrack "This is a Sedan," but it is moving like a "Sports Car."
*   **Solution:** Check the `object_class` input. If it is "Unknown," let the **Live Learning** system run. It will create a new profile that fits the data.

### 2. Profile Status (Learning State)
The system tracks how "refined" its knowledge of the object is.

*   **Status: Baseline (Cold Start):** The system is using the default file (e.g., `vehicle_sedan.json`). It knows generic physics.
*   **Status: Learning:** The system is observing drift. It is gathering data (requires ~100-200 frames).
*   **Status: Refined:** The system has generated a **Live Profile**.
    *   *Benefit:* Predictions are now tighter. The "Prediction Tree" will shrink (pruning impossible moves) to match the object's actual capabilities.

---

## Part III: Configuration & Tuning Parameters

This section explains the key parameters you can adjust in `TrackingConfig` and what they do to the visualization.

### 1. Geometry Parameters

#### `mode` (Tracking Mode)
*   **Discrete:**
    *   *Visual:* You see Ghost Objects at fixed distances (e.g., 1 meter away, 2 meters away).
    *   *Use Case:* Fast objects, large open spaces.
*   **Proportional:**
    *   *Visual:* Ghost Objects hug the object tighter (e.g., 10% of the object's size).
    *   *Use Case:* Precision tracking, tight spaces, robotic arms.
*   **Dual:**
    *   *Visual:* You see both layers.
    *   *Use Case:* General purpose (Recommended).

#### `recursion_depth` (Lookahead)
*   **Value: 1:**
    *   *Visual:* Only one layer of Ghost Objects.
    *   *Performance:* Fastest.
    *   *Use Case:* High-speed combat, fast reflexes, low-CPU embedded devices.
*   **Value: 2:**
    *   *Visual:* Ghost Objects have children (Grandchildren predictions).
    *   *Performance:* Moderate.
    *   *Use Case:* Standard tracking (Recommended).
*   **Value: 3+:**
    *   *Visual:* Complex branching tree.
    *   *Performance:* Heavy.
    *   *Use Case:* Slow, complex maneuvering (e.g., drone navigating a maze).

### 2. Velocity Parameters

#### `enable_velocity_weighting`
*   **ON (Recommended):** Ghost Objects in the direction of travel turn Red (High Confidence). Objects behind turn Blue (Low Confidence).
*   **OFF:** All Ghost Objects are the same color (Uniform Confidence).
    *   *Use Case:* Only use OFF if the object moves randomly (e.g., a fly or bouncing ball).

#### `velocity_method`
*   **Ema (Exponential Moving Average):** Smoothes out jitter. Good for noisy sensors.
*   **LastDelta:** Reacts instantly to speed changes. Good for high-framerate sensors (LiDAR/Event Cameras).
*   **Lsq (Least Squares):** Most stable, but has a delay. Good for slow, long-term trends.

### 3. Learning Parameters

#### `enable_object_type`
*   **ON:** The system loads profiles (Sedan, Drone, Human) and enforces physics. Anomalies trigger if physics are violated.
*   **OFF:** The system treats everything as a "Generic Moving Object." No physics constraints.

#### `enable_drift_analysis`
*   **ON:** Required for Learning. The system measures error.
*   **OFF:** The system predicts but never checks if it was right. (No Learning, No Anomaly Detection).

---

## Part IV: Persistence & Data Management

### 1. The Profile Database
PentaTrack learns over time. By default, this knowledge is lost on restart. You must enable **Persistence**.

**The File:** `profiles.json` (or your specified path).
**Structure:**
```json
{
  "learned_sedan_01": {
    "max_lateral_drift_ms": 12.5, // Learned that this specific car drifts more than average
    "max_speed_ms": 45.0,
    ...
  }
}
```

### 2. Operational Procedure for Learning
If you are deploying a new class of object (e.g., a new type of delivery drone):

1.  **Start Empty:** Delete or archive old profiles.
2.  **Run Mission:** Let the system track the object.
3.  **Monitor Status:** Watch for "Status: Refined" in the UI.
4.  **Save:** Trigger the `save_profiles` command (usually on shutdown or via a UI button).
5.  **Deploy:** Next time, the system starts with "Knowledge" of how this drone moves.

---

## Part V: Troubleshooting Guide for Operators

### Symptom 1: "The Ghost Objects are lagging behind the real object."
*   **Diagnosis:** The velocity estimation is lagging.
*   **Fix:**
    1.  Check sensor frame rate. (Is it dropping frames?)
    2.  Switch `velocity_method` to `LastDelta` for faster reaction.
    3.  Reduce `recursion_depth` to lower CPU load.

### Symptom 2: "The Intercept Point is jittering/wildly jumping."
*   **Diagnosis:** The target is changing speed/heading rapidly, or sensor noise is high.
*   **Fix:**
    1.  Increase `softmax_temperature` (makes direction bias softer).
    2.  Increase `history_window` (smoothes velocity over a longer time).
    3.  Check if the "Drift Leader" is switching constantly (indicates erratic movement).

### Symptom 3: "Everything is flagged as an Anomaly."
*   **Diagnosis:** The physics profile is too strict, or the Coordinate Frame is wrong.
*   **Fix:**
    1.  **Critical Check:** Is the sensor moving? If the drone is flying but sending "Relative Coordinates," stationary trees will look like moving anomalies.
        *   *Solution:* Enable "World Frame" transformation in the System Bus.
    2.  If frames are correct, manually edit the JSON profile to increase `max_lateral_drift` thresholds.

### Symptom 4: "Track lost immediately after detection."
*   **Diagnosis:** ID mismatch. The detector changes the ID every frame.
*   **Fix:** This is an upstream issue (YOLO/Detector), not PentaTrack. Ensure the upstream detector maintains ID consistency (e.g., using SORT or DeepSORT).
