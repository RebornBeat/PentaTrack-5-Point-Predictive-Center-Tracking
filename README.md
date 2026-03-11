# PentaTrack — 5-Point Predictive Center Tracking

**Recursive multi-center bounding box prediction for real-time object tracking and movement forecasting.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](#license)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-green.svg)](#requirements)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange.svg)](#roadmap)

---

## Overview

PentaTrack is a movement prediction framework built on a simple but powerful idea: at any given moment, an object's next position can be described by **five predictive centers** — the current neutral center plus one center for each cardinal movement direction. Each of those predicted centers then becomes its own origin for another layer of five predictions, creating a **recursive prediction tree** that can anticipate complex trajectories before they happen.

The core system is intentionally minimal — 5 centers, bounding box in, predictions out. But PentaTrack is designed to scale up through **optional extensions** that can be enabled independently or combined:

- **Diagonal Centers** — Expand from 5-point to 9-point with four diagonal directions.
- **Rich Node Metadata** — Attach velocity, confidence, timestamps, and parent lineage to every center.
- **Velocity-Weighted Prediction** — Bias the prediction tree toward observed motion so it predicts where an object is *likely* going, not just where it *could* go.
- **Inter-Center Drift Analysis** — Measure drift vectors from every predicted center to the actual new position, capturing how objects move *between* prediction points rather than chasing grid resolution.
- **Velocity-Adaptive Drift** — Automatically adjust drift measurement sensitivity based on the object's speed so data stays meaningful at any velocity.
- **Object-Type Awareness** — Attach object classification (car, human, drone, aircraft, etc.) to constrain predictions within the physical realm of possibility for that object type, eliminating false positives.
- **Homing Intercept Prediction** — Project where a target will be when a pursuing object reaches it, with continuously refining intercept zones that converge as distance closes.

Each extension is opt-in. You can run bare 5-point tracking with zero overhead, or stack everything for a fully weighted, drift-aware, object-classified, intercept-capable prediction engine. The architecture doesn't care — everything composes cleanly.

This is not traditional single-centroid tracking. PentaTrack maps the *possibility space* of an object's movement at every frame.

---

## Core Concept

### The 5-Point Bounding Box Model

Every tracked object begins with a standard bounding box and its computed **neutral center** at coordinate origin `(0, 0, 0)`:

```
            +Z (Up)
              |
              |
  -X (Left) ─── C ─── +X (Right)
              |
              |
            -Z (Down)

        +Y (Forward / Away) ──→ into depth
        -Y (Backward / Towards) ──→ out of depth
```

From the neutral center **C**, four directional centers are projected:

| Center | Direction     | Offset        |
|--------|---------------|---------------|
| C₀     | Neutral       | `(0, 0, 0)`  |
| C₊ₓ   | Right         | `(+δ, 0, 0)` |
| C₋ₓ   | Left          | `(-δ, 0, 0)` |
| C₊z   | Up            | `(0, 0, +δ)` |
| C₋z   | Down          | `(0, 0, -δ)` |

This gives us **5 centers** at any given point in time — a pentagonal prediction field around the object.

### Coordinate System

| Axis | Positive (+)         | Negative (-)            |
|------|----------------------|-------------------------|
| X    | Right                | Left                    |
| Y    | Forward / Away       | Backward / Towards      |
| Z    | Up                   | Down                    |

> **Note:** The Y-axis (forward/backward, i.e. depth) is supported at the architecture level and can be enabled for 3D tracking scenarios (e.g. drone altitude + distance), expanding the base model to **7 centers** when active.

### What Does "±1" Mean?

The displacement magnitude `δ` is configurable and operates in two modes:

- **Full displacement (`discrete`)** — A shift of `±1` means the entire bounding box has moved by one full box-width in that direction. This represents a complete, committed movement.
- **Fractional displacement (`proportional`)** — A shift of `±δ` represents a percentage of the bounding box dimension (e.g., `δ = 0.10` means 10% of box width). This captures micro-movements and drift.

Both modes can run **simultaneously** (`dual` mode). When they do, each direction produces two predicted centers (one full, one fractional), expanding the prediction field:

| Mode           | Centers per Frame | Description                          |
|----------------|-------------------|--------------------------------------|
| `discrete`     | 5                 | Neutral + 4 cardinal (full shift)    |
| `proportional` | 5                 | Neutral + 4 cardinal (fractional)    |
| `dual`         | 9                 | Neutral + 4 full + 4 fractional      |
| `3d-discrete`  | 7                 | + Forward/Backward (full shift)      |
| `3d-dual`      | 13                | + Forward/Backward (full + fractional)|

### Recursive Center Expansion

Here's where PentaTrack gets interesting. Each predicted center **itself becomes a new origin**, spawning its own set of 5 (or more) child centers:

```
                        C₊z
                       / | \
                 C₋ₓ  C₀  C₊ₓ
                       |
                      C₋z
                     ↙  ↓  ↘
              C₋ₓ'  C₀'  C₊ₓ'     ← second-level predictions
                     |
                    C₊z'
```

At **depth 1**: 5 centers (base prediction).
At **depth 2**: up to 25 centers (each of the 5 spawns 5 more).
At **depth N**: up to `5^N` centers.

In practice, pruning strategies collapse redundant or low-probability branches to keep computation bounded. But the key insight remains: **every center is always a potential new origin**, and the system can look arbitrarily far ahead by increasing recursion depth.

---

## Optional Extensions

PentaTrack's core 5-point model is complete on its own. The following extensions are independent modules that can be enabled individually or in any combination to add capability without changing the base tracking logic.

### Extension: Diagonal Centers

**Enable with:** `enable_diagonals=True`

Adds four diagonal prediction centers between the cardinals, expanding the model from 5-point to **9-point**:

```
  (-X,+Z)       +Z (Up)       (+X,+Z)
       ╲          |           ╱
         ╲        |         ╱
 -X (Left) ───── C ───── +X (Right)
         ╱        |         ╲
       ╱          |           ╲
  (-X,-Z)       -Z (Down)     (+X,-Z)
```

**Additional Diagonal Centers:**

| Center   | Direction     | Offset            |
|----------|---------------|-------------------|
| C₊ₓ₊z  | Up-Right      | `(+δ, 0, +δ)`    |
| C₊ₓ₋z  | Down-Right    | `(+δ, 0, -δ)`    |
| C₋ₓ₊z  | Up-Left       | `(-δ, 0, +δ)`    |
| C₋ₓ₋z  | Down-Left     | `(-δ, 0, -δ)`    |

The four diagonals capture the reality that most real-world movement doesn't happen in pure cardinal directions. A drone banking right while descending, a pedestrian cutting diagonally across a street, a ball arcing up-right off a bat — these all land naturally in the diagonal prediction zones.

**Center counts with diagonals enabled:**

| Mode                         | Centers per Frame | Description                                        |
|------------------------------|-------------------|----------------------------------------------------|
| `discrete` + diagonals       | 9                 | Neutral + 4 cardinal + 4 diagonal (full shift)     |
| `proportional` + diagonals   | 9                 | Neutral + 4 cardinal + 4 diagonal (fractional)     |
| `dual` + diagonals           | 17                | Neutral + 8 full + 8 fractional                    |
| `3d-discrete` + diagonals    | 27                | + Y-axis cardinal, diagonal, and corner directions  |
| `3d-dual` + diagonals        | 53                | + Y-axis full + fractional for all 26 directions   |

**Recursive impact:** With diagonals enabled, each center spawns 9 children instead of 5. At depth 2 this means up to 81 centers (vs 25 without diagonals). Pruning keeps this practical — a typical depth-2 expansion with diagonals + pruning produces **15–30 live centers** focused along the actual motion corridor.

```
                              C₀ (root)
               ╱    ╱    |    ╲    ╲    ╲
          C₊ₓ   C₋ₓ   C₊z  C₋z  C₊ₓ₊z ... (depth 1, 9 nodes)
          ╱|╲   ╱|╲   ╱|╲   ╱|╲
         ... ... ... ... ...            (depth 2, up to 81 nodes)
```

---

### Extension: Rich Node Metadata

**Enable with:** `enable_metadata=True`

By default, each center is a simple `(x, y, z)` coordinate. With metadata enabled, every center becomes a **rich data node** — because the center is always tracked, you can attach additional information to each center in the prediction tree.

Each center node stores:

```
CenterNode {
    position:    (x, y, z)       # Spatial coordinates
    velocity:    (vx, vy, vz)    # Estimated velocity vector at this point
    confidence:  float           # Probability weight (0.0 – 1.0)
    timestamp:   float           # Frame time or system clock
    parent:      CenterNode?     # Reference to the origin center that spawned this node
    depth:       int             # Recursion depth (0 = root neutral center)
    label:       str             # Human-readable direction label
    children:    CenterNode[]    # Child predictions (populated on recursive expansion)
}
```

#### Why This Matters

**Velocity** lets each center carry forward the object's current motion vector, so child predictions inherit and propagate momentum rather than treating each frame as stateless.

**Confidence** is the probability weight assigned to this center. When combined with the [Velocity-Weighted Prediction](#extension-velocity-weighted-prediction) extension, the tree becomes biased — not all branches are equal.

**Timestamp** anchors every prediction in time, enabling temporal queries like "where was the highest-confidence center 200ms ago?" and time-delta velocity calculations between frames.

**Parent** gives every center full lineage. You can walk backward from any leaf node to reconstruct the exact predicted path that led there. This is critical for trajectory analysis — if a depth-3 child center ends up matching the actual object position, you can trace back through its parent chain to understand *which sequence of movements* the object followed.

**Depth** lets you distinguish between immediate predictions (depth 1) and speculative lookahead (depth 3+), and apply different confidence thresholds at each level.

#### Center Node as a Graph

Because every center knows its parent and children, the full prediction state at any frame is a **rooted tree (or DAG after pruning)**:

```
                         C₀ (root, depth 0)
               ╱    ╱    |    ╲    ╲
            C₊ₓ   C₋ₓ   C₊z  C₋z  ... (depth 1)
            ╱|╲   ╱|╲   ╱|╲   ╱|╲
           ... ... ... ... ...          (depth 2)
```

Each node is a fully self-contained prediction with all the metadata needed to evaluate, compare, and route decisions downstream. This structure is the same whether you're running 5-point, 9-point (with diagonals), or full 3D — the metadata layer is independent of the geometric model.

---

### Extension: Velocity-Weighted Prediction

**Enable with:** `enable_velocity_weighting=True`
**Requires:** `enable_metadata=True` (velocity weighting uses the confidence and velocity fields on each node)

The raw center model — whether 5-point or 9-point — gives you all *possible* directions uniformly. Velocity weighting tells you which ones *actually matter* right now by biasing the prediction tree toward observed motion.

#### The Problem with Uniform Prediction

Without weighting, every directional center gets equal probability. An object moving hard to the right has the same prediction weight on its left-center as its right-center. That's geometrically complete but practically wasteful — you're spending computation on branches the object is almost certainly not taking.

#### How Velocity Weighting Works

PentaTrack computes a velocity vector from the object's recent center history (configurable window, default 30 frames). This velocity is then decomposed into directional components and mapped onto the prediction centers as **probability weights**.

**Example — 5-point model, object moving to the right (`velocity ≈ +X`):**

```
                     C₊z
                     0.20
                      |
     C₋ₓ  0.10 ──── C₀  0.05 ──── C₊ₓ  0.45
                      |
                     C₋z
                     0.20
```

**Example — 9-point model (diagonals enabled), same velocity:**

```
               C₋ₓ₊z        C₊z         C₊ₓ₊z
               0.02         0.05         0.08
                  ╲          |           ╱
                    ╲        |         ╱
     C₋ₓ  0.03 ──── C₀  0.02 ──── C₊ₓ  0.45
                    ╱        |         ╲
                  ╱          |           ╲
               C₋ₓ₋z        C₋z        C₊ₓ₋z
               0.02         0.05         0.08

                    ──── velocity ≈ +X ────→

     Weights sum to ≈ 0.80 (remaining 0.20 reserved
     for Y-axis or redistributed if 2D-only)
```

The weights are **not hardcoded** — they're computed every frame from observed motion:

| Center   | Weight | Reasoning                                            |
|----------|--------|------------------------------------------------------|
| C₊ₓ     | 0.45   | Dominant motion direction — highest confidence        |
| C₊ₓ₊z   | 0.08   | Right + Up diagonal — plausible if climbing *(9pt)*   |
| C₊ₓ₋z   | 0.08   | Right + Down diagonal — plausible if descending *(9pt)* |
| C₊z     | 0.05   | Pure up — possible but not current trend              |
| C₋z     | 0.05   | Pure down — possible but not current trend            |
| C₋ₓ     | 0.03   | Against motion — unlikely but kept for reversal       |
| C₋ₓ₊z   | 0.02   | Against motion + up — very unlikely *(9pt)*           |
| C₋ₓ₋z   | 0.02   | Against motion + down — very unlikely *(9pt)*         |
| C₀       | 0.02   | Stationary — unlikely given strong +X velocity        |

*(Items marked 9pt only appear when diagonals are enabled)*

#### Weight Distribution Strategies

PentaTrack supports multiple strategies for converting a velocity vector into center weights:

**Cosine similarity (`cosine`)** — Each center's direction vector is compared against the velocity vector. Weight is proportional to `cos(θ)` between them. This is the default and produces smooth, intuitive distributions.

**Softmax projection (`softmax`)** — The dot product of velocity with each center direction is run through a softmax function with configurable temperature. Lower temperature = sharper bias toward the dominant direction. Higher temperature = more uniform spread.

**Exponential decay (`decay`)** — Weight falls off exponentially with angular distance from the velocity vector. Produces very sharp distributions — useful when you want to aggressively prune unlikely branches.

**Hybrid (`hybrid`)** — Blends cosine similarity for cardinal centers with softmax for diagonals, letting you tune how aggressively diagonal predictions are weighted relative to cardinals. *(Only meaningful when diagonals are enabled; falls back to cosine in 5-point mode.)*

```python
config = TrackingConfig(
    enable_velocity_weighting=True,
    weight_strategy="cosine",      # "cosine", "softmax", "decay", "hybrid"
    softmax_temperature=1.0,       # Lower = sharper (only for softmax/hybrid)
    min_weight=0.01,               # Floor — no center ever drops to true zero
)
```

#### Weight Propagation Through Recursion

When a center spawns children during recursive expansion, the child weights are computed as:

```
child.confidence = parent.confidence × parent.weight_for_child_direction × confidence_decay
```

This means high-confidence parent centers produce high-confidence children, while low-probability parents generate low-probability subtrees that get pruned early. The result is a prediction tree that's **dense where motion is happening and sparse where it isn't**.

#### Velocity Estimation Methods

Velocity is computed from the center history buffer using configurable methods:

**Simple difference (`last_delta`)** — `v = center[t] - center[t-1]`. Fast, noisy.

**Weighted moving average (`wma`)** — Recent frames weighted more heavily. Smooth, slight lag.

**Exponential moving average (`ema`)** — Standard EMA with configurable alpha. Good balance of responsiveness and smoothing.

**Least-squares fit (`lsq`)** — Fits a line through the last N centers. Most stable, highest latency.

```python
config = TrackingConfig(
    velocity_method="ema",         # "last_delta", "wma", "ema", "lsq"
    velocity_ema_alpha=0.3,        # Smoothing factor for EMA
    history_window=30,             # Frames used for velocity estimation
)
```

---

### Extension: Inter-Center Drift Analysis

**Enable with:** `enable_drift_analysis=True`

This is the extension that bridges the gap between PentaTrack's discrete prediction centers and the continuous reality of object motion. The core insight: an object's actual new position will almost never land exactly on one of the 5 or 9 predicted centers. It will land *between* them. Rather than chasing that precision by subdividing δ into finer and finer fractions (10% → 1% → 0.1%), which creates exponential center bloat with no guarantee of an exact match, drift analysis takes the opposite approach — **accept the coarse grid and measure how the object relates to it**.

#### Why Not Just Add More Centers?

The naive solution to the "object doesn't land on a center" problem is to increase resolution. Break 10% displacement into 1%, or 0.1%, hoping a center lands close enough. This is a trap:

- At 1% resolution with 5-point base, you're generating 50+ centers per direction per frame
- You'd potentially need to go to 0.01% or smaller to reliably "catch" real motion
- The computational cost scales with the number of centers, not the quality of data
- Even at extreme resolution, real motion is continuous — you still can't guarantee a grid hit

Drift analysis solves this by treating it as a **measurement problem instead of a resolution problem**. You keep the coarse grid (5 or 9 centers) and instead measure the displacement vector from every center to the actual new position.

#### How It Works

Each frame, after the object's actual new center is detected, PentaTrack computes a **drift vector from every existing predicted center** to the actual new position:

```
Frame N: Object at position P_old, 5 centers predicted
Frame N+1: Object detected at P_new (actual)

drift_from_neutral  = P_new - C₀_old       # Drift relative to "stayed still"
drift_from_right    = P_new - C₊ₓ_old      # Drift relative to "moved right"
drift_from_left     = P_new - C₋ₓ_old      # Drift relative to "moved left"
drift_from_up       = P_new - C₊z_old      # Drift relative to "moved up"
drift_from_down     = P_new - C₋z_old      # Drift relative to "moved down"
```

With 9-point (diagonals enabled), you get 9 drift vectors. Each one tells a different story:

```
                Frame N                              Frame N+1
          Predicted Centers                      Actual + Drift Vectors

              C₊z                                     C₊z ╌╌╌╌→ ●
               |                                       |      ╱ P_new
     C₋ₓ ─── C₀ ─── C₊ₓ              C₋ₓ ╌╌╌→ C₀ ╌╌→ C₊ₓ ╌→
               |                                       |    (smallest
              C₋z                                     C₋z    drift!)

     ╌╌→ = drift vector from that center to actual position
     Shortest drift = closest prediction = most accurate center
```

**The center with the smallest drift magnitude tells you which prediction was closest.** The direction and magnitude of *all* drift vectors together give you a multi-perspective motion profile far richer than a single centroid displacement:

- "The object moved right" → single-centroid tells you this
- "The object moved right, with slight upward drift relative to the pure-right prediction, accelerating drift from neutral, and the left-center drift is growing non-linearly" → multi-center drift tells you this

Over multiple frames, the **drift history per center** reveals patterns that pure positional tracking can't see: curved trajectories appear as rotating drift vectors, acceleration appears as growing drift magnitudes, and sudden direction changes appear as drift-leader switches (which center has the smallest drift flips from one to another).

#### Drift Accuracy Tracking

Each center accumulates a rolling accuracy score based on how close its prediction was to reality:

```
DriftRecord {
    center_label:     str          # Which center this drift was measured from
    drift_vector:     (dx, dy, dz) # Vector from predicted center to actual position
    drift_magnitude:  float        # |drift_vector| — smaller = better prediction
    frame:            int          # Frame number
    timestamp:        float        # Time of measurement
}
```

The center with the best (lowest) average drift over the recent window becomes the **drift leader** — the most reliable predictor for the current motion pattern. This feeds directly into velocity weighting (if enabled) and into homing intercept calculations (if enabled).

```python
config = TrackingConfig(
    enable_drift_analysis=True,
    drift_history_window=30,       # Frames of drift history to retain
)

prediction = tracker.update(bbox)

# Access drift data for the most recent frame
for drift in prediction.drift_vectors:
    print(f"Drift from {drift.center_label}: "
          f"vector={drift.drift_vector}, mag={drift.drift_magnitude:.4f}")

# Find the drift leader (most accurate center)
leader = prediction.drift_leader()
print(f"Most accurate predictor: {leader.center_label} "
      f"(avg drift: {leader.avg_magnitude:.4f})")
```

---

### Extension: Velocity-Adaptive Drift

**Enable with:** `enable_adaptive_drift=True`
**Requires:** `enable_drift_analysis=True`

Standard drift analysis measures the vector from each predicted center to the actual position once per frame. This works well for slow-moving objects where frame-to-frame displacement is a fraction of δ. But fast-moving objects can blow past the entire bounding box in a single frame, making raw drift vectors enormous and potentially meaningless at a fixed measurement rate.

Velocity-adaptive drift solves this by making the drift engine **velocity-aware** — it automatically adjusts its measurement sensitivity, weighting, and interpretation based on how fast the target is moving.

#### Speed Regimes

| Regime         | Velocity Range               | Drift Behavior                          | Adaptation                                      |
|----------------|------------------------------|-----------------------------------------|-------------------------------------------------|
| **Slow**       | `< 0.25δ` per frame         | Small, precise drift vectors            | Standard per-frame measurement; all 5/9 vectors equally informative |
| **Medium**     | `0.25δ – 1.0δ` per frame    | Moderate drift; some centers overshoot  | Recent drift measurements weighted more heavily; drift magnitude normalized by velocity |
| **Fast**       | `1.0δ – 5.0δ` per frame     | Object overshoots multiple centers      | Sub-frame interpolation activated; drift vectors measured against interpolated intermediate positions |
| **Extreme**    | `> 5.0δ` per frame          | Object traverses entire prediction field| Drift measured relative to velocity-extrapolated centers; fixed-grid drift becomes secondary to trajectory-fit drift |

#### What Adapts

**Measurement weighting** — At low speed, drift from all centers is weighted equally. As speed increases, drift measurements from the drift leader (closest-predicting center) gain weight while opposing-direction drift measurements are down-weighted, since an object moving fast to the right generates large but uninformative drift values from the left-center.

**Normalization** — Raw drift magnitudes scale with velocity (faster = bigger drift numbers). The adaptive system normalizes drift by the object's current speed so that drift values remain comparable across velocity changes. A drift of 0.1δ means the same thing whether the object is moving at 0.5δ/frame or 5.0δ/frame.

**Rate of drift change** — For accelerating objects, the drift itself isn't the key signal — the rate of drift *change* is. The adaptive system tracks `d(drift)/dt` and uses it to detect acceleration, deceleration, and trajectory curvature that pure drift values miss.

**Interpolation** — At high velocities, the system interpolates sub-frame positions between the last known center and the new detected center, then computes drift against each interpolation point. This recovers information that would be lost if you only measured drift at the coarse frame rate.

```python
config = TrackingConfig(
    enable_drift_analysis=True,
    enable_adaptive_drift=True,
    drift_speed_thresholds=[0.25, 1.0, 5.0],  # δ/frame boundaries for speed regimes
    drift_interpolation_steps=4,                # Sub-frame interpolation points for fast objects
    drift_normalize_by_velocity=True,           # Normalize drift magnitudes by speed
    drift_acceleration_tracking=True,           # Track d(drift)/dt for accel detection
)
```

---

### Extension: Object-Type Awareness

**Enable with:** `enable_object_type=True`
**Enhances:** `enable_drift_analysis=True`, `enable_adaptive_drift=True` (drift extensions gain physics-constrained boundaries)

Every object type has a **physical realm of possibility** for how it can move. A car on a road is constrained to lane-width lateral drift and road-following trajectories. A human walking can suddenly stop, turn, or trip and fall. An airplane in flight has a minimum turn radius and can't instantly reverse. A helicopter can hover but is susceptible to translational lift drift. A boat is constrained by water resistance and hull dynamics.

Object-type awareness lets PentaTrack attach a classification to each tracked object and use it to **constrain predictions within physically plausible boundaries** and **flag anomalous drift as potential events** (crash, fall, loss of control, evasive maneuver).

#### Object Classification Hierarchy

```
ObjectType {
    category:       str           # Top-level: "vehicle", "human", "animal", "aircraft", "watercraft"
    type:           str           # Specific: "sedan", "motorcycle", "pedestrian", "eagle", "helicopter"
    model:          str?          # Exact (optional): "Toyota Camry 2024", "DJI Mavic 3", "Boeing 737-800"
    constraints:    DriftProfile  # Physical movement boundaries for this object type
}
```

The classification is hierarchical — each level adds more specific constraints:

**Category level** provides broad physics: "vehicle" means ground-constrained, can't move vertically unless airborne (which itself is an anomaly flag). "Aircraft" means 3D movement is expected but turn radius is limited by aerodynamics.

**Type level** refines the constraints: "motorcycle" has tighter lateral drift limits than "semi-truck" but can lean into turns. "Helicopter" can hover (near-zero drift is normal) while "fixed-wing" cannot (zero drift = stalling).

**Model level** (optional) provides the tightest constraints: a specific aircraft model has known stall speeds, turn rates, and climb limits. A specific car model has known wheelbase and turning radius. This level enables the most aggressive false-positive rejection but requires the most specific input data.

#### Drift Profiles

Each object type maps to a `DriftProfile` that defines the physical boundaries of possible movement:

```
DriftProfile {
    max_lateral_drift:    float    # Maximum plausible lateral (X) displacement per frame
    max_vertical_drift:   float    # Maximum plausible vertical (Z) displacement per frame
    max_longitudinal_drift: float  # Maximum plausible forward/back (Y) displacement per frame
    max_turn_rate:        float    # Maximum angular velocity (degrees/frame)
    can_hover:            bool     # Can the object hold position (zero drift)?
    can_reverse:          bool     # Can the object move backward?
    min_speed:            float    # Minimum sustained speed (0 for cars, stall speed for aircraft)
    max_speed:            float    # Maximum plausible speed
    anomaly_thresholds:   dict     # Per-axis drift thresholds that trigger anomaly flags
}
```

#### How It Constrains Predictions

When drift analysis is enabled alongside object-type awareness, the system gains two capabilities:

**1. Prediction Pruning by Physical Plausibility**

Before the prediction tree is built, the system checks each candidate center against the object's drift profile. Centers that require physically impossible movement are pruned before they're ever expanded:

```
Object: sedan, velocity ≈ 60mph heading +X

  C₊z (Up) → requires vertical lift → sedan can't fly → PRUNED
  C₋z (Down) → requires downward movement → already on ground → PRUNED
  C₊ₓ (Right/Forward) → within max_lateral_drift → KEPT
  C₋ₓ (Left/Reverse) → within turn-rate limits → KEPT (reduced weight)
  C₀ (Stop) → below min braking distance → REDUCED WEIGHT
```

This doesn't remove centers from the model — it assigns them near-zero or zero confidence based on what's physically possible for that object type. The 5/9-point geometry stays intact; the weighting becomes object-aware.

**2. Anomaly Detection via Drift Boundary Violations**

When an observed drift *exceeds* the drift profile's boundaries, that's not just a tracking error — it's potentially a real-world event:

| Drift Anomaly                           | Object Type    | Possible Real Event                                    |
|-----------------------------------------|----------------|--------------------------------------------------------|
| Lateral drift exceeds max for speed     | Car            | Skidding, hydroplaning, or crash in progress           |
| Vertical drift on ground-constrained    | Motorcycle     | Airborne (ramp, collision, fall)                       |
| Drift suddenly drops to zero            | Fixed-wing     | Engine failure / stall (below min_speed threshold)     |
| Extreme lateral + rotation spike        | Helicopter     | Loss of tail rotor / spinning                          |
| Drift exceeds human sprint speed        | Pedestrian     | Running, falling, or being carried by vehicle          |
| Sudden reversal exceeding turn_rate     | Boat           | Capsizing or collision event                           |
| Drift pattern matches known failure     | Any            | Matched against historical anomaly signatures          |

These anomaly flags don't stop tracking — they annotate the prediction with event likelihood so downstream systems (autopilot, alert system, analytics) can react appropriately.

#### Historical Drift Profiles

When `enable_object_type=True` with drift analysis, PentaTrack maintains a **per-object-type drift history database**. Over time, the system builds empirical drift profiles from observed data:

```python
# After tracking hundreds of "sedan" objects:
sedan_profile = tracker.get_empirical_profile("vehicle", "sedan")
# Returns observed drift distributions, not just theoretical maximums
# e.g., "95th percentile lateral drift at 30mph = 0.03δ"
```

This means the system **learns what normal drift looks like** for each object type from real tracking data, making anomaly detection increasingly precise over time without manual tuning.

```python
config = TrackingConfig(
    enable_object_type=True,
    object_type_category="vehicle",
    object_type_type="sedan",
    object_type_model=None,                  # Optional: "Toyota Camry 2024"
    anomaly_sensitivity=0.95,                # Percentile threshold for anomaly flags
    build_empirical_profiles=True,           # Learn drift profiles from observed data
    profile_database_path="./drift_profiles.db",  # Where to store learned profiles
)

# Or set object type dynamically per tracked object
tracker.set_object_type(
    track_id="obj_42",
    category="aircraft",
    type="helicopter",
    model="DJI Mavic 3"
)
```

---

### Extension: Homing Intercept Prediction

**Enable with:** `enable_homing=True`
**Requires:** `enable_drift_analysis=True` (intercept projections use multi-center drift data)

This extension answers the question that matters in pursuit and intercept scenarios: **"Where will the target be when I get there?"** — not where it is now, and not where it might go eventually, but the specific position it will occupy at the moment of arrival.

#### The Intercept Problem

You have two objects: the **homing object** (a drone, a robotic arm, a guided interceptor — with known speed and position) and the **target** (being tracked by PentaTrack). A simple approach is to aim at the target's current position, but by the time the homing object arrives, the target has moved. Aim at a single predicted position, and the prediction might be wrong.

PentaTrack's approach: use the 5/9 drift vectors from the target to compute **multiple intercept projections**, weight them by recent drift accuracy, and produce a **probability-weighted intercept zone** that continuously refines as distance closes.

#### How It Works

```
HOMING INTERCEPT LOOP (runs every frame):

  1. MEASURE    → Compute 5/9 drift vectors on target (from drift analysis)
  2. ESTIMATE   → Derive target velocity from drift history
  3. COMPUTE    → time_to_arrival = distance(homing, target) / homing_speed
  4. PROJECT    → For each center, project forward by time_to_arrival:
                   intercept_point[i] = C[i] + (drift_vector[i] × time_to_arrival)
  5. WEIGHT     → Weight each intercept_point by that center's drift accuracy:
                   best drift leader → highest intercept confidence
  6. AIM        → Weighted average of intercept points = aim target
  7. REPEAT     → As distance shrinks, projections converge
```

**Visualized over time:**

```
  T = 5 seconds out                     T = 2 seconds out

       ╱ intercept from C₊z                 ╱ from C₊z
      ● (low conf)                          ● (low)
     ╱                                     ╱
    ╱   ● intercept from C₊ₓ    →        ● from C₊ₓ    → tighter
   ╱   (high conf)                       (high)            cluster
  ●─── ★ weighted aim point        ●─── ★ weighted aim
   ╲   ● intercept from C₀              ● from C₀
    ╲  (med conf)                        (med)
     ╲
      ● intercept from C₋ₓ
       (low conf)


  T = 0.5 seconds out

    ●●★● ← all intercepts nearly converged
          minimal projection window
          essentially real-time tracking
```

The key property: **as the homing object closes distance, the intercept zone collapses**. Early in pursuit, the zone might span a wide area because the projection window is long and uncertainty is high. In the final moments, time-to-arrival is so small that all 5/9 intercept projections converge toward the same point — the system degrades gracefully into pure real-time tracking.

#### Reference Frames

The homing object's own motion affects what drift means. PentaTrack supports two reference frames:

**World frame (`reference_frame="world"`)** — Drift and intercept are computed in absolute coordinates. Use when the tracking system (camera, sensor) is stationary or its motion is already compensated. The standard for fixed-camera tracking and ground-station-computed guidance.

**Homing frame (`reference_frame="homing"`)** — Drift is computed relative to the homing object's position and velocity. This gives you closing-rate-adjusted drift that directly answers "how is the target moving relative to me?" — the question that matters for final approach guidance. Use when computing guidance from the homing object's own sensor.

```python
config = TrackingConfig(
    enable_homing=True,
    enable_drift_analysis=True,       # Required by homing
    reference_frame="world",          # "world" or "homing"
    homing_speed=10.0,                # Homing object speed (units/second)
    intercept_convergence_log=True,   # Log how the intercept zone tightens over time
)

# Per-frame update with homing object position
prediction = tracker.update(
    target_bbox=[120, 200, 280, 400],
    homing_position=(500, 0, 300),    # Homing object's current (x, y, z)
    homing_velocity=(−8.0, 0, −2.0),  # Homing object's velocity vector
)

# Get the intercept prediction
intercept = prediction.intercept
print(f"Aim point: {intercept.weighted_position}")
print(f"Time to arrival: {intercept.time_to_arrival:.2f}s")
print(f"Intercept zone radius: {intercept.zone_radius:.3f}")  # Shrinks as distance closes
print(f"Confidence: {intercept.confidence:.2%}")

# Access individual intercept projections per center
for proj in intercept.projections:
    print(f"  {proj.from_center}: {proj.position} (conf: {proj.confidence:.3f})")
```

#### Object-Type Enhanced Intercept

When combined with [Object-Type Awareness](#extension-object-type-awareness), intercept prediction becomes physically constrained. The system knows the target can't move in ways that violate its drift profile, so impossible intercept projections are automatically suppressed:

```
Target: sedan on highway, velocity ≈ +X at 70mph
Homing: drone at 45mph, 200m behind

Intercept projection from C₊z → target would need to fly upward → sedan can't fly → SUPPRESSED
Intercept projection from C₋ₓ → target would need to instantly reverse at 70mph → exceeds turn rate → SUPPRESSED
Intercept projection from C₊ₓ → target continues forward → physically consistent → HIGH CONFIDENCE
Intercept projection from C₊ₓ₊z → target continues forward with climb → impossible on ground → SUPPRESSED
```

This dramatically tightens the intercept zone early in pursuit, because physically impossible projections are eliminated before they can widen the uncertainty spread.

---

## Extension Combinations

The extensions compose freely. Here's how center counts and capabilities scale:

| Configuration                                  | Centers (depth 1) | Per-Center Data                | Weighting       | Drift        | Intercept |
|------------------------------------------------|--------------------|--------------------------------|-----------------|--------------|-----------|
| Base 5-point                                   | 5                  | `(x, y, z)` only              | Uniform         | —            | —         |
| Base + metadata                                | 5                  | Full CenterNode                | Uniform         | —            | —         |
| Base + diagonals                               | 9                  | `(x, y, z)` only              | Uniform         | —            | —         |
| Base + diagonals + metadata                    | 9                  | Full CenterNode                | Uniform         | —            | —         |
| Base + metadata + velocity                     | 5                  | Full CenterNode                | Velocity-biased | —            | —         |
| Base + drift                                   | 5                  | `(x, y, z)` + drift vectors   | Uniform         | Per-frame    | —         |
| Base + drift + adaptive                        | 5                  | `(x, y, z)` + drift vectors   | Uniform         | Velocity-adaptive | —    |
| Base + drift + object-type                     | 5                  | `(x, y, z)` + drift + class   | Physics-constrained | Per-frame + anomaly | — |
| Base + drift + homing                          | 5                  | `(x, y, z)` + drift vectors   | Uniform         | Per-frame    | Active    |
| All extensions enabled                         | 9                  | Full CenterNode + class + drift| Velocity + physics | Adaptive + anomaly | Active + constrained |
| All + 3D Y-axis                                | 27                 | Full CenterNode + class + drift| Velocity + physics | Adaptive + anomaly | Active + constrained |
| All + 3D + dual mode                           | 53                 | Full CenterNode + class + drift| Velocity + physics | Adaptive + anomaly | Active + constrained |

The design principle: **start minimal, enable what you need.** A drone tracking a single target in 2D might only need base 5-point. A multi-object autonomous vehicle system might run the full stack. A homing interceptor needs drift + homing but might skip diagonals if the target moves primarily in one axis. Same engine, different configuration.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                       PentaTrack Core                        │
├──────────────┬────────────────┬───────────────────────────────┤
│   Detector   │  Center Engine │  Prediction Tree              │
│   ─────────  │  ────────────  │  ────────────────             │
│   BBox input │  Neutral calc  │  5-point base expansion       │
│   YOLO/custom│  δ projection  │  Recursive spawning           │
│   Frame sync │  Mode select   │  Branch pruning               │
│              │                │  Depth control                │
├──────────────┴────────────────┴───────────────────────────────┤
│                    Optional Extensions                        │
├──────────────┬────────────────┬───────────────────────────────┤
│  Diagonals   │  Velocity      │  Node Metadata                │
│  ──────────  │  Weighting     │  Manager                      │
│  4 diagonal  │  ──────────    │  ─────────────                │
│  centers     │  Cosine/Softmax│  Position, velocity           │
│  9pt compass │  Weight distrib│  Confidence, timestamp         │
│  3D corners  │  Propagation   │  Parent refs, depth            │
│              │  Min-weight    │  Lineage traversal             │
├──────────────┼────────────────┼───────────────────────────────┤
│  Drift       │  Adaptive      │  Object-Type                  │
│  Analysis    │  Drift         │  Awareness                    │
│  ──────────  │  ──────────    │  ─────────────                │
│  Multi-center│  Speed regime  │  Category/type/model          │
│  drift vecs  │  detection     │  Drift profiles               │
│  Drift leader│  Normalization │  Physical constraints          │
│  Per-center  │  Sub-frame     │  Anomaly detection             │
│  accuracy    │  interpolation │  Empirical learning            │
│  Drift hist  │  Accel tracking│  False-positive rejection      │
├──────────────┼────────────────┼───────────────────────────────┤
│  Velocity    │  Homing        │                               │
│  Estimator   │  Intercept     │                               │
│  ──────────  │  ──────────    │                               │
│  EMA/WMA/LSQ │  Multi-proj    │                               │
│  History buf │  Convergence   │                               │
│  Alpha tuning│  Ref frames    │                               │
│              │  Aim point     │                               │
├──────────────┴────────────────┴───────────────────────────────┤
│                        Tracker State                          │
│  Per-object center history, velocity vectors, probability     │
│  weights, trajectory buffer, parent-child graph, drift        │
│  history, object profiles, intercept state                    │
├──────────────────────────────────────────────────────────────┤
│                     Output / Integration                      │
│  Raw center streams • Predicted trajectory • Confidence map • │
│  Lineage trace • Drift vectors • Anomaly flags • Intercept •  │
│  Drone autopilot interface • Viz overlay                      │
└──────────────────────────────────────────────────────────────┘
```

---

## Use Cases

- **Drone Object Tracking** — Predict where a target is heading before the drone's gimbal or flight path adjusts. Feed PentaTrack centers into a PID controller for smoother pursuit. With velocity weighting enabled, the dominant-weight center becomes the aim point, with diagonal centers providing anticipatory correction for banking or climbing targets. Enable drift analysis for sub-center precision on smooth trajectories.
- **Autonomous Vehicle Perception** — Anticipate pedestrian or vehicle movement from bounding box streams. The recursive depth allows multi-step lookahead. Enable diagonals to capture the reality that pedestrians rarely move in pure cardinal directions. With metadata + velocity weighting, parent-chain lineage analysis can detect intention changes (e.g., a pedestrian moving right suddenly shifts weight to up-right = stepping off curb). Object-type awareness with "pedestrian" classification constrains predictions to walking speed and flags sprint-speed drift as a running or falling event.
- **Sports Analytics** — Track player or ball movement with directional prediction for play analysis and broadcast augmentation. Velocity weighting naturally captures acceleration, deceleration, and direction changes. Full node metadata enables post-game trajectory reconstruction. Adaptive drift handles the wide speed range from stationary set pieces to full-sprint breakaways.
- **Surveillance & Security** — Predict subject trajectory across camera feeds. Predictive centers can trigger alerts before a subject reaches a boundary. With metadata enabled, confidence-stamped and timestamped centers enable forensic timeline reconstruction. Object-type awareness flags anomalous movement (person running where walking is expected, vehicle drifting outside lane boundaries).
- **Robotics / Pick-and-Place** — Predict object drift on conveyor belts or in manipulation tasks where sub-frame position matters. Fractional displacement mode captures sub-pixel drift, and enabling diagonals handles objects moving at angles to the belt direction.
- **Homing / Intercept Systems** — Drones pursuing a target, robotic arms reaching for moving objects, or any system where one object needs to arrive at another's future position. The homing extension provides continuously refining intercept zones. Combined with object-type awareness, physically impossible intercept projections are suppressed, tightening the aim point earlier in pursuit.

---

## Quick Start

### Minimal (5-Point Core)

```python
from pentatrack import PentaTracker, TrackingConfig

# Bare minimum — 5-point tracking, no extensions
config = TrackingConfig(
    mode="dual",              # "discrete", "proportional", or "dual"
    delta_discrete=1.0,       # Full displacement magnitude
    delta_proportional=0.10,  # 10% fractional displacement
    recursion_depth=2,        # How many levels of center expansion
)

tracker = PentaTracker(config)

# Feed a bounding box [x_min, y_min, x_max, y_max]
bbox = [120, 200, 280, 400]
prediction = tracker.update(bbox)

# Access the 5 base centers
for center in prediction.centers:
    print(f"{center.label}: ({center.x:.2f}, {center.y:.2f}, {center.z:.2f})")

# Access recursive child centers at depth 2
for center in prediction.centers:
    for child in center.children:
        print(f"  └─ {child.label}: ({child.x:.2f}, {child.y:.2f}, {child.z:.2f})")

# Get the highest-probability next position
best = prediction.most_likely()
print(f"Predicted next center: {best}")
```

### Full Stack (All Extensions)

```python
from pentatrack import PentaTracker, TrackingConfig

# Everything enabled
config = TrackingConfig(
    mode="dual",
    delta_discrete=1.0,
    delta_proportional=0.10,
    recursion_depth=2,
    # Geometry extensions
    enable_diagonals=True,             # 5pt → 9pt compass model
    enable_y_axis=False,               # Set True for 3D / depth tracking
    # Data extensions
    enable_metadata=True,              # Rich CenterNode data
    enable_velocity_weighting=True,    # Bias tree toward observed motion
    weight_strategy="cosine",
    velocity_method="ema",
    velocity_ema_alpha=0.3,
    # Drift extensions
    enable_drift_analysis=True,        # Multi-center drift vectors
    enable_adaptive_drift=True,        # Velocity-aware drift sensitivity
    drift_history_window=30,
    drift_normalize_by_velocity=True,
    # Object awareness
    enable_object_type=True,           # Physical constraint system
    object_type_category="vehicle",
    object_type_type="sedan",
    build_empirical_profiles=True,
    # Homing
    enable_homing=True,                # Intercept prediction
    reference_frame="world",
    homing_speed=10.0,
)

tracker = PentaTracker(config)

bbox = [120, 200, 280, 400]
prediction = tracker.update(bbox)

# Access the 9 base centers with full metadata
for center in prediction.centers:
    print(f"{center.label}: pos={center.position}, "
          f"vel={center.velocity}, conf={center.confidence:.3f}, "
          f"t={center.timestamp:.4f}")

# Access recursive child centers at depth 2
for center in prediction.centers:
    for child in center.children:
        print(f"  └─ {child.label}: pos={child.position}, "
              f"conf={child.confidence:.3f}, parent={child.parent.label}")

# Get the highest-probability next position
best = prediction.most_likely()
print(f"Predicted next center: {best.position} (conf: {best.confidence:.3f})")

# Trace lineage from any node back to root
leaf = prediction.deepest_most_likely()
path = leaf.trace_lineage()  # Returns [root, ..., parent, leaf]
print(f"Predicted trajectory: {[n.label for n in path]}")

# Get velocity-weighted probability distribution
weights = prediction.weight_distribution()
for label, weight in sorted(weights.items(), key=lambda x: -x[1]):
    print(f"  {label}: {weight:.2%}")

# Access drift analysis
for drift in prediction.drift_vectors:
    print(f"Drift from {drift.center_label}: mag={drift.drift_magnitude:.4f}")
leader = prediction.drift_leader()
print(f"Drift leader: {leader.center_label}")

# Check for anomalies (object-type aware)
for anomaly in prediction.anomalies:
    print(f"ANOMALY: {anomaly.event_type} — {anomaly.description} "
          f"(conf: {anomaly.confidence:.2%})")

# Homing intercept (if homing enabled)
intercept = prediction.intercept
print(f"Aim: {intercept.weighted_position}, "
      f"ETA: {intercept.time_to_arrival:.2f}s, "
      f"Zone: {intercept.zone_radius:.3f}")
```

---

## Configuration Reference

### Core Parameters

| Parameter             | Type    | Default  | Description                                              |
|-----------------------|---------|----------|----------------------------------------------------------|
| `mode`                | str     | `"dual"` | Displacement mode: `discrete`, `proportional`, or `dual` |
| `delta_discrete`      | float   | `1.0`    | Magnitude for full-shift displacement                    |
| `delta_proportional`  | float   | `0.10`   | Fraction of bbox dimension for micro-shift               |
| `recursion_depth`     | int     | `1`      | Levels of recursive center expansion                     |
| `prune_threshold`     | float   | `0.05`   | Drop branches below this probability weight              |
| `confidence_decay`    | float   | `0.85`   | Per-depth confidence multiplier for child centers        |

### Extension Toggles

| Parameter                   | Type    | Default  | Description                                           |
|-----------------------------|---------|----------|-------------------------------------------------------|
| `enable_diagonals`          | bool    | `False`  | Add 4 diagonal centers (5pt → 9pt)                    |
| `enable_y_axis`             | bool    | `False`  | Add forward/backward depth axis                       |
| `enable_metadata`           | bool    | `False`  | Attach rich data to each center node                  |
| `enable_velocity_weighting` | bool    | `False`  | Bias prediction tree by observed velocity             |
| `enable_drift_analysis`     | bool    | `False`  | Multi-center drift vector measurement                 |
| `enable_adaptive_drift`     | bool    | `False`  | Velocity-aware drift sensitivity                      |
| `enable_object_type`        | bool    | `False`  | Physical constraints by object classification         |
| `enable_homing`             | bool    | `False`  | Intercept prediction for pursuit scenarios            |

### Velocity Weighting Parameters (when enabled)

| Parameter              | Type    | Default    | Description                                               |
|------------------------|---------|------------|-----------------------------------------------------------|
| `weight_strategy`      | str     | `"cosine"` | Weight distribution: `cosine`, `softmax`, `decay`, `hybrid`|
| `softmax_temperature`  | float   | `1.0`      | Sharpness for softmax strategy (lower = sharper)          |
| `min_weight`           | float   | `0.01`     | Floor weight — no center drops to true zero               |
| `velocity_method`      | str     | `"ema"`    | Velocity estimation: `last_delta`, `wma`, `ema`, `lsq`   |
| `velocity_ema_alpha`   | float   | `0.3`      | Smoothing factor for EMA velocity estimation              |
| `history_window`       | int     | `30`       | Number of past frames used for velocity estimation        |

### Drift Analysis Parameters (when enabled)

| Parameter                      | Type    | Default          | Description                                               |
|--------------------------------|---------|------------------|-----------------------------------------------------------|
| `drift_history_window`         | int     | `30`             | Frames of drift history to retain per center              |
| `drift_speed_thresholds`       | list    | `[0.25, 1.0, 5.0]` | δ/frame boundaries for speed regime classification     |
| `drift_interpolation_steps`    | int     | `4`              | Sub-frame interpolation points for fast objects           |
| `drift_normalize_by_velocity`  | bool    | `True`           | Normalize drift magnitudes by current speed               |
| `drift_acceleration_tracking`  | bool    | `True`           | Track d(drift)/dt for acceleration detection              |

### Object-Type Parameters (when enabled)

| Parameter                | Type    | Default  | Description                                                  |
|--------------------------|---------|----------|--------------------------------------------------------------|
| `object_type_category`   | str     | `None`   | Top-level class: `vehicle`, `human`, `animal`, `aircraft`, `watercraft` |
| `object_type_type`       | str     | `None`   | Specific type: `sedan`, `motorcycle`, `pedestrian`, `helicopter`, etc. |
| `object_type_model`      | str     | `None`   | Exact model (optional): `Toyota Camry 2024`, `DJI Mavic 3`  |
| `anomaly_sensitivity`    | float   | `0.95`   | Percentile threshold for anomaly flags                       |
| `build_empirical_profiles` | bool  | `False`  | Learn drift profiles from observed tracking data             |
| `profile_database_path`  | str     | `None`   | File path for persistent empirical profile storage           |

### Homing Intercept Parameters (when enabled)

| Parameter                    | Type    | Default    | Description                                            |
|------------------------------|---------|------------|--------------------------------------------------------|
| `reference_frame`            | str     | `"world"`  | Drift reference: `world` (absolute) or `homing` (relative) |
| `homing_speed`               | float   | `None`     | Homing object speed (units/second)                     |
| `intercept_convergence_log`  | bool    | `False`    | Log intercept zone convergence over time               |

---

## How It Differs from Traditional Tracking

| Aspect                 | Traditional (Kalman/SORT)          | PentaTrack                                       |
|------------------------|------------------------------------|--------------------------------------------------|
| Centers per object     | 1 (centroid)                       | 5 base, up to 27+ with extensions                |
| Prediction model       | Linear/constant velocity           | Spatial possibility tree (recursive)             |
| Directional coverage   | Single vector                      | Cardinal + optional diagonal + optional depth    |
| Movement granularity   | Single predicted position          | Full + fractional + directional                  |
| Lookahead              | 1 step (typically)                 | N steps via recursion depth                      |
| Node metadata          | Position only                      | Optional: velocity, confidence, timestamp, lineage |
| Probability model      | Gaussian uncertainty               | Optional: velocity-weighted directional distribution |
| Drift analysis         | Not applicable                     | Optional: multi-center drift vectors per frame   |
| Speed adaptation       | Fixed model                        | Optional: velocity-adaptive drift sensitivity    |
| Object awareness       | None                               | Optional: physics-constrained by object type     |
| Anomaly detection      | Outlier rejection                  | Optional: drift boundary violations = real events |
| Intercept prediction   | External computation               | Optional: built-in homing with convergent zones  |
| Trajectory analysis    | Smoothed path                      | Optional: full parent-chain lineage reconstruction |
| Output                 | Next estimated position            | Weighted probability field of positions          |

PentaTrack doesn't replace Kalman filters or SORT — it adds a **spatial prediction layer** on top of any detection pipeline. Feed it bounding boxes from YOLO, Detectron, or any detector and it handles the rest. With velocity weighting enabled, the weighted centers can also be fed *into* a Kalman filter as informed priors, combining the strengths of both approaches.

---

## Roadmap

### Core (Complete)
- [x] Core 5-point center engine (X/Z cardinal axes)
- [x] Discrete and proportional displacement modes
- [x] Dual-mode simultaneous tracking
- [x] Recursive center expansion with depth control

### Extensions (Complete)
- [x] Diagonal center expansion (optional 9-point compass model)
- [x] Center node data model (position, velocity, confidence, timestamp, parent)
- [x] Velocity-weighted prediction (cosine, softmax, decay, hybrid)
- [x] Velocity estimation methods (EMA, WMA, LSQ, last-delta)
- [x] Parent-chain lineage traversal
- [x] Confidence propagation through recursion
- [x] Inter-center drift analysis (multi-point drift vectors per frame)
- [x] Drift accuracy tracking and drift leader identification
- [x] Velocity-adaptive drift (speed regime detection, normalization, interpolation)
- [x] Drift acceleration tracking (d(drift)/dt)
- [x] Object-type awareness (category/type/model classification hierarchy)
- [x] Physics-constrained prediction pruning via drift profiles
- [x] Anomaly detection via drift boundary violations
- [x] Historical empirical drift profile learning
- [x] Homing intercept prediction (multi-projection convergent zones)
- [x] World and homing reference frame support

### Planned
- [ ] Y-axis (depth/forward-backward) integration → 27-point 3D model
- [ ] Adaptive pruning (auto-tune threshold from motion characteristics)
- [ ] YOLO v8/v11 detection bridge
- [ ] Drone autopilot integration (MAVLink)
- [ ] Real-time visualization overlay with weight heatmap and drift vectors
- [ ] Multi-object tracking with shared prediction fields
- [ ] Cross-object interaction prediction (collision, convergence)
- [ ] GPU-accelerated prediction tree expansion (CUDA kernels)
- [ ] Temporal prediction (time-to-arrival at boundary centers)
- [ ] Multi-homing (multiple pursuers coordinating on one target)
- [ ] Evasion prediction (target aware of homing, anticipate countermeasures)
- [ ] Object-type auto-classification from drift signature matching

---

## Project Structure

```
pentatrack/
├── pentatrack/
│   ├── __init__.py
│   ├── core.py                # 5-point center computation engine
│   ├── config.py              # TrackingConfig dataclass
│   ├── prediction_tree.py     # Recursive center expansion
│   ├── pruning.py             # Branch pruning strategies
│   ├── tracker.py             # PentaTracker main interface
│   ├── extensions/
│   │   ├── __init__.py
│   │   ├── diagonals.py       # 4 diagonal centers (5pt → 9pt)
│   │   ├── metadata.py        # CenterNode rich data model
│   │   ├── velocity.py        # Velocity estimation (EMA, WMA, LSQ)
│   │   ├── weighting.py       # Weight distribution strategies
│   │   ├── drift.py           # Inter-center drift analysis engine
│   │   ├── adaptive_drift.py  # Velocity-adaptive drift sensitivity
│   │   ├── object_type.py     # Object classification and drift profiles
│   │   └── homing.py          # Homing intercept prediction
│   └── utils/
│       ├── bbox.py            # Bounding box utilities
│       ├── coords.py          # Coordinate system helpers
│       ├── lineage.py         # Parent-chain traversal tools
│       ├── profiles.py        # Drift profile database management
│       └── viz.py             # Visualization, heatmap, and drift overlay tools
├── examples/
│   ├── basic_tracking.py      # Minimal 5-point usage
│   ├── with_diagonals.py      # 9-point compass model
│   ├── velocity_weighting.py  # Full weighted prediction
│   ├── lineage_trace.py       # Trajectory reconstruction
│   ├── drift_analysis.py      # Multi-center drift measurement
│   ├── adaptive_drift.py      # Speed-aware drift demo
│   ├── object_types.py        # Object classification and anomaly detection
│   ├── homing_intercept.py    # Pursuit and intercept demo
│   ├── drone_pursuit.py       # Drone integration example
│   └── webcam_demo.py         # Live camera demo
├── profiles/
│   └── defaults/
│       ├── vehicle.json       # Default drift profiles for vehicle types
│       ├── human.json         # Default drift profiles for human movement
│       ├── aircraft.json      # Default drift profiles for aircraft types
│       ├── watercraft.json    # Default drift profiles for watercraft types
│       └── animal.json        # Default drift profiles for animal types
├── tests/
├── README.md
├── LICENSE
└── pyproject.toml
```

---

## Requirements

- Python 3.10+
- NumPy
- OpenCV (optional — for visualization and camera input)
- Ultralytics (optional — for YOLO detection bridge)

---

## License

MIT — use it, fork it, build on it.

---

## Contributing

This project is in active early development. If you're working in object tracking, drone systems, or computer vision and this concept resonates, open an issue or reach out. The prediction model is intentionally framework-agnostic — contributions toward integrations with specific detection pipelines, autopilot systems, or hardware platforms are welcome.

---

> *"Every movement has a center. Every center predicts the next. Velocity decides which one matters. Drift reveals what the grid can't see."*
