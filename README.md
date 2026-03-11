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

Each extension is opt-in. You can run bare 5-point tracking with zero overhead, or stack all three for a fully weighted, diagonal-aware, metadata-rich prediction engine. The architecture doesn't care — everything composes cleanly.

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

## Extension Combinations

The extensions compose freely. Here's how center counts and capabilities scale:

| Configuration                                  | Centers (depth 1) | Per-Center Data                | Weighting     |
|------------------------------------------------|--------------------|--------------------------------|---------------|
| Base 5-point                                   | 5                  | `(x, y, z)` only              | Uniform       |
| Base + metadata                                | 5                  | Full CenterNode                | Uniform       |
| Base + diagonals                               | 9                  | `(x, y, z)` only              | Uniform       |
| Base + diagonals + metadata                    | 9                  | Full CenterNode                | Uniform       |
| Base + metadata + velocity                     | 5                  | Full CenterNode                | Velocity-biased |
| Base + diagonals + metadata + velocity         | 9                  | Full CenterNode                | Velocity-biased |
| All + 3D Y-axis                                | 27                 | Full CenterNode                | Velocity-biased |
| All + 3D + dual mode                           | 53                 | Full CenterNode                | Velocity-biased |

The design principle: **start minimal, enable what you need.** A drone tracking a single target in 2D might only need base 5-point. A multi-object autonomous vehicle system might run the full stack. Same engine, different configuration.

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
├──────────────┤────────────────┤───────────────────────────────┤
│  Velocity    │                │                               │
│  Estimator   │                │                               │
│  ──────────  │                │                               │
│  EMA/WMA/LSQ │                │                               │
│  History buf │                │                               │
│  Alpha tuning│                │                               │
├──────────────┴────────────────┴───────────────────────────────┤
│                        Tracker State                          │
│  Per-object center history, velocity vectors, probability     │
│  weights, trajectory buffer, parent-child g?
