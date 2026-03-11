# PentaTrack — 5-Point Predictive Center Tracking

**Recursive multi-center bounding box prediction for real-time object tracking and movement forecasting.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](#license)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-green.svg)](#requirements)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange.svg)](#roadmap)

---

## Overview

PentaTrack is a movement prediction framework built on a simple but powerful idea: at any given moment, an object's next position can be described by **five predictive centers** — the current neutral center plus one center for each cardinal movement direction. Each of those predicted centers then becomes its own origin for another layer of five predictions, creating a **recursive prediction tree** that can anticipate complex trajectories before they happen.

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

| Center | Direction     | Offset      |
|--------|---------------|-------------|
| C₀     | Neutral       | `(0, 0, 0)` |
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

> **Note:** The Y-axis (forward/backward, i.e. depth) is supported at the architecture level and can be enabled for 3D tracking scenarios (e.g. drone altitude + distance), expanding the model to **7 centers** when active.

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

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  PentaTrack Core                │
├─────────────┬───────────────┬───────────────────┤
│  Detector   │  Center Engine │  Prediction Tree  │
│  ─────────  │  ────────────  │  ───────────────  │
│  BBox input │  Neutral calc  │  Recursive expan. │
│  YOLO/custom│  δ projection  │  Branch pruning   │
│  Frame sync │  Mode select   │  Depth control    │
├─────────────┴───────────────┴───────────────────┤
│                 Tracker State                    │
│  Per-object center history, velocity vectors,   │
│  probability weights, trajectory buffer         │
├─────────────────────────────────────────────────┤
│              Output / Integration               │
│  Raw center streams • Predicted trajectory •    │
│  Confidence map • Drone autopilot interface •   │
│  Visualization overlay                          │
└─────────────────────────────────────────────────┘
```

---

## Use Cases

- **Drone Object Tracking** — Predict where a target is heading before the drone's gimbal or flight path adjusts. Feed PentaTrack centers into a PID controller for smoother pursuit.
- **Autonomous Vehicle Perception** — Anticipate pedestrian or vehicle movement from bounding box streams. The recursive depth allows multi-step lookahead.
- **Sports Analytics** — Track player or ball movement with directional prediction for play analysis and broadcast augmentation.
- **Surveillance & Security** — Predict subject trajectory across camera feeds. Predictive centers can trigger alerts before a subject reaches a boundary.
- **Robotics / Pick-and-Place** — Predict object drift on conveyor belts or in manipulation tasks where sub-frame position matters.

---

## Quick Start

```python
from pentatrack import PentaTracker, TrackingConfig

# Configure the tracker
config = TrackingConfig(
    mode="dual",              # "discrete", "proportional", or "dual"
    delta_discrete=1.0,       # Full displacement magnitude
    delta_proportional=0.10,  # 10% fractional displacement
    recursion_depth=2,        # How many levels of center expansion
    enable_y_axis=False,      # Set True for 3D / depth tracking
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

---

## Configuration Reference

| Parameter             | Type    | Default  | Description                                              |
|-----------------------|---------|----------|----------------------------------------------------------|
| `mode`                | str     | `"dual"` | Displacement mode: `discrete`, `proportional`, or `dual` |
| `delta_discrete`      | float   | `1.0`    | Magnitude for full-shift displacement                    |
| `delta_proportional`  | float   | `0.10`   | Fraction of bbox dimension for micro-shift               |
| `recursion_depth`     | int     | `1`      | Levels of recursive center expansion                     |
| `enable_y_axis`       | bool    | `False`  | Enable forward/backward (depth) predictions              |
| `prune_threshold`     | float   | `0.05`   | Drop branches below this probability weight              |
| `history_window`      | int     | `30`     | Number of past frames used for velocity estimation       |
| `confidence_decay`    | float   | `0.85`   | Per-depth confidence multiplier for child centers         |

---

## How It Differs from Traditional Tracking

| Aspect                | Traditional (Kalman/SORT)         | PentaTrack                              |
|------------------------|-----------------------------------|-----------------------------------------|
| Centers per object     | 1 (centroid)                      | 5+ (pentagonal prediction field)        |
| Prediction model       | Linear/constant velocity          | Spatial possibility tree (recursive)    |
| Movement granularity   | Single predicted position         | Full + fractional + directional         |
| Lookahead              | 1 step (typically)                | N steps via recursion depth             |
| Output                 | Next estimated position           | Weighted probability field of positions |

PentaTrack doesn't replace Kalman filters or SORT — it adds a **spatial prediction layer** on top of any detection pipeline. Feed it bounding boxes from YOLO, Detectron, or any detector and it handles the rest.

---

## Roadmap

- [x] Core 5-point center engine (X/Z axes)
- [x] Discrete and proportional displacement modes
- [x] Dual-mode simultaneous tracking
- [x] Recursive center expansion with depth control
- [ ] Y-axis (depth/forward-backward) integration
- [ ] Probability weighting from velocity history
- [ ] Branch pruning optimization
- [ ] YOLO v8/v11 detection bridge
- [ ] Drone autopilot integration (MAVLink)
- [ ] Real-time visualization overlay
- [ ] Multi-object tracking with shared prediction fields
- [ ] GPU-accelerated prediction tree expansion

---

## Project Structure

```
pentatrack/
├── pentatrack/
│   ├── __init__.py
│   ├── core.py              # Center computation engine
│   ├── config.py            # TrackingConfig dataclass
│   ├── prediction_tree.py   # Recursive center expansion
│   ├── pruning.py           # Branch pruning strategies
│   ├── tracker.py           # PentaTracker main interface
│   └── utils/
│       ├── bbox.py          # Bounding box utilities
│       ├── coords.py        # Coordinate system helpers
│       └── viz.py           # Visualization tools
├── examples/
│   ├── basic_tracking.py
│   ├── drone_pursuit.py
│   └── webcam_demo.py
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

> *"Every movement has a center. Every center predicts the next."*
