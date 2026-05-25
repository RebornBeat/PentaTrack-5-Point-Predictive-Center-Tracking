use crate::config::TrackingConfig;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Label for a predicted center.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CenterLabel {
    Neutral,
    Right,
    Left,
    Up,
    Down,
    Forward,
    Backward,
    UpRight,
    UpLeft,
    DownRight,
    DownLeft,
    ForwardRight,
    ForwardLeft,
    BackwardRight,
    BackwardLeft,
    ForwardUp,
    ForwardDown,
    BackwardUp,
    BackwardDown,
}

impl CenterLabel {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Neutral => "C0",
            Self::Right => "C+x",
            Self::Left => "C-x",
            Self::Up => "C+z",
            Self::Down => "C-z",
            Self::Forward => "C+y",
            Self::Backward => "C-y",
            Self::UpRight => "C+x+z",
            Self::UpLeft => "C-x+z",
            Self::DownRight => "C+x-z",
            Self::DownLeft => "C-x-z",
            Self::ForwardRight => "C+x+y",
            Self::ForwardLeft => "C-x+y",
            Self::BackwardRight => "C+x-y",
            Self::BackwardLeft => "C-x-y",
            Self::ForwardUp => "C+y+z",
            Self::ForwardDown => "C+y-z",
            Self::BackwardUp => "C-y+z",
            Self::BackwardDown => "C-y-z",
        }
    }

    /// Returns the unit offset vector for this label.
    pub fn unit_offset(&self) -> Vector3<f32> {
        match self {
            Self::Neutral => Vector3::new(0.0, 0.0, 0.0),
            Self::Right => Vector3::new(1.0, 0.0, 0.0),
            Self::Left => Vector3::new(-1.0, 0.0, 0.0),
            Self::Up => Vector3::new(0.0, 0.0, 1.0),
            Self::Down => Vector3::new(0.0, 0.0, -1.0),
            Self::Forward => Vector3::new(0.0, 1.0, 0.0),
            Self::Backward => Vector3::new(0.0, -1.0, 0.0),
            Self::UpRight => Vector3::new(1.0, 0.0, 1.0).normalize(),
            Self::UpLeft => Vector3::new(-1.0, 0.0, 1.0).normalize(),
            Self::DownRight => Vector3::new(1.0, 0.0, -1.0).normalize(),
            Self::DownLeft => Vector3::new(-1.0, 0.0, -1.0).normalize(),
            _ => Vector3::zeros(),
        }
    }
}

/// A single predicted center — the core unit of PentaTrack's output.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CenterNode {
    pub label: CenterLabel,
    pub position: Vector3<f32>,
    pub confidence: f32,
    pub depth: u32,
    /// Velocity at this center (from velocity weighting extension).
    pub velocity: Option<Vector3<f32>>,
    /// Parent center index in the flattened prediction tree (None = root).
    pub parent_idx: Option<usize>,
    /// Child center indices in the flattened tree.
    pub children: Vec<usize>,
}

/// Complete prediction for one tracked object at one time step.
#[derive(Debug, Clone)]
pub struct Prediction {
    /// Flattened center tree. Index 0 is always the neutral (root) center.
    pub centers: Vec<CenterNode>,
    pub drift_leader_label: Option<CenterLabel>,
    pub anomaly_flags: Vec<String>,
}

impl Prediction {
    pub fn most_likely(&self) -> &CenterNode {
        self.centers
            .iter()
            .max_by(|a, b| a.confidence.partial_cmp(&b.confidence).unwrap())
            .expect("Prediction must have at least one center")
    }

    pub fn weight_distribution(&self) -> Vec<(&CenterLabel, f32)> {
        self.centers
            .iter()
            .filter(|c| c.depth == 1)
            .map(|c| (&c.label, c.confidence))
            .collect()
    }
}

/// Compute the base center positions from a bounding box center.
pub fn compute_base_centers(
    bb_center: Vector3<f32>,
    config: &TrackingConfig,
) -> Vec<(CenterLabel, Vector3<f32>)> {
    let mut centers = vec![(CenterLabel::Neutral, bb_center)];

    let cardinal = [
        CenterLabel::Right,
        CenterLabel::Left,
        CenterLabel::Up,
        CenterLabel::Down,
    ];

    for label in cardinal {
        let offset = label.unit_offset();
        if matches!(
            config.mode,
            crate::config::TrackingMode::Discrete | crate::config::TrackingMode::Dual
        ) {
            centers.push((label.clone(), bb_center + offset * config.delta_discrete));
        }
        if matches!(
            config.mode,
            crate::config::TrackingMode::Proportional | crate::config::TrackingMode::Dual
        ) {
            // Proportional: offset is fraction of delta (proportional scale is separate)
            if !matches!(config.mode, crate::config::TrackingMode::Dual) {
                centers.push((label, bb_center + offset * config.delta_proportional));
            }
        }
    }

    if config.enable_diagonals {
        let diagonals = [
            CenterLabel::UpRight,
            CenterLabel::UpLeft,
            CenterLabel::DownRight,
            CenterLabel::DownLeft,
        ];
        for label in diagonals {
            let offset = label.unit_offset();
            centers.push((label, bb_center + offset * config.delta_discrete));
        }
    }

    if config.enable_y_axis {
        centers.push((
            CenterLabel::Forward,
            bb_center + Vector3::new(0.0, config.delta_discrete, 0.0),
        ));
        centers.push((
            CenterLabel::Backward,
            bb_center - Vector3::new(0.0, config.delta_discrete, 0.0),
        ));
    }

    centers
}
