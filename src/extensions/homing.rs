use crate::core::center_engine::{CenterLabel, CenterNode};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterceptProjection {
    pub from_center: CenterLabel,
    pub position: Vector3<f32>,
    pub confidence: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterceptPrediction {
    /// Confidence-weighted average intercept position.
    pub weighted_position: Vector3<f32>,
    /// Time to arrival in seconds.
    pub time_to_arrival_s: f32,
    /// Zone radius: spread of intercept projections. Shrinks as distance closes.
    pub zone_radius_m: f32,
    /// Overall confidence in [0, 1].
    pub confidence: f32,
    pub projections: Vec<InterceptProjection>,
}

/// Compute homing intercept prediction from a set of predicted centers.
///
/// `centers`: the depth-1 centers with their current positions.
/// `homing_position`: position of the homing object (chaser).
/// `homing_speed_ms`: speed of the homing object (m/s).
/// `target_velocity`: estimated target velocity (for projection).
pub fn compute_intercept(
    centers: &[CenterNode],
    homing_position: &Vector3<f32>,
    homing_speed_ms: f32,
    target_velocity: &Vector3<f32>,
) -> InterceptPrediction {
    let target_center = centers
        .iter()
        .find(|c| c.depth == 0)
        .map(|c| c.position)
        .unwrap_or_else(|| {
            let conf_sum: f32 = centers.iter().map(|c| c.confidence).sum();
            if conf_sum > 0.0 {
                centers
                    .iter()
                    .fold(Vector3::zeros(), |a, c| a + c.position * c.confidence)
                    / conf_sum
            } else {
                Vector3::zeros()
            }
        });

    let distance_m = (target_center - homing_position).norm();
    let time_to_arrival_s = if homing_speed_ms > 0.0 {
        distance_m / homing_speed_ms
    } else {
        f32::MAX
    };

    let depth1_centers: Vec<&CenterNode> = centers.iter().filter(|c| c.depth == 1).collect();

    let projections: Vec<InterceptProjection> = depth1_centers
        .iter()
        .map(|c| {
            // Project target center forward by time_to_arrival
            let projected = c.position + target_velocity * time_to_arrival_s;
            InterceptProjection {
                from_center: c.label.clone(),
                position: projected,
                confidence: c.confidence,
            }
        })
        .collect();

    // Weighted average of projections
    let total_conf: f32 = projections.iter().map(|p| p.confidence).sum();
    let weighted_pos = if total_conf > 0.0 {
        projections
            .iter()
            .fold(Vector3::zeros(), |a, p| a + p.position * p.confidence)
            / total_conf
    } else {
        target_center
    };

    // Zone radius: RMS distance from weighted position
    let zone_radius = if !projections.is_empty() {
        let sum_sq: f32 = projections
            .iter()
            .map(|p| (p.position - weighted_pos).norm_squared() * p.confidence)
            .sum();
        (sum_sq / total_conf.max(1e-6)).sqrt()
    } else {
        0.0
    };

    InterceptPrediction {
        weighted_position: weighted_pos,
        time_to_arrival_s,
        zone_radius_m: zone_radius,
        confidence: total_conf / depth1_centers.len().max(1) as f32,
        projections,
    }
}
