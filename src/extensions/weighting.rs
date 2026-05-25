use crate::config::WeightStrategy;
use crate::core::center_engine::CenterLabel;
use nalgebra::Vector3;

/// Compute probability weights for each center label given current velocity.
pub fn compute_weights(
    velocity: &Vector3<f32>,
    labels: &[CenterLabel],
    strategy: WeightStrategy,
    softmax_temperature: f32,
    min_weight: f32,
) -> Vec<(CenterLabel, f32)> {
    let speed = velocity.norm();
    if speed < 1e-6 {
        // Uniform distribution when stationary
        let w = 1.0 / labels.len() as f32;
        return labels
            .iter()
            .map(|l| (l.clone(), w.max(min_weight)))
            .collect();
    }

    let vel_dir = velocity / speed;

    let scores: Vec<f32> = labels
        .iter()
        .map(|label| {
            let offset = label.unit_offset();
            if offset.norm() < 1e-6 {
                return 0.5;
            } // Neutral center
            let cos_sim = vel_dir.dot(&offset).max(0.0);
            match strategy {
                WeightStrategy::Cosine => cos_sim,
                WeightStrategy::Decay => (2.0 * cos_sim - 1.0).max(0.0).exp(),
                WeightStrategy::Softmax | WeightStrategy::Hybrid => {
                    (cos_sim / softmax_temperature).exp()
                }
            }
        })
        .collect();

    let sum: f32 = scores.iter().sum();
    let normalized: Vec<f32> = if sum > 0.0 {
        scores.iter().map(|s| (s / sum).max(min_weight)).collect()
    } else {
        vec![min_weight; labels.len()]
    };

    // Re-normalize after applying min_weight floor
    let sum2: f32 = normalized.iter().sum();
    labels
        .iter()
        .zip(normalized.iter())
        .map(|(l, &w)| (l.clone(), w / sum2))
        .collect()
}
