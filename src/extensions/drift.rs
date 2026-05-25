use crate::core::center_engine::CenterLabel;
use nalgebra::Vector3;
use omni_sense_core::Timestamp;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriftRecord {
    pub center_label: CenterLabel,
    pub drift_vector: Vector3<f32>,
    pub drift_magnitude: f32,
    pub timestamp: Timestamp,
}

/// Tracks drift history per center label.
pub struct DriftAnalyzer {
    pub history_window: usize,
    records: Vec<DriftRecord>,
}

impl DriftAnalyzer {
    pub fn new(history_window: usize) -> Self {
        Self {
            history_window,
            records: Vec::new(),
        }
    }

    /// Record drift from each predicted center to the actual new position.
    pub fn record_drift(
        &mut self,
        predicted_centers: &[(CenterLabel, Vector3<f32>)],
        actual_position: Vector3<f32>,
        timestamp: Timestamp,
    ) {
        for (label, center_pos) in predicted_centers {
            let drift = actual_position - center_pos;
            let magnitude = drift.norm();
            self.records.push(DriftRecord {
                center_label: label.clone(),
                drift_vector: drift,
                drift_magnitude: magnitude,
                timestamp,
            });
        }
        // Trim to window
        let max_records = self.history_window * predicted_centers.len();
        while self.records.len() > max_records {
            self.records.remove(0);
        }
    }

    /// Find the drift leader: center with lowest average drift magnitude.
    pub fn drift_leader(&self) -> Option<CenterLabel> {
        let mut sums: std::collections::HashMap<String, (f32, u32)> =
            std::collections::HashMap::new();
        for rec in &self.records {
            let entry = sums
                .entry(rec.center_label.as_str().to_string())
                .or_insert((0.0, 0));
            entry.0 += rec.drift_magnitude;
            entry.1 += 1;
        }
        sums.iter()
            .filter(|(_, (_, count))| *count > 0)
            .min_by(|(_, (sum_a, n_a)), (_, (sum_b, n_b))| {
                (sum_a / *n_a as f32)
                    .partial_cmp(&(sum_b / *n_b as f32))
                    .unwrap()
            })
            .and_then(|(label_str, _)| {
                // Map back to CenterLabel
                match label_str.as_str() {
                    "C0" => Some(CenterLabel::Neutral),
                    "C+x" => Some(CenterLabel::Right),
                    "C-x" => Some(CenterLabel::Left),
                    "C+z" => Some(CenterLabel::Up),
                    "C-z" => Some(CenterLabel::Down),
                    _ => None,
                }
            })
    }
}
