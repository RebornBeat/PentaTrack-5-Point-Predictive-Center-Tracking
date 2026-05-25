use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriftProfile {
    pub max_lateral_drift_ms: f32,
    pub max_vertical_drift_ms: f32,
    pub max_longitudinal_drift_ms: f32,
    pub max_turn_rate_deg_s: f32,
    pub can_hover: bool,
    pub can_reverse: bool,
    pub min_speed_ms: f32,
    pub max_speed_ms: f32,
    pub anomaly_thresholds: AnomalyThresholds,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnomalyThresholds {
    pub lateral_ms: f32,
    pub approach_accel_ms2: f32,
    pub closing_velocity_ms: f32,
}

/// Well-known object taxonomies. Projects substitute their own.
pub struct ObjectTypeTaxonomy;

impl ObjectTypeTaxonomy {
    pub fn human_walking() -> DriftProfile {
        DriftProfile {
            max_lateral_drift_ms: 3.0,
            max_vertical_drift_ms: 0.3,
            max_longitudinal_drift_ms: 5.0,
            max_turn_rate_deg_s: 180.0,
            can_hover: true,
            can_reverse: true,
            min_speed_ms: 0.0,
            max_speed_ms: 8.0,
            anomaly_thresholds: AnomalyThresholds {
                lateral_ms: 5.0,
                approach_accel_ms2: 3.0,
                closing_velocity_ms: 10.0,
            },
        }
    }

    pub fn vehicle_sedan() -> DriftProfile {
        DriftProfile {
            max_lateral_drift_ms: 10.0,
            max_vertical_drift_ms: 0.5,
            max_longitudinal_drift_ms: 40.0,
            max_turn_rate_deg_s: 60.0,
            can_hover: false,
            can_reverse: true,
            min_speed_ms: 0.0,
            max_speed_ms: 55.0,
            anomaly_thresholds: AnomalyThresholds {
                lateral_ms: 15.0,
                approach_accel_ms2: 8.0,
                closing_velocity_ms: 25.0,
            },
        }
    }

    pub fn consumer_multirotors() -> DriftProfile {
        DriftProfile {
            max_lateral_drift_ms: 20.0,
            max_vertical_drift_ms: 10.0,
            max_longitudinal_drift_ms: 30.0,
            max_turn_rate_deg_s: 360.0,
            can_hover: true,
            can_reverse: true,
            min_speed_ms: 0.0,
            max_speed_ms: 35.0,
            anomaly_thresholds: AnomalyThresholds {
                lateral_ms: 25.0,
                approach_accel_ms2: 10.0,
                closing_velocity_ms: 30.0,
            },
        }
    }

    pub fn get(category: &str, type_name: &str) -> DriftProfile {
        match (category, type_name) {
            ("human", _) => Self::human_walking(),
            ("vehicle", "sedan") | ("vehicle", _) => Self::vehicle_sedan(),
            ("aircraft", "multirotors") | ("uav", _) => Self::consumer_multirotors(),
            _ => Self::human_walking(), // Fallback
        }
    }
}

/// Check if observed drift violates the profile and return anomaly flags.
pub fn check_anomalies(
    profile: &DriftProfile,
    lateral_ms: f32,
    longitudinal_ms: f32,
    accel_ms2: f32,
) -> Vec<String> {
    let mut flags = Vec::new();
    if lateral_ms > profile.anomaly_thresholds.lateral_ms {
        flags.push(format!("LateralDriftAnomaly:{:.2}ms", lateral_ms));
    }
    if accel_ms2 > profile.anomaly_thresholds.approach_accel_ms2 {
        flags.push(format!("ApproachAccelerationAnomaly:{:.2}ms2", accel_ms2));
    }
    flags
}
