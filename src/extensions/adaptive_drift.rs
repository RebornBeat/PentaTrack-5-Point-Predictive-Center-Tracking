/// Speed regime for adaptive drift sensitivity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedRegime {
    Slow,
    Medium,
    Fast,
    Extreme,
}

/// Classifies current speed into a regime using configurable thresholds.
pub fn classify_speed(speed_ms: f32, thresholds: &[f32; 3]) -> SpeedRegime {
    if speed_ms < thresholds[0] {
        SpeedRegime::Slow
    } else if speed_ms < thresholds[1] {
        SpeedRegime::Medium
    } else if speed_ms < thresholds[2] {
        SpeedRegime::Fast
    } else {
        SpeedRegime::Extreme
    }
}

/// Normalize drift magnitude by current speed for cross-velocity comparison.
pub fn normalize_drift(drift_magnitude: f32, speed_ms: f32) -> f32 {
    if speed_ms < 1e-6 {
        return drift_magnitude;
    }
    drift_magnitude / speed_ms
}
