use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackingConfig {
    // Core
    pub mode: TrackingMode,
    pub delta_discrete: f32,
    pub delta_proportional: f32,
    pub recursion_depth: u32,
    pub prune_threshold: f32,
    pub confidence_decay: f32,

    // Extensions
    pub enable_diagonals: bool,
    pub enable_y_axis: bool,
    pub enable_metadata: bool,
    pub enable_velocity_weighting: bool,
    pub weight_strategy: WeightStrategy,
    pub softmax_temperature: f32,
    pub min_weight: f32,
    pub velocity_method: VelocityMethod,
    pub history_window: usize,

    pub enable_drift_analysis: bool,
    pub enable_adaptive_drift: bool,
    pub drift_history_window: usize,
    pub drift_normalize_by_velocity: bool,
    pub drift_speed_thresholds: [f32; 3],
    pub drift_interpolation_steps: u32,
    pub drift_acceleration_tracking: bool,

    pub enable_object_type: bool,
    pub object_type_category: String,
    pub object_type_type: String,
    pub object_type_model: Option<String>,
    pub anomaly_sensitivity: f32,
    pub build_empirical_profiles: bool,
    pub profile_database_path: Option<String>,

    pub enable_homing: bool,
    pub reference_frame: ReferenceFrame,
    pub homing_speed: f32,
    pub intercept_convergence_log: bool,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            mode: TrackingMode::Dual,
            delta_discrete: 1.0,
            delta_proportional: 0.10,
            recursion_depth: 1,
            prune_threshold: 0.05,
            confidence_decay: 0.85,
            enable_diagonals: false,
            enable_y_axis: false,
            enable_metadata: false,
            enable_velocity_weighting: false,
            weight_strategy: WeightStrategy::Cosine,
            softmax_temperature: 1.0,
            min_weight: 0.01,
            velocity_method: VelocityMethod::Ema { alpha: 0.3 },
            history_window: 30,
            enable_drift_analysis: false,
            enable_adaptive_drift: false,
            drift_history_window: 30,
            drift_normalize_by_velocity: true,
            drift_speed_thresholds: [0.25, 1.0, 5.0],
            drift_interpolation_steps: 4,
            drift_acceleration_tracking: true,
            enable_object_type: false,
            object_type_category: String::new(),
            object_type_type: String::new(),
            object_type_model: None,
            anomaly_sensitivity: 0.95,
            build_empirical_profiles: false,
            profile_database_path: None,
            enable_homing: false,
            reference_frame: ReferenceFrame::World,
            homing_speed: 10.0,
            intercept_convergence_log: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TrackingMode {
    Discrete,
    Proportional,
    Dual,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum WeightStrategy {
    Cosine,
    Softmax,
    Decay,
    Hybrid,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum VelocityMethod {
    LastDelta,
    Wma,
    Ema { alpha: f32 },
    Lsq,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ReferenceFrame {
    World,
    Homing,
}
