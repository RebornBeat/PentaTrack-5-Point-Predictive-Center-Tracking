use crate::config::TrackingConfig;
use crate::core::center_engine::{CenterNode, Prediction};
use crate::core::prediction_tree::build_prediction_tree;
use crate::core::pruning::{limit_nodes, prune_below_threshold};
use crate::extensions::drift::DriftAnalyzer;
use crate::extensions::homing::{compute_intercept, InterceptPrediction};
use crate::extensions::object_type::{DriftProfile, ObjectTypeTaxonomy};
use crate::extensions::velocity::VelocityEstimator;
use crate::extensions::weighting::compute_weights;
use nalgebra::Vector3;
use omni_sense_core::DetectionEvent;
use std::collections::HashMap;

/// Complete prediction for one tracked object.
pub struct PentaPrediction {
    pub track_id: String,
    pub prediction: Prediction,
    pub velocity_estimate: Vector3<f32>,
    pub intercept: Option<InterceptPrediction>,
}

impl PentaPrediction {
    pub fn most_likely(&self) -> &CenterNode {
        self.prediction.most_likely()
    }
}

/// Per-track state maintained across frames.
struct TrackState {
    velocity_estimator: VelocityEstimator,
    drift_analyzer: Option<DriftAnalyzer>,
    drift_profile: Option<DriftProfile>,
    last_position: Option<Vector3<f32>>,
}

/// Main PentaTrack interface.
pub struct PentaTracker {
    config: TrackingConfig,
    tracks: HashMap<String, TrackState>,
}

impl PentaTracker {
    pub fn new(config: TrackingConfig) -> Self {
        Self {
            config,
            tracks: HashMap::new(),
        }
    }

    /// Update with a DetectionEvent and return the updated prediction.
    pub fn update(&mut self, detection: DetectionEvent) -> PentaPrediction {
        let track_id = detection.source_sensor.0.clone();
        let position = detection.position;
        let timestamp_ns = detection.timestamp.monotonic_ns;

        let state = self
            .tracks
            .entry(track_id.clone())
            .or_insert_with(|| TrackState {
                velocity_estimator: VelocityEstimator::new(
                    self.config.velocity_method.clone(),
                    self.config.history_window,
                ),
                drift_analyzer: if self.config.enable_drift_analysis {
                    Some(DriftAnalyzer::new(self.config.drift_history_window))
                } else {
                    None
                },
                drift_profile: if self.config.enable_object_type {
                    Some(ObjectTypeTaxonomy::get(
                        &self.config.object_type_category,
                        &self.config.object_type_type,
                    ))
                } else {
                    None
                },
                last_position: None,
            });

        // Record position for velocity estimation
        state.velocity_estimator.record(position, timestamp_ns);
        let velocity = state.velocity_estimator.estimate();

        // Compute center weights if velocity weighting is enabled
        let base_labels = crate::core::center_engine::compute_base_centers(position, &self.config);
        let labels: Vec<crate::core::center_engine::CenterLabel> =
            base_labels.iter().map(|(l, _)| l.clone()).collect();

        let weights = if self.config.enable_velocity_weighting {
            Some(crate::extensions::weighting::compute_weights(
                &velocity,
                &labels,
                self.config.weight_strategy,
                self.config.softmax_temperature,
                self.config.min_weight,
            ))
        } else {
            None
        };

        // Build prediction tree
        let mut prediction = build_prediction_tree(position, &self.config, weights.as_ref());

        // Populate velocity in metadata if enabled
        if self.config.enable_metadata {
            for node in &mut prediction.centers {
                node.velocity = Some(velocity);
            }
        }

        // Drift analysis
        if let Some(ref mut drift_analyzer) = state.drift_analyzer {
            let predicted_centers: Vec<_> = base_labels.clone();
            if let Some(last_pos) = state.last_position {
                drift_analyzer.record_drift(&predicted_centers, position, detection.timestamp);
            }
            prediction.drift_leader_label = drift_analyzer.drift_leader();
        }

        // Anomaly detection
        if let Some(ref profile) = state.drift_profile {
            let lateral = velocity.x.abs().max(velocity.y.abs());
            let longitudinal = velocity.norm();
            let accel = if let Some(last_pos) = state.last_position {
                (velocity - (position - last_pos)).norm()
            } else {
                0.0
            };
            prediction.anomaly_flags = crate::extensions::object_type::check_anomalies(
                profile,
                lateral,
                longitudinal,
                accel,
            );
        }

        // Prune
        prune_below_threshold(&mut prediction, self.config.prune_threshold);
        limit_nodes(&mut prediction, 200);

        state.last_position = Some(position);

        // Homing intercept
        let intercept = if self.config.enable_homing {
            Some(compute_intercept(
                &prediction.centers,
                &Vector3::zeros(), // Application provides homing position
                self.config.homing_speed,
                &velocity,
            ))
        } else {
            None
        };

        PentaPrediction {
            track_id,
            prediction,
            velocity_estimate: velocity,
            intercept,
        }
    }

    /// Update with a homing object position for intercept computation.
    pub fn update_with_homing(
        &mut self,
        detection: DetectionEvent,
        homing_position: Vector3<f32>,
        homing_velocity: Vector3<f32>,
    ) -> PentaPrediction {
        let mut pred = self.update(detection);
        if self.config.enable_homing {
            pred.intercept = Some(compute_intercept(
                &pred.prediction.centers,
                &homing_position,
                self.config.homing_speed,
                &pred.velocity_estimate,
            ));
        }
        pred
    }
}
