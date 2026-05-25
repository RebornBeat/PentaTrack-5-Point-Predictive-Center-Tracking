//! PentaTrack — 5-Point Predictive Center Tracking
//!
//! Consumes DetectionEvent from OMNI-SENSE. Produces CenterNode prediction trees.
//! All extensions are opt-in via TrackingConfig fields.

pub mod config;
pub mod core;
pub mod extensions;
pub mod tracker;
pub mod utils;

pub use config::{ReferenceFrame, TrackingConfig, TrackingMode, VelocityMethod, WeightStrategy};
pub use core::center_engine::{CenterNode, Prediction};
pub use extensions::homing::InterceptPrediction;
pub use extensions::object_type::{DriftProfile, ObjectTypeTaxonomy};
pub use tracker::{PentaPrediction, PentaTracker};
