//! Load drift profiles from JSON files.

use crate::extensions::object_type::DriftProfile;
use std::collections::HashMap;

pub fn load_profiles_from_json(
    json: &str,
) -> Result<HashMap<String, DriftProfile>, serde_json::Error> {
    serde_json::from_str(json)
}
