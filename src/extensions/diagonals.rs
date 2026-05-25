// Diagonals are handled directly in center_engine::compute_base_centers
// when config.enable_diagonals = true.
// This module documents the diagonal label set for reference.

use crate::core::center_engine::CenterLabel;

pub fn diagonal_labels() -> Vec<CenterLabel> {
    vec![
        CenterLabel::UpRight,
        CenterLabel::UpLeft,
        CenterLabel::DownRight,
        CenterLabel::DownLeft,
    ]
}

pub fn all_labels_with_diagonals() -> Vec<CenterLabel> {
    vec![
        CenterLabel::Neutral,
        CenterLabel::Right,
        CenterLabel::Left,
        CenterLabel::Up,
        CenterLabel::Down,
        CenterLabel::UpRight,
        CenterLabel::UpLeft,
        CenterLabel::DownRight,
        CenterLabel::DownLeft,
    ]
}
