use crate::config::TrackingConfig;
use crate::core::center_engine::{compute_base_centers, CenterLabel, CenterNode, Prediction};
use nalgebra::Vector3;

/// Build the full prediction tree to the configured recursion depth.
pub fn build_prediction_tree(
    root_position: Vector3<f32>,
    config: &TrackingConfig,
    weights: Option<&Vec<(CenterLabel, f32)>>,
) -> Prediction {
    let mut nodes: Vec<CenterNode> = Vec::new();

    // Root node (depth 0)
    nodes.push(CenterNode {
        label: CenterLabel::Neutral,
        position: root_position,
        confidence: 1.0,
        depth: 0,
        velocity: None,
        parent_idx: None,
        children: Vec::new(),
    });

    // Expand recursively up to config.recursion_depth
    expand_node(0, root_position, 0, config, weights, &mut nodes);

    // Set parent's children references
    for i in 1..nodes.len() {
        if let Some(parent) = nodes[i].parent_idx {
            nodes[parent].children.push(i);
        }
    }

    Prediction {
        centers: nodes,
        drift_leader_label: None,
        anomaly_flags: Vec::new(),
    }
}

fn expand_node(
    parent_idx: usize,
    parent_pos: Vector3<f32>,
    depth: u32,
    config: &TrackingConfig,
    weights: Option<&Vec<(CenterLabel, f32)>>,
    nodes: &mut Vec<CenterNode>,
) {
    if depth >= config.recursion_depth {
        return;
    }

    let base_centers = compute_base_centers(parent_pos, config);
    let parent_conf = nodes[parent_idx].confidence;

    for (label, position) in base_centers {
        // Skip neutral at depth > 0 (avoid infinite recursion toward same point)
        if depth > 0 && label == CenterLabel::Neutral {
            continue;
        }

        // Compute weight for this center
        let weight = if let Some(ref wts) = weights {
            wts.iter()
                .find(|(l, _)| *l == label)
                .map(|(_, w)| *w)
                .unwrap_or(config.min_weight)
        } else {
            1.0 / count_non_neutral_labels(config) as f32
        };

        let confidence = (parent_conf * weight * config.confidence_decay).max(0.0);

        // Prune below threshold
        if confidence < config.prune_threshold {
            continue;
        }

        let child_idx = nodes.len();
        nodes.push(CenterNode {
            label: label.clone(),
            position,
            confidence,
            depth: depth + 1,
            velocity: None,
            parent_idx: Some(parent_idx),
            children: Vec::new(),
        });

        // Recurse
        expand_node(child_idx, position, depth + 1, config, weights, nodes);
    }
}

fn count_non_neutral_labels(config: &TrackingConfig) -> usize {
    let base = 4; // Right, Left, Up, Down
    let diag = if config.enable_diagonals { 4 } else { 0 };
    let y = if config.enable_y_axis { 2 } else { 0 };
    base + diag + y
}
