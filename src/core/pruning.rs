use crate::core::center_engine::Prediction;

/// Remove nodes below the confidence threshold from the prediction.
pub fn prune_below_threshold(prediction: &mut Prediction, threshold: f32) {
    prediction
        .centers
        .retain(|c| c.confidence >= threshold || c.depth == 0);
}

/// Limit tree to at most `max_nodes` nodes, keeping highest-confidence.
pub fn limit_nodes(prediction: &mut Prediction, max_nodes: usize) {
    if prediction.centers.len() <= max_nodes {
        return;
    }
    let mut sorted: Vec<usize> = (0..prediction.centers.len()).collect();
    sorted.sort_by(|&a, &b| {
        prediction.centers[b]
            .confidence
            .partial_cmp(&prediction.centers[a].confidence)
            .unwrap()
    });
    let keep: std::collections::HashSet<usize> = sorted[..max_nodes].iter().cloned().collect();
    let mut new_centers: Vec<_> = prediction
        .centers
        .iter()
        .enumerate()
        .filter(|(i, _)| keep.contains(i))
        .map(|(_, c)| c.clone())
        .collect();
    // Reset children (simplified: drop stale child references)
    for c in &mut new_centers {
        c.children.clear();
    }
    prediction.centers = new_centers;
}
