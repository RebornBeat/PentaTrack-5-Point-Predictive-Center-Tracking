// Metadata extension: attaches additional per-node data (velocity, timestamp, lineage).
// When enable_metadata = true, CenterNode.velocity is populated and lineage can be traversed.
// The CenterNode struct already contains velocity: Option<Vector3<f32>> and parent_idx.
// This module provides utility functions for lineage traversal.

use crate::core::center_engine::{CenterNode, Prediction};

/// Trace the lineage of a node back to the root (depth 0).
/// Returns Vec<node_index> from root to the specified node.
pub fn trace_lineage(prediction: &Prediction, node_idx: usize) -> Vec<usize> {
    let mut path = vec![node_idx];
    let mut current = node_idx;
    while let Some(parent) = prediction.centers[current].parent_idx {
        path.push(parent);
        current = parent;
    }
    path.reverse();
    path
}

/// Find the deepest most-likely node: the leaf with the highest confidence.
pub fn deepest_most_likely(prediction: &Prediction) -> usize {
    let max_depth = prediction
        .centers
        .iter()
        .map(|c| c.depth)
        .max()
        .unwrap_or(0);
    prediction
        .centers
        .iter()
        .enumerate()
        .filter(|(_, c)| c.depth == max_depth)
        .max_by(|(_, a), (_, b)| a.confidence.partial_cmp(&b.confidence).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0)
}
