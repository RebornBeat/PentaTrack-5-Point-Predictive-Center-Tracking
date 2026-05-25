//! Text-based visualization utilities for debugging prediction trees.

use crate::core::center_engine::Prediction;

/// Print the prediction tree to stdout for debugging.
pub fn print_tree(prediction: &Prediction) {
    println!("PentaTrack Prediction Tree:");
    print_node(prediction, 0, "");
}

fn print_node(prediction: &Prediction, node_idx: usize, prefix: &str) {
    let node = &prediction.centers[node_idx];
    println!(
        "{}[d{}] {} pos=({:.2},{:.2},{:.2}) conf={:.3}",
        prefix,
        node.depth,
        node.label.as_str(),
        node.position.x,
        node.position.y,
        node.position.z,
        node.confidence
    );
    let child_prefix = format!("{}  ", prefix);
    for &child_idx in &node.children {
        print_node(prediction, child_idx, &child_prefix);
    }
}
