use nalgebra::Vector3;

/// Convert spherical coordinates (range, azimuth, elevation) to Cartesian.
pub fn spherical_to_cartesian(range_m: f32, azimuth_rad: f32, elevation_rad: f32) -> Vector3<f32> {
    let cos_el = elevation_rad.cos();
    Vector3::new(
        range_m * cos_el * azimuth_rad.cos(),
        range_m * cos_el * azimuth_rad.sin(),
        range_m * elevation_rad.sin(),
    )
}

/// Convert Cartesian to spherical (range, azimuth, elevation).
pub fn cartesian_to_spherical(pos: &Vector3<f32>) -> (f32, f32, f32) {
    let range = pos.norm();
    let elevation = if range > 1e-6 {
        (pos.z / range).asin()
    } else {
        0.0
    };
    let azimuth = pos.y.atan2(pos.x);
    (range, azimuth, elevation)
}
