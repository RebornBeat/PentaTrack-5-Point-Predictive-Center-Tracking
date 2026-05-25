use nalgebra::Vector3;

/// A 3D bounding box defined by minimum and maximum corners.
#[derive(Debug, Clone)]
pub struct BoundingBox3D {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

impl BoundingBox3D {
    pub fn center(&self) -> Vector3<f32> {
        (self.min + self.max) / 2.0
    }

    pub fn dimensions(&self) -> Vector3<f32> {
        self.max - self.min
    }

    pub fn from_2d_with_height(
        x_min: f32,
        y_min: f32,
        x_max: f32,
        y_max: f32,
        z_min: f32,
        z_max: f32,
    ) -> Self {
        Self {
            min: Vector3::new(x_min, y_min, z_min),
            max: Vector3::new(x_max, y_max, z_max),
        }
    }
}
