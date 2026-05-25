use crate::config::VelocityMethod;
use nalgebra::Vector3;

/// History of object positions for velocity estimation.
pub struct VelocityEstimator {
    pub method: VelocityMethod,
    pub history: Vec<(f64, Vector3<f32>)>, // (timestamp_ns, position)
    pub max_history: usize,
    ema_velocity: Vector3<f32>,
}

impl VelocityEstimator {
    pub fn new(method: VelocityMethod, max_history: usize) -> Self {
        Self {
            method,
            history: Vec::new(),
            max_history,
            ema_velocity: Vector3::zeros(),
        }
    }

    pub fn record(&mut self, position: Vector3<f32>, timestamp_ns: u64) {
        self.history.push((timestamp_ns as f64, position));
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }
    }

    pub fn estimate(&mut self) -> Vector3<f32> {
        if self.history.len() < 2 {
            return Vector3::zeros();
        }

        match &self.method {
            VelocityMethod::LastDelta => {
                let n = self.history.len();
                let (t1, p1) = self.history[n - 2];
                let (t2, p2) = self.history[n - 1];
                let dt = ((t2 - t1) * 1e-9) as f32;
                if dt > 0.0 {
                    (p2 - p1) / dt
                } else {
                    Vector3::zeros()
                }
            }
            VelocityMethod::Ema { alpha } => {
                let n = self.history.len();
                let (t1, p1) = self.history[n - 2];
                let (t2, p2) = self.history[n - 1];
                let dt = ((t2 - t1) * 1e-9) as f32;
                let instant = if dt > 0.0 {
                    (p2 - p1) / dt
                } else {
                    Vector3::zeros()
                };
                self.ema_velocity = self.ema_velocity * (1.0 - alpha) + instant * alpha;
                self.ema_velocity
            }
            VelocityMethod::Wma => {
                let n = self.history.len().min(10);
                let recent = &self.history[self.history.len() - n..];
                let mut weighted_vel = Vector3::zeros();
                let mut total_w = 0.0f32;
                for i in 1..recent.len() {
                    let (t0, p0) = recent[i - 1];
                    let (t1, p1) = recent[i];
                    let dt = ((t1 - t0) * 1e-9) as f32;
                    if dt > 0.0 {
                        let w = i as f32;
                        weighted_vel += (p1 - p0) / dt * w;
                        total_w += w;
                    }
                }
                if total_w > 0.0 {
                    weighted_vel / total_w
                } else {
                    Vector3::zeros()
                }
            }
            VelocityMethod::Lsq => {
                // Linear least-squares velocity over history window
                let n = self.history.len();
                let t0 = self.history[0].0;
                let times: Vec<f32> = self
                    .history
                    .iter()
                    .map(|(t, _)| ((t - t0) * 1e-9) as f32)
                    .collect();
                let positions: Vec<Vector3<f32>> = self.history.iter().map(|(_, p)| *p).collect();

                let t_mean = times.iter().sum::<f32>() / n as f32;
                let mut num = Vector3::zeros();
                let mut den = 0.0f32;
                for i in 0..n {
                    let dt = times[i] - t_mean;
                    num += (positions[i]
                        - positions.iter().fold(Vector3::zeros(), |a, p| a + p) / n as f32)
                        * dt;
                    den += dt * dt;
                }
                if den > 0.0 {
                    num / den
                } else {
                    Vector3::zeros()
                }
            }
        }
    }
}
