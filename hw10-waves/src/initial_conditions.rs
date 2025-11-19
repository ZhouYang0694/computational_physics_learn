use crate::config::{GaussianConfig, VelocityMode};

pub fn gaussian_profile(positions: &[f64], params: &GaussianConfig) -> Vec<f64> {
    let mut values: Vec<f64> = positions
        .iter()
        .map(|&x| {
            let shift = x - params.center;
            params.amplitude * (-params.k * shift * shift).exp()
        })
        .collect();

    if let Some(first) = values.first_mut() {
        *first = 0.0;
    }
    if let Some(last) = values.last_mut() {
        *last = 0.0;
    }

    values
}

pub fn gaussian_velocity(
    positions: &[f64],
    params: &GaussianConfig,
    displacement: &[f64],
    wave_speed: f64,
) -> Vec<f64> {
    match params.velocity {
        VelocityMode::Zero => vec![0.0; positions.len()],
        VelocityMode::RightTraveling => {
            let mut values: Vec<f64> = positions
                .iter()
                .zip(displacement.iter())
                .map(|(&x, &y)| {
                    let shift = x - params.center;
                    let derivative = -2.0 * params.k * shift * y;
                    -wave_speed * derivative
                })
                .collect();

            if let Some(first) = values.first_mut() {
                *first = 0.0;
            }
            if let Some(last) = values.last_mut() {
                *last = 0.0;
            }

            values
        }
    }
}
