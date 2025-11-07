use anyhow::{Result, anyhow};
use serde::Serialize;

use crate::dynamics::SimulationSample;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct OrbitalElements {
    pub specific_energy: f64,
    pub specific_angular_momentum: f64,
    pub eccentricity: f64,
}

pub fn compute_orbital_elements(sample: &SimulationSample, mu: f64) -> Result<OrbitalElements> {
    if !sample.radius.is_finite() || sample.radius <= 0.0 {
        return Err(anyhow!(
            "Invalid radius {:.6} for orbital element calculation",
            sample.radius
        ));
    }

    let speed_sq = sample.vx * sample.vx + sample.vy * sample.vy;
    let specific_energy = 0.5 * speed_sq - mu / sample.radius;
    let specific_angular_momentum = (sample.x * sample.vy - sample.y * sample.vx).abs();

    let ecc_argument = 1.0
        + (2.0 * specific_energy * specific_angular_momentum * specific_angular_momentum)
            / (mu * mu);

    let eccentricity = if ecc_argument < 0.0 {
        0.0
    } else {
        ecc_argument.sqrt()
    };

    Ok(OrbitalElements {
        specific_energy,
        specific_angular_momentum,
        eccentricity,
    })
}
