use anyhow::{Context, Result};

use crate::analysis::OrbitalElements;
use crate::config::{
    EnsembleSettings, LyapunovSettings, SimulationParams, VelocityScanSettings,
};
use crate::ensemble::{run_ensemble, EnsembleConfig, LyapunovDiagnostics, SimulationResultMetadata};

#[derive(Debug, Clone, Copy)]
pub struct VelocityScanPoint {
    pub vy: f64,
    pub eccentricity: f64,
    pub lyapunov: Option<f64>,
    pub diagnostics: LyapunovDiagnostics,
    pub orbital: OrbitalElements,
}

#[derive(Debug, Clone, Copy)]
pub struct VelocityScanMetadata {
    pub vy_min: f64,
    pub vy_max: f64,
    pub sample_count: usize,
}

#[derive(Debug, Clone)]
pub struct VelocityScanResult {
    pub points: Vec<VelocityScanPoint>,
    pub simulation: SimulationResultMetadata,
    pub scan: VelocityScanMetadata,
    pub lyapunov: LyapunovSettings,
}

pub fn run_velocity_scan(
    base: &SimulationParams,
    ensemble: &EnsembleSettings,
    lyapunov: LyapunovSettings,
    scan: &VelocityScanSettings,
) -> Result<VelocityScanResult> {
    let mut points = Vec::with_capacity(scan.sample_count);
    let mut captured_metadata: Option<SimulationResultMetadata> = None;

    let vy_values = linspace(scan.vy_min, scan.vy_max, scan.sample_count);

    for (idx, vy) in vy_values.iter().enumerate() {
        println!(
            "[velocity-scan] running sample {}/{} with v_y = {:.6}",
            idx + 1,
            vy_values.len(),
            vy
        );

        let mut params = base.clone();
        params.initial_state.vy = *vy;

        let ensemble_cfg = EnsembleConfig {
            sample_count: ensemble.sample_count,
            delta_theta0: params.delta_theta0,
            lyapunov,
        };

        let (ensemble_result, _runs) = run_ensemble(&params, &ensemble_cfg)
            .with_context(|| format!("Ensemble failed for v_y = {:.6}", vy))?;

        if captured_metadata.is_none() {
            captured_metadata = Some(ensemble_result.metadata);
        }

        points.push(VelocityScanPoint {
            vy: *vy,
            eccentricity: ensemble_result.orbital.eccentricity,
            lyapunov: ensemble_result.lyapunov.slope,
            diagnostics: ensemble_result.lyapunov,
            orbital: ensemble_result.orbital,
        });
    }

    let metadata = captured_metadata.unwrap_or(SimulationResultMetadata {
        mu: base.mu,
        dt: base.dt,
        total_time: base.total_time,
        delta_theta0: base.delta_theta0,
        initial_state: base.initial_state,
    });

    Ok(VelocityScanResult {
        points,
        simulation: metadata,
        scan: VelocityScanMetadata {
            vy_min: scan.vy_min,
            vy_max: scan.vy_max,
            sample_count: scan.sample_count,
        },
        lyapunov,
    })
}

fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count <= 1 {
        return vec![start];
    }

    let step = (end - start) / (count as f64 - 1.0);
    (0..count).map(|i| start + step * i as f64).collect()
}
