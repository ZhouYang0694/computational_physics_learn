use std::f64::consts::PI;

use anyhow::{Context, Result};
use serde_json;

use crate::analysis::{compute_orbital_elements, OrbitalElements};
use crate::config::{LyapunovSettings, SimulationParams};
use crate::dynamics::{SimulationResult, propagate};
use crate::output::{AverageArtifacts, write_average_csv, write_average_json};
use crate::plotting::{render_delta_theta_average, render_lyapunov_overlay};

#[derive(Debug, Clone)]
pub struct EnsembleConfig {
    pub sample_count: usize,
    pub delta_theta0: f64,
    pub lyapunov: LyapunovSettings,
}

#[derive(Debug)]
pub struct EnsembleResult {
    pub averaged_samples: Vec<f64>,
    pub time_grid: Vec<f64>,
    pub theta_samples: Vec<f64>,
    pub metadata: SimulationResultMetadata,
    pub orbital: OrbitalElements,
    pub lyapunov: LyapunovDiagnostics,
}

pub fn run_ensemble(
    params: &SimulationParams,
    ensemble_config: &EnsembleConfig,
) -> Result<(
    EnsembleResult,
    Vec<(f64, SimulationResult, OrbitalElements)>,
)> {
    let sample_count = ensemble_config.sample_count.max(1);
    let mut accumulated: Vec<f64> = Vec::new();
    let mut time_grid: Vec<f64> = Vec::new();
    let mut per_simulation = Vec::with_capacity(sample_count);
    let mut theta_values = Vec::with_capacity(sample_count);
    let mut reference_metadata: Option<SimulationResultMetadata> = None;
    let mut reference_orbital: Option<OrbitalElements> = None;

    for (index, theta0) in theta_samples(sample_count).into_iter().enumerate() {
        println!(
            "[ensemble] running sample {}/{} with theta0 = {:.6}",
            index + 1,
            sample_count,
            theta0
        );

        let mut modified_params = params.clone();
        modified_params.initial_state.theta = theta0;
        modified_params.delta_theta0 = ensemble_config.delta_theta0;

        let sim_result = propagate(&modified_params)
            .with_context(|| format!("Simulation failed for theta0 = {:.6}", theta0))?;
        let orbital = compute_orbital_elements(
            sim_result
                .samples
                .last()
                .expect("simulation produced samples"),
            modified_params.mu,
        )?;

        if accumulated.is_empty() {
            accumulated.resize(sim_result.samples.len(), 0.0);
            time_grid = sim_result
                .samples
                .iter()
                .map(|sample| sample.time)
                .collect();
            reference_metadata = Some(SimulationResultMetadata {
                mu: sim_result.metadata.mu,
                dt: sim_result.metadata.dt,
                total_time: sim_result.metadata.total_time,
                delta_theta0: modified_params.delta_theta0,
                initial_state: sim_result.metadata.initial_state,
            });
            reference_orbital = Some(orbital);
        }

        for (acc, sample) in accumulated.iter_mut().zip(sim_result.samples.iter()) {
            *acc += sample.delta_theta;
        }

        per_simulation.push((theta0, sim_result, orbital));
        theta_values.push(theta0);
    }

    let averaged_samples: Vec<f64> = accumulated
        .into_iter()
        .map(|value| value / sample_count as f64)
        .collect();

    let metadata_ref = reference_metadata
        .as_ref()
        .expect("metadata captured");
    let lyapunov = analyze_rising_segment(
        &time_grid,
        &averaged_samples,
        metadata_ref.dt,
        &ensemble_config.lyapunov,
    );

    Ok((
        EnsembleResult {
            averaged_samples,
            time_grid,
            theta_samples: theta_values,
            metadata: reference_metadata.expect("metadata captured"),
            orbital: reference_orbital.expect("orbital captured"),
            lyapunov,
        },
        per_simulation,
    ))
}

fn theta_samples(count: usize) -> Vec<f64> {
    if count == 1 {
        return vec![0.0];
    }

    let step = (2.0 * PI) / (count as f64);
    (0..count).map(|i| -PI + i as f64 * step).collect()
}

pub fn export_ensemble(artifacts: &AverageArtifacts, ensemble: &EnsembleResult) -> Result<()> {
    if artifacts.toggles.csv {
        write_average_csv(
            &artifacts.average_csv,
            &ensemble.time_grid,
            &ensemble.averaged_samples,
            &artifacts.csv,
        )?;
    }

    if artifacts.toggles.json {
        let metadata_json =
            build_average_metadata_json(&ensemble.metadata, &ensemble.theta_samples);
        let orbital_json = serde_json::to_value(&ensemble.orbital)
            .context("Failed to serialize orbital elements for ensemble JSON")?;
        let lyapunov_json = build_lyapunov_json(&ensemble.lyapunov);
        write_average_json(
            &artifacts.average_json,
            &ensemble.time_grid,
            &ensemble.averaged_samples,
            &artifacts.json,
            &metadata_json,
            &orbital_json,
            Some(&lyapunov_json),
        )?;
    }

    if artifacts.toggles.png || artifacts.toggles.svg {
        render_delta_theta_average(artifacts, &ensemble.time_grid, &ensemble.averaged_samples)?;
    }

    if artifacts.toggles.lyapunov_png || artifacts.toggles.lyapunov_svg {
        render_lyapunov_overlay(
            artifacts,
            &ensemble.time_grid,
            &ensemble.averaged_samples,
            &ensemble.lyapunov,
        )?;
    }

    Ok(())
}

#[derive(Debug, Clone, Copy)]
pub struct SimulationResultMetadata {
    pub mu: f64,
    pub dt: f64,
    pub total_time: f64,
    pub delta_theta0: f64,
    pub initial_state: crate::state::RigidBodyState,
}

#[derive(Debug, Clone, Copy)]
pub struct LyapunovDiagnostics {
    pub burn_in_time: f64,
    pub log_fraction: f64,
    pub pre_burn_min: f64,
    pub pre_burn_max: f64,
    pub pre_burn_log_min: f64,
    pub pre_burn_log_max: f64,
    pub post_burn_min: Option<f64>,
    pub post_burn_max: Option<f64>,
    pub post_burn_log_min: Option<f64>,
    pub post_burn_log_max: Option<f64>,
    pub segment_start_time: Option<f64>,
    pub segment_end_time: Option<f64>,
    pub threshold_log10: Option<f64>,
    pub segment_start_index: Option<usize>,
    pub segment_end_index: Option<usize>,
    pub segment_points: usize,
    pub slope: Option<f64>,
    pub intercept: Option<f64>,
}

fn build_average_metadata_json(
    metadata: &SimulationResultMetadata,
    theta_samples: &[f64],
) -> serde_json::Value {
    let mut initial_state = serde_json::Map::new();
    initial_state.insert("x".into(), serde_json::json!(metadata.initial_state.x));
    initial_state.insert("y".into(), serde_json::json!(metadata.initial_state.y));
    initial_state.insert("vx".into(), serde_json::json!(metadata.initial_state.vx));
    initial_state.insert("vy".into(), serde_json::json!(metadata.initial_state.vy));
    initial_state.insert(
        "omega".into(),
        serde_json::json!(metadata.initial_state.omega),
    );
    initial_state.insert(
        "theta".into(),
        serde_json::Value::Array(
            theta_samples
                .iter()
                .map(|theta| serde_json::json!(theta))
                .collect(),
        ),
    );

    serde_json::json!({
        "delta_theta0": metadata.delta_theta0,
        "dt": metadata.dt,
        "total_time": metadata.total_time,
        "mu": metadata.mu,
        "initial_state": serde_json::Value::Object(initial_state)
    })
}

fn analyze_rising_segment(
    time: &[f64],
    values: &[f64],
    dt: f64,
    config: &LyapunovSettings,
) -> LyapunovDiagnostics {
    if values.is_empty() || time.len() != values.len() || dt <= 0.0 {
        return LyapunovDiagnostics {
            burn_in_time: config.burn_in_time,
            log_fraction: config.log_fraction,
            pre_burn_min: 0.0,
            pre_burn_max: 0.0,
            pre_burn_log_min: f64::NEG_INFINITY,
            pre_burn_log_max: f64::NEG_INFINITY,
            post_burn_min: None,
            post_burn_max: None,
            post_burn_log_min: None,
            post_burn_log_max: None,
            segment_start_time: None,
            segment_end_time: None,
            threshold_log10: None,
            segment_start_index: None,
            segment_end_index: None,
            segment_points: 0,
            slope: None,
            intercept: None,
        };
    }

    let mut pre_min = f64::INFINITY;
    let mut pre_max = f64::NEG_INFINITY;
    for &value in values {
        if value < pre_min {
            pre_min = value;
        }
        if value > pre_max {
            pre_max = value;
        }
    }

    let pre_burn_log_min = pre_min.log10();
    let pre_burn_log_max = pre_max.log10();

    let burn_index = (config.burn_in_time / dt).floor().max(0.0) as usize;
    let capped_burn_index = burn_index.min(values.len());
    let post_slice = &values[capped_burn_index..];

    if post_slice.is_empty() {
        return LyapunovDiagnostics {
            burn_in_time: config.burn_in_time,
            log_fraction: config.log_fraction,
            pre_burn_min: pre_min,
            pre_burn_max: pre_max,
            pre_burn_log_min,
            pre_burn_log_max,
            post_burn_min: None,
            post_burn_max: None,
            post_burn_log_min: None,
            post_burn_log_max: None,
            segment_start_time: None,
            segment_end_time: None,
            threshold_log10: None,
            segment_start_index: None,
            segment_end_index: None,
            segment_points: 0,
            slope: None,
            intercept: None,
        };
    }

    let mut post_min = f64::INFINITY;
    let mut post_max = f64::NEG_INFINITY;
    for &value in post_slice {
        if value < post_min {
            post_min = value;
        }
        if value > post_max {
            post_max = value;
        }
    }

    let post_burn_log_min = post_min.log10();
    let post_burn_log_max = post_max.log10();
    let log_range = (post_burn_log_max - post_burn_log_min).max(0.0);
    let fraction = config.log_fraction.clamp(0.0, 1.0);
    let threshold = post_burn_log_min + fraction * log_range;

    let mut segment_start: Option<usize> = None;
    let mut segment_end: Option<usize> = None;

    for (offset, &value) in post_slice.iter().enumerate() {
        let log_value = value.log10();
        if log_value < threshold {
            let absolute_index = capped_burn_index + offset;
            segment_start = Some(absolute_index);
            let mut end_index = absolute_index;
            let mut next_index = absolute_index + 1;
            while next_index < values.len() && values[next_index].log10() < threshold {
                end_index = next_index;
                next_index += 1;
            }
            segment_end = Some(end_index);
            break;
        }
    }

    let mut segment_points = 0usize;
    let mut slope = None;
    let mut intercept = None;
    let mut segment_start_time = None;
    let mut segment_end_time = None;

    if let (Some(start_idx), Some(end_idx)) = (segment_start, segment_end) {
        if end_idx >= start_idx {
            let slice_end = end_idx + 1;
            let segment_time = &time[start_idx..slice_end];
            let segment_values = &values[start_idx..slice_end];
            segment_points = segment_time.len();
            if segment_points >= 2 {
                let log_values: Vec<f64> = segment_values.iter().map(|v| v.ln()).collect();
                if let Some((m, b)) = linear_regression(segment_time, &log_values) {
                    slope = Some(m);
                    intercept = Some(b);
                }
            }
            segment_start_time = Some(time[start_idx]);
            segment_end_time = Some(time[end_idx]);
        }
    }

    LyapunovDiagnostics {
        burn_in_time: config.burn_in_time,
        log_fraction: config.log_fraction,
        pre_burn_min: pre_min,
        pre_burn_max: pre_max,
        pre_burn_log_min,
        pre_burn_log_max,
        post_burn_min: Some(post_min),
        post_burn_max: Some(post_max),
        post_burn_log_min: Some(post_burn_log_min),
        post_burn_log_max: Some(post_burn_log_max),
        segment_start_time,
        segment_end_time,
        threshold_log10: Some(threshold),
        segment_start_index: segment_start,
        segment_end_index: segment_end,
        segment_points,
        slope,
        intercept,
    }
}

fn build_lyapunov_json(diagnostics: &LyapunovDiagnostics) -> serde_json::Value {
    let pre_burn = serde_json::json!({
        "min": diagnostics.pre_burn_min,
        "max": diagnostics.pre_burn_max,
        "log10_min": diagnostics.pre_burn_log_min,
        "log10_max": diagnostics.pre_burn_log_max,
    });

    let post_burn = match (
        diagnostics.post_burn_min,
        diagnostics.post_burn_max,
        diagnostics.post_burn_log_min,
        diagnostics.post_burn_log_max,
    ) {
        (Some(min), Some(max), Some(log_min), Some(log_max)) => serde_json::json!({
            "min": min,
            "max": max,
            "log10_min": log_min,
            "log10_max": log_max,
        }),
        _ => serde_json::Value::Null,
    };

    let rising_segment = if let (
        Some(start_time),
        Some(end_time),
        Some(start_idx),
        Some(end_idx),
    ) = (
        diagnostics.segment_start_time,
        diagnostics.segment_end_time,
        diagnostics.segment_start_index,
        diagnostics.segment_end_index,
    )
    {
        serde_json::json!({
            "start_time": start_time,
            "end_time": end_time,
            "start_index": start_idx,
            "end_index": end_idx,
            "threshold_log10": diagnostics.threshold_log10,
        })
    } else {
        serde_json::json!({
            "start_time": serde_json::Value::Null,
            "end_time": serde_json::Value::Null,
            "start_index": serde_json::Value::Null,
            "end_index": serde_json::Value::Null,
            "threshold_log10": diagnostics.threshold_log10,
        })
    };

    serde_json::json!({
        "burn_in_time": diagnostics.burn_in_time,
        "log_fraction": diagnostics.log_fraction,
        "pre_burn": pre_burn,
        "post_burn": post_burn,
        "rising_segment": rising_segment,
        "regression": {
            "point_count": diagnostics.segment_points,
            "slope": diagnostics.slope,
            "intercept": diagnostics.intercept,
            "lyapunov_estimate": diagnostics.slope,
            "log_base": "e",
        }
    })
}

fn linear_regression(x: &[f64], y: &[f64]) -> Option<(f64, f64)> {
    if x.len() != y.len() || x.len() < 2 {
        return None;
    }

    let n = x.len() as f64;
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_x2: f64 = x.iter().map(|v| v * v).sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-12 {
        return None;
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;
    Some((slope, intercept))
}
