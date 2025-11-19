mod config;
mod initial_conditions;
mod output;
mod solver;

use std::collections::VecDeque;
use std::env;

use anyhow::{Context, Result, ensure};

use crate::config::Config;
use crate::initial_conditions::{gaussian_profile, gaussian_velocity};
use crate::output::OutputWriter;
use crate::solver::Solver;

fn main() -> Result<()> {
    let config_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "configs/stability_scan.toml".to_string());
    let mut config = Config::from_file(&config_path)?;

    println!("Loaded configuration from {config_path}");
    if let Some(name) = &config.experiment {
        println!("Experiment tag: {name}");
    }

    let (adjusted_length, length_changed) =
        adjust_to_multiple(config.physical.string_length, config.simulation.dx);
    if length_changed {
        println!(
            "Adjusted string_length from {:.6} m to {:.6} m to align with dx = {:.6} m",
            config.physical.string_length, adjusted_length, config.simulation.dx
        );
        config.physical.string_length = adjusted_length;
    }

    let (adjusted_time, time_changed) =
        adjust_to_multiple(config.simulation.total_time, config.simulation.dt);
    if time_changed {
        println!(
            "Extended total_time from {:.6} s to {:.6} s to align with dt = {:.6} s",
            config.simulation.total_time, adjusted_time, config.simulation.dt
        );
        config.simulation.total_time = adjusted_time;
    }

    let solver = Solver::new(
        config.physical.string_length,
        &config.simulation,
        config.physical.wave_speed,
    )?;

    let dt = solver.dt();
    let total_steps = compute_steps(config.simulation.total_time, dt)
        .with_context(|| "failed to compute total steps".to_string())?;
    let frame_schedule = build_frame_schedule(total_steps, config.output.cadence);
    if frame_schedule.len() != config.output.cadence {
        println!(
            "Requested {} frames, will output {} frames after deduplication.",
            config.output.cadence,
            frame_schedule.len()
        );
    }
    let mut pending_frames: VecDeque<usize> = frame_schedule.into_iter().collect();

    let positions = solver.grid().positions.clone();
    let mut displacement0 = gaussian_profile(&positions, &config.gaussian);
    let velocity0 = gaussian_velocity(
        &positions,
        &config.gaussian,
        &displacement0,
        config.physical.wave_speed,
    );

    // Ensure fixed boundary nodes remain zero after Gaussian construction.
    enforce_dirichlet(&mut displacement0);

    let y_limit = compute_y_limit(&displacement0);
    let x_min = *positions.first().expect("grid must contain nodes");
    let x_max = *positions.last().expect("grid must contain nodes");

    let output_writer = OutputWriter::new(&config.output, x_min, x_max, y_limit)?;

    println!(
        "Grid points: {}, dt = {:.5e}s, dx = {:.5e}m, CFL = {:.3}",
        positions.len(),
        dt,
        solver.grid().dx,
        solver.cfl()
    );
    if let (Some(first), Some(last)) = (pending_frames.front(), pending_frames.back()) {
        println!(
            "Total steps: {total_steps}, frames scheduled: {} (first step = {}, last step = {})",
            pending_frames.len(),
            first,
            last
        );
    } else {
        println!(
            "Total steps: {total_steps}, frames scheduled: {}",
            pending_frames.len()
        );
    }

    solver.run(
        displacement0,
        velocity0,
        total_steps,
        1,
        |step, time, state| {
            if let Some(&target) = pending_frames.front() {
                if step == target {
                    output_writer.write_step(step, time, &positions, state)?;
                    pending_frames.pop_front();
                }
            }
            Ok(())
        },
    )?;

    println!(
        "Simulation complete. Results stored in {}",
        config.output.resolved_path().display()
    );

    Ok(())
}

fn compute_steps(duration: f64, dt: f64) -> Result<usize> {
    ensure!(duration >= 0.0, "duration must be non-negative");
    let raw = duration / dt;
    let rounded = raw.round();
    ensure!(
        (raw - rounded).abs() <= 1e-6,
        "duration must be an integer multiple of dt (ratio = {raw})"
    );
    Ok(rounded as usize)
}

fn enforce_dirichlet(values: &mut [f64]) {
    if let Some(first) = values.first_mut() {
        *first = 0.0;
    }
    if let Some(last) = values.last_mut() {
        *last = 0.0;
    }
}

fn compute_y_limit(values: &[f64]) -> f64 {
    let max_amp = values.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    let inflated = max_amp * 1.1;
    if inflated <= 1e-6 { 0.05 } else { inflated }
}

fn adjust_to_multiple(value: f64, step: f64) -> (f64, bool) {
    if step <= 0.0 {
        return (value, false);
    }
    let ratio = value / step;
    let rounded = ratio.round();
    if (ratio - rounded).abs() <= 1e-8 {
        (value, false)
    } else {
        let adjusted = ratio.ceil() * step;
        (adjusted, true)
    }
}

fn build_frame_schedule(total_steps: usize, frames: usize) -> Vec<usize> {
    if frames == 0 {
        return Vec::new();
    }
    if total_steps == 0 {
        return vec![0];
    }
    if frames == 1 {
        return vec![0];
    }

    let mut schedule = Vec::with_capacity(frames);
    let denom = (frames - 1) as f64;
    for i in 0..frames {
        let step = ((i as f64 / denom) * total_steps as f64).round() as usize;
        if schedule.last().copied() != Some(step) {
            schedule.push(step);
        }
    }
    if *schedule.last().unwrap() != total_steps {
        schedule.push(total_steps);
    }
    schedule
}
