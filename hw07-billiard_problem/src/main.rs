mod boundary;
mod cli;
mod config;
mod detector;
mod math;
mod output;
mod physics;
mod render;
mod sampling;
mod simulation;

use std::path::Path;
use std::time::Instant;

use anyhow::{Result, anyhow};
use clap::Parser;

use crate::cli::{CliOptions, DetectorOverride};
use crate::config::{ConfigLoader, DetectorMode};
use crate::output::OutputBundle;
use crate::simulation::SimulationContext;

fn main() -> Result<()> {
    let cli = CliOptions::parse();

    let config_path = normalize_config_path(&cli.config)?;
    let mut app_config = ConfigLoader::load_from_path(&config_path)?;

    if let Some(detector_override) = cli.detector {
        apply_detector_override(detector_override, &mut app_config)?;
    }

    println!("Configuration summary:");
    for line in app_config.summary_lines() {
        println!("  - {line}");
    }

    let boundary = boundary::build_boundary(&app_config.simulation.boundary)?;
    println!(
        "  -> geometry check: {}, bounding radius {:.3}",
        boundary.shape_label(),
        boundary.bounding_radius()
    );

    if cli.boundary_preview {
        println!("Boundary preview flag is deprecated in the current build.");
    }

    if cli.dry_run {
        println!("Dry-run requested; exiting without running simulation.");
        return Ok(());
    }

    let start = Instant::now();
    let mut sim_context = SimulationContext::new(&app_config.simulation)?;
    sim_context.run()?;

    let bundle = OutputBundle {
        samples: &sim_context.samples,
        plot: &app_config.simulation.plot,
        output: &app_config.simulation.output,
        output_dir: &app_config.resolved_output_dir,
        boundary: Some(sim_context.boundary.as_ref()),
    };
    let csv_files = bundle.write_csv()?;
    let plot_files = bundle.write_plots()?;

    println!(
        "Simulation finished in {:.3?}; generated {} CSVs and {} plots.",
        start.elapsed(),
        csv_files.len(),
        plot_files.len()
    );
    for file in csv_files.into_iter().chain(plot_files) {
        println!("  -> {}", file.display());
    }
    Ok(())
}

fn normalize_config_path(path: &Path) -> Result<std::path::PathBuf> {
    if path.exists() {
        return Ok(path.to_path_buf());
    }

    Err(anyhow!(
        "configuration file {} does not exist",
        path.display()
    ))
}

fn apply_detector_override(
    detector_override: DetectorOverride,
    app_config: &mut config::AppConfig,
) -> Result<()> {
    match detector_override {
        DetectorOverride::StepBack => {
            if app_config.simulation.detector.step_size.is_none() {
                return Err(anyhow!(
                    "step-back detector override requires `detector.step_size` in the configuration"
                ));
            }
            app_config.simulation.detector.mode = DetectorMode::StepBack;
        }
        DetectorOverride::Analytic => {
            app_config.simulation.detector.mode = DetectorMode::Analytic;
        }
    }
    Ok(())
}
