use std::path::PathBuf;

use anyhow::{Context, Result, anyhow};
use hw08_chaotic_tumbling::analysis::{self, compute_orbital_elements};
use hw08_chaotic_tumbling::config::SimulationParams;
use hw08_chaotic_tumbling::dynamics::SimulationResult;
use hw08_chaotic_tumbling::output::{ensure_directory, resolve_artifacts, write_csv, write_json};
use hw08_chaotic_tumbling::plotting::render_all;
use hw08_chaotic_tumbling::{config, dynamics};

fn main() -> Result<()> {
    let config_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("config/simulation.toml"));

    let params = config::load_from_file(&config_path)
        .with_context(|| format!("Failed to load config from {}", config_path.display()))?;

    let artifacts = resolve_artifacts(&params.output);
    ensure_directory(&artifacts.directory)?;

    let (result, orbital) = run_primary_simulation(&params)?;

    write_csv(&artifacts.data_csv, &result.samples, &artifacts.data.csv)?;
    write_json(
        &artifacts.data_json,
        &result.metadata,
        &result.samples,
        &orbital,
        &artifacts.data.json,
    )?;

    render_all(&result, &artifacts)?;

    println!("Simulation complete.");
    println!("Specific energy ε = {:.6}", orbital.specific_energy);
    println!(
        "Specific angular momentum h = {:.6}",
        orbital.specific_angular_momentum
    );
    println!("Eccentricity e = {:.6}", orbital.eccentricity);

    Ok(())
}

fn run_primary_simulation(
    params: &SimulationParams,
) -> Result<(SimulationResult, analysis::OrbitalElements)> {
    let result = dynamics::propagate(params)?;
    let last_sample = result
        .samples
        .last()
        .ok_or_else(|| anyhow!("Simulation produced zero samples"))?;
    let orbital = compute_orbital_elements(last_sample, params.mu)?;
    Ok((result, orbital))
}
