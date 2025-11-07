use std::path::PathBuf;

use anyhow::{Context, Result};
use hw08_chaotic_tumbling::config::load_ensemble_from_file;
use hw08_chaotic_tumbling::ensemble::{EnsembleConfig, export_ensemble, run_ensemble};
use hw08_chaotic_tumbling::output::{ensure_directory, resolve_average};

fn main() -> Result<()> {
    let config_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("config/ensemble.toml"));

    let config = load_ensemble_from_file(&config_path)
        .with_context(|| format!("Failed to load ensemble config {}", config_path.display()))?;

    ensure_directory(&config.output.directory)?;

    let ensemble_cfg = EnsembleConfig {
        sample_count: config.ensemble.sample_count,
        delta_theta0: config.base.delta_theta0,
        lyapunov: config.lyapunov,
    };

    println!(
        "[ensemble] starting run with {} samples, Δθ0 = {:.6e}",
        ensemble_cfg.sample_count, ensemble_cfg.delta_theta0
    );

    let (ensemble_result, _runs) = run_ensemble(&config.base, &ensemble_cfg)?;

    let artifacts = resolve_average(&config.output);
    export_ensemble(&artifacts, &ensemble_result)?;

    println!(
        "[ensemble] completed. Average data points: {}. Outputs in {}",
        ensemble_result.time_grid.len(),
        artifacts.directory.display()
    );

    Ok(())
}
