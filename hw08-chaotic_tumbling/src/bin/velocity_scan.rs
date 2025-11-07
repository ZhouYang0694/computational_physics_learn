use std::path::PathBuf;

use anyhow::{Context, Result};
use hw08_chaotic_tumbling::config::load_velocity_scan_from_file;
use hw08_chaotic_tumbling::output::{
    ensure_directory, resolve_velocity_scan, write_velocity_scan_csv, write_velocity_scan_json,
};
use hw08_chaotic_tumbling::plotting::render_lyapunov_vs_ecc;
use hw08_chaotic_tumbling::scan::run_velocity_scan;

fn main() -> Result<()> {
    let config_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("config/velocity_scan.toml"));

    let config = load_velocity_scan_from_file(&config_path).with_context(|| {
        format!(
            "Failed to load velocity scan config {}",
            config_path.display()
        )
    })?;

    ensure_directory(&config.output.directory)?;

    let scan_result = run_velocity_scan(
        &config.base,
        &config.ensemble,
        config.lyapunov,
        &config.scan,
    )?;

    let artifacts = resolve_velocity_scan(&config.output);

    if artifacts.toggles.csv {
        write_velocity_scan_csv(&artifacts.csv, &scan_result.points)?;
    }

    if artifacts.toggles.json {
        write_velocity_scan_json(
            &artifacts.json,
            &scan_result.points,
            &scan_result.simulation,
            &scan_result.scan,
            &scan_result.lyapunov,
        )?;
    }

    if artifacts.toggles.png || artifacts.toggles.svg {
        render_lyapunov_vs_ecc(&artifacts, &scan_result.points)?;
    }

    println!(
        "[velocity-scan] completed. Samples: {}. Outputs in {}",
        scan_result.points.len(),
        artifacts.directory.display()
    );

    Ok(())
}
