use std::path::PathBuf;

use clap::{Parser, ValueEnum};

/// Command line options for the billiard simulator.
#[derive(Parser, Debug)]
#[command(author, version, about = "Configurable 2D billiard simulator")]
pub struct CliOptions {
    /// Path to the simulation TOML configuration file.
    #[arg(long, value_name = "FILE", default_value = "config/default.toml")]
    pub config: PathBuf,

    /// Override the collision detector configured in the TOML file.
    #[arg(long, value_enum)]
    pub detector: Option<DetectorOverride>,

    /// Display configuration summary without running the simulation.
    #[arg(long)]
    pub dry_run: bool,

    /// Render boundary-only preview image for verification.
    #[arg(long)]
    pub boundary_preview: bool,
}

/// Supported detector overrides exposed on the CLI.
#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum DetectorOverride {
    StepBack,
    Analytic,
}
