use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail, ensure};
use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct Config {
    pub simulation: SimulationConfig,
    pub physical: PhysicalConfig,
    pub gaussian: GaussianConfig,
    pub output: OutputConfig,
    #[serde(default)]
    pub experiment: Option<String>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct SimulationConfig {
    pub dt: f64,
    pub dx: f64,
    pub total_time: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct PhysicalConfig {
    pub wave_speed: f64,
    pub string_length: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct GaussianConfig {
    #[serde(default = "default_amplitude")]
    pub amplitude: f64,
    pub center: f64,
    pub k: f64,
    #[serde(default)]
    pub velocity: VelocityMode,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(rename_all = "snake_case")]
pub enum VelocityMode {
    Zero,
    RightTraveling,
}

impl Default for VelocityMode {
    fn default() -> Self {
        VelocityMode::Zero
    }
}

#[derive(Debug, Deserialize, Clone)]
pub struct OutputConfig {
    #[serde(default = "default_base_dir")]
    pub base_dir: String,
    pub directory: String,
    pub cadence: usize,
    #[serde(default)]
    pub write_csv: bool,
    #[serde(default = "default_pixels_per_unit")]
    pub pixels_per_unit: f64,
}

impl Config {
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path_ref = path.as_ref();
        let content = fs::read_to_string(path_ref)
            .with_context(|| format!("failed to read config file {:?}", path_ref))?;
        let config: Config = toml::from_str(&content)
            .with_context(|| "failed to parse TOML configuration".to_string())?;
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> Result<()> {
        ensure!(self.simulation.dt > 0.0, "dt must be positive");
        ensure!(self.simulation.dx > 0.0, "dx must be positive");
        ensure!(
            self.simulation.total_time > 0.0,
            "total_time must be positive"
        );
        ensure!(
            self.physical.wave_speed > 0.0,
            "wave_speed must be positive"
        );
        ensure!(
            self.physical.string_length > 0.0,
            "string_length must be positive"
        );
        ensure!(
            self.output.cadence >= 1,
            "output cadence must be at least one frame"
        );
        ensure!(
            self.output.pixels_per_unit > 0.0,
            "pixels_per_unit must be positive"
        );
        ensure!(
            self.gaussian.center >= 0.0 && self.gaussian.center <= self.physical.string_length,
            "gaussian center must lie within [0, string_length]",
        );

        if self.gaussian.k < 0.0 {
            bail!("gaussian k must be non-negative");
        }

        let output_dir = self.output.resolved_path();
        fs::create_dir_all(&output_dir)
            .with_context(|| format!("failed to create output directory {:?}", output_dir))?;

        Ok(())
    }
}

impl OutputConfig {
    pub fn resolved_path(&self) -> PathBuf {
        let path = Path::new(&self.directory);
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            let mut root = PathBuf::from(&self.base_dir);
            root.push(path);
            root
        }
    }
}

fn default_amplitude() -> f64 {
    1.0
}

fn default_base_dir() -> String {
    "output".to_string()
}

fn default_pixels_per_unit() -> f64 {
    800.0
}
