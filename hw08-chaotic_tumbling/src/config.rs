use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use serde::Deserialize;

use crate::state::RigidBodyState;

#[derive(Debug, Deserialize)]
struct ConfigRoot {
    simulation: SimulationSection,
    output: OutputSection,
}

#[derive(Debug, Deserialize)]
struct EnsembleConfigRoot {
    simulation: SimulationSection,
    #[serde(default)]
    output: Option<OutputSection>,
    ensemble_output: EnsembleOutputSection,
    ensemble: EnsembleSection,
    #[serde(default)]
    lyapunov: Option<LyapunovSection>,
}

#[derive(Debug, Deserialize)]
struct VelocityScanConfigRoot {
    simulation: SimulationSection,
    #[serde(default)]
    output: Option<OutputSection>,
    ensemble: EnsembleSection,
    #[serde(default)]
    lyapunov: Option<LyapunovSection>,
    scan: VelocityScanSection,
    scan_output: VelocityScanOutputSection,
}

#[derive(Debug, Deserialize)]
struct SimulationSection {
    mu: f64,
    total_time: f64,
    dt: f64,
    #[serde(default = "default_delta_theta0")]
    delta_theta0: f64,
    #[serde(rename = "initial_state")]
    initial_state: InitialStateSection,
}

fn default_delta_theta0() -> f64 {
    0.0
}

#[derive(Debug, Deserialize)]
struct InitialStateSection {
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
    theta: f64,
    omega: f64,
}

#[derive(Debug, Deserialize, Clone)]
struct OutputSection {
    directory: PathBuf,
    theta_png: PathBuf,
    theta_svg: PathBuf,
    omega_png: PathBuf,
    omega_svg: PathBuf,
    trajectory_png: PathBuf,
    trajectory_svg: PathBuf,
    data_csv: PathBuf,
    data_json: PathBuf,
    delta_theta_png: PathBuf,
    delta_theta_svg: PathBuf,
    #[serde(default)]
    toggles: OutputTogglesSection,
    #[serde(default)]
    data: DataSection,
}

impl Default for OutputSection {
    fn default() -> Self {
        OutputSection {
            directory: PathBuf::from("unused"),
            theta_png: PathBuf::from("theta.png"),
            theta_svg: PathBuf::from("theta.svg"),
            omega_png: PathBuf::from("omega.png"),
            omega_svg: PathBuf::from("omega.svg"),
            trajectory_png: PathBuf::from("trajectory.png"),
            trajectory_svg: PathBuf::from("trajectory.svg"),
            data_csv: PathBuf::from("timeseries.csv"),
            data_json: PathBuf::from("timeseries.json"),
            delta_theta_png: PathBuf::from("delta_theta.png"),
            delta_theta_svg: PathBuf::from("delta_theta.svg"),
            toggles: OutputTogglesSection::default(),
            data: DataSection::default(),
        }
    }
}

#[derive(Debug, Deserialize)]
struct EnsembleSection {
    #[serde(default = "default_sample_count")]
    sample_count: usize,
    #[serde(default)]
    delta_theta0: Option<f64>,
}

fn default_sample_count() -> usize {
    8
}

#[derive(Debug, Deserialize, Clone)]
struct LyapunovSection {
    #[serde(default)]
    burn_in_time: f64,
    #[serde(default = "default_log_fraction")]
    log_fraction: f64,
}

impl Default for LyapunovSection {
    fn default() -> Self {
        Self {
            burn_in_time: 0.0,
            log_fraction: default_log_fraction(),
        }
    }
}

fn default_log_fraction() -> f64 {
    0.5
}

#[derive(Debug, Deserialize)]
struct EnsembleOutputSection {
    directory: PathBuf,
    average_png: PathBuf,
    average_svg: PathBuf,
    average_csv: PathBuf,
    average_json: PathBuf,
    lyapunov_png: PathBuf,
    lyapunov_svg: PathBuf,
    #[serde(default)]
    toggles: EnsembleOutputTogglesSection,
    #[serde(default)]
    csv: AverageCsvSection,
    #[serde(default)]
    json: AverageJsonSection,
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct EnsembleOutputTogglesSection {
    #[serde(default = "default_true")]
    png: bool,
    #[serde(default = "default_true")]
    svg: bool,
    #[serde(default = "default_true")]
    csv: bool,
    #[serde(default = "default_true")]
    json: bool,
    #[serde(default = "default_true")]
    lyapunov_png: bool,
    #[serde(default = "default_true")]
    lyapunov_svg: bool,
}

impl Default for EnsembleOutputTogglesSection {
    fn default() -> Self {
        Self {
            png: true,
            svg: true,
            csv: true,
            json: true,
            lyapunov_png: true,
            lyapunov_svg: true,
        }
    }
}

#[derive(Debug, Deserialize)]
struct AverageCsvSection {
    #[serde(default = "default_true")]
    enabled: bool,
    #[serde(default = "default_average_csv_fields")]
    fields: Vec<String>,
}

impl Default for AverageCsvSection {
    fn default() -> Self {
        Self {
            enabled: true,
            fields: default_average_csv_fields(),
        }
    }
}

#[derive(Debug, Deserialize)]
struct AverageJsonSection {
    #[serde(default = "default_true")]
    enabled: bool,
    #[serde(default = "default_true")]
    include_metadata: bool,
    #[serde(default = "default_true")]
    include_orbital: bool,
    #[serde(default = "default_true")]
    include_samples: bool,
    #[serde(default = "default_average_json_fields")]
    sample_fields: Vec<String>,
}

impl Default for AverageJsonSection {
    fn default() -> Self {
        Self {
            enabled: true,
            include_metadata: true,
            include_orbital: true,
            include_samples: true,
            sample_fields: default_average_json_fields(),
        }
    }
}

#[derive(Debug, Deserialize)]
struct VelocityScanSection {
    vy_min: f64,
    vy_max: f64,
    #[serde(default = "default_scan_samples")]
    sample_count: usize,
}

fn default_scan_samples() -> usize {
    8
}

#[derive(Debug, Deserialize)]
struct VelocityScanOutputSection {
    directory: PathBuf,
    plot_png: PathBuf,
    plot_svg: PathBuf,
    csv: PathBuf,
    json: PathBuf,
    #[serde(default)]
    toggles: VelocityScanOutputTogglesSection,
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct VelocityScanOutputTogglesSection {
    #[serde(default = "default_true")]
    png: bool,
    #[serde(default = "default_true")]
    svg: bool,
    #[serde(default = "default_true")]
    csv: bool,
    #[serde(default = "default_true")]
    json: bool,
}

impl Default for VelocityScanOutputTogglesSection {
    fn default() -> Self {
        Self {
            png: true,
            svg: true,
            csv: true,
            json: true,
        }
    }
}

#[derive(Debug, Deserialize, Clone, Copy)]
struct OutputTogglesSection {
    #[serde(default = "default_true")]
    theta: bool,
    #[serde(default = "default_true")]
    omega: bool,
    #[serde(default = "default_true")]
    delta_theta: bool,
    #[serde(default = "default_true")]
    trajectory: bool,
}

fn default_true() -> bool {
    true
}

impl Default for OutputTogglesSection {
    fn default() -> Self {
        Self {
            theta: true,
            omega: true,
            delta_theta: true,
            trajectory: true,
        }
    }
}

#[derive(Debug, Deserialize, Default, Clone)]
struct DataSection {
    #[serde(default)]
    csv: CsvDataSection,
    #[serde(default)]
    json: JsonDataSection,
}

#[derive(Debug, Deserialize, Clone)]
struct CsvDataSection {
    #[serde(default = "default_true")]
    enabled: bool,
    #[serde(default = "default_csv_fields")]
    fields: Vec<String>,
}

impl Default for CsvDataSection {
    fn default() -> Self {
        Self {
            enabled: true,
            fields: default_csv_fields(),
        }
    }
}

#[derive(Debug, Deserialize, Clone)]
struct JsonDataSection {
    #[serde(default = "default_true")]
    enabled: bool,
    #[serde(default = "default_true")]
    include_metadata: bool,
    #[serde(default = "default_true")]
    include_orbital: bool,
    #[serde(default = "default_true")]
    include_samples: bool,
    #[serde(default = "default_csv_fields")]
    sample_fields: Vec<String>,
}

impl Default for JsonDataSection {
    fn default() -> Self {
        Self {
            enabled: true,
            include_metadata: true,
            include_orbital: true,
            include_samples: true,
            sample_fields: default_csv_fields(),
        }
    }
}

fn default_csv_fields() -> Vec<String> {
    vec![
        "time".into(),
        "x".into(),
        "y".into(),
        "vx".into(),
        "vy".into(),
        "radius".into(),
        "theta_wrapped_primary".into(),
        "theta_wrapped_secondary".into(),
        "delta_theta".into(),
    ]
}

fn default_average_csv_fields() -> Vec<String> {
    vec!["time".into(), "delta_theta_avg".into()]
}

fn default_average_json_fields() -> Vec<String> {
    vec!["time".into(), "delta_theta_avg".into()]
}

#[derive(Debug, Clone)]
pub struct OutputPaths {
    pub directory: PathBuf,
    pub theta_png: PathBuf,
    pub theta_svg: PathBuf,
    pub omega_png: PathBuf,
    pub omega_svg: PathBuf,
    pub trajectory_png: PathBuf,
    pub trajectory_svg: PathBuf,
    pub data_csv: PathBuf,
    pub data_json: PathBuf,
    pub delta_theta_png: PathBuf,
    pub delta_theta_svg: PathBuf,
    pub toggles: OutputToggles,
    pub data: DataConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct OutputToggles {
    pub theta: bool,
    pub omega: bool,
    pub delta_theta: bool,
    pub trajectory: bool,
}

#[derive(Debug, Clone)]
pub struct DataConfig {
    pub csv: CsvExportConfig,
    pub json: JsonExportConfig,
}

#[derive(Debug, Clone)]
pub struct CsvExportConfig {
    pub enabled: bool,
    pub fields: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct JsonExportConfig {
    pub enabled: bool,
    pub include_metadata: bool,
    pub include_orbital: bool,
    pub include_samples: bool,
    pub sample_fields: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct SimulationParams {
    pub mu: f64,
    pub total_time: f64,
    pub dt: f64,
    pub delta_theta0: f64,
    pub initial_state: RigidBodyState,
    pub output: OutputPaths,
}

#[derive(Debug, Clone)]
pub struct EnsembleParams {
    pub base: SimulationParams,
    pub ensemble: EnsembleSettings,
    pub lyapunov: LyapunovSettings,
    pub output: EnsembleOutputPaths,
}

#[derive(Debug, Clone)]
pub struct EnsembleSettings {
    pub sample_count: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct LyapunovSettings {
    pub burn_in_time: f64,
    pub log_fraction: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct VelocityScanSettings {
    pub vy_min: f64,
    pub vy_max: f64,
    pub sample_count: usize,
}

#[derive(Debug, Clone)]
pub struct VelocityScanOutputPaths {
    pub directory: PathBuf,
    pub plot_png: PathBuf,
    pub plot_svg: PathBuf,
    pub csv: PathBuf,
    pub json: PathBuf,
    pub toggles: VelocityScanOutputToggles,
}

#[derive(Debug, Clone, Copy)]
pub struct VelocityScanOutputToggles {
    pub png: bool,
    pub svg: bool,
    pub csv: bool,
    pub json: bool,
}

#[derive(Debug, Clone)]
pub struct VelocityScanParams {
    pub base: SimulationParams,
    pub ensemble: EnsembleSettings,
    pub lyapunov: LyapunovSettings,
    pub scan: VelocityScanSettings,
    pub output: VelocityScanOutputPaths,
}

#[derive(Debug, Clone)]
pub struct EnsembleOutputPaths {
    pub directory: PathBuf,
    pub average_png: PathBuf,
    pub average_svg: PathBuf,
    pub average_csv: PathBuf,
    pub average_json: PathBuf,
    pub lyapunov_png: PathBuf,
    pub lyapunov_svg: PathBuf,
    pub toggles: AverageOutputToggles,
    pub csv: AverageCsvConfig,
    pub json: AverageJsonConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct AverageOutputToggles {
    pub png: bool,
    pub svg: bool,
    pub csv: bool,
    pub json: bool,
    pub lyapunov_png: bool,
    pub lyapunov_svg: bool,
}

#[derive(Debug, Clone)]
pub struct AverageCsvConfig {
    pub enabled: bool,
    pub fields: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct AverageJsonConfig {
    pub enabled: bool,
    pub include_metadata: bool,
    pub include_orbital: bool,
    pub include_samples: bool,
    pub sample_fields: Vec<String>,
}

pub fn load_from_file(path: impl AsRef<Path>) -> Result<SimulationParams> {
    let raw = fs::read_to_string(path.as_ref())
        .with_context(|| format!("Failed to read config file {}", path.as_ref().display()))?;

    let parsed: ConfigRoot =
        toml::from_str(&raw).context("Failed to parse simulation configuration")?;
    load_from_sections(&parsed.simulation, &parsed.output)
}

pub fn load_ensemble_from_file(path: impl AsRef<Path>) -> Result<EnsembleParams> {
    let raw = fs::read_to_string(path.as_ref())
        .with_context(|| format!("Failed to read config file {}", path.as_ref().display()))?;

    let parsed: EnsembleConfigRoot =
        toml::from_str(&raw).context("Failed to parse ensemble configuration")?;

    let output_section = parsed.output.clone().unwrap_or_default();
    let mut base = load_from_sections(&parsed.simulation, &output_section)?;
    let delta_theta0 = parsed
        .ensemble
        .delta_theta0
        .unwrap_or(parsed.simulation.delta_theta0);
    base.delta_theta0 = delta_theta0;

    let ensemble_output = &parsed.ensemble_output;
    let lyapunov_section = parsed.lyapunov.unwrap_or_default();

    Ok(EnsembleParams {
        base,
        ensemble: EnsembleSettings {
            sample_count: parsed.ensemble.sample_count,
        },
        lyapunov: LyapunovSettings {
            burn_in_time: lyapunov_section.burn_in_time.max(0.0),
            log_fraction: lyapunov_section.log_fraction,
        },
        output: EnsembleOutputPaths {
            directory: ensemble_output.directory.clone(),
            average_png: ensemble_output.average_png.clone(),
            average_svg: ensemble_output.average_svg.clone(),
            average_csv: ensemble_output.average_csv.clone(),
            average_json: ensemble_output.average_json.clone(),
            lyapunov_png: ensemble_output.lyapunov_png.clone(),
            lyapunov_svg: ensemble_output.lyapunov_svg.clone(),
            toggles: AverageOutputToggles {
                png: ensemble_output.toggles.png,
                svg: ensemble_output.toggles.svg,
                csv: ensemble_output.toggles.csv,
                json: ensemble_output.toggles.json,
                lyapunov_png: ensemble_output.toggles.lyapunov_png,
                lyapunov_svg: ensemble_output.toggles.lyapunov_svg,
            },
            csv: AverageCsvConfig {
                enabled: ensemble_output.csv.enabled,
                fields: ensemble_output.csv.fields.clone(),
            },
            json: AverageJsonConfig {
                enabled: ensemble_output.json.enabled,
                include_metadata: ensemble_output.json.include_metadata,
                include_orbital: ensemble_output.json.include_orbital,
                include_samples: ensemble_output.json.include_samples,
                sample_fields: ensemble_output.json.sample_fields.clone(),
            },
        },
    })
}

pub fn load_velocity_scan_from_file(path: impl AsRef<Path>) -> Result<VelocityScanParams> {
    let raw = fs::read_to_string(path.as_ref())
        .with_context(|| format!("Failed to read config file {}", path.as_ref().display()))?;

    let parsed: VelocityScanConfigRoot =
        toml::from_str(&raw).context("Failed to parse velocity scan configuration")?;

    let output_section = parsed.output.clone().unwrap_or_default();
    let mut base = load_from_sections(&parsed.simulation, &output_section)?;

    let delta_theta0 = parsed
        .ensemble
        .delta_theta0
        .unwrap_or(parsed.simulation.delta_theta0);
    base.delta_theta0 = delta_theta0;

    let lyapunov_section = parsed.lyapunov.unwrap_or_default();
    let scan_settings = parsed.scan;
    let scan_output = parsed.scan_output;

    Ok(VelocityScanParams {
        base,
        ensemble: EnsembleSettings {
            sample_count: parsed.ensemble.sample_count,
        },
        lyapunov: LyapunovSettings {
            burn_in_time: lyapunov_section.burn_in_time.max(0.0),
            log_fraction: lyapunov_section.log_fraction,
        },
        scan: VelocityScanSettings {
            vy_min: scan_settings.vy_min,
            vy_max: scan_settings.vy_max,
            sample_count: scan_settings.sample_count.max(1),
        },
        output: VelocityScanOutputPaths {
            directory: scan_output.directory.clone(),
            plot_png: scan_output.plot_png.clone(),
            plot_svg: scan_output.plot_svg.clone(),
            csv: scan_output.csv.clone(),
            json: scan_output.json.clone(),
            toggles: VelocityScanOutputToggles {
                png: scan_output.toggles.png,
                svg: scan_output.toggles.svg,
                csv: scan_output.toggles.csv,
                json: scan_output.toggles.json,
            },
        },
    })
}

fn load_from_sections(
    simulation: &SimulationSection,
    output: &OutputSection,
) -> Result<SimulationParams> {
    if simulation.dt <= 0.0 {
        return Err(anyhow!("Time step dt must be positive"));
    }

    if simulation.total_time <= 0.0 {
        return Err(anyhow!("Total time must be positive"));
    }

    let initial_state = RigidBodyState {
        x: simulation.initial_state.x,
        y: simulation.initial_state.y,
        vx: simulation.initial_state.vx,
        vy: simulation.initial_state.vy,
        theta: simulation.initial_state.theta,
        omega: simulation.initial_state.omega,
    };

    Ok(SimulationParams {
        mu: simulation.mu,
        total_time: simulation.total_time,
        dt: simulation.dt,
        delta_theta0: simulation.delta_theta0,
        initial_state,
        output: OutputPaths {
            directory: output.directory.clone(),
            theta_png: output.theta_png.clone(),
            theta_svg: output.theta_svg.clone(),
            omega_png: output.omega_png.clone(),
            omega_svg: output.omega_svg.clone(),
            trajectory_png: output.trajectory_png.clone(),
            trajectory_svg: output.trajectory_svg.clone(),
            data_csv: output.data_csv.clone(),
            data_json: output.data_json.clone(),
            delta_theta_png: output.delta_theta_png.clone(),
            delta_theta_svg: output.delta_theta_svg.clone(),
            toggles: OutputToggles {
                theta: output.toggles.theta,
                omega: output.toggles.omega,
                delta_theta: output.toggles.delta_theta,
                trajectory: output.toggles.trajectory,
            },
            data: DataConfig {
                csv: CsvExportConfig {
                    enabled: output.data.csv.enabled,
                    fields: output.data.csv.fields.clone(),
                },
                json: JsonExportConfig {
                    enabled: output.data.json.enabled,
                    include_metadata: output.data.json.include_metadata,
                    include_orbital: output.data.json.include_orbital,
                    include_samples: output.data.json.include_samples,
                    sample_fields: output.data.json.sample_fields.clone(),
                },
            },
        },
    })
}
