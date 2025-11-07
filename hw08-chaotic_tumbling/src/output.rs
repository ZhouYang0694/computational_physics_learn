use std::fs::{self, File};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use serde_json;

use crate::analysis::OrbitalElements;
use crate::config::{
    AverageCsvConfig, AverageJsonConfig, AverageOutputToggles, CsvExportConfig, DataConfig,
    EnsembleOutputPaths, JsonExportConfig, LyapunovSettings, OutputPaths, OutputToggles,
    VelocityScanOutputPaths, VelocityScanOutputToggles,
};
use crate::dynamics::{SimulationMetadata, SimulationSample};
use crate::ensemble::SimulationResultMetadata;
use crate::scan::{VelocityScanMetadata, VelocityScanPoint};

#[derive(Debug, Clone)]
pub struct OutputArtifacts {
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

#[derive(Debug, Clone)]
pub struct AverageArtifacts {
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

#[derive(Debug, Clone)]
pub struct VelocityScanArtifacts {
    pub directory: PathBuf,
    pub plot_png: PathBuf,
    pub plot_svg: PathBuf,
    pub csv: PathBuf,
    pub json: PathBuf,
    pub toggles: VelocityScanOutputToggles,
}

pub fn resolve_artifacts(paths: &OutputPaths) -> OutputArtifacts {
    let directory = paths.directory.clone();

    OutputArtifacts {
        directory: directory.clone(),
        theta_png: resolve_path(&directory, &paths.theta_png),
        theta_svg: resolve_path(&directory, &paths.theta_svg),
        omega_png: resolve_path(&directory, &paths.omega_png),
        omega_svg: resolve_path(&directory, &paths.omega_svg),
        trajectory_png: resolve_path(&directory, &paths.trajectory_png),
        trajectory_svg: resolve_path(&directory, &paths.trajectory_svg),
        data_csv: resolve_path(&directory, &paths.data_csv),
        data_json: resolve_path(&directory, &paths.data_json),
        delta_theta_png: resolve_path(&directory, &paths.delta_theta_png),
        delta_theta_svg: resolve_path(&directory, &paths.delta_theta_svg),
        toggles: paths.toggles,
        data: paths.data.clone(),
    }
}

pub fn resolve_average(paths: &EnsembleOutputPaths) -> AverageArtifacts {
    let directory = paths.directory.clone();

    AverageArtifacts {
        directory: directory.clone(),
        average_png: resolve_path(&directory, &paths.average_png),
        average_svg: resolve_path(&directory, &paths.average_svg),
        average_csv: resolve_path(&directory, &paths.average_csv),
        average_json: resolve_path(&directory, &paths.average_json),
        lyapunov_png: resolve_path(&directory, &paths.lyapunov_png),
        lyapunov_svg: resolve_path(&directory, &paths.lyapunov_svg),
        toggles: paths.toggles,
        csv: paths.csv.clone(),
        json: paths.json.clone(),
    }
}

pub fn resolve_velocity_scan(paths: &VelocityScanOutputPaths) -> VelocityScanArtifacts {
    let directory = paths.directory.clone();

    VelocityScanArtifacts {
        directory: directory.clone(),
        plot_png: resolve_path(&directory, &paths.plot_png),
        plot_svg: resolve_path(&directory, &paths.plot_svg),
        csv: resolve_path(&directory, &paths.csv),
        json: resolve_path(&directory, &paths.json),
        toggles: paths.toggles,
    }
}

fn resolve_path(base: &Path, relative: &Path) -> PathBuf {
    if relative.is_absolute() {
        relative.to_path_buf()
    } else {
        base.join(relative)
    }
}

pub fn ensure_directory(path: &Path) -> Result<()> {
    if !path.exists() {
        fs::create_dir_all(path)
            .with_context(|| format!("Failed to create output directory {}", path.display()))?;
    }
    Ok(())
}

pub fn write_csv(
    path: &Path,
    samples: &[SimulationSample],
    config: &CsvExportConfig,
) -> Result<()> {
    if !config.enabled {
        return Ok(());
    }

    let fields = parse_sample_fields(&config.fields)?;
    if fields.is_empty() {
        return Ok(());
    }

    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }
    let mut writer = csv::Writer::from_path(path)
        .with_context(|| format!("Unable to create CSV file {}", path.display()))?;

    writer.write_record(fields.iter().map(|field| field.header()))?;

    for sample in samples {
        let row: Vec<String> = fields.iter().map(|field| field.format(sample)).collect();
        writer
            .write_record(&row)
            .with_context(|| format!("Failed to write sample at t={:.6}", sample.time))?;
    }

    writer
        .flush()
        .with_context(|| format!("Failed to flush CSV writer for {}", path.display()))
}

pub fn write_json(
    path: &Path,
    metadata: &SimulationMetadata,
    samples: &[SimulationSample],
    orbital: &OrbitalElements,
    config: &JsonExportConfig,
) -> Result<()> {
    if !config.enabled {
        return Ok(());
    }

    let sample_fields = if config.include_samples {
        parse_sample_fields(&config.sample_fields)?
    } else {
        Vec::new()
    };

    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }

    let mut root = serde_json::Map::new();

    if config.include_metadata {
        root.insert(
            "metadata".into(),
            serde_json::to_value(metadata)
                .context("Failed to serialize metadata for JSON export")?,
        );
    }

    if config.include_orbital {
        root.insert(
            "orbital_elements".into(),
            serde_json::to_value(orbital)
                .context("Failed to serialize orbital elements for JSON export")?,
        );
    }

    if config.include_samples {
        if sample_fields.is_empty() {
            root.insert(
                "samples".into(),
                serde_json::to_value(samples)
                    .context("Failed to serialize samples for JSON export")?,
            );
        } else {
            let mut json_samples = Vec::with_capacity(samples.len());
            for sample in samples {
                let mut map = serde_json::Map::new();
                for field in &sample_fields {
                    map.insert(
                        field.header().into(),
                        serde_json::Number::from_f64(field.value(sample))
                            .map(serde_json::Value::Number)
                            .unwrap_or(serde_json::Value::Null),
                    );
                }
                json_samples.push(serde_json::Value::Object(map));
            }
            root.insert("samples".into(), serde_json::Value::Array(json_samples));
        }
    }

    let file = File::create(path)
        .with_context(|| format!("Unable to create JSON file {}", path.display()))?;

    serde_json::to_writer_pretty(file, &serde_json::Value::Object(root))
        .with_context(|| format!("Failed to write JSON payload to {}", path.display()))
}

pub fn write_average_csv(
    path: &Path,
    time: &[f64],
    values: &[f64],
    config: &AverageCsvConfig,
) -> Result<()> {
    if !config.enabled {
        return Ok(());
    }

    if time.len() != values.len() {
        return Err(anyhow!("Average CSV: time and value arrays must match"));
    }

    let fields = if config.fields.is_empty() {
        default_average_csv_fields()
    } else {
        config.fields.clone()
    };

    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }

    let mut writer = csv::Writer::from_path(path)
        .with_context(|| format!("Unable to create CSV file {}", path.display()))?;
    writer.write_record(fields.iter())?;

    for (t, value) in time.iter().zip(values.iter()) {
        let mut row = Vec::with_capacity(fields.len());
        for field in &fields {
            match field.as_str() {
                "time" => row.push(format!("{:.12e}", t)),
                "delta_theta_avg" => row.push(format!("{:.12e}", value)),
                other => {
                    return Err(anyhow!(format!(
                        "Unsupported average CSV field '{}'",
                        other
                    )));
                }
            }
        }
        writer.write_record(row)?;
    }

    writer
        .flush()
        .with_context(|| format!("Failed to flush CSV writer for {}", path.display()))
}

pub fn write_average_json(
    path: &Path,
    time: &[f64],
    values: &[f64],
    config: &AverageJsonConfig,
    metadata: &serde_json::Value,
    orbital: &serde_json::Value,
    lyapunov: Option<&serde_json::Value>,
) -> Result<()> {
    if !config.enabled {
        return Ok(());
    }

    if time.len() != values.len() {
        return Err(anyhow!("Average JSON: time and value arrays must match"));
    }

    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }

    let fields = if config.sample_fields.is_empty() {
        default_average_json_fields()
    } else {
        config.sample_fields.clone()
    };

    let samples: Vec<_> = time
        .iter()
        .zip(values.iter())
        .map(|(t, value)| {
            let mut map = serde_json::Map::new();
            for field in &fields {
                match field.as_str() {
                    "time" => {
                        map.insert("time".into(), serde_json::json!(t));
                    }
                    "delta_theta_avg" => {
                        map.insert("delta_theta_avg".into(), serde_json::json!(value));
                    }
                    other => {
                        map.insert(other.into(), serde_json::Value::Null);
                    }
                }
            }
            serde_json::Value::Object(map)
        })
        .collect();

    let mut root = serde_json::Map::new();

    if config.include_metadata {
        root.insert("metadata".into(), metadata.clone());
    }

    if config.include_orbital {
        root.insert("orbital_elements".into(), orbital.clone());
    }

    if let Some(lyapunov_json) = lyapunov {
        root.insert("lyapunov".into(), lyapunov_json.clone());
    }

    if config.include_samples {
        root.insert("samples".into(), serde_json::Value::Array(samples));
    }

    let file = File::create(path)
        .with_context(|| format!("Unable to create JSON file {}", path.display()))?;

    serde_json::to_writer_pretty(file, &serde_json::Value::Object(root))
        .with_context(|| format!("Failed to write JSON payload to {}", path.display()))
}

pub fn write_velocity_scan_csv(path: &Path, points: &[VelocityScanPoint]) -> Result<()> {
    if points.is_empty() {
        return Ok(());
    }

    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }

    let mut writer = csv::Writer::from_path(path)
        .with_context(|| format!("Unable to create CSV file {}", path.display()))?;

    writer.write_record(&["vy", "eccentricity", "lyapunov"])?;

    for point in points {
        let lyapunov_str = match point.lyapunov {
            Some(value) => format!("{:.12e}", value),
            None => String::from(""),
        };
        writer.write_record(&[
            format!("{:.12e}", point.vy),
            format!("{:.12e}", point.eccentricity),
            lyapunov_str,
        ])?;
    }

    writer
        .flush()
        .with_context(|| format!("Failed to flush CSV writer for {}", path.display()))
}

pub fn write_velocity_scan_json(
    path: &Path,
    points: &[VelocityScanPoint],
    metadata: &SimulationResultMetadata,
    scan: &VelocityScanMetadata,
    lyapunov: &LyapunovSettings,
) -> Result<()> {
    if let Some(parent) = path.parent() {
        ensure_directory(parent)?;
    }

    let mut root = serde_json::Map::new();

    root.insert(
        "simulation".into(),
        serde_json::json!({
            "mu": metadata.mu,
            "dt": metadata.dt,
            "total_time": metadata.total_time,
            "delta_theta0": metadata.delta_theta0,
            "initial_state": {
                "x": metadata.initial_state.x,
                "y": metadata.initial_state.y,
                "vx": metadata.initial_state.vx,
                "vy": metadata.initial_state.vy,
                "theta": metadata.initial_state.theta,
                "omega": metadata.initial_state.omega,
            }
        }),
    );

    root.insert(
        "scan".into(),
        serde_json::json!({
            "vy_min": scan.vy_min,
            "vy_max": scan.vy_max,
            "sample_count": scan.sample_count,
        }),
    );

    root.insert(
        "lyapunov_settings".into(),
        serde_json::json!({
            "burn_in_time": lyapunov.burn_in_time,
            "log_fraction": lyapunov.log_fraction,
        }),
    );

    let samples: Vec<_> = points
        .iter()
        .map(|point| {
            serde_json::json!({
                "vy": point.vy,
                "eccentricity": point.eccentricity,
                "lyapunov": point.lyapunov,
                "lyapunov_diagnostics": {
                    "point_count": point.diagnostics.segment_points,
                    "slope": point.diagnostics.slope,
                    "intercept": point.diagnostics.intercept,
                    "threshold_log10": point.diagnostics.threshold_log10,
                    "segment": {
                        "start_time": point.diagnostics.segment_start_time,
                        "end_time": point.diagnostics.segment_end_time,
                        "start_index": point.diagnostics.segment_start_index,
                        "end_index": point.diagnostics.segment_end_index,
                    }
                },
                "orbital": {
                    "specific_energy": point.orbital.specific_energy,
                    "specific_angular_momentum": point.orbital.specific_angular_momentum,
                    "eccentricity": point.orbital.eccentricity,
                }
            })
        })
        .collect();

    root.insert("samples".into(), serde_json::Value::Array(samples));

    let file = File::create(path)
        .with_context(|| format!("Unable to create JSON file {}", path.display()))?;

    serde_json::to_writer_pretty(file, &serde_json::Value::Object(root))
        .with_context(|| format!("Failed to write JSON payload to {}", path.display()))
}

#[derive(Debug, Clone, Copy)]
enum SampleField {
    Time,
    X,
    Y,
    Vx,
    Vy,
    Radius,
    ThetaPrimary,
    ThetaWrappedPrimary,
    OmegaPrimary,
    ThetaSecondary,
    ThetaWrappedSecondary,
    OmegaSecondary,
    DeltaTheta,
}

impl SampleField {
    fn from_str(field: &str) -> Option<Self> {
        match field {
            "time" => Some(Self::Time),
            "x" => Some(Self::X),
            "y" => Some(Self::Y),
            "vx" => Some(Self::Vx),
            "vy" => Some(Self::Vy),
            "radius" => Some(Self::Radius),
            "theta_primary" => Some(Self::ThetaPrimary),
            "theta_wrapped_primary" => Some(Self::ThetaWrappedPrimary),
            "omega_primary" => Some(Self::OmegaPrimary),
            "theta_secondary" => Some(Self::ThetaSecondary),
            "theta_wrapped_secondary" => Some(Self::ThetaWrappedSecondary),
            "omega_secondary" => Some(Self::OmegaSecondary),
            "delta_theta" => Some(Self::DeltaTheta),
            _ => None,
        }
    }

    fn header(&self) -> &'static str {
        match self {
            Self::Time => "time",
            Self::X => "x",
            Self::Y => "y",
            Self::Vx => "vx",
            Self::Vy => "vy",
            Self::Radius => "radius",
            Self::ThetaPrimary => "theta_primary",
            Self::ThetaWrappedPrimary => "theta_wrapped_primary",
            Self::OmegaPrimary => "omega_primary",
            Self::ThetaSecondary => "theta_secondary",
            Self::ThetaWrappedSecondary => "theta_wrapped_secondary",
            Self::OmegaSecondary => "omega_secondary",
            Self::DeltaTheta => "delta_theta",
        }
    }

    fn value(&self, sample: &SimulationSample) -> f64 {
        match self {
            Self::Time => sample.time,
            Self::X => sample.x,
            Self::Y => sample.y,
            Self::Vx => sample.vx,
            Self::Vy => sample.vy,
            Self::Radius => sample.radius,
            Self::ThetaPrimary => sample.theta_primary,
            Self::ThetaWrappedPrimary => sample.theta_wrapped_primary,
            Self::OmegaPrimary => sample.omega_primary,
            Self::ThetaSecondary => sample.theta_secondary,
            Self::ThetaWrappedSecondary => sample.theta_wrapped_secondary,
            Self::OmegaSecondary => sample.omega_secondary,
            Self::DeltaTheta => sample.delta_theta,
        }
    }

    fn format(&self, sample: &SimulationSample) -> String {
        format!("{:.12e}", self.value(sample))
    }
}

fn parse_sample_fields(fields: &[String]) -> Result<Vec<SampleField>> {
    let mut parsed = Vec::with_capacity(fields.len());
    for field in fields {
        let trimmed = field.trim();
        let sample_field = SampleField::from_str(trimmed)
            .ok_or_else(|| anyhow!("Unsupported sample field '{}' in export config", trimmed))?;
        parsed.push(sample_field);
    }
    Ok(parsed)
}

fn default_average_csv_fields() -> Vec<String> {
    vec!["time".into(), "delta_theta_avg".into()]
}

fn default_average_json_fields() -> Vec<String> {
    vec!["time".into(), "delta_theta_avg".into()]
}
