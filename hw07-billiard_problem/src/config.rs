use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use serde::Deserialize;
use std::ffi::OsStr;
use std::path::Component;

/// Load and validate simulation configuration from a TOML file.
pub struct ConfigLoader;

impl ConfigLoader {
    pub fn load_from_path(path: &Path) -> Result<AppConfig> {
        let raw = fs::read_to_string(path)
            .with_context(|| format!("failed to read configuration file {}", path.display()))?;
        let sim: SimulationConfig = toml::from_str(&raw)
            .with_context(|| format!("failed to parse configuration file {}", path.display()))?;

        sim.validate()
            .with_context(|| format!("invalid configuration in {}", path.display()))?;

        let output_dir = sim
            .output
            .resolve_output_dir(path)
            .context("output directory validation failed")?;

        Ok(AppConfig {
            source_path: path.to_path_buf(),
            simulation: sim,
            resolved_output_dir: output_dir,
        })
    }
}

/// Wrapper that holds both the parsed simulation configuration and derived paths.
#[derive(Debug, Clone)]
pub struct AppConfig {
    pub source_path: PathBuf,
    pub simulation: SimulationConfig,
    pub resolved_output_dir: PathBuf,
}

impl AppConfig {
    /// Human friendly description of key configuration choices.
    pub fn summary_lines(&self) -> Vec<String> {
        let boundary = match &self.simulation.boundary {
            BoundaryConfig::Polygon { vertices } => {
                format!("boundary: polygon with {} vertices", vertices.len())
            }
            BoundaryConfig::Stadium { radius, alpha } => {
                format!("boundary: stadium r={radius}, alpha={alpha}")
            }
            BoundaryConfig::Bezier { segments } => {
                format!("boundary: bezier with {} segments", segments.len())
            }
        };
        let detector = format!(
            "detector: {:?} (step={:?}, tol={})",
            self.simulation.detector.mode,
            self.simulation.detector.step_size,
            self.simulation.detector.tolerance
        );
        let initial = format!(
            "initial state: pos=({}, {}), vel=({}, {})",
            self.simulation.initial_state.position[0],
            self.simulation.initial_state.position[1],
            self.simulation.initial_state.velocity[0],
            self.simulation.initial_state.velocity[1],
        );
        let termination = format!(
            "termination: max_time={:?}, max_collisions={:?}",
            self.simulation.termination.max_time, self.simulation.termination.max_collisions,
        );
        let sampling = format!(
            "sampling: poincare_y={}, eps={}",
            self.simulation.sampling.poincare_plane_y, self.simulation.sampling.zero_cross_eps
        );
        let outputs = format!(
            "output dir: {} (csv={}, png={}, svg={})",
            self.resolved_output_dir.display(),
            self.simulation.output.export_csv,
            self.simulation.output.export_png,
            self.simulation.output.export_svg
        );

        vec![boundary, detector, initial, termination, sampling, outputs]
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct BezierSegmentConfig {
    pub start: [f64; 2],
    pub ctrl1: [f64; 2],
    pub ctrl2: [f64; 2],
    pub end: [f64; 2],
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum BoundaryConfig {
    Polygon { vertices: Vec<[f64; 2]> },
    Stadium { radius: f64, alpha: f64 },
    Bezier { segments: Vec<BezierSegmentConfig> },
}

#[derive(Debug, Clone, Deserialize)]
pub struct InitialStateConfig {
    pub position: [f64; 2],
    pub velocity: [f64; 2],
}

#[derive(Debug, Clone, Deserialize)]
pub struct DetectorConfig {
    #[serde(default)]
    pub mode: DetectorMode,

    /// Optional step size for step-back detector.
    #[serde(default)]
    pub step_size: Option<f64>,

    /// Root-finding or refinement tolerance.
    #[serde(default = "DetectorConfig::default_tolerance")]
    pub tolerance: f64,
}

impl DetectorConfig {
    const fn default_tolerance() -> f64 {
        1e-6
    }
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            mode: DetectorMode::default(),
            step_size: None,
            tolerance: Self::default_tolerance(),
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DetectorMode {
    StepBack,
    Analytic,
}

impl Default for DetectorMode {
    fn default() -> Self {
        DetectorMode::StepBack
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct TerminationConfig {
    pub max_time: Option<f64>,
    pub max_collisions: Option<u64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SamplingConfig {
    pub poincare_plane_y: f64,
    #[serde(default = "SamplingConfig::default_zero_cross_eps")]
    pub zero_cross_eps: f64,
}

impl SamplingConfig {
    const fn default_zero_cross_eps() -> f64 {
        1e-9
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct PlotConfig {
    #[serde(default = "PlotConfig::default_width")]
    pub width_px: u32,
    #[serde(default = "PlotConfig::default_height")]
    pub height_px: u32,
    #[serde(default = "PlotConfig::default_line_width")]
    pub line_width: f64,
}

impl PlotConfig {
    const fn default_width() -> u32 {
        1200
    }
    const fn default_height() -> u32 {
        900
    }
    const fn default_line_width() -> f64 {
        2.0
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct OutputConfig {
    pub directory: PathBuf,
    pub base_name: String,
    pub export_csv: bool,
    pub export_png: bool,
    pub export_svg: bool,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            directory: PathBuf::from("output"),
            base_name: "billiard".to_string(),
            export_csv: true,
            export_png: true,
            export_svg: true,
        }
    }
}

impl OutputConfig {
    fn resolve_output_dir(&self, _config_path: &Path) -> Result<PathBuf> {
        let dir = if self.directory.is_absolute() {
            self.directory.clone()
        } else {
            let mut sanitized = PathBuf::new();
            for component in self.directory.components() {
                match component {
                    Component::CurDir => {}
                    Component::ParentDir => {
                        return Err(anyhow!(
                            "output directory cannot contain parent references ('..'): {}",
                            self.directory.display()
                        ));
                    }
                    Component::Normal(part) => sanitized.push(part),
                    Component::Prefix(_) | Component::RootDir => {
                        return Err(anyhow!(
                            "unexpected path prefix in output directory: {}",
                            self.directory.display()
                        ));
                    }
                }
            }

            if sanitized.as_os_str().is_empty() {
                PathBuf::from("output")
            } else {
                let mut parts = sanitized.components();
                let prefixed = match parts.next() {
                    Some(Component::Normal(first)) if first == OsStr::new("output") => sanitized,
                    Some(Component::Normal(_)) => PathBuf::from("output").join(&sanitized),
                    Some(Component::CurDir) | Some(Component::ParentDir) => unreachable!(),
                    None => PathBuf::from("output"),
                    Some(Component::Prefix(_)) | Some(Component::RootDir) => unreachable!(),
                };
                prefixed
            }
        };

        let normalized = dir.components().collect::<Vec<_>>();

        if normalized.is_empty() {
            return Err(anyhow!("output directory resolves to an empty path"));
        }

        // Ensure output path stays under top-level "output"
        if normalized.first().and_then(|c| c.as_os_str().to_str()) != Some("output") {
            return Err(anyhow!(
                "output directory must be within the project 'output' folder; got {}",
                dir.display()
            ));
        }

        Ok(dir)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct SimulationConfig {
    pub boundary: BoundaryConfig,
    pub initial_state: InitialStateConfig,
    #[serde(default)]
    pub detector: DetectorConfig,
    pub termination: TerminationConfig,
    pub sampling: SamplingConfig,
    pub plot: PlotConfig,
    #[serde(default)]
    pub output: OutputConfig,
}

impl SimulationConfig {
    fn validate(&self) -> Result<()> {
        self.validate_boundary()?;
        self.validate_initial_state()?;
        self.validate_detector()?;
        Ok(())
    }

    fn validate_boundary(&self) -> Result<()> {
        match &self.boundary {
            BoundaryConfig::Polygon { vertices } => {
                if vertices.len() < 3 {
                    return Err(anyhow!(
                        "polygon boundary requires at least 3 vertices, got {}",
                        vertices.len()
                    ));
                }
            }
            BoundaryConfig::Stadium { radius, alpha } => {
                if *radius <= 0.0 {
                    return Err(anyhow!(
                        "stadium boundary radius must be positive, got {radius}"
                    ));
                }
                if *alpha < 0.0 {
                    return Err(anyhow!(
                        "stadium boundary alpha must be non-negative, got {alpha}"
                    ));
                }
            }
            BoundaryConfig::Bezier { segments } => {
                if segments.is_empty() {
                    return Err(anyhow!(
                        "bezier boundary requires at least one cubic segment"
                    ));
                }

                let mut prev_end = segments[0].end;
                let first_start = segments[0].start;
                for seg in segments.iter().skip(1) {
                    if distance_sq(prev_end, seg.start) > 1e-9 {
                        return Err(anyhow!(
                            "bezier segments must form a continuous chain: end {:?} does not match next start {:?}",
                            prev_end,
                            seg.start
                        ));
                    }
                    prev_end = seg.end;
                }
                if distance_sq(prev_end, first_start) > 1e-9 {
                    return Err(anyhow!(
                        "bezier segments must form a closed loop; last end {:?} does not match first start {:?}",
                        prev_end,
                        first_start
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_initial_state(&self) -> Result<()> {
        let vx = self.initial_state.velocity[0];
        let vy = self.initial_state.velocity[1];
        let speed_sq = vx * vx + vy * vy;
        if speed_sq <= f64::EPSILON {
            return Err(anyhow!(
                "initial velocity magnitude must be positive; received ({vx}, {vy})"
            ));
        }
        Ok(())
    }

    fn validate_detector(&self) -> Result<()> {
        if matches!(self.detector.mode, DetectorMode::StepBack) {
            let step = self
                .detector
                .step_size
                .ok_or_else(|| anyhow!("step-back detector requires `step_size`"))?;
            if step <= 0.0 {
                return Err(anyhow!(
                    "step-back detector step size must be positive, got {step}"
                ));
            }
        }
        if self.detector.tolerance <= 0.0 {
            return Err(anyhow!(
                "detector tolerance must be positive, got {}",
                self.detector.tolerance
            ));
        }
        Ok(())
    }
}

fn distance_sq(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    dx * dx + dy * dy
}
