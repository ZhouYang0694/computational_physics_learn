use std::{error::Error, fs};

use ising_core::{InitMode, Lattice};
use plotters::prelude::*;
use rand::{rngs::StdRng, SeedableRng};
use serde::Deserialize;

const CONFIG_PATH: &str = "config/assignment_1.toml";
const OUTPUT_DIR: &str = "output/assignment_1";
const MT_FIG: &str = "output/assignment_1/magnetization_vs_temperature.png";
const MC_FIG: &str = "output/assignment_1/magnetization_vs_steps.png";
const M8_FIG: &str = "output/assignment_1/m8_vs_temperature.png";
const T_CRITICAL: f64 = 2.269; // Requested Tc reference (J = 1, kB = 1)
const T_MONITOR: f64 = 2.269; // Track relaxation near criticality

#[derive(Deserialize)]
struct Config {
    lattice_size: usize,
    equilibration_sweeps: usize,
    measurement_sweeps: usize,
    seed: u64,
    init_mode: InitModeConfig,
    temperatures: Vec<f64>,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum InitModeConfig {
    AllUp,
    Random,
}

impl From<InitModeConfig> for InitMode {
    fn from(value: InitModeConfig) -> Self {
        match value {
            InitModeConfig::AllUp => InitMode::AllUp,
            InitModeConfig::Random => InitMode::Random,
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let config = load_config(CONFIG_PATH)?;
    fs::create_dir_all(OUTPUT_DIR)?;

    // Use separate RNG streams to keep plots reproducible but independent.
    let mut rng_mt = StdRng::seed_from_u64(config.seed);
    let mut rng_monitor = StdRng::seed_from_u64(config.seed.wrapping_add(1));

    let mt_data = simulate_magnetization_vs_temperature(&config, &mut rng_mt);
    plot_magnetization_vs_temperature(&mt_data)?;
    plot_m8_vs_temperature(&mt_data)?;

    let mc_trace = simulate_relaxation_trace(&config, &mut rng_monitor);
    plot_magnetization_trace(&mc_trace)?;

    Ok(())
}

fn load_config(path: &str) -> Result<Config, Box<dyn Error>> {
    let content =
        fs::read_to_string(path).map_err(|e| format!("Failed to read config at {path}: {e}"))?;
    let cfg: Config = toml::from_str(&content)
        .map_err(|e| format!("Failed to parse TOML config at {path}: {e}"))?;
    if cfg.lattice_size == 0 {
        return Err("Config must set lattice_size > 0".into());
    }
    if cfg.equilibration_sweeps == 0 || cfg.measurement_sweeps == 0 {
        return Err("Config must set sweeps > 0".into());
    }
    if cfg.temperatures.is_empty() {
        return Err("Config must provide at least one temperature".into());
    }
    Ok(cfg)
}

/// Run independent simulations for each temperature to build the M-T curve.
fn simulate_magnetization_vs_temperature(config: &Config, rng: &mut StdRng) -> Vec<(f64, f64)> {
    let mut data = Vec::with_capacity(config.temperatures.len());
    for &t in &config.temperatures {
        let mut lattice = Lattice::new(config.lattice_size, config.init_mode.into(), rng);

        for _ in 0..config.equilibration_sweeps {
            lattice.sweep(rng, t, 0.0);
        }

        let mut m_acc = 0.0;
        for _ in 0..config.measurement_sweeps {
            lattice.sweep(rng, t, 0.0);
            // Use |m| so spontaneous flips do not average to zero below Tc.
            m_acc += lattice.magnetization_per_spin().abs();
        }
        let m_avg = m_acc / (config.measurement_sweeps as f64);
        data.push((t, m_avg));
    }
    data
}

/// Track magnetization versus Monte Carlo steps near Tc to illustrate relaxation/equilibration.
fn simulate_relaxation_trace(config: &Config, rng: &mut StdRng) -> Vec<(usize, f64)> {
    let mut lattice = Lattice::new(config.lattice_size, config.init_mode.into(), rng);
    let total_steps = config.equilibration_sweeps + config.measurement_sweeps;
    let mut trace = Vec::with_capacity(total_steps);
    for step in 0..total_steps {
        lattice.sweep(rng, T_MONITOR, 0.0);
        trace.push((step, lattice.magnetization_per_spin()));
    }
    trace
}

fn plot_magnetization_vs_temperature(data: &[(f64, f64)]) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new(MT_FIG, (900, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut ordered = data.to_vec();
    ordered.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let t_min = ordered
        .iter()
        .map(|(t, _)| *t)
        .fold(f64::INFINITY, f64::min);
    let t_max = ordered
        .iter()
        .map(|(t, _)| *t)
        .fold(f64::NEG_INFINITY, f64::max);
    let m_max = data
        .iter()
        .map(|(_, m)| *m)
        .fold(f64::NEG_INFINITY, f64::max)
        .max(1e-6); // avoid zero height

    let mut chart = ChartBuilder::on(&root)
        .caption("2D Ising: Magnetization vs Temperature", ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(50)
        .y_label_area_size(60)
        .build_cartesian_2d(t_min..t_max, 0.0..m_max)?;

    chart
        .configure_mesh()
        .x_desc("Temperature T")
        .y_desc("Magnetization per spin |m|")
        .label_style(("sans-serif", 16))
        .draw()?;

    chart
        .draw_series(LineSeries::new(
            ordered.iter().cloned(),
            ShapeStyle::from(&BLUE.mix(0.8)).stroke_width(3),
        ))?
        .label("Monte Carlo")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    // Mark the theoretical critical temperature.
    chart.draw_series(LineSeries::new(
        vec![(T_CRITICAL, 0.0), (T_CRITICAL, m_max)],
        ShapeStyle::from(&RED.mix(0.8)).stroke_width(2),
    ))?;

    chart
        .configure_series_labels()
        .border_style(&BLACK)
        .background_style(&WHITE.mix(0.8))
        .label_font(("sans-serif", 16))
        .draw()?;

    root.present()?;
    Ok(())
}

/// Plot M^8 vs T with a linear fit (useful for estimating critical exponent behavior).
fn plot_m8_vs_temperature(data: &[(f64, f64)]) -> Result<(), Box<dyn Error>> {
    // Restrict to requested window 2.0 <= T <= 2.26.
    let mut ordered: Vec<(f64, f64)> = data
        .iter()
        .copied()
        .filter(|(t, _)| *t >= 2.0 && *t <= 2.26)
        .collect();
    ordered.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let transformed: Vec<(f64, f64)> = ordered.iter().map(|(t, m)| (*t, m.powi(8))).collect();

    if transformed.len() < 2 {
        return Err("Need at least two points with 2.0 <= T <= 2.6 to plot M^8 vs T".into());
    }

    let (slope, intercept, r2) = linear_regression(&transformed)?;
    println!(
        "M^8 vs T fit: slope = {:.6}, intercept = {:.6}, R^2 = {:.4}",
        slope, intercept, r2
    );

    // Fit line over the T-range
    let t_min = transformed.first().unwrap().0;
    let t_max = transformed.last().unwrap().0;
    let fit_line = vec![
        (t_min, slope * t_min + intercept),
        (t_max, slope * t_max + intercept),
    ];

    let y_min = transformed
        .iter()
        .map(|(_, y)| *y)
        .fold(f64::INFINITY, f64::min)
        .min(
            fit_line
                .iter()
                .map(|(_, y)| *y)
                .fold(f64::INFINITY, f64::min),
        );
    let y_max = transformed
        .iter()
        .map(|(_, y)| *y)
        .fold(f64::NEG_INFINITY, f64::max)
        .max(
            fit_line
                .iter()
                .map(|(_, y)| *y)
                .fold(f64::NEG_INFINITY, f64::max),
        );
    let y_span = (y_max - y_min).abs().max(1e-9);

    let root = BitMapBackend::new(M8_FIG, (900, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("M^8 vs Temperature", ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(60)
        .y_label_area_size(70)
        .build_cartesian_2d(
            t_min..t_max,
            (y_min - 0.05 * y_span)..(y_max + 0.05 * y_span),
        )?;

    chart
        .configure_mesh()
        .x_desc("Temperature T")
        .y_desc("M^8")
        .label_style(("sans-serif", 16))
        .draw()?;

    chart.draw_series(PointSeries::of_element(
        transformed.clone(),
        5,
        &BLUE,
        &|coord, size, style| EmptyElement::at(coord) + Circle::new((0, 0), size, style.filled()),
    ))?;

    chart
        .draw_series(LineSeries::new(
            fit_line.iter().cloned(),
            ShapeStyle::from(&RED.mix(0.8)).stroke_width(2),
        ))?
        .label(format!("fit: slope={:.6}", slope))
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .border_style(&BLACK)
        .background_style(&WHITE.mix(0.8))
        .label_font(("sans-serif", 16))
        .draw()?;

    root.present()?;
    Ok(())
}

/// Transform to (ln(Tc - T), ln M) for T < Tc and perform linear fit to extract critical exponent beta.
/// Simple least-squares linear regression returning (slope, intercept, R^2).
fn linear_regression(data: &[(f64, f64)]) -> Result<(f64, f64, f64), Box<dyn Error>> {
    let n = data.len() as f64;
    if n < 2.0 {
        return Err("Need at least two points for regression".into());
    }
    let (sum_x, sum_y, sum_xx, sum_xy, sum_yy) = data.iter().fold(
        (0.0, 0.0, 0.0, 0.0, 0.0),
        |(sx, sy, sxx, sxy, syy), (x, y)| (sx + x, sy + y, sxx + x * x, sxy + x * y, syy + y * y),
    );
    let mean_x = sum_x / n;
    let mean_y = sum_y / n;
    let denom = sum_xx - n * mean_x * mean_x;
    if denom.abs() < 1e-12 {
        return Err("Degenerate regression (zero variance in x)".into());
    }
    let slope = (sum_xy - n * mean_x * mean_y) / denom;
    let intercept = mean_y - slope * mean_x;

    // Compute R^2.
    let ss_tot = sum_yy - n * mean_y * mean_y;
    let ss_res = data
        .iter()
        .map(|(x, y)| {
            let y_fit = slope * *x + intercept;
            (y - y_fit).powi(2)
        })
        .sum::<f64>();
    let r2 = 1.0 - ss_res / ss_tot.max(1e-12);

    Ok((slope, intercept, r2))
}

fn plot_magnetization_trace(trace: &[(usize, f64)]) -> Result<(), Box<dyn Error>> {
    let root = BitMapBackend::new(MC_FIG, (900, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let max_step = trace.last().map(|(s, _)| *s).unwrap_or(0);
    let x_max = if max_step == 0 { 1 } else { max_step };
    let m_min = trace.iter().map(|(_, m)| *m).fold(f64::INFINITY, f64::min);
    let m_max = trace
        .iter()
        .map(|(_, m)| *m)
        .fold(f64::NEG_INFINITY, f64::max);
    let span = (m_max - m_min).abs().max(1e-6);
    let y_low = m_min - 0.05 * span;
    let y_high = m_max + 0.05 * span;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("Magnetization vs MC sweeps at T = {:.3}", T_MONITOR),
            ("sans-serif", 28),
        )
        .margin(20)
        .x_label_area_size(60)
        .y_label_area_size(60)
        .build_cartesian_2d(0usize..x_max, y_low..y_high)?;

    chart
        .configure_mesh()
        .x_desc("Monte Carlo sweeps")
        .y_desc("Magnetization per spin m")
        .label_style(("sans-serif", 16))
        .draw()?;

    chart.draw_series(LineSeries::new(
        trace.iter().map(|(s, m)| (*s, *m)),
        ShapeStyle::from(&GREEN.mix(0.9)).stroke_width(2),
    ))?;

    root.present()?;
    Ok(())
}
