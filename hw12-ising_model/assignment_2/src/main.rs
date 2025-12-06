use std::{error::Error, fs};

use ising_core::{InitMode, Lattice};
use plotters::prelude::*;
use rand::{rngs::StdRng, SeedableRng};
use serde::Deserialize;

const CONFIG_PATH: &str = "config/assignment_2.toml";
const OUTPUT_DIR: &str = "output/assignment_2";

// Hard-coded physics knobs per assignment instructions.
const TEMPERATURES: [f64; 3] = [100.0, 30.0, 10.0];
const FIELD_MIN: f64 = -10.0;
const FIELD_MAX: f64 = 10.0;
const FIELD_STEP: f64 = 1.0;

#[derive(Deserialize)]
struct Config {
    lattice_size: usize,
    equilibration_sweeps: usize,
    measurement_sweeps: usize,
    seed: u64,
    init_mode: InitModeConfig,
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

    // Independent RNG streams for each temperature improve reproducibility.
    let datasets: Vec<_> = TEMPERATURES
        .iter()
        .enumerate()
        .map(|(i, &t)| {
            let mut rng = StdRng::seed_from_u64(config.seed.wrapping_add(17 * (i as u64 + 1)));
            let fields = field_grid(FIELD_MIN, FIELD_MAX, FIELD_STEP);
            let values = simulate_field_scan(&config, t, &fields, &mut rng);
            (t, values)
        })
        .collect();

    for (temp, data) in &datasets {
        plot_magnetization_vs_field(*temp, data)?;
    }
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
    Ok(cfg)
}

fn field_grid(min: f64, max: f64, step: f64) -> Vec<f64> {
    let mut values = Vec::new();
    let mut h = min;
    while h <= max + 1e-12 {
        values.push(h);
        h += step;
    }
    values
}

/// Scan external field H at fixed temperature T. Magnetization is averaged over measurement sweeps.
fn simulate_field_scan(
    config: &Config,
    temperature: f64,
    fields: &[f64],
    rng: &mut StdRng,
) -> Vec<(f64, f64)> {
    let mut data = Vec::with_capacity(fields.len());
    for &h in fields {
        // New lattice each field value to isolate responses.
        let mut lattice = Lattice::new(config.lattice_size, config.init_mode.into(), rng);
        for _ in 0..config.equilibration_sweeps {
            lattice.sweep(rng, temperature, h);
        }
        let mut m_acc = 0.0;
        for _ in 0..config.measurement_sweeps {
            lattice.sweep(rng, temperature, h);
            m_acc += lattice.magnetization_per_spin();
        }
        let m_avg = m_acc / (config.measurement_sweeps as f64);
        data.push((h, m_avg));
    }
    data
}

fn plot_magnetization_vs_field(temp: f64, data: &[(f64, f64)]) -> Result<(), Box<dyn Error>> {
    let path = format!("{OUTPUT_DIR}/magnetization_vs_field_T{:.0}.png", temp);
    let root = BitMapBackend::new(&path, (900, 600)).into_drawing_area();
    root.fill(&WHITE)?;

    let h_min = FIELD_MIN;
    let h_max = FIELD_MAX;

    // Mean-field reference m = tanh(H / T) at this temperature.
    let theory: Vec<(f64, f64)> = (0..200)
        .map(|i| {
            let frac = i as f64 / 199.0;
            let h = FIELD_MIN + frac * (FIELD_MAX - FIELD_MIN);
            (h, (h / temp).tanh())
        })
        .collect();

    // Determine y-limits from both Monte Carlo and theory curves.
    let mut m_min = f64::INFINITY;
    let mut m_max = f64::NEG_INFINITY;
    for &(_, m) in data {
        m_min = m_min.min(m);
        m_max = m_max.max(m);
    }
    for &(_, m) in &theory {
        m_min = m_min.min(m);
        m_max = m_max.max(m);
    }
    let padding = 0.1 * (m_max.abs().max(m_min.abs()).max(1e-6));
    let y_low = m_min - padding;
    let y_high = m_max + padding;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("Magnetization vs Field (T = {:.0})", temp),
            ("sans-serif", 28),
        )
        .margin(20)
        .x_label_area_size(50)
        .y_label_area_size(60)
        .build_cartesian_2d(h_min..h_max, y_low..y_high)?;

    chart
        .configure_mesh()
        .x_desc("External field H")
        .y_desc("Magnetization per spin m")
        .label_style(("sans-serif", 16))
        .draw()?;

    chart
        .draw_series(LineSeries::new(
            data.iter().cloned(),
            ShapeStyle::from(&BLUE.mix(0.8)).stroke_width(3),
        ))?
        .label("Monte Carlo")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .draw_series(LineSeries::new(
            theory.into_iter(),
            ShapeStyle::from(&RED.mix(0.8)).stroke_width(2),
        ))?
        .label("Mean-field (m = tanh(H/T))")
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
