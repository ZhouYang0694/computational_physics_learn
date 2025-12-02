use hw11_random_systems::output::RunContext;
use hw11_random_systems::stats::{ExponentialFit, exponential_fit};
use plotters::prelude::*;
use rand::SeedableRng;
use rand::prelude::*;
use serde::Deserialize;
use std::error::Error;
use std::fs;
use std::path::Path;

fn main() -> Result<(), Box<dyn Error>> {
    let config = load_diffusion_config()?;
    let ctx = RunContext::new("coffee")?;
    let result = simulate_diffusion(&config);
    write_outputs(&ctx, &config, &result)?;
    Ok(())
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
struct DiffusionConfig {
    size: usize,
    hole_length: usize,
    max_steps: usize,
    molecules: usize,
    initial_block: usize,
    seed: u64,
    snapshot_interval: usize,
}

impl Default for DiffusionConfig {
    fn default() -> Self {
        Self {
            size: 50,
            hole_length: 10,
            max_steps: 400,
            molecules: 800,
            initial_block: 10,
            seed: 7,
            snapshot_interval: 50,
        }
    }
}

fn load_diffusion_config() -> Result<DiffusionConfig, Box<dyn Error>> {
    let path = Path::new("config/coffee.toml");
    if path.exists() {
        let text = fs::read_to_string(path)?;
        let cfg: DiffusionConfig = toml::from_str(&text)?;
        Ok(cfg)
    } else {
        Ok(DiffusionConfig::default())
    }
}

struct DiffusionResult {
    history: Vec<(usize, usize)>,
    escaped: usize,
    fit: Option<ExponentialFit>,
    snapshots: Vec<Snapshot>,
}

struct Snapshot {
    step: usize,
    positions: Vec<(i32, i32)>,
}

fn simulate_diffusion(config: &DiffusionConfig) -> DiffusionResult {
    let mut rng = StdRng::seed_from_u64(config.seed);
    let mut molecules = initialize_molecules(config, &mut rng);
    let mut history = Vec::new();
    let mut total_escaped = 0usize;
    let mut snapshots = Vec::new();

    record_snapshot(0, &molecules, config.snapshot_interval, &mut snapshots);

    history.push((0, molecules.len()));
    for step in 1..=config.max_steps {
        let escaped = diffuse_step(&mut molecules, config, &mut rng);
        total_escaped += escaped;
        history.push((step, molecules.len()));
        record_snapshot(step, &molecules, config.snapshot_interval, &mut snapshots);
        if molecules.is_empty() {
            break;
        }
    }

    let fit = prepare_exponential_fit(&history);
    DiffusionResult {
        history,
        escaped: total_escaped,
        fit,
        snapshots,
    }
}

fn record_snapshot(
    step: usize,
    molecules: &[(i32, i32)],
    interval: usize,
    snapshots: &mut Vec<Snapshot>,
) {
    if interval == 0 {
        return;
    }
    if step % interval == 0 {
        snapshots.push(Snapshot {
            step,
            positions: molecules.to_vec(),
        });
    }
}

fn initialize_molecules<R: Rng>(config: &DiffusionConfig, rng: &mut R) -> Vec<(i32, i32)> {
    let start = (config.size / 2).saturating_sub(config.initial_block / 2) as i32;
    let mut positions = Vec::with_capacity(config.molecules);
    for _ in 0..config.molecules {
        let x = start + rng.gen_range(0..config.initial_block) as i32;
        let y = start + rng.gen_range(0..config.initial_block) as i32;
        positions.push((x, y));
    }
    positions
}

fn diffuse_step<R: Rng>(
    molecules: &mut Vec<(i32, i32)>,
    config: &DiffusionConfig,
    rng: &mut R,
) -> usize {
    let mut escaped = 0;
    let mut next_positions = Vec::with_capacity(molecules.len());
    for &pos in molecules.iter() {
        if let Some(p) = move_one(pos, config, rng) {
            next_positions.push(p);
        } else {
            escaped += 1;
        }
    }
    *molecules = next_positions;
    escaped
}

fn move_one<R: Rng>(pos: (i32, i32), config: &DiffusionConfig, rng: &mut R) -> Option<(i32, i32)> {
    let dir = rng.gen_range(0..4);
    let (dx, dy) = match dir {
        0 => (1, 0),
        1 => (-1, 0),
        2 => (0, 1),
        _ => (0, -1),
    };
    let mut nx = pos.0 + dx;
    let mut ny = pos.1 + dy;
    let max = config.size as i32 - 1;
    let mid = config.size as i32 / 2;
    let hole_start = mid - config.hole_length as i32 / 2;
    let hole_end = hole_start + config.hole_length as i32 - 1;

    // Reflect at solid boundaries; only the hole lets particles escape.
    if nx > max {
        if pos.0 == max && pos.1 >= hole_start && pos.1 <= hole_end {
            return None;
        }
        nx = pos.0 - dx;
    } else if nx < 0 {
        nx = pos.0 - dx;
    }

    if ny > max || ny < 0 {
        ny = pos.1 - dy;
    }

    nx = nx.clamp(0, max);
    ny = ny.clamp(0, max);
    Some((nx, ny))
}

fn prepare_exponential_fit(history: &[(usize, usize)]) -> Option<ExponentialFit> {
    let mut times = Vec::new();
    let mut counts = Vec::new();
    for (t, n) in history.iter() {
        if *n > 0 {
            times.push(*t as f64);
            counts.push(*n as f64);
        }
    }
    exponential_fit(&times, &counts)
}

fn write_outputs(
    ctx: &RunContext,
    config: &DiffusionConfig,
    result: &DiffusionResult,
) -> Result<(), Box<dyn Error>> {
    let plot_path = ctx.file_path("n_vs_t.png");
    plot_history(&result.history, result.fit, &plot_path)?;

    let mut rows = Vec::new();
    for (t, n) in &result.history {
        rows.push(vec![t.to_string(), n.to_string()]);
    }
    ctx.write_csv("history.csv", &["time_step", "molecules_inside"], &rows)?;

    if !result.snapshots.is_empty() {
        write_snapshots(ctx, config.size, &result.snapshots)?;
    }

    let mut log_lines = vec![
        "Coffee cup diffusion with escape hole".to_string(),
        format!("grid_size = {}", config.size),
        format!("hole_length = {}", config.hole_length),
        format!("initial_molecules = {}", config.molecules),
        format!("simulation_steps = {}", config.max_steps),
        format!("escaped_molecules = {}", result.escaped),
        format!(
            "remaining_inside = {}",
            result.history.last().map(|(_, n)| *n).unwrap_or(0)
        ),
        format!("snapshot_interval = {}", config.snapshot_interval),
        format!("snapshots_saved = {}", result.snapshots.len()),
    ];
    if let Some(fit) = result.fit {
        log_lines.push(format!("fit_tau = {:.3}", fit.tau));
        log_lines.push(format!("fit_n0 = {:.3}", fit.n0));
        log_lines.push(format!("fit_r2 = {:.4}", fit.r2));
    } else {
        log_lines.push("fit_failed = insufficient data".to_string());
    }
    ctx.write_lines("simulation.log", &log_lines)?;
    Ok(())
}

fn write_snapshots(
    ctx: &RunContext,
    size: usize,
    snapshots: &[Snapshot],
) -> Result<(), Box<dyn Error>> {
    if snapshots.is_empty() {
        return Ok(());
    }
    let dir = ctx.file_path("snapshots");
    fs::create_dir_all(&dir)?;
    for snap in snapshots {
        let file_path = dir.join(format!("snapshot_{:05}.png", snap.step));
        render_snapshot(size, &snap.positions, &file_path)?;
    }
    Ok(())
}

fn render_snapshot(
    size: usize,
    positions: &[(i32, i32)],
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    let mut counts = vec![0u32; size * size];
    for &(x, y) in positions {
        if x >= 0 && x < size as i32 && y >= 0 && y < size as i32 {
            counts[grid_index(size, x as usize, y as usize)] += 1;
        }
    }
    let max_count = counts.iter().copied().max().unwrap_or(0).max(1) as f64;

    let dim = 600;
    let area = BitMapBackend::new(path, (dim, dim)).into_drawing_area();
    area.fill(&BLACK)?;
    let (width, height) = area.dim_in_pixel();
    let cell_w = width as f64 / size as f64;
    let cell_h = height as f64 / size as f64;

    for y in 0..size {
        for x in 0..size {
            let count = counts[grid_index(size, x, y)];
            if count == 0 {
                continue;
            }
            let intensity = (count as f64 / max_count).min(1.0);
            let color = RGBColor(
                (50.0 + 200.0 * intensity) as u8,
                (30.0 + 80.0 * (1.0 - intensity)) as u8,
                (120.0 + 100.0 * (1.0 - intensity)) as u8,
            );
            let x0 = (x as f64 * cell_w).round() as i32;
            let x1 = ((x + 1) as f64 * cell_w).round() as i32;
            let y0 = (y as f64 * cell_h).round() as i32;
            let y1 = ((y + 1) as f64 * cell_h).round() as i32;
            area.draw(&Rectangle::new([(x0, y0), (x1, y1)], color.filled()))?;
        }
    }
    area.present()?;
    Ok(())
}

fn grid_index(size: usize, x: usize, y: usize) -> usize {
    y * size + x
}

fn plot_history(
    history: &[(usize, usize)],
    fit: Option<ExponentialFit>,
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    let max_t = history.last().map(|(t, _)| *t).unwrap_or(0).max(1);
    let max_n = history.iter().map(|(_, n)| *n).max().unwrap_or(1);
    let y_max = (max_n as f64 * 1.1).max(1.0);

    let area = BitMapBackend::new(path, (900, 600)).into_drawing_area();
    area.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&area)
        .caption("Molecules remaining inside", ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(50)
        .y_label_area_size(60)
        .build_cartesian_2d(0..max_t, 0f64..y_max)?;

    chart
        .configure_mesh()
        .x_desc("time step")
        .y_desc("molecules inside")
        .draw()?;

    chart
        .draw_series(LineSeries::new(
            history.iter().map(|(t, n)| (*t, *n as f64)),
            ShapeStyle::from(&BLUE).stroke_width(2),
        ))?
        .label("simulation")
        .legend(|(x, y)| PathElement::new([(x - 10, y), (x + 10, y)], &BLUE));

    chart.draw_series(
        history
            .iter()
            .map(|(t, n)| Circle::new((*t, *n as f64), 3, BLUE.filled())),
    )?;

    if let Some(fit) = fit {
        let mut series = Vec::with_capacity(max_t + 1);
        for t in 0..=max_t {
            let y = fit.n0 * (-(t as f64) / fit.tau).exp();
            series.push((t, y));
        }
        chart
            .draw_series(LineSeries::new(
                series,
                ShapeStyle::from(&RED).stroke_width(2),
            ))?
            .label("exp fit")
            .legend(|(x, y)| PathElement::new([(x - 10, y), (x + 10, y)], &RED));
    }

    chart
        .configure_series_labels()
        .border_style(&BLACK)
        .draw()?;
    area.present()?;
    Ok(())
}
