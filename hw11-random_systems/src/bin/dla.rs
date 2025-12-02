use hw11_random_systems::output::RunContext;
use hw11_random_systems::stats::{LinearFit, linear_regression};
use plotters::prelude::*;
use rand::SeedableRng;
use rand::prelude::*;
use serde::Deserialize;
use std::error::Error;
use std::f64::consts::PI;
use std::fs;
use std::path::Path;

fn main() -> Result<(), Box<dyn Error>> {
    let config = load_dla_config()?;
    let ctx = RunContext::new("dla")?;
    let result = simulate_dla(&config);
    write_outputs(&ctx, &config, &result)?;
    Ok(())
}

#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
struct DlaConfig {
    grid_size: usize,
    num_particles: usize,
    step_limit: usize,
    spawn_margin: f64,
    kill_margin: f64,
    seed: u64,
    fit_burn_in_fraction: f64,
    fit_burn_out_fraction: f64,
}

impl Default for DlaConfig {
    fn default() -> Self {
        Self {
            grid_size: 401,
            num_particles: 3500,
            step_limit: 20_000,
            spawn_margin: 4.0,
            kill_margin: 12.0,
            seed: 42,
            fit_burn_in_fraction: 0.15,
            fit_burn_out_fraction: 0.15,
        }
    }
}

fn load_dla_config() -> Result<DlaConfig, Box<dyn Error>> {
    let path = Path::new("config/dla.toml");
    if path.exists() {
        let text = fs::read_to_string(path)?;
        let cfg: DlaConfig = toml::from_str(&text)?;
        Ok(cfg)
    } else {
        Ok(DlaConfig::default())
    }
}

struct DlaResult {
    grid: Vec<bool>,
    grid_size: usize,
    center: i32,
    cluster_radius: f64,
    mass_radius: Vec<(f64, f64)>,
    fit: Option<LinearFit>,
    log_points_total: usize,
    fit_points_used: usize,
    fit_window: Option<(usize, usize)>,
}

fn simulate_dla(config: &DlaConfig) -> DlaResult {
    let size = config.grid_size;
    let center = (size / 2) as i32;
    let mut grid = vec![false; size * size];
    let mut rng = StdRng::seed_from_u64(config.seed);

    set_cell(&mut grid, size, center, center, true);
    let mut cluster_radius = 1.0;
    let max_radius = (size / 2 - 2) as f64;
    let mut spawn_radius = 4.0;
    let mut kill_radius = spawn_radius + config.kill_margin;

    for _ in 0..config.num_particles {
        if let Some(pt) = release_particle(
            &mut rng,
            &mut grid,
            size,
            center,
            spawn_radius,
            kill_radius,
            config.step_limit,
        ) {
            let dist = distance(center, pt.0, pt.1);
            if dist > cluster_radius {
                cluster_radius = dist;
                let mut target_spawn = cluster_radius + config.spawn_margin;
                target_spawn = target_spawn.min(max_radius - config.kill_margin);
                target_spawn = target_spawn.max(cluster_radius + 1.0);
                target_spawn = target_spawn.max(4.0);
                spawn_radius = target_spawn;
                kill_radius = (spawn_radius + config.kill_margin).min(max_radius);
            }
        }
    }

    let mass_radius = compute_mass_radius(&grid, size, center);
    let log_points: Vec<(f64, f64)> = mass_radius
        .iter()
        .filter_map(|(r, m)| {
            if *r > 1.0 && *m > 0.0 {
                Some((r.log10(), m.log10()))
            } else {
                None
            }
        })
        .collect();
    let fit_window = select_middle_window(
        log_points.len(),
        config.fit_burn_in_fraction,
        config.fit_burn_out_fraction,
    );
    let (fit, used_window) = if let Some((start, end)) = fit_window {
        if end - start >= 2 {
            let xs: Vec<f64> = log_points[start..end].iter().map(|(x, _)| *x).collect();
            let ys: Vec<f64> = log_points[start..end].iter().map(|(_, y)| *y).collect();
            (linear_regression(&xs, &ys), Some((start, end)))
        } else {
            (None, None)
        }
    } else {
        (None, None)
    };
    let fit_points_used = used_window.map(|(s, e)| e - s).unwrap_or(0);

    DlaResult {
        grid,
        grid_size: size,
        center,
        cluster_radius,
        mass_radius,
        fit,
        log_points_total: log_points.len(),
        fit_points_used,
        fit_window: used_window,
    }
}

fn release_particle<R: Rng>(
    rng: &mut R,
    grid: &mut [bool],
    size: usize,
    center: i32,
    spawn_radius: f64,
    kill_radius: f64,
    step_limit: usize,
) -> Option<(i32, i32)> {
    let mut steps = 0;
    let mut pos = spawn_new_particle(rng, center, spawn_radius, size);
    loop {
        if steps >= step_limit {
            steps = 0;
            pos = spawn_new_particle(rng, center, spawn_radius, size);
            continue;
        }
        steps += 1;

        let dir = rng.gen_range(0..4);
        match dir {
            0 => pos.0 += 1,
            1 => pos.0 -= 1,
            2 => pos.1 += 1,
            _ => pos.1 -= 1,
        }

        if !in_bounds(pos, size) || distance(center, pos.0, pos.1) > kill_radius {
            steps = 0;
            pos = spawn_new_particle(rng, center, spawn_radius, size);
            continue;
        }

        if has_neighbor(grid, size, pos.0, pos.1) {
            set_cell(grid, size, pos.0, pos.1, true);
            return Some(pos);
        }
    }
}

fn spawn_new_particle<R: Rng>(rng: &mut R, center: i32, radius: f64, size: usize) -> (i32, i32) {
    let angle = rng.gen_range(0.0..1.0) * 2.0 * PI;
    let mut x = center + (radius * angle.cos()).round() as i32;
    let mut y = center + (radius * angle.sin()).round() as i32;
    let max = size as i32 - 2;
    let min = 1;
    x = x.clamp(min, max);
    y = y.clamp(min, max);
    (x, y)
}

fn compute_mass_radius(grid: &[bool], size: usize, center: i32) -> Vec<(f64, f64)> {
    let max_radius = (size / 2) as usize;
    let mut cumulative = vec![0u32; max_radius + 1];

    for y in 0..size {
        for x in 0..size {
            if grid[index(size, x, y)] {
                let dx = x as i32 - center;
                let dy = y as i32 - center;
                let dist = ((dx * dx + dy * dy) as f64).sqrt();
                let mut idx = dist.floor() as usize;
                if idx > max_radius {
                    idx = max_radius;
                }
                for r in idx..=max_radius {
                    cumulative[r] += 1;
                }
            }
        }
    }

    log_sample_mass_radius(&cumulative)
}

fn log_sample_mass_radius(cumulative: &[u32]) -> Vec<(f64, f64)> {
    let len = cumulative.len();
    if len <= 2 {
        return cumulative
            .iter()
            .enumerate()
            .map(|(r, &m)| (r as f64, m as f64))
            .collect();
    }
    let max_radius = (len - 1) as f64;
    if max_radius <= 1.0 {
        return cumulative
            .iter()
            .enumerate()
            .map(|(r, &m)| (r as f64, m as f64))
            .collect();
    }

    let log_max = max_radius.ln();
    if !log_max.is_finite() || log_max <= 0.0 {
        return cumulative
            .iter()
            .enumerate()
            .map(|(r, &m)| (r as f64, m as f64))
            .collect();
    }

    let sample_count = len - 1;
    let mut samples = Vec::with_capacity(sample_count);
    for i in 0..sample_count {
        let t = i as f64 / (sample_count - 1) as f64;
        let radius = (t * log_max).exp().max(1.0);
        let mass = interpolate_cumulative(cumulative, radius);
        samples.push((radius, mass));
    }
    samples
}

fn interpolate_cumulative(cumulative: &[u32], radius: f64) -> f64 {
    if cumulative.is_empty() {
        return 0.0;
    }
    let max_index = cumulative.len() - 1;
    if max_index == 0 {
        return cumulative[0] as f64;
    }
    let clamped = radius.clamp(0.0, max_index as f64);
    let low = clamped.floor() as usize;
    let high = clamped.ceil() as usize;
    if low == high {
        cumulative[low] as f64
    } else {
        let frac = clamped - low as f64;
        let low_val = cumulative[low] as f64;
        let high_val = cumulative[high.min(max_index)] as f64;
        low_val + frac * (high_val - low_val)
    }
}

fn select_middle_window(
    total_points: usize,
    burn_in_fraction: f64,
    burn_out_fraction: f64,
) -> Option<(usize, usize)> {
    if total_points < 2 {
        return None;
    }
    let burn_in = burn_in_fraction.clamp(0.0, 0.49);
    let burn_out = burn_out_fraction.clamp(0.0, 0.49);
    let start = ((burn_in * total_points as f64).floor() as usize).min(total_points);
    let mut end = total_points.saturating_sub((burn_out * total_points as f64).floor() as usize);
    end = end.clamp(start, total_points);
    if end - start >= 2 {
        Some((start, end))
    } else {
        None
    }
}

fn write_outputs(
    ctx: &RunContext,
    config: &DlaConfig,
    result: &DlaResult,
) -> Result<(), Box<dyn Error>> {
    let fractal_path = ctx.file_path("fractal.png");
    render_fractal(
        &result.grid,
        result.grid_size,
        result.center,
        result.cluster_radius,
        &fractal_path,
    )?;

    let plot_path = ctx.file_path("mass_radius.png");
    plot_mass_radius(
        &result.mass_radius,
        result.fit,
        result.fit_window,
        &plot_path,
    )?;

    let mut rows = Vec::new();
    for (radius, mass) in &result.mass_radius {
        let log_r = if *radius > 0.0 { radius.log10() } else { 0.0 };
        let log_m = if *mass > 0.0 { mass.log10() } else { 0.0 };
        rows.push(vec![
            format!("{radius:.4}"),
            format!("{mass:.4}"),
            format!("{log_r:.4}"),
            format!("{log_m:.4}"),
        ]);
    }
    ctx.write_csv(
        "mass_radius.csv",
        &["radius", "mass", "log_radius", "log_mass"],
        &rows,
    )?;

    let mut log_lines = vec![
        "Diffusion-limited aggregation simulation".to_string(),
        format!("grid_size = {}", config.grid_size),
        format!("num_particles = {}", config.num_particles),
        format!("spawn_margin = {:.2}", config.spawn_margin),
        format!("kill_margin = {:.2}", config.kill_margin),
        format!("cluster_radius = {:.3}", result.cluster_radius),
        format!("fit_burn_in_fraction = {:.2}", config.fit_burn_in_fraction),
        format!(
            "fit_burn_out_fraction = {:.2}",
            config.fit_burn_out_fraction
        ),
        format!("fit_points_total = {}", result.log_points_total),
        format!("fit_points_used = {}", result.fit_points_used),
    ];
    if let Some(fit) = result.fit {
        log_lines.push(format!("fit_slope (D) = {:.4}", fit.slope));
        log_lines.push(format!("fit_intercept = {:.4}", fit.intercept));
        log_lines.push(format!("fit_r2 = {:.4}", fit.r2));
    } else {
        log_lines.push("fit_failed = not enough data".to_string());
    }
    ctx.write_lines("simulation.log", &log_lines)?;
    Ok(())
}

fn render_fractal(
    grid: &[bool],
    size: usize,
    center: i32,
    radius: f64,
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    let dim = 900;
    let area = BitMapBackend::new(path, (dim, dim)).into_drawing_area();
    area.fill(&BLACK)?;
    let (width, height) = area.dim_in_pixel();
    let cell_w = width as f64 / size as f64;
    let cell_h = height as f64 / size as f64;

    for y in 0..size {
        for x in 0..size {
            if grid[index(size, x, y)] {
                let dx = x as i32 - center;
                let dy = y as i32 - center;
                let dist = ((dx * dx + dy * dy) as f64).sqrt();
                let norm = (dist / radius.max(1.0)).min(1.0);
                let color = RGBColor(
                    (30.0 + 200.0 * (1.0 - norm)) as u8,
                    (60.0 + 150.0 * norm) as u8,
                    (120.0 + 100.0 * norm) as u8,
                );
                let x0 = (x as f64 * cell_w).round() as i32;
                let x1 = ((x + 1) as f64 * cell_w).round() as i32;
                let y0 = (y as f64 * cell_h).round() as i32;
                let y1 = ((y + 1) as f64 * cell_h).round() as i32;
                area.draw(&Rectangle::new([(x0, y0), (x1, y1)], color.filled()))?;
            }
        }
    }
    area.present()?;
    Ok(())
}

fn plot_mass_radius(
    data: &[(f64, f64)],
    fit: Option<LinearFit>,
    fit_window: Option<(usize, usize)>,
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    let scatter: Vec<(f64, f64)> = data
        .iter()
        .cloned()
        .filter(|(r, m)| *r > 0.0 && *m > 0.0)
        .collect();
    if scatter.is_empty() {
        let area = BitMapBackend::new(path, (900, 600)).into_drawing_area();
        area.fill(&WHITE)?;
        area.draw(&Text::new(
            "Not enough positive samples",
            (60, 300),
            ("sans-serif", 28).into_font(),
        ))?;
        area.present()?;
        return Ok(());
    }

    let r_min = scatter
        .iter()
        .map(|(r, _)| *r)
        .fold(f64::INFINITY, f64::min);
    let r_max = scatter
        .iter()
        .map(|(r, _)| *r)
        .fold(f64::NEG_INFINITY, f64::max);
    let m_min = scatter
        .iter()
        .map(|(_, m)| *m)
        .fold(f64::INFINITY, f64::min);
    let m_max = scatter
        .iter()
        .map(|(_, m)| *m)
        .fold(f64::NEG_INFINITY, f64::max);

    if !r_min.is_finite() || !r_max.is_finite() || !m_min.is_finite() || !m_max.is_finite() {
        return Ok(());
    }

    let area = BitMapBackend::new(path, (900, 600)).into_drawing_area();
    area.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&area)
        .caption("Mass-radius scaling", ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(50)
        .y_label_area_size(60)
        .build_cartesian_2d((r_min..r_max).log_scale(), (m_min..m_max).log_scale())?;

    chart
        .configure_mesh()
        .x_desc("r")
        .y_desc("M")
        .x_label_formatter(&|v| format!("{:.1}", v))
        .y_label_formatter(&|v| format!("{:.1}", v))
        .draw()?;

    chart.draw_series(
        scatter
            .iter()
            .map(|&(r, m)| Circle::new((r, m), 3, BLUE.filled())),
    )?;

    if let (Some(fit), Some((start, end))) = (fit, fit_window) {
        let log_points: Vec<(f64, f64)> = data
            .iter()
            .filter_map(|(r, mass)| {
                if *r > 1.0 && *mass > 0.0 {
                    Some((r.log10(), mass.log10()))
                } else {
                    None
                }
            })
            .collect();
        if end <= log_points.len() && start < end {
            let x0 = log_points[start].0;
            let x1 = log_points[end - 1].0;
            let y0 = fit.slope * x0 + fit.intercept;
            let y1 = fit.slope * x1 + fit.intercept;
            let r0 = 10f64.powf(x0);
            let r1 = 10f64.powf(x1);
            let m0 = 10f64.powf(y0);
            let m1 = 10f64.powf(y1);
            chart
                .draw_series(LineSeries::new(
                    vec![(r0, m0), (r1, m1)],
                    ShapeStyle::from(&RED).stroke_width(2),
                ))?
                .label(format!("slope = {:.3}", fit.slope))
                .legend(|(x, y)| PathElement::new([(x - 10, y), (x + 10, y)], &RED));
        }
    }

    chart
        .configure_series_labels()
        .border_style(&BLACK)
        .draw()?;
    area.present()?;
    Ok(())
}

fn in_bounds(pos: (i32, i32), size: usize) -> bool {
    let max = size as i32 - 1;
    pos.0 > 0 && pos.0 < max && pos.1 > 0 && pos.1 < max
}

fn has_neighbor(grid: &[bool], size: usize, x: i32, y: i32) -> bool {
    for (dx, dy) in &[(1, 0), (-1, 0), (0, 1), (0, -1)] {
        let nx = x + dx;
        let ny = y + dy;
        if nx >= 0 && nx < size as i32 && ny >= 0 && ny < size as i32 {
            if grid[index(size, nx as usize, ny as usize)] {
                return true;
            }
        }
    }
    false
}

fn set_cell(grid: &mut [bool], size: usize, x: i32, y: i32, value: bool) {
    if x >= 0 && x < size as i32 && y >= 0 && y < size as i32 {
        let idx = index(size, x as usize, y as usize);
        grid[idx] = value;
    }
}

fn index(size: usize, x: usize, y: usize) -> usize {
    y * size + x
}

fn distance(center: i32, x: i32, y: i32) -> f64 {
    let dx = x - center;
    let dy = y - center;
    ((dx * dx + dy * dy) as f64).sqrt()
}
