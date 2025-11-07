use std::fs;
use std::path::Path;

use anyhow::{Context, Result, anyhow};
use plotters::coord::Shift;
use plotters::prelude::*;
use plotters::style::text_anchor::{HPos, Pos, VPos};

use crate::dynamics::{SimulationResult, SimulationSample};
use crate::ensemble::LyapunovDiagnostics;
use crate::output::{AverageArtifacts, OutputArtifacts, VelocityScanArtifacts};
use crate::scan::VelocityScanPoint;

const CANVAS_SIZE: (u32, u32) = (680, 540);
const TRAJECTORY_CANVAS: (u32, u32) = (620, 620);

pub fn render_all(result: &SimulationResult, artifacts: &OutputArtifacts) -> Result<()> {
    if result.samples.is_empty() {
        return Err(anyhow!("No samples available for plotting"));
    }

    if artifacts.toggles.theta {
        draw_theta_chart_png(result, &artifacts.theta_png)?;
        draw_theta_chart_svg(result, &artifacts.theta_svg)?;
    }

    if artifacts.toggles.omega {
        draw_omega_chart_png(result, &artifacts.omega_png)?;
        draw_omega_chart_svg(result, &artifacts.omega_svg)?;
    }

    if artifacts.toggles.delta_theta {
        draw_delta_theta_chart_png(result, &artifacts.delta_theta_png)?;
        draw_delta_theta_chart_svg(result, &artifacts.delta_theta_svg)?;
    }

    if artifacts.toggles.trajectory {
        draw_trajectory_png(result, &artifacts.trajectory_png)?;
        draw_trajectory_svg(result, &artifacts.trajectory_svg)?;
    }

    Ok(())
}

pub fn render_delta_theta_average(
    artifacts: &AverageArtifacts,
    time: &[f64],
    values: &[f64],
) -> Result<()> {
    if time.is_empty() || values.is_empty() {
        return Err(anyhow!("No data available for Δθ average plot"));
    }
    if time.len() != values.len() {
        return Err(anyhow!("Mismatched lengths for Δθ average plot"));
    }

    if artifacts.toggles.png {
        ensure_parent(&artifacts.average_png)?;
        let backend_png = BitMapBackend::new(&artifacts.average_png, CANVAS_SIZE);
        let png_area = backend_png.into_drawing_area();
        draw_average_chart(png_area, time, values, None)?;
    }

    if artifacts.toggles.svg {
        ensure_parent(&artifacts.average_svg)?;
        let backend_svg = SVGBackend::new(&artifacts.average_svg, CANVAS_SIZE);
        let svg_area = backend_svg.into_drawing_area();
        draw_average_chart(svg_area, time, values, None)?;
    }

    Ok(())
}

pub fn render_lyapunov_overlay(
    artifacts: &AverageArtifacts,
    time: &[f64],
    values: &[f64],
    diagnostics: &LyapunovDiagnostics,
) -> Result<()> {
    if time.is_empty() || values.is_empty() {
        return Err(anyhow!("No data available for Lyapunov overlay plot"));
    }
    if time.len() != values.len() {
        return Err(anyhow!(
            "Mismatched lengths for Lyapunov overlay plot: {} vs {}",
            time.len(),
            values.len()
        ));
    }

    if artifacts.toggles.lyapunov_png {
        ensure_parent(&artifacts.lyapunov_png)?;
        let backend_png = BitMapBackend::new(&artifacts.lyapunov_png, CANVAS_SIZE);
        let png_area = backend_png.into_drawing_area();
        draw_average_chart(png_area, time, values, Some(diagnostics))?;
    }

    if artifacts.toggles.lyapunov_svg {
        ensure_parent(&artifacts.lyapunov_svg)?;
        let backend_svg = SVGBackend::new(&artifacts.lyapunov_svg, CANVAS_SIZE);
        let svg_area = backend_svg.into_drawing_area();
        draw_average_chart(svg_area, time, values, Some(diagnostics))?;
    }

    Ok(())
}

pub fn render_lyapunov_vs_ecc(
    artifacts: &VelocityScanArtifacts,
    points: &[VelocityScanPoint],
) -> Result<()> {
    if points.is_empty() {
        return Err(anyhow!("No samples available for Lyapunov vs eccentricity plot"));
    }

    let valid: Vec<(f64, f64)> = points
        .iter()
        .filter_map(|p| p.lyapunov.map(|lambda| (p.eccentricity, lambda)))
        .collect();

    if valid.is_empty() {
        return Err(anyhow!(
            "No valid Lyapunov estimates available for plotting"
        ));
    }

    if artifacts.toggles.png {
        ensure_parent(&artifacts.plot_png)?;
        let backend_png = BitMapBackend::new(&artifacts.plot_png, CANVAS_SIZE);
        let png_area = backend_png.into_drawing_area();
        draw_lyapunov_vs_ecc_chart(png_area, &valid)?;
    }

    if artifacts.toggles.svg {
        ensure_parent(&artifacts.plot_svg)?;
        let backend_svg = SVGBackend::new(&artifacts.plot_svg, CANVAS_SIZE);
        let svg_area = backend_svg.into_drawing_area();
        draw_lyapunov_vs_ecc_chart(svg_area, &valid)?;
    }

    Ok(())
}

fn draw_theta_chart_png(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = BitMapBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_theta_chart(drawing_area, result)
}

fn draw_theta_chart_svg(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = SVGBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_theta_chart(drawing_area, result)
}

fn draw_omega_chart_png(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = BitMapBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_omega_chart(drawing_area, result)
}

fn draw_omega_chart_svg(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = SVGBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_omega_chart(drawing_area, result)
}

fn draw_delta_theta_chart_png(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = BitMapBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_delta_theta_chart(drawing_area, result)
}

fn draw_delta_theta_chart_svg(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = SVGBackend::new(path, CANVAS_SIZE);
    let drawing_area = backend.into_drawing_area();
    draw_delta_theta_chart(drawing_area, result)
}

fn draw_trajectory_png(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = BitMapBackend::new(path, TRAJECTORY_CANVAS);
    let drawing_area = backend.into_drawing_area();
    draw_trajectory(drawing_area, result)
}

fn draw_trajectory_svg(result: &SimulationResult, path: &Path) -> Result<()> {
    ensure_parent(path)?;
    let backend = SVGBackend::new(path, TRAJECTORY_CANVAS);
    let drawing_area = backend.into_drawing_area();
    draw_trajectory(drawing_area, result)
}

fn draw_theta_chart<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    draw_time_series_chart(
        drawing_area,
        result,
        "Hyperion θ versus time",
        "θ (radians)",
        |sample| sample.theta_wrapped_primary,
    )
}

fn draw_omega_chart<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    draw_time_series_chart(
        drawing_area,
        result,
        "Hyperion ω versus time",
        "ω (radians/yr)",
        |sample| sample.omega_primary,
    )
}

fn draw_delta_theta_chart<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    draw_time_series_chart_log(
        drawing_area,
        result,
        "Hyperion Δθ versus time",
        "Δθ (radians)",
        |sample| sample.delta_theta,
    )
}

fn draw_time_series_chart<DB, F>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
    title: &str,
    y_label: &str,
    value_accessor: F,
) -> Result<()>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
    F: Fn(&SimulationSample) -> f64,
{
    let samples = &result.samples;

    let time_start = samples.first().map(|s| s.time).unwrap_or(0.0);
    let time_end = samples.last().map(|s| s.time).unwrap_or(time_start + 1.0);

    let (y_min, y_max) = min_max(samples.iter().map(|sample| value_accessor(sample)));
    let y_span = (y_max - y_min).abs();
    let mut y_padding = if y_span < 1e-9 {
        y_max.abs().max(1.0) * 0.05
    } else {
        y_span * 0.05
    };
    let mut y_lower = y_min - y_padding;
    let mut y_upper = y_max + y_padding;
    if (y_upper - y_lower).abs() < 1e-9 {
        y_padding = y_max.abs().max(1.0) * 0.05;
        y_lower -= y_padding;
        y_upper += y_padding;
    }
    let y_range = y_lower..y_upper;

    let root = drawing_area;
    root.fill(&WHITE)?;

    let (title_area, chart_area) = root.split_vertically(36);
    let title_style_base = ("sans-serif", 28).into_text_style(&title_area);
    let title_style = title_style_base.pos(Pos::new(HPos::Center, VPos::Center));
    let title_dims = title_area.dim_in_pixel();
    title_area.draw_text(
        title,
        &title_style,
        (title_dims.0 as i32 / 2, title_dims.1 as i32 / 2),
    )?;

    let mut chart = ChartBuilder::on(&chart_area)
        .margin_left(52)
        .margin_right(18)
        .margin_bottom(40)
        .margin_top(6)
        .set_label_area_size(LabelAreaPosition::Left, 58)
        .set_label_area_size(LabelAreaPosition::Bottom, 45)
        .build_cartesian_2d(time_start..time_end, y_range.clone())?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("time (yr)")
        .y_desc(y_label)
        .y_label_formatter(&|value| format_decimal_tick(*value))
        .label_style(("sans-serif", 18))
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

    chart.draw_series(LineSeries::new(
        samples
            .iter()
            .map(|sample| (sample.time, value_accessor(sample))),
        &BLACK,
    ))?;

    chart.draw_series(std::iter::once(PathElement::new(
        vec![
            (time_start, y_lower),
            (time_end, y_lower),
            (time_end, y_upper),
            (time_start, y_upper),
            (time_start, y_lower),
        ],
        &BLACK,
    )))?;

    chart_area
        .present()
        .map_err(|e| anyhow!("Failed to render time series chart: {:?}", e))?;
    Ok(())
}

fn draw_time_series_chart_log<DB, F>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
    title: &str,
    y_label: &str,
    value_accessor: F,
) -> Result<()>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
    F: Fn(&SimulationSample) -> f64,
{
    let samples = &result.samples;

    let time_start = samples.first().map(|s| s.time).unwrap_or(0.0);
    let time_end = samples.last().map(|s| s.time).unwrap_or(time_start + 1.0);

    let mut min_positive = f64::INFINITY;
    let mut max_value = 0.0;
    for value in samples.iter().map(|sample| value_accessor(sample)) {
        if value > 0.0 && value < min_positive {
            min_positive = value;
        }
        if value > max_value {
            max_value = value;
        }
    }

    if !min_positive.is_finite() {
        min_positive = 1e-12;
    }
    if max_value <= 0.0 {
        max_value = min_positive * 10.0;
    }

    // Clamp upper range to the next power of ten
    let mut max_ceiling = if max_value <= 0.0 {
        max_value
    } else {
        let exp = (max_value.log10().ceil() as i64).max(0);
        10f64.powi(exp as i32)
    };
    if max_ceiling <= max_value {
        max_ceiling *= 10.0;
    }

    let root = drawing_area;
    root.fill(&WHITE)?;

    let (title_area, chart_area) = root.split_vertically(36);
    let title_style_base = ("sans-serif", 28).into_text_style(&title_area);
    let title_style = title_style_base.pos(Pos::new(HPos::Center, VPos::Center));
    let title_dims = title_area.dim_in_pixel();
    title_area.draw_text(
        title,
        &title_style,
        (title_dims.0 as i32 / 2, title_dims.1 as i32 / 2),
    )?;

    let mut chart = ChartBuilder::on(&chart_area)
        .margin_left(52)
        .margin_right(18)
        .margin_bottom(40)
        .margin_top(6)
        .set_label_area_size(LabelAreaPosition::Left, 58)
        .set_label_area_size(LabelAreaPosition::Bottom, 45)
        .build_cartesian_2d(
            time_start..time_end,
            (min_positive..max_ceiling).log_scale(),
        )?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("time (yr)")
        .y_desc(y_label)
        .y_label_formatter(&|value| format_log_tick(*value))
        .label_style(("sans-serif", 18))
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

    chart.draw_series(LineSeries::new(
        samples.iter().map(|sample| {
            let value = value_accessor(sample);
            let sanitized = if value > 0.0 { value } else { min_positive };
            (sample.time, sanitized)
        }),
        &BLACK,
    ))?;

    chart.draw_series(std::iter::once(PathElement::new(
        vec![
            (time_start, min_positive),
            (time_end, min_positive),
            (time_end, max_ceiling),
            (time_start, max_ceiling),
            (time_start, min_positive),
        ],
        &BLACK,
    )))?;

    chart_area
        .present()
        .map_err(|e| anyhow!("Failed to render time series chart: {:?}", e))?;
    Ok(())
}

fn draw_trajectory<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    result: &SimulationResult,
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    let samples = &result.samples;
    let root = drawing_area;
    root.fill(&WHITE)?;

    let (title_area, chart_area) = root.split_vertically(36);
    let title_style_base = ("sans-serif", 28).into_text_style(&title_area);
    let title_style = title_style_base.pos(Pos::new(HPos::Center, VPos::Center));
    let title_dims = title_area.dim_in_pixel();
    title_area.draw_text(
        "Hyperion trajectory",
        &title_style,
        (title_dims.0 as i32 / 2, title_dims.1 as i32 / 2),
    )?;

    let (x_min, x_max) = min_max(samples.iter().map(|s| s.x));
    let (y_min, y_max) = min_max(samples.iter().map(|s| s.y));
    let x_span = (x_max - x_min).abs();
    let y_span = (y_max - y_min).abs();
    let mut x_pad = if x_span < 1e-9 {
        x_max.abs().max(1.0) * 0.05
    } else {
        x_span * 0.08
    };
    let mut y_pad = if y_span < 1e-9 {
        y_max.abs().max(1.0) * 0.05
    } else {
        y_span * 0.08
    };

    let mut x_lower = x_min - x_pad;
    let mut x_upper = x_max + x_pad;
    let mut y_lower = y_min - y_pad;
    let mut y_upper = y_max + y_pad;

    if (x_upper - x_lower).abs() < 1e-9 {
        x_pad = x_max.abs().max(1.0) * 0.05;
        x_lower -= x_pad;
        x_upper += x_pad;
    }

    if (y_upper - y_lower).abs() < 1e-9 {
        y_pad = y_max.abs().max(1.0) * 0.05;
        y_lower -= y_pad;
        y_upper += y_pad;
    }

    let mut chart = ChartBuilder::on(&chart_area)
        .margin_left(52)
        .margin_right(18)
        .margin_bottom(45)
        .margin_top(6)
        .set_label_area_size(LabelAreaPosition::Left, 58)
        .set_label_area_size(LabelAreaPosition::Bottom, 50)
        .build_cartesian_2d(x_lower..x_upper, y_lower..y_upper)?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("x")
        .y_desc("y")
        .label_style(("sans-serif", 18))
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

    chart.draw_series(LineSeries::new(samples.iter().map(|s| (s.x, s.y)), &BLACK))?;

    chart.draw_series(PointSeries::of_element(
        vec![(samples[0].x, samples[0].y)],
        5,
        ShapeStyle::from(&BLACK).filled(),
        &|coord, size, style| {
            EmptyElement::at(coord)
                + Circle::new((0, 0), size, style)
                + Text::new("start", (10, -10), ("sans-serif", 18).into_font())
        },
    ))?;

    chart.draw_series(std::iter::once(PathElement::new(
        vec![
            (x_lower, y_lower),
            (x_upper, y_lower),
            (x_upper, y_upper),
            (x_lower, y_upper),
            (x_lower, y_lower),
        ],
        &BLACK,
    )))?;

    chart_area
        .present()
        .map_err(|e| anyhow!("Failed to render trajectory chart: {:?}", e))?;
    Ok(())
}

fn draw_average_chart<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    time: &[f64],
    values: &[f64],
    diagnostics: Option<&LyapunovDiagnostics>,
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    let time_start = *time.first().unwrap_or(&0.0);
    let time_end = *time.last().unwrap_or(&(time_start + 1.0));

    let mut min_positive = f64::INFINITY;
    let mut max_value = 0.0;
    for &value in values {
        if value > 0.0 && value < min_positive {
            min_positive = value;
        }
        if value > max_value {
            max_value = value;
        }
    }

    if let Some(diag) = diagnostics {
        if let (Some(start_idx), Some(end_idx), Some(slope), Some(intercept)) = (
            diag.segment_start_index,
            diag.segment_end_index,
            diag.slope,
            diag.intercept,
        ) {
            if end_idx >= start_idx && end_idx < time.len() {
                let start_time = time[start_idx];
                let end_time = time[end_idx];
                let start_val = (slope * start_time + intercept).exp();
                let end_val = (slope * end_time + intercept).exp();

                if start_val.is_finite() && start_val > 0.0 {
                    if start_val < min_positive {
                        min_positive = start_val;
                    }
                    if start_val > max_value {
                        max_value = start_val;
                    }
                }

                if end_val.is_finite() && end_val > 0.0 {
                    if end_val < min_positive {
                        min_positive = end_val;
                    }
                    if end_val > max_value {
                        max_value = end_val;
                    }
                }
            }
        }
    }

    if !min_positive.is_finite() {
        min_positive = 1e-12;
    }
    if max_value <= 0.0 {
        max_value = min_positive * 10.0;
    }

    let mut max_ceiling = if max_value <= 0.0 {
        max_value
    } else {
        10f64.powi(max_value.log10().ceil() as i32)
    };
    if max_ceiling <= max_value {
        max_ceiling *= 10.0;
    }

    let title_text = if diagnostics.is_some() {
        "Hyperion Δθ̄ with Lyapunov fit"
    } else {
        "Hyperion Δθ̄ versus time"
    };

    let root = drawing_area;
    root.fill(&WHITE)?;

    let (title_area, chart_area) = root.split_vertically(36);
    let title_style_base = ("sans-serif", 28).into_text_style(&title_area);
    let title_style = title_style_base.pos(Pos::new(HPos::Center, VPos::Center));
    let title_dims = title_area.dim_in_pixel();
    title_area.draw_text(
        title_text,
        &title_style,
        (title_dims.0 as i32 / 2, title_dims.1 as i32 / 2),
    )?;

    let mut chart = ChartBuilder::on(&chart_area)
        .margin_left(52)
        .margin_right(18)
        .margin_bottom(40)
        .margin_top(6)
        .set_label_area_size(LabelAreaPosition::Left, 58)
        .set_label_area_size(LabelAreaPosition::Bottom, 45)
        .build_cartesian_2d(
            time_start..time_end,
            (min_positive..max_ceiling).log_scale(),
        )?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("time (yr)")
        .y_desc("Δθ̄ (radians)")
        .y_label_formatter(&|value| format_log_tick(*value))
        .label_style(("sans-serif", 18))
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

    chart.draw_series(LineSeries::new(
        time.iter().zip(values.iter()).map(|(&t, &value)| {
            let sanitized = if value > 0.0 { value } else { min_positive };
            (t, sanitized)
        }),
        &BLACK,
    ))?;

    if let Some(diag) = diagnostics {
        if let (Some(start_idx), Some(end_idx), Some(slope), Some(intercept)) = (
            diag.segment_start_index,
            diag.segment_end_index,
            diag.slope,
            diag.intercept,
        ) {
            if end_idx >= start_idx && end_idx < time.len() && diag.segment_points >= 2 {
                let start_time = time[start_idx];
                let end_time = time[end_idx];
                let start_val = (slope * start_time + intercept).exp();
                let end_val = (slope * end_time + intercept).exp();
                if start_val.is_finite()
                    && start_val > 0.0
                    && end_val.is_finite()
                    && end_val > 0.0
                {
                    chart.draw_series(LineSeries::new(
                        vec![(start_time, start_val), (end_time, end_val)],
                        Palette99::pick(1).stroke_width(3),
                    ))?;

                    let annotation = format!(
                        "λ ≈ {:.4} ({} pts)",
                        slope,
                        diag.segment_points
                    );

                    chart_area.draw(&Text::new(
                        annotation,
                        (30, 36),
                        ("sans-serif", 20).into_font(),
                    ))?;
                }
            }
        }
    }

    chart.draw_series(std::iter::once(PathElement::new(
        vec![
            (time_start, min_positive),
            (time_end, min_positive),
            (time_end, max_ceiling),
            (time_start, max_ceiling),
            (time_start, min_positive),
        ],
        &BLACK,
    )))?;

    chart_area
        .present()
        .map_err(|e| anyhow!("Failed to render average chart: {:?}", e))?;
    Ok(())
}

fn draw_lyapunov_vs_ecc_chart<DB: DrawingBackend>(
    drawing_area: DrawingArea<DB, Shift>,
    points: &[(f64, f64)],
) -> Result<()>
where
    DB::ErrorType: 'static,
{
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;

    for &(x, y) in points {
        if x < x_min {
            x_min = x;
        }
        if x > x_max {
            x_max = x;
        }
        if y < y_min {
            y_min = y;
        }
        if y > y_max {
            y_max = y;
        }
    }

    if !x_min.is_finite() || !x_max.is_finite() {
        return Err(anyhow!("Invalid eccentricity range for plotting"));
    }
    if !y_min.is_finite() || !y_max.is_finite() {
        return Err(anyhow!("Invalid Lyapunov range for plotting"));
    }

    if (x_max - x_min).abs() < f64::EPSILON {
        x_min -= 0.05;
        x_max += 0.05;
    } else {
        let pad = (x_max - x_min) * 0.08;
        x_min -= pad;
        x_max += pad;
    }

    if (y_max - y_min).abs() < f64::EPSILON {
        y_min -= 0.05;
        y_max += 0.05;
    } else {
        let pad = (y_max - y_min) * 0.08;
        y_min -= pad;
        y_max += pad;
    }

    let root = drawing_area;
    root.fill(&WHITE)?;

    let (title_area, chart_area) = root.split_vertically(36);
    let title_style_base = ("sans-serif", 28).into_text_style(&title_area);
    let title_style = title_style_base.pos(Pos::new(HPos::Center, VPos::Center));
    let dims = title_area.dim_in_pixel();
    title_area.draw_text(
        "Lyapunov exponent vs. eccentricity",
        &title_style,
        (dims.0 as i32 / 2, dims.1 as i32 / 2),
    )?;

    let mut chart = ChartBuilder::on(&chart_area)
        .margin_left(60)
        .margin_right(20)
        .margin_top(6)
        .margin_bottom(45)
        .set_label_area_size(LabelAreaPosition::Left, 58)
        .set_label_area_size(LabelAreaPosition::Bottom, 50)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart
        .configure_mesh()
        .disable_mesh()
        .x_desc("eccentricity")
        .y_desc("Lyapunov exponent λ")
        .label_style(("sans-serif", 18))
        .axis_desc_style(("sans-serif", 20))
        .y_label_formatter(&|v| format_decimal_tick(*v))
        .x_label_formatter(&|v| format_decimal_tick(*v))
        .draw()?;

    chart.draw_series(points.iter().map(|&(x, y)| {
        Circle::new((x, y), 5, Palette99::pick(4).filled())
    }))?;

    chart.draw_series(std::iter::once(PathElement::new(
        vec![(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), (x_min, y_min)],
        &BLACK,
    )))?;

    chart_area
        .present()
        .map_err(|e| anyhow!("Failed to render Lyapunov vs eccentricity chart: {:?}", e))?;
    Ok(())
}

fn min_max<I>(values: I) -> (f64, f64)
where
    I: Iterator<Item = f64>,
{
    let mut iter = values.peekable();
    if iter.peek().is_none() {
        return (0.0, 1.0);
    }

    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;

    for val in iter {
        if val < min {
            min = val;
        }
        if val > max {
            max = val;
        }
    }

    if (max - min).abs() < f64::EPSILON {
        let epsilon = if min.abs() < 1.0 {
            1.0
        } else {
            min.abs() * 0.05
        };
        (min - epsilon, max + epsilon)
    } else {
        (min, max)
    }
}

fn format_log_tick(value: f64) -> String {
    if !value.is_finite() || value <= 0.0 {
        return "0".into();
    }

    let log10 = value.log10();
    let exponent = log10.round();
    if (log10 - exponent).abs() < 5e-4 {
        format!("1e{}", exponent as i32)
    } else {
        format!("{:.1e}", value)
    }
}

fn format_decimal_tick(value: f64) -> String {
    if value.abs() >= 1e4 || value.abs() < 1e-3 {
        format!("{:.1e}", value)
    } else {
        format!("{:.6}", value)
            .trim_end_matches('0')
            .trim_end_matches('.')
            .to_string()
    }
}

fn ensure_parent(path: &Path) -> Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.exists() {
            fs::create_dir_all(parent)
                .with_context(|| format!("Failed to create plot directory {}", parent.display()))?;
        }
    }
    Ok(())
}
