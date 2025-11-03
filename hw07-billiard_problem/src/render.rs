use std::f64::consts::{PI, TAU};
use std::fmt::Write;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use nalgebra::Point2;
use plotters::coord::Shift;
use plotters::prelude::*;
use plotters::style::FontTransform;
use plotters::style::text_anchor::{HPos, Pos, VPos};

use crate::boundary::{Boundary2D, BoundarySegment};
use crate::config::{OutputConfig, PlotConfig};
use crate::sampling::SimulationSamples;

const ARC_SAMPLES: usize = 192;
const BEZIER_SAMPLES: usize = 192;
const TITLE_FONT: i32 = 30;
const AXIS_FONT: i32 = 24;
const TICK_FONT: i32 = 18;
const TICK_LENGTH_PX: f64 = 10.0;

const LEFT_MARGIN: f64 = 85.0;
const RIGHT_MARGIN: f64 = 55.0;
const TOP_MARGIN: f64 = 70.0;
const BOTTOM_MARGIN: f64 = 95.0;

pub fn draw_boundary_preview(
    boundary: &dyn Boundary2D,
    plot: &PlotConfig,
    output_dir: &Path,
    output: &OutputConfig,
) -> Result<Vec<PathBuf>> {
    fs::create_dir_all(output_dir)
        .with_context(|| format!("failed to create output directory {}", output_dir.display()))?;

    let mut generated = Vec::new();

    if output.export_png {
        let png_path = output_dir.join(format!("{}_boundary.png", output.base_name));
        draw_boundary_png(boundary, plot, &png_path)?;
        generated.push(png_path);
    }

    if output.export_svg {
        let svg_path = output_dir.join(format!("{}_boundary.svg", output.base_name));
        draw_boundary_svg(boundary, plot, &svg_path)?;
        generated.push(svg_path);
    }

    Ok(generated)
}

pub fn draw_trajectory_plot(
    boundary: Option<&dyn Boundary2D>,
    samples: &SimulationSamples,
    plot: &PlotConfig,
    output: &OutputConfig,
    output_dir: &Path,
) -> Result<Vec<PathBuf>> {
    if !output.export_png && !output.export_svg {
        return Ok(Vec::new());
    }

    let mut files = Vec::new();
    let mut extent = samples.trajectory_extent().max(1.0);
    if let Some(boundary) = boundary {
        extent = extent.max(boundary.bounding_radius());
    }

    let min_x = -extent;
    let max_x = extent;
    let min_y = -extent;
    let max_y = extent;
    let ticks = symmetric_ticks(extent);
    let layout = PlotLayout::new(plot.width_px as f64, plot.height_px as f64);

    if output.export_png {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let png_path = output_dir.join(format!("{}_trajectory.png", output.base_name));
        let backend = BitMapBackend::new(&png_path, (plot.width_px, plot.height_px));
        let drawing = backend.into_drawing_area();
        drawing
            .fill(&WHITE)
            .map_err(|e| anyhow!("failed to clear drawing area: {e}"))?;
        draw_axes_png_generic(
            &drawing,
            &layout,
            min_x,
            max_x,
            min_y,
            max_y,
            &ticks,
            &ticks,
            "Trajectory",
            "x",
            "y",
        )?;
        if let Some(boundary) = boundary {
            draw_boundary_shape_png(
                &drawing, boundary, &layout, min_x, max_x, min_y, max_y, plot,
            )?;
        }
        draw_trajectory_segments_png(&drawing, samples, &layout, min_x, max_x, min_y, max_y, plot)?;
        drawing
            .present()
            .map_err(|e| anyhow!("failed to write trajectory image: {e}"))?;
        drop(drawing);
        files.push(png_path);
    }

    if output.export_svg {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let svg_path = output_dir.join(format!("{}_trajectory.svg", output.base_name));
        let mut svg = String::new();
        let width = plot.width_px.max(100) as f64;
        let height = plot.height_px.max(100) as f64;
        writeln!(
            svg,
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##
        )
        .unwrap();
        writeln!(
            svg,
            r#"<rect x="0" y="0" width="{width}" height="{height}" fill="white"/>"#
        )
        .unwrap();
        draw_axes_svg_generic(
            &mut svg,
            &layout,
            min_x,
            max_x,
            min_y,
            max_y,
            &ticks,
            &ticks,
            "Trajectory",
            "x",
            "y",
            width,
            height,
        );
        if let Some(boundary) = boundary {
            let mut path = String::new();
            build_boundary_path_svg(
                boundary,
                min_x,
                max_x,
                min_y,
                max_y,
                &layout,
                plot.line_width.max(1.0),
                &mut path,
            );
            svg.push_str(&path);
        }
        draw_trajectory_segments_svg(
            &mut svg,
            samples,
            &layout,
            min_x,
            max_x,
            min_y,
            max_y,
            plot.line_width.max(1.0),
        );
        writeln!(svg, "</svg>").unwrap();
        fs::write(&svg_path, svg)
            .with_context(|| format!("failed to write trajectory SVG {}", svg_path.display()))?;
        files.push(svg_path);
    }

    Ok(files)
}

pub fn draw_phase_plot(
    samples: &SimulationSamples,
    plot: &PlotConfig,
    output: &OutputConfig,
    output_dir: &Path,
) -> Result<Vec<PathBuf>> {
    if !output.export_png && !output.export_svg {
        return Ok(Vec::new());
    }

    let (range_x, range_vx) = samples.phase_extent();
    let (min_x, max_x) = range_x.widen_symmetric(1.0);
    let (min_vx, max_vx) = range_vx.widen_symmetric(1.0);
    let x_ticks = ticks_for_range(min_x, max_x);
    let vx_ticks = ticks_for_range(min_vx, max_vx);
    let layout = PlotLayout::new(plot.width_px as f64, plot.height_px as f64);

    let mut files = Vec::new();
    if output.export_png {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let path = output_dir.join(format!("{}_phase.png", output.base_name));
        let backend = BitMapBackend::new(&path, (plot.width_px, plot.height_px));
        let drawing = backend.into_drawing_area();
        drawing
            .fill(&WHITE)
            .map_err(|e| anyhow!("failed to clear drawing area: {e}"))?;
        draw_axes_png_generic(
            &drawing,
            &layout,
            min_x,
            max_x,
            min_vx,
            max_vx,
            &x_ticks,
            &vx_ticks,
            "Phase space",
            "x",
            "v_x",
        )?;
        draw_phase_segments_png(
            &drawing, samples, &layout, min_x, max_x, min_vx, max_vx, plot,
        )?;
        drawing
            .present()
            .map_err(|e| anyhow!("failed to write phase image: {e}"))?;
        drop(drawing);
        files.push(path);
    }

    if output.export_svg {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let path = output_dir.join(format!("{}_phase.svg", output.base_name));
        let width = plot.width_px.max(100) as f64;
        let height = plot.height_px.max(100) as f64;
        let mut svg = String::new();
        writeln!(
            svg,
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##
        )
        .unwrap();
        writeln!(
            svg,
            r#"<rect x="0" y="0" width="{width}" height="{height}" fill="white"/>"#
        )
        .unwrap();
        draw_axes_svg_generic(
            &mut svg,
            &layout,
            min_x,
            max_x,
            min_vx,
            max_vx,
            &x_ticks,
            &vx_ticks,
            "Phase space",
            "x",
            "v_x",
            width,
            height,
        );
        draw_phase_segments_svg(
            &mut svg,
            samples,
            &layout,
            min_x,
            max_x,
            min_vx,
            max_vx,
            plot.line_width.max(1.0),
        );
        writeln!(svg, "</svg>").unwrap();
        fs::write(&path, svg)
            .with_context(|| format!("failed to write phase SVG {}", path.display()))?;
        files.push(path);
    }

    Ok(files)
}

pub fn draw_poincare_plot(
    samples: &SimulationSamples,
    plot: &PlotConfig,
    output: &OutputConfig,
    output_dir: &Path,
) -> Result<Vec<PathBuf>> {
    if !output.export_png && !output.export_svg {
        return Ok(Vec::new());
    }

    let (range_x, range_vx) = samples.poincare_extent();
    let (min_x, max_x) = range_x.widen_symmetric(1.0);
    let (min_vx, max_vx) = range_vx.widen_symmetric(1.0);
    let x_ticks = ticks_for_range(min_x, max_x);
    let vx_ticks = ticks_for_range(min_vx, max_vx);
    let layout = PlotLayout::new(plot.width_px as f64, plot.height_px as f64);

    let mut files = Vec::new();
    if output.export_png {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let path = output_dir.join(format!("{}_poincare.png", output.base_name));
        let backend = BitMapBackend::new(&path, (plot.width_px, plot.height_px));
        let drawing = backend.into_drawing_area();
        drawing
            .fill(&WHITE)
            .map_err(|e| anyhow!("failed to clear drawing area: {e}"))?;
        draw_axes_png_generic(
            &drawing,
            &layout,
            min_x,
            max_x,
            min_vx,
            max_vx,
            &x_ticks,
            &vx_ticks,
            "Poincaré section",
            "x",
            "v_x",
        )?;
        draw_poincare_points_png(&drawing, samples, &layout, min_x, max_x, min_vx, max_vx)?;
        drawing
            .present()
            .map_err(|e| anyhow!("failed to write Poincaré image: {e}"))?;
        drop(drawing);
        files.push(path);
    }

    if output.export_svg {
        fs::create_dir_all(output_dir).with_context(|| {
            format!("failed to create output directory {}", output_dir.display())
        })?;
        let path = output_dir.join(format!("{}_poincare.svg", output.base_name));
        let width = plot.width_px.max(100) as f64;
        let height = plot.height_px.max(100) as f64;
        let mut svg = String::new();
        writeln!(
            svg,
            r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##
        )
        .unwrap();
        writeln!(
            svg,
            r#"<rect x="0" y="0" width="{width}" height="{height}" fill="white"/>"#
        )
        .unwrap();
        draw_axes_svg_generic(
            &mut svg,
            &layout,
            min_x,
            max_x,
            min_vx,
            max_vx,
            &x_ticks,
            &vx_ticks,
            "Poincaré section",
            "x",
            "v_x",
            width,
            height,
        );
        draw_poincare_points_svg(&mut svg, samples, &layout, min_x, max_x, min_vx, max_vx);
        writeln!(svg, "</svg>").unwrap();
        fs::write(&path, svg)
            .with_context(|| format!("failed to write Poincaré SVG {}", path.display()))?;
        files.push(path);
    }

    Ok(files)
}

fn draw_boundary_png(
    boundary: &dyn Boundary2D,
    plot: &PlotConfig,
    output_path: &Path,
) -> Result<()> {
    let width = plot.width_px.max(100);
    let height = plot.height_px.max(100);
    let layout = PlotLayout::new(width as f64, height as f64);

    let backend = BitMapBackend::new(output_path, (width, height));
    let drawing = backend.into_drawing_area();
    drawing
        .fill(&WHITE)
        .map_err(|e| anyhow!("failed to clear drawing area: {e}"))?;

    let extent = boundary.bounding_radius().max(1.0) * 1.1;
    let (min_x, max_x, min_y, max_y) = (-extent, extent, -extent, extent);

    draw_axes_png(
        &drawing,
        &layout,
        min_x,
        max_x,
        min_y,
        max_y,
        boundary.shape_label(),
    )?;
    draw_boundary_shape_png(
        &drawing, boundary, &layout, min_x, max_x, min_y, max_y, plot,
    )?;

    drawing
        .present()
        .map_err(|e| anyhow!("failed to write preview image: {e}"))
}

fn draw_axes_png(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    shape_label: &str,
) -> Result<()> {
    let ticks = symmetric_ticks(max_x.abs().max(max_y.abs()));
    draw_axes_png_generic(
        drawing,
        layout,
        min_x,
        max_x,
        min_y,
        max_y,
        &ticks,
        &ticks,
        &format!("{} boundary", capitalize(shape_label)),
        "x",
        "y",
    )
}

fn draw_axes_png_generic(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    layout: &PlotLayout,
    data_min_x: f64,
    data_max_x: f64,
    data_min_y: f64,
    data_max_y: f64,
    x_ticks: &[f64],
    y_ticks: &[f64],
    title: &str,
    x_label: &str,
    y_label: &str,
) -> Result<()> {
    let frame_points = vec![
        (layout.data_left, layout.data_top),
        (layout.data_right, layout.data_top),
        (layout.data_right, layout.data_bottom),
        (layout.data_left, layout.data_bottom),
        (layout.data_left, layout.data_top),
    ];
    drawing
        .draw(&PathElement::new(
            to_backend_points(frame_points),
            ShapeStyle::from(&BLACK),
        ))
        .map_err(|e| anyhow!("failed to draw frame: {e}"))?;

    let tick_style_bottom = ("sans-serif", TICK_FONT)
        .into_font()
        .into_text_style(drawing)
        .pos(Pos::new(HPos::Center, VPos::Top));
    let tick_style_left = ("sans-serif", TICK_FONT)
        .into_font()
        .into_text_style(drawing)
        .pos(Pos::new(HPos::Right, VPos::Center));

    for value in x_ticks {
        let x = project_x(*value, data_min_x, data_max_x, layout);
        let tick = vec![
            (x, layout.data_bottom),
            (x, layout.data_bottom + TICK_LENGTH_PX),
        ];
        drawing
            .draw(&PathElement::new(
                to_backend_points(tick),
                ShapeStyle::from(&BLACK),
            ))
            .map_err(|e| anyhow!("failed to draw x tick: {e}"))?;
        let label = format_tick(*value);
        drawing
            .draw_text(
                &label,
                &tick_style_bottom,
                (
                    round_i32(x),
                    round_i32(
                        (layout.data_bottom + TICK_LENGTH_PX + 4.0)
                            .min(layout.total_height() - 5.0),
                    ),
                ),
            )
            .map_err(|e| anyhow!("failed to draw x tick label: {e}"))?;
    }

    for value in y_ticks {
        let y = project_y(*value, data_min_y, data_max_y, layout);
        let tick = vec![
            (layout.data_left, y),
            (layout.data_left - TICK_LENGTH_PX, y),
        ];
        drawing
            .draw(&PathElement::new(
                to_backend_points(tick),
                ShapeStyle::from(&BLACK),
            ))
            .map_err(|e| anyhow!("failed to draw y tick: {e}"))?;
        let label = format_tick(*value);
        drawing
            .draw_text(
                &label,
                &tick_style_left,
                (
                    round_i32((layout.data_left - TICK_LENGTH_PX - 6.0).max(5.0)),
                    round_i32(y),
                ),
            )
            .map_err(|e| anyhow!("failed to draw y tick label: {e}"))?;
    }

    let x_label_style = ("sans-serif", AXIS_FONT)
        .into_font()
        .into_text_style(drawing)
        .pos(Pos::new(HPos::Center, VPos::Top));
    drawing
        .draw_text(
            x_label,
            &x_label_style,
            (
                round_i32(layout.data_center_x()),
                round_i32(
                    (layout.data_bottom + 3.5 * TICK_LENGTH_PX).min(layout.total_height() - 25.0),
                ),
            ),
        )
        .map_err(|e| anyhow!("failed to draw x axis label: {e}"))?;

    let y_label_style = ("sans-serif", AXIS_FONT)
        .into_font()
        .into_text_style(drawing)
        .transform(FontTransform::Rotate90)
        .pos(Pos::new(HPos::Center, VPos::Center));
    drawing
        .draw_text(
            y_label,
            &y_label_style,
            (
                round_i32((layout.data_left - 3.5 * TICK_LENGTH_PX).max(20.0)),
                round_i32(layout.data_center_y()),
            ),
        )
        .map_err(|e| anyhow!("failed to draw y axis label: {e}"))?;

    let title_style = ("sans-serif", TITLE_FONT)
        .into_font()
        .into_text_style(drawing)
        .pos(Pos::new(HPos::Center, VPos::Bottom));
    drawing
        .draw_text(
            title,
            &title_style,
            (
                round_i32(layout.data_center_x()),
                round_i32((layout.data_top - 1.5 * TICK_LENGTH_PX).max(0.0)),
            ),
        )
        .map_err(|e| anyhow!("failed to draw title: {e}"))?;

    Ok(())
}

fn draw_axes_svg_generic(
    svg: &mut String,
    layout: &PlotLayout,
    data_min_x: f64,
    data_max_x: f64,
    data_min_y: f64,
    data_max_y: f64,
    x_ticks: &[f64],
    y_ticks: &[f64],
    title: &str,
    x_label: &str,
    y_label: &str,
    width: f64,
    height: f64,
) {
    writeln!(
        svg,
        r#"<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="none" stroke="black" stroke-width="1"/>"#,
        x = layout.data_left,
        y = layout.data_top,
        w = layout.data_width(),
        h = layout.data_height()
    )
    .unwrap();

    for value in x_ticks {
        let x = project_x(*value, data_min_x, data_max_x, layout);
        let y1 = layout.data_bottom;
        let y2 = layout.data_bottom + TICK_LENGTH_PX;
        writeln!(
            svg,
            r#"<line x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" stroke="black" stroke-width="1"/>"#,
            x = x,
            y1 = y1,
            y2 = y2
        )
        .unwrap();
        let label = format_tick(*value);
        writeln!(
            svg,
            r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{font}" text-anchor="middle" dominant-baseline="hanging">{label}</text>"#,
            x = x,
            y = (y2 + 4.0).min(height - 5.0),
            font = TICK_FONT,
            label = label
        )
        .unwrap();
    }

    for value in y_ticks {
        let y = project_y(*value, data_min_y, data_max_y, layout);
        let x1 = layout.data_left;
        let x2 = layout.data_left - TICK_LENGTH_PX;
        writeln!(
            svg,
            r#"<line x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" stroke="black" stroke-width="1"/>"#,
            x1 = x1,
            x2 = x2,
            y = y
        )
        .unwrap();
        let label = format_tick(*value);
        writeln!(
            svg,
            r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{font}" text-anchor="end" dominant-baseline="middle">{label}</text>"#,
            x = (x2 - 6.0).max(5.0),
            y = y,
            font = TICK_FONT,
            label = label
        )
        .unwrap();
    }

    writeln!(
        svg,
        r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{font}" text-anchor="middle" dominant-baseline="hanging">{x_label}</text>"#,
        x = layout.data_center_x(),
        y = (layout.data_bottom + 3.5 * TICK_LENGTH_PX).min(height - 25.0),
        font = AXIS_FONT,
        x_label = x_label
    )
    .unwrap();
    writeln!(
        svg,
        r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{font}" text-anchor="middle" transform="rotate(-90 {x} {y})">{y_label}</text>"#,
        x = (layout.data_left - 3.5 * TICK_LENGTH_PX).max(20.0),
        y = layout.data_center_y(),
        font = AXIS_FONT,
        y_label = y_label
    )
    .unwrap();
    writeln!(
        svg,
        r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{font}" text-anchor="middle">{title}</text>"#,
        x = width / 2.0,
        y = (layout.data_top - 1.5 * TICK_LENGTH_PX).max(30.0),
        font = TITLE_FONT,
        title = title
    )
    .unwrap();
}

fn draw_boundary_shape_png(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    boundary: &dyn Boundary2D,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    plot: &PlotConfig,
) -> Result<()> {
    let stroke_width = plot.line_width.max(1.0).round() as u32;
    let boundary_style = ShapeStyle::from(&BLACK).stroke_width(stroke_width);

    for segment in boundary.segments() {
        match segment {
            BoundarySegment::Line { start, end } => {
                let points = vec![
                    project_point_layout(start, min_x, max_x, min_y, max_y, layout),
                    project_point_layout(end, min_x, max_x, min_y, max_y, layout),
                ];
                drawing
                    .draw(&PathElement::new(
                        to_backend_points(points),
                        boundary_style.clone(),
                    ))
                    .map_err(|e| anyhow!("failed to draw boundary line: {e}"))?;
            }
            BoundarySegment::Arc {
                center,
                radius,
                start_angle,
                end_angle,
                clockwise,
            } => {
                let samples = sample_arc_points(center, radius, start_angle, end_angle, clockwise);
                let mut projected = Vec::with_capacity(samples.len());
                for point in samples {
                    projected.push(project_point_layout(
                        point, min_x, max_x, min_y, max_y, layout,
                    ));
                }
                drawing
                    .draw(&PathElement::new(
                        to_backend_points(projected),
                        boundary_style.clone(),
                    ))
                    .map_err(|e| anyhow!("failed to draw boundary arc: {e}"))?;
            }
            BoundarySegment::Bezier {
                start,
                ctrl1,
                ctrl2,
                end,
            } => {
                let samples = sample_cubic_points(start, ctrl1, ctrl2, end, BEZIER_SAMPLES);
                let mut projected = Vec::with_capacity(samples.len());
                for point in samples {
                    projected.push(project_point_layout(
                        point, min_x, max_x, min_y, max_y, layout,
                    ));
                }
                drawing
                    .draw(&PathElement::new(
                        to_backend_points(projected),
                        boundary_style.clone(),
                    ))
                    .map_err(|e| anyhow!("failed to draw boundary bezier: {e}"))?;
            }
        }
    }

    Ok(())
}

fn draw_boundary_svg(
    boundary: &dyn Boundary2D,
    plot: &PlotConfig,
    output_path: &Path,
) -> Result<()> {
    let width = plot.width_px.max(100) as f64;
    let height = plot.height_px.max(100) as f64;
    let layout = PlotLayout::new(width, height);

    let extent = boundary.bounding_radius().max(1.0) * 1.1;
    let (min_x, max_x, min_y, max_y) = (-extent, extent, -extent, extent);

    let mut path_data = String::new();
    let mut current: Option<(f64, f64)> = None;
    let data_span = max_x - min_x;
    let radius_scale = layout.data_width() / data_span;

    for segment in boundary.segments() {
        match segment {
            BoundarySegment::Line { start, end } => {
                let start_px = project_point_layout(start, min_x, max_x, min_y, max_y, &layout);
                let end_px = project_point_layout(end, min_x, max_x, min_y, max_y, &layout);
                if !points_match(current, start_px) {
                    write!(&mut path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }
                write!(&mut path_data, "L {:.6} {:.6} ", end_px.0, end_px.1).unwrap();
                current = Some(end_px);
            }
            BoundarySegment::Arc {
                center,
                radius,
                start_angle,
                end_angle,
                clockwise,
            } => {
                let start_pt = point_from_angle(center, radius, start_angle);
                let end_pt = point_from_angle(center, radius, end_angle);

                let start_px = project_point_layout(start_pt, min_x, max_x, min_y, max_y, &layout);
                let end_px = project_point_layout(end_pt, min_x, max_x, min_y, max_y, &layout);

                if !points_match(current, start_px) {
                    write!(&mut path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }

                let sweep_flag = if clockwise { 1 } else { 0 };
                let large_arc = if angular_span(start_angle, end_angle, clockwise) > PI {
                    1
                } else {
                    0
                };
                let radius_px = radius * radius_scale;
                write!(
                    &mut path_data,
                    "A {:.6} {:.6} 0 {} {} {:.6} {:.6} ",
                    radius_px, radius_px, large_arc, sweep_flag, end_px.0, end_px.1
                )
                .unwrap();
                current = Some(end_px);
            }
            BoundarySegment::Bezier {
                start,
                ctrl1,
                ctrl2,
                end,
            } => {
                let start_px = project_point_layout(start, min_x, max_x, min_y, max_y, &layout);
                let ctrl1_px = project_point_layout(ctrl1, min_x, max_x, min_y, max_y, &layout);
                let ctrl2_px = project_point_layout(ctrl2, min_x, max_x, min_y, max_y, &layout);
                let end_px = project_point_layout(end, min_x, max_x, min_y, max_y, &layout);
                if !points_match(current, start_px) {
                    write!(&mut path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }
                write!(
                    &mut path_data,
                    "C {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} ",
                    ctrl1_px.0, ctrl1_px.1, ctrl2_px.0, ctrl2_px.1, end_px.0, end_px.1
                )
                .unwrap();
                current = Some(end_px);
            }
        }
    }

    path_data.push('Z');

    let mut svg = String::new();
    writeln!(
        &mut svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##,
        width = width,
        height = height
    )
    .unwrap();
    writeln!(
        &mut svg,
        r#"<rect x="0" y="0" width="{width}" height="{height}" fill="white"/>"#,
        width = width,
        height = height
    )
    .unwrap();

    let ticks = symmetric_ticks(max_x.abs().max(max_y.abs()));
    draw_axes_svg_generic(
        &mut svg,
        &layout,
        min_x,
        max_x,
        min_y,
        max_y,
        &ticks,
        &ticks,
        &format!("{} boundary", capitalize(boundary.shape_label())),
        "x",
        "y",
        width,
        height,
    );

    // boundary path
    writeln!(
        &mut svg,
        r#"<path d="{path}" fill="none" stroke="black" stroke-width="{stroke}"/>"#,
        path = path_data.trim(),
        stroke = plot.line_width.max(1.0)
    )
    .unwrap();

    writeln!(&mut svg, "</svg>").unwrap();

    fs::write(output_path, svg)
        .with_context(|| format!("failed to write SVG file {}", output_path.display()))
}

fn sample_arc_points(
    center: Point2<f64>,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
    clockwise: bool,
) -> Vec<Point2<f64>> {
    let mut points = Vec::with_capacity(ARC_SAMPLES + 1);
    let total = angular_span(start_angle, end_angle, clockwise);
    let step = total / ARC_SAMPLES as f64;

    for i in 0..=ARC_SAMPLES {
        let angle = if clockwise {
            start_angle - step * i as f64
        } else {
            start_angle + step * i as f64
        };
        let normalized = normalize_angle(angle);
        points.push(point_from_angle(center, radius, normalized));
    }
    points
}

fn sample_cubic_points(
    start: Point2<f64>,
    ctrl1: Point2<f64>,
    ctrl2: Point2<f64>,
    end: Point2<f64>,
    samples: usize,
) -> Vec<Point2<f64>> {
    let steps = samples.max(8);
    let mut points = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        points.push(evaluate_cubic(start, ctrl1, ctrl2, end, t));
    }
    points
}

fn evaluate_cubic(
    start: Point2<f64>,
    ctrl1: Point2<f64>,
    ctrl2: Point2<f64>,
    end: Point2<f64>,
    t: f64,
) -> Point2<f64> {
    let mt = 1.0 - t;
    let mt2 = mt * mt;
    let t2 = t * t;
    let a = mt2 * mt;
    let b = 3.0 * mt2 * t;
    let c = 3.0 * mt * t2;
    let d = t2 * t;
    Point2::new(
        start.x * a + ctrl1.x * b + ctrl2.x * c + end.x * d,
        start.y * a + ctrl1.y * b + ctrl2.y * c + end.y * d,
    )
}

fn angular_span(start: f64, end: f64, clockwise: bool) -> f64 {
    if clockwise {
        let mut diff = start - end;
        if diff <= 0.0 {
            diff += TAU;
        }
        diff
    } else {
        let mut diff = end - start;
        if diff <= 0.0 {
            diff += TAU;
        }
        diff
    }
}

fn normalize_angle(mut angle: f64) -> f64 {
    while angle < 0.0 {
        angle += TAU;
    }
    while angle >= TAU {
        angle -= TAU;
    }
    angle
}

fn point_from_angle(center: Point2<f64>, radius: f64, angle: f64) -> Point2<f64> {
    Point2::new(
        center.x + radius * angle.cos(),
        center.y + radius * angle.sin(),
    )
}

fn symmetric_ticks(max_abs: f64) -> Vec<f64> {
    let step = nice_step(max_abs, 5);
    if step <= 0.0 {
        return vec![0.0];
    }
    let count = (max_abs / step).ceil() as i32;
    let mut ticks = Vec::new();
    for i in -count..=count {
        let value = (i as f64) * step;
        if value.abs() <= max_abs + 1e-9 {
            ticks.push(value);
        }
    }
    if ticks.is_empty() {
        ticks.push(0.0);
    }
    ticks
}

fn nice_step(max_abs: f64, desired: usize) -> f64 {
    if max_abs <= f64::EPSILON {
        return 1.0;
    }
    let range = max_abs * 2.0;
    let desired = desired.max(2) as f64;
    nice_number(range / (desired - 1.0), true)
}

fn nice_number(value: f64, round: bool) -> f64 {
    if value == 0.0 {
        return 0.0;
    }
    let exponent = value.abs().log10().floor();
    let fraction = value / 10f64.powf(exponent);
    let nice_fraction = if round {
        if fraction < 1.5 {
            1.0
        } else if fraction < 3.0 {
            2.0
        } else if fraction < 7.0 {
            5.0
        } else {
            10.0
        }
    } else if fraction <= 1.0 {
        1.0
    } else if fraction <= 2.0 {
        2.0
    } else if fraction <= 5.0 {
        5.0
    } else {
        10.0
    };
    nice_fraction * 10f64.powf(exponent)
}

fn format_tick(value: f64) -> String {
    if value.abs() < 1e-9 {
        "0".to_string()
    } else {
        let rounded = (value * 1000.0).round() / 1000.0;
        let mut text = format!("{rounded:.3}");
        while text.contains('.') && text.ends_with('0') {
            text.pop();
        }
        if text.ends_with('.') {
            text.pop();
        }
        text
    }
}

fn capitalize(text: &str) -> String {
    let mut chars = text.chars();
    match chars.next() {
        None => String::new(),
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
    }
}

#[derive(Clone, Copy)]
struct PlotLayout {
    _width: f64,
    height: f64,
    data_left: f64,
    data_right: f64,
    data_top: f64,
    data_bottom: f64,
}

impl PlotLayout {
    fn new(width: f64, height: f64) -> Self {
        let left = LEFT_MARGIN.min(width / 2.0);
        let right = RIGHT_MARGIN.min(width / 2.0);
        let top = TOP_MARGIN.min(height / 2.0);
        let bottom = BOTTOM_MARGIN.min(height / 2.0);

        let avail_width = (width - left - right).max(10.0);
        let avail_height = (height - top - bottom).max(10.0);
        let data_size = avail_width.min(avail_height);
        let extra_w = (avail_width - data_size) / 2.0;
        let extra_h = (avail_height - data_size) / 2.0;
        let data_left = left + extra_w;
        let data_right = data_left + data_size;
        let data_top = top + extra_h;
        let data_bottom = data_top + data_size;

        Self {
            _width: width,
            height,
            data_left,
            data_right,
            data_top,
            data_bottom,
        }
    }

    fn data_width(&self) -> f64 {
        self.data_right - self.data_left
    }

    fn data_height(&self) -> f64 {
        self.data_bottom - self.data_top
    }

    fn data_center_x(&self) -> f64 {
        (self.data_left + self.data_right) / 2.0
    }

    fn data_center_y(&self) -> f64 {
        (self.data_top + self.data_bottom) / 2.0
    }

    fn total_height(&self) -> f64 {
        self.height
    }
}

fn project_point_layout(
    point: Point2<f64>,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    layout: &PlotLayout,
) -> (f64, f64) {
    (
        project_x(point.x, min_x, max_x, layout),
        project_y(point.y, min_y, max_y, layout),
    )
}

fn project_x(value: f64, min: f64, max: f64, layout: &PlotLayout) -> f64 {
    let span = (max - min).max(f64::EPSILON);
    layout.data_left + (value - min) / span * layout.data_width()
}

fn project_y(value: f64, min: f64, max: f64, layout: &PlotLayout) -> f64 {
    let span = (max - min).max(f64::EPSILON);
    layout.data_bottom - (value - min) / span * layout.data_height()
}

fn points_match(a: Option<(f64, f64)>, b: (f64, f64)) -> bool {
    if let Some((ax, ay)) = a {
        (ax - b.0).abs() < 1e-6 && (ay - b.1).abs() < 1e-6
    } else {
        false
    }
}

fn to_backend_points(points: Vec<(f64, f64)>) -> Vec<(i32, i32)> {
    points
        .into_iter()
        .map(|(x, y)| (round_i32(x), round_i32(y)))
        .collect()
}

fn round_i32(value: f64) -> i32 {
    value.round() as i32
}

fn draw_trajectory_segments_png(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    plot: &PlotConfig,
) -> Result<()> {
    let style = ShapeStyle::from(&BLACK).stroke_width(plot.line_width.max(1.0).round() as u32);
    for segment in samples.trajectory() {
        let start =
            project_point_layout(segment.start.position, min_x, max_x, min_y, max_y, layout);
        let end = project_point_layout(segment.end.position, min_x, max_x, min_y, max_y, layout);
        drawing
            .draw(&PathElement::new(
                to_backend_points(vec![start, end]),
                style.clone(),
            ))
            .map_err(|e| anyhow!("failed to draw trajectory segment: {e}"))?;
    }
    Ok(())
}

fn draw_trajectory_segments_svg(
    svg: &mut String,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    stroke: f64,
) {
    for segment in samples.trajectory() {
        let start =
            project_point_layout(segment.start.position, min_x, max_x, min_y, max_y, layout);
        let end = project_point_layout(segment.end.position, min_x, max_x, min_y, max_y, layout);
        writeln!(
            svg,
            r#"<path d="M {:.6} {:.6} L {:.6} {:.6}" fill="none" stroke="black" stroke-width="{stroke}"/>"#,
            start.0,
            start.1,
            end.0,
            end.1,
            stroke = stroke
        )
        .unwrap();
    }
}

fn draw_phase_segments_png(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_vx: f64,
    max_vx: f64,
    plot: &PlotConfig,
) -> Result<()> {
    let style = ShapeStyle::from(&BLACK).stroke_width(plot.line_width.max(1.0).round() as u32);
    for segment in samples.phase_space() {
        let start = (
            project_x(segment.start_x, min_x, max_x, layout),
            project_y(segment.vx, min_vx, max_vx, layout),
        );
        let end = (
            project_x(segment.end_x, min_x, max_x, layout),
            project_y(segment.vx, min_vx, max_vx, layout),
        );
        drawing
            .draw(&PathElement::new(
                to_backend_points(vec![start, end]),
                style.clone(),
            ))
            .map_err(|e| anyhow!("failed to draw phase segment: {e}"))?;
    }
    Ok(())
}

fn draw_phase_segments_svg(
    svg: &mut String,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_vx: f64,
    max_vx: f64,
    stroke: f64,
) {
    for segment in samples.phase_space() {
        let start = (
            project_x(segment.start_x, min_x, max_x, layout),
            project_y(segment.vx, min_vx, max_vx, layout),
        );
        let end = (
            project_x(segment.end_x, min_x, max_x, layout),
            project_y(segment.vx, min_vx, max_vx, layout),
        );
        writeln!(
            svg,
            r#"<path d="M {:.6} {:.6} L {:.6} {:.6}" fill="none" stroke="black" stroke-width="{stroke}"/>"#,
            start.0,
            start.1,
            end.0,
            end.1,
            stroke = stroke
        )
        .unwrap();
    }
}

fn draw_poincare_points_png(
    drawing: &DrawingArea<BitMapBackend, Shift>,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_vx: f64,
    max_vx: f64,
) -> Result<()> {
    for point in samples.poincare() {
        let x = project_x(point.x, min_x, max_x, layout);
        let y = project_y(point.vx, min_vx, max_vx, layout);
        drawing
            .draw(&Circle::new(
                (round_i32(x), round_i32(y)),
                3,
                BLACK.filled(),
            ))
            .map_err(|e| anyhow!("failed to draw Poincaré point: {e}"))?;
    }
    Ok(())
}

fn draw_poincare_points_svg(
    svg: &mut String,
    samples: &SimulationSamples,
    layout: &PlotLayout,
    min_x: f64,
    max_x: f64,
    min_vx: f64,
    max_vx: f64,
) {
    for point in samples.poincare() {
        let x = project_x(point.x, min_x, max_x, layout);
        let y = project_y(point.vx, min_vx, max_vx, layout);
        writeln!(
            svg,
            r#"<circle cx="{x}" cy="{y}" r="3" fill="black"/>"#,
            x = x,
            y = y
        )
        .unwrap();
    }
}

fn ticks_for_range(min: f64, max: f64) -> Vec<f64> {
    if !min.is_finite() || !max.is_finite() || (max - min).abs() < f64::EPSILON {
        return vec![-1.0, -0.5, 0.0, 0.5, 1.0];
    }
    let span = max - min;
    let rough_step = nice_number(span / 4.0, true);
    if !rough_step.is_finite() || rough_step <= 0.0 {
        return vec![min, (min + max) / 2.0, max];
    }
    let start = (min / rough_step).floor() * rough_step;
    let end = (max / rough_step).ceil() * rough_step;
    let mut ticks = Vec::new();
    let mut value = start;
    while value <= end + 1e-9 {
        if value >= min - 1e-9 && value <= max + 1e-9 {
            ticks.push(value);
        }
        value += rough_step;
    }
    if ticks.is_empty() {
        ticks.push(min);
        if (max - min).abs() > f64::EPSILON {
            let mid = (min + max) / 2.0;
            if (mid - min).abs() > 1e-9 {
                ticks.push(mid);
            }
            ticks.push(max);
        }
    }
    ticks
}

fn build_boundary_path_svg(
    boundary: &dyn Boundary2D,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    layout: &PlotLayout,
    stroke: f64,
    out: &mut String,
) {
    let mut path_data = String::new();
    let mut current: Option<(f64, f64)> = None;
    let radius_scale = layout.data_width() / (max_x - min_x).max(f64::EPSILON);
    for segment in boundary.segments() {
        match segment {
            BoundarySegment::Line { start, end } => {
                let start_px = project_point_layout(start, min_x, max_x, min_y, max_y, layout);
                let end_px = project_point_layout(end, min_x, max_x, min_y, max_y, layout);
                if !points_match(current, start_px) {
                    write!(path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }
                write!(path_data, "L {:.6} {:.6} ", end_px.0, end_px.1).unwrap();
                current = Some(end_px);
            }
            BoundarySegment::Arc {
                center,
                radius,
                start_angle,
                end_angle,
                clockwise,
            } => {
                let start_pt = point_from_angle(center, radius, start_angle);
                let end_pt = point_from_angle(center, radius, end_angle);
                let start_px = project_point_layout(start_pt, min_x, max_x, min_y, max_y, layout);
                let end_px = project_point_layout(end_pt, min_x, max_x, min_y, max_y, layout);
                if !points_match(current, start_px) {
                    write!(path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }
                let sweep_flag = if clockwise { 1 } else { 0 };
                let large_arc = if angular_span(start_angle, end_angle, clockwise) > PI {
                    1
                } else {
                    0
                };
                let radius_px = radius * radius_scale;
                write!(
                    path_data,
                    "A {:.6} {:.6} 0 {} {} {:.6} {:.6} ",
                    radius_px, radius_px, large_arc, sweep_flag, end_px.0, end_px.1
                )
                .unwrap();
                current = Some(end_px);
            }
            BoundarySegment::Bezier {
                start,
                ctrl1,
                ctrl2,
                end,
            } => {
                let start_px = project_point_layout(start, min_x, max_x, min_y, max_y, layout);
                let ctrl1_px = project_point_layout(ctrl1, min_x, max_x, min_y, max_y, layout);
                let ctrl2_px = project_point_layout(ctrl2, min_x, max_x, min_y, max_y, layout);
                let end_px = project_point_layout(end, min_x, max_x, min_y, max_y, layout);
                if !points_match(current, start_px) {
                    write!(path_data, "M {:.6} {:.6} ", start_px.0, start_px.1).unwrap();
                }
                write!(
                    path_data,
                    "C {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} ",
                    ctrl1_px.0, ctrl1_px.1, ctrl2_px.0, ctrl2_px.1, end_px.0, end_px.1
                )
                .unwrap();
                current = Some(end_px);
            }
        }
    }
    path_data.push('Z');
    writeln!(
        out,
        r#"<path d="{path}" fill="none" stroke="black" stroke-width="{stroke}"/>"#,
        path = path_data.trim(),
        stroke = stroke
    )
    .unwrap();
}
