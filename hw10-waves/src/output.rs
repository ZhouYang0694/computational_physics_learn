use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

use anyhow::{Context, Result, ensure};
use plotters::prelude::*;

use crate::config::OutputConfig;

pub struct OutputWriter {
    target_dir: PathBuf,
    write_csv: bool,
    canvas_size: (u32, u32),
    padding: Padding,
    y_limit: f64,
    x_min: f64,
    x_max: f64,
}

struct Padding {
    left: u32,
    right: u32,
    top: u32,
    bottom: u32,
}

impl OutputWriter {
    pub fn new(config: &OutputConfig, x_min: f64, x_max: f64, y_limit: f64) -> Result<Self> {
        ensure!(y_limit > 0.0, "y_limit must be positive");
        ensure!(x_max > x_min, "x range must have positive length");

        let dir = config.resolved_path();
        fs::create_dir_all(&dir)
            .with_context(|| format!("failed to create output directory {:?}", dir))?;

        let scale = config.pixels_per_unit;
        ensure!(scale > 0.0, "pixels_per_unit must be positive");
        let x_span = x_max - x_min;
        let plot_width = ((x_span * scale).ceil() as u32).max(1);
        let plot_height = ((2.0 * y_limit * scale).ceil() as u32).max(1);

        let padding = Padding {
            left: 80,
            right: 30,
            top: 20,
            bottom: 60,
        };

        let canvas_size = (
            plot_width + padding.left + padding.right,
            plot_height + padding.top + padding.bottom,
        );

        Ok(Self {
            target_dir: dir,
            write_csv: config.write_csv,
            canvas_size,
            padding,
            y_limit,
            x_min,
            x_max,
        })
    }

    pub fn write_step(
        &self,
        step: usize,
        time: f64,
        positions: &[f64],
        displacement: &[f64],
    ) -> Result<()> {
        ensure!(!positions.is_empty(), "positions array must not be empty");
        ensure!(
            positions.len() == displacement.len(),
            "positions and displacement lengths differ"
        );
        self.write_plot(step, time, positions, displacement)?;
        if self.write_csv {
            self.write_csv_data(step, time, positions, displacement)?;
        }
        Ok(())
    }

    fn write_plot(
        &self,
        step: usize,
        time: f64,
        positions: &[f64],
        displacement: &[f64],
    ) -> Result<()> {
        let filename = format!("step_{step:05}.png");
        let path = self.target_dir.join(filename);
        let root = BitMapBackend::new(&path, self.canvas_size).into_drawing_area();
        root.fill(&WHITE)?;

        let mut chart = ChartBuilder::on(&root)
            .margin(self.padding.top)
            .set_label_area_size(LabelAreaPosition::Left, self.padding.left)
            .set_label_area_size(LabelAreaPosition::Bottom, self.padding.bottom)
            .caption(format!("t = {time:.4} s"), ("sans-serif", 24))
            .build_cartesian_2d(self.x_min..self.x_max, -self.y_limit..self.y_limit)?;

        let axis_style = ShapeStyle::from(&BLACK).stroke_width(2);

        chart
            .configure_mesh()
            .disable_mesh()
            .x_desc("x (m)")
            .y_desc("y (m)")
            .axis_style(axis_style)
            .draw()?;

        chart.draw_series(LineSeries::new(
            positions.iter().copied().zip(displacement.iter().copied()),
            &BLACK,
        ))?;

        root.present()?;
        Ok(())
    }

    fn write_csv_data(
        &self,
        step: usize,
        time: f64,
        positions: &[f64],
        displacement: &[f64],
    ) -> Result<()> {
        let filename = format!("step_{step:05}.csv");
        let path = self.target_dir.join(filename);
        let mut file =
            File::create(&path).with_context(|| format!("failed to create csv file {:?}", path))?;
        writeln!(file, "# step={step}, time={time}")?;
        writeln!(file, "x,y")?;
        for (x, y) in positions.iter().zip(displacement.iter()) {
            writeln!(file, "{x},{y}")?;
        }
        Ok(())
    }
}
