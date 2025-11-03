use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::boundary::Boundary2D;
use crate::config::{OutputConfig, PlotConfig};
use crate::render;
use crate::sampling::{HistoryRow, SimulationSamples};

pub struct OutputBundle<'a> {
    pub samples: &'a SimulationSamples,
    pub plot: &'a PlotConfig,
    pub output: &'a OutputConfig,
    pub output_dir: &'a Path,
    pub boundary: Option<&'a dyn Boundary2D>,
}

impl<'a> OutputBundle<'a> {
    pub fn write_csv(&self) -> Result<Vec<PathBuf>> {
        if !self.output.export_csv {
            return Ok(Vec::new());
        }
        fs::create_dir_all(self.output_dir).with_context(|| {
            format!(
                "failed to create output directory {}",
                self.output_dir.display()
            )
        })?;

        let mut written = Vec::new();
        let history_path = self
            .output_dir
            .join(format!("{}_history.csv", self.output.base_name));
        write_history_csv(history_path.as_path(), self.samples.history())?;
        written.push(history_path);

        Ok(written)
    }

    pub fn write_plots(&self) -> Result<Vec<PathBuf>> {
        let mut files = Vec::new();
        files.extend(render::draw_trajectory_plot(
            self.boundary,
            self.samples,
            self.plot,
            self.output,
            self.output_dir,
        )?);

        files.extend(render::draw_phase_plot(
            self.samples,
            self.plot,
            self.output,
            self.output_dir,
        )?);

        files.extend(render::draw_poincare_plot(
            self.samples,
            self.plot,
            self.output,
            self.output_dir,
        )?);

        Ok(files)
    }
}

fn write_history_csv(path: &Path, rows: &[HistoryRow]) -> Result<()> {
    let file = File::create(path)
        .with_context(|| format!("failed to create history CSV {}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "t,bounce_id,x,y,vx,vy,event").context("failed to write CSV header")?;

    for row in rows {
        let (vx, vy) = (row.velocity.x, row.velocity.y);
        let event = row
            .event
            .map(event_label)
            .unwrap_or_else(|| "none".to_string());
        writeln!(
            writer,
            "{:.12},{},{:.12},{:.12},{:.12},{:.12},{}",
            row.time, row.bounce_id, row.position.x, row.position.y, vx, vy, event
        )
        .context("failed to write CSV row")?;
    }

    writer.flush().context("failed to flush CSV writer")
}

fn event_label(event: crate::sampling::SampleEvent) -> String {
    match event {
        crate::sampling::SampleEvent::SegmentStart => "segment_start",
        crate::sampling::SampleEvent::SegmentEnd => "segment_end",
        crate::sampling::SampleEvent::Collision => "collision",
        crate::sampling::SampleEvent::Poincare => "poincare",
    }
    .to_string()
}
