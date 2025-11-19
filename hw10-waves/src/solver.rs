use anyhow::{Result, ensure};

use crate::config::SimulationConfig;

#[derive(Debug, Clone)]
pub struct Grid {
    pub positions: Vec<f64>,
    pub dx: f64,
}

impl Grid {
    pub fn new(length: f64, dx: f64) -> Result<Self> {
        ensure!(dx > 0.0, "dx must be positive");
        let steps_f = length / dx;
        let steps = steps_f.round() as usize;
        let approx_length = steps as f64 * dx;
        ensure!(
            (approx_length - length).abs() <= dx * 1e-8,
            "string length must be an integer multiple of dx"
        );
        ensure!(
            (steps_f - steps as f64).abs() <= 1e-8,
            "string length divided by dx must be close to an integer"
        );

        let points = steps + 1;
        ensure!(points >= 3, "grid must contain at least three nodes");

        let positions = (0..points).map(|i| i as f64 * dx).collect();
        Ok(Self { positions, dx })
    }

    pub fn len(&self) -> usize {
        self.positions.len()
    }
}

pub struct Solver {
    grid: Grid,
    dt: f64,
    wave_speed: f64,
    cfl_sq: f64,
}

impl Solver {
    pub fn new(length: f64, sim: &SimulationConfig, wave_speed: f64) -> Result<Self> {
        let grid = Grid::new(length, sim.dx)?;
        let cfl = wave_speed * sim.dt / grid.dx;
        let cfl_sq = cfl * cfl;
        Ok(Self {
            grid,
            dt: sim.dt,
            wave_speed,
            cfl_sq,
        })
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn dt(&self) -> f64 {
        self.dt
    }

    pub fn cfl(&self) -> f64 {
        self.wave_speed * self.dt / self.grid.dx
    }

    pub fn run<F>(
        &self,
        mut displacement0: Vec<f64>,
        velocity0: Vec<f64>,
        total_steps: usize,
        output_interval_steps: usize,
        mut callback: F,
    ) -> Result<()>
    where
        F: FnMut(usize, f64, &[f64]) -> Result<()>,
    {
        let points = self.grid.len();
        ensure!(
            displacement0.len() == points,
            "displacement length mismatch"
        );
        ensure!(velocity0.len() == points, "velocity length mismatch");
        ensure!(
            output_interval_steps >= 1,
            "output interval must be at least one step"
        );

        // Step 0
        callback(0, 0.0, &displacement0)?;

        if total_steps == 0 {
            return Ok(());
        }

        let mut current = vec![0.0; points];
        let mut next = vec![0.0; points];

        self.first_step(&mut current, &displacement0, &velocity0);
        let mut step = 1;
        let mut time = self.dt;
        if step % output_interval_steps == 0 {
            callback(step, time, &current)?;
        }

        while step < total_steps {
            self.advance(&mut next, &current, &displacement0);
            displacement0.copy_from_slice(&current);
            current.copy_from_slice(&next);
            step += 1;
            time = step as f64 * self.dt;
            if step % output_interval_steps == 0 {
                callback(step, time, &current)?;
            }
        }

        Ok(())
    }

    fn first_step(&self, next: &mut [f64], current: &[f64], velocity: &[f64]) {
        let n = current.len();
        next[0] = 0.0;
        next[n - 1] = 0.0;
        for i in 1..n - 1 {
            let laplacian = current[i + 1] + current[i - 1] - 2.0 * current[i];
            next[i] = current[i] + self.dt * velocity[i] + 0.5 * self.cfl_sq * laplacian;
        }
    }

    fn advance(&self, next: &mut [f64], current: &[f64], previous: &[f64]) {
        let n = current.len();
        next[0] = 0.0;
        next[n - 1] = 0.0;
        for i in 1..n - 1 {
            let laplacian = current[i + 1] + current[i - 1] - 2.0 * current[i];
            next[i] = 2.0 * current[i] - previous[i] + self.cfl_sq * laplacian;
        }
    }
}
