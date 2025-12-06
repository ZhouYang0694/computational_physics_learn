//! Core utilities for simulating the 2D Ising model with periodic boundary conditions.
//! The implementation is deliberately small and geared toward classroom-scale Monte Carlo
//! experiments, so the interface exposes the essential physics pieces: lattice storage,
//! Metropolis sweep, and observables (energy, magnetization).

use rand::Rng;

/// Lattice initialization strategy.
#[derive(Clone, Copy, Debug)]
pub enum InitMode {
    /// All spins start aligned up (+1).
    AllUp,
    /// Spins are drawn randomly from {+1, -1}.
    Random,
}

/// Square lattice storing spins as ±1.
pub struct Lattice {
    size: usize,
    spins: Vec<i8>,
}

impl Lattice {
    /// Create a new lattice of side length `size` using the chosen initialization.
    pub fn new(size: usize, init: InitMode, rng: &mut impl Rng) -> Self {
        let mut spins = Vec::with_capacity(size * size);
        match init {
            InitMode::AllUp => spins.resize(size * size, 1),
            InitMode::Random => {
                for _ in 0..size * size {
                    let spin = if rng.gen_bool(0.5) { 1 } else { -1 };
                    spins.push(spin);
                }
            }
        }
        Self { size, spins }
    }

    /// Perform one Metropolis sweep (L² trials) at temperature `temperature` with external field `field`.
    ///
    /// The acceptance uses ΔE = 2 s_i (Σ_neighbors s_j + H) with J = 1, k_B = 1.
    pub fn sweep(&mut self, rng: &mut impl Rng, temperature: f64, field: f64) {
        let l = self.size;
        let trials = l * l;
        for _ in 0..trials {
            let x = rng.gen_range(0..l);
            let y = rng.gen_range(0..l);
            let idx = self.index(x, y);
            let spin = self.spins[idx] as f64;
            let neighbor_sum = self.local_neighbor_sum(x, y) as f64;
            let delta_e = 2.0 * spin * (neighbor_sum + field);
            if delta_e <= 0.0 || rng.gen::<f64>() < (-delta_e / temperature).exp() {
                self.spins[idx] = -self.spins[idx];
            }
        }
    }

    /// Total magnetization M = Σ s_i.
    pub fn total_magnetization(&self) -> f64 {
        self.spins.iter().map(|&s| s as f64).sum()
    }

    /// Magnetization per spin m = M / N.
    pub fn magnetization_per_spin(&self) -> f64 {
        self.total_magnetization() / (self.spins.len() as f64)
    }

    /// Total energy E = - Σ⟨ij⟩ s_i s_j - H Σ s_i with periodic boundary conditions.
    /// Pair interactions are counted once using right and down neighbors.
    pub fn total_energy(&self, field: f64) -> f64 {
        let l = self.size;
        let mut interaction = 0.0;
        for y in 0..l {
            for x in 0..l {
                let s = self.spins[self.index(x, y)] as f64;
                let s_right = self.spins[self.index((x + 1) % l, y)] as f64;
                let s_down = self.spins[self.index(x, (y + 1) % l)] as f64;
                interaction += -s * (s_right + s_down);
            }
        }
        let field_term = -field * self.total_magnetization();
        interaction + field_term
    }

    /// Energy per spin e = E / N.
    pub fn energy_per_spin(&self, field: f64) -> f64 {
        self.total_energy(field) / (self.spins.len() as f64)
    }

    /// Current lattice side length.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Internal: map (x, y) -> linear index.
    #[inline]
    fn index(&self, x: usize, y: usize) -> usize {
        y * self.size + x
    }

    /// Sum of four nearest neighbors for site (x, y) with periodic boundaries.
    #[inline]
    fn local_neighbor_sum(&self, x: usize, y: usize) -> i8 {
        let l = self.size as isize;
        let xi = x as isize;
        let yi = y as isize;
        let neighbors = [
            ((xi + 1).rem_euclid(l) as usize, y),
            ((xi - 1).rem_euclid(l) as usize, y),
            (x, (yi + 1).rem_euclid(l) as usize),
            (x, (yi - 1).rem_euclid(l) as usize),
        ];
        neighbors
            .iter()
            .map(|&(nx, ny)| self.spins[self.index(nx, ny)])
            .sum()
    }
}
