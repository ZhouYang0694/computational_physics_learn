# Wave Equation Finite-Difference Experiments

This project solves the 1D wave equation for a vibrating string with fixed endpoints using the explicit second-order finite-difference stencil you specified. Configuration is read from TOML files, allowing you to change the grid spacing, time step, frame count, and Gaussian initial-condition parameters without recompiling.

## Running simulations

```
cargo run --release -- <path-to-config>
```

Two ready-to-run scenarios live under `configs/`:

- `configs/stability_scan.toml` — Gaussian displacement with zero initial velocity. Adjust `dt`/`dx` in the file to inspect the CFL stability limit.
- `configs/right_traveling.toml` — Gaussian packet configured to travel to the right without splitting.

Results are written as PNG plots (and optional CSV data) under `output/<directory>/` where `<directory>` comes from the configuration file. Each plot uses a fixed axis frame: the y-axis remains symmetric about zero with a 10% amplitude buffer taken from the initial Gaussian, and the x/y scales reflect physical units using `output.pixels_per_unit` to control clarity. The `output.cadence` field now specifies how many frames to save (initial snapshot included), distributed uniformly from start to finish.

## Configuration overview

```toml
[simulation]
dt = 1e-5      # time step (s)
dx = 0.005     # spatial step (m)
total_time = 0.01

[physical]
wave_speed = 300.0
string_length = 1.0

[gaussian]
amplitude = 1.0
center = 0.5
k = 400.0
velocity = "zero"          # or "right_traveling"

[output]
base_dir = "output"
directory = "stability_scan"
cadence = 50          # total number of frames (including the initial step)
write_csv = false
pixels_per_unit = 800.0
```

If `string_length` is not an integer multiple of `dx`, or `total_time` is not an integer multiple of `dt`, the program automatically extends them to the next admissible multiple and reports the adjustment in the terminal.

## Single-direction traveling packet

A right-moving solution of the continuous wave equation can be written as `y(x, t) = f(x - c t)` for any smooth profile `f`. Differentiating at `t = 0` gives

```
y(x, 0)   = f(x)
∂_t y(x, 0) = -c f'(x)
```

Setting the initial velocity to `y_t(x, 0) = -c f'(x)` therefore yields a packet that travels towards `+x` without reflection (until it meets the fixed boundary). Within the code, choosing `velocity = "right_traveling"` automatically applies this relationship to the Gaussian `f(x) = exp(-k (x - x0)^2)`.

## Next steps

- Modify `dt`/`dx` in the stability configuration to observe when the CFL condition `c Δt / Δx ≤ 1` is violated.
- Enable CSV dumps by setting `output.write_csv = true` to perform quantitative post-processing (energy estimates, etc.).
- Extend `initial_conditions.rs` if you want additional waveforms beyond the Gaussian template.
