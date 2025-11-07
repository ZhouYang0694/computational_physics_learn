use anyhow::{Result, anyhow};
use serde::Serialize;

use crate::config::SimulationParams;
use crate::state::RigidBodyState;

#[derive(Debug, Clone, Serialize)]
pub struct SimulationSample {
    pub time: f64,
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
    pub theta_primary: f64,
    pub theta_wrapped_primary: f64,
    pub omega_primary: f64,
    pub theta_secondary: f64,
    pub theta_wrapped_secondary: f64,
    pub omega_secondary: f64,
    pub delta_theta: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SimulationMetadata {
    pub mu: f64,
    pub dt: f64,
    pub total_time: f64,
    pub initial_state: RigidBodyState,
    pub delta_theta0: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct SimulationResult {
    pub metadata: SimulationMetadata,
    pub samples: Vec<SimulationSample>,
}

pub fn propagate(params: &SimulationParams) -> Result<SimulationResult> {
    let mut x = params.initial_state.x;
    let mut y = params.initial_state.y;
    let mut vx = params.initial_state.vx;
    let mut vy = params.initial_state.vy;

    let mut theta_primary = params.initial_state.theta;
    let mut omega_primary = params.initial_state.omega;

    let mut theta_secondary = params.initial_state.theta + params.delta_theta0;
    let mut omega_secondary = params.initial_state.omega;

    let dt = params.dt;
    let total_steps = (params.total_time / dt).ceil() as usize;

    let mut samples = Vec::with_capacity(total_steps + 1);
    samples.push(make_sample(
        0.0,
        x,
        y,
        vx,
        vy,
        theta_primary,
        omega_primary,
        theta_secondary,
        omega_secondary,
    ));

    let mu = params.mu;

    for step in 0..total_steps {
        let radius = (x * x + y * y).sqrt();
        if radius <= f64::EPSILON {
            return Err(anyhow!(
                "Radius collapsed to zero at step {} (t = {:.6})",
                step,
                step as f64 * dt
            ));
        }

        let r2 = radius * radius;
        let r3 = r2 * radius;
        let r5 = r3 * r2;

        let ax = -mu * x / r3;
        let ay = -mu * y / r3;

        let torque_primary = orientation_torque(x, y, theta_primary);
        let torque_secondary = orientation_torque(x, y, theta_secondary);
        let domega_primary = -3.0 * mu / r5 * torque_primary;
        let domega_secondary = -3.0 * mu / r5 * torque_secondary;

        vx += ax * dt;
        vy += ay * dt;
        omega_primary += domega_primary * dt;
        omega_secondary += domega_secondary * dt;

        x += vx * dt;
        y += vy * dt;
        theta_primary += omega_primary * dt;
        theta_secondary += omega_secondary * dt;

        let time = ((step + 1) as f64) * dt;
        samples.push(make_sample(
            time,
            x,
            y,
            vx,
            vy,
            theta_primary,
            omega_primary,
            theta_secondary,
            omega_secondary,
        ));
    }

    Ok(SimulationResult {
        metadata: SimulationMetadata {
            mu,
            dt,
            total_time: params.total_time,
            initial_state: params.initial_state,
            delta_theta0: params.delta_theta0,
        },
        samples,
    })
}

fn orientation_torque(x: f64, y: f64, theta: f64) -> f64 {
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    (x * sin_theta - y * cos_theta) * (x * cos_theta + y * sin_theta)
}

fn make_sample(
    time: f64,
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
    theta_primary: f64,
    omega_primary: f64,
    theta_secondary: f64,
    omega_secondary: f64,
) -> SimulationSample {
    let radius = (x * x + y * y).sqrt();
    let theta_wrapped_primary = RigidBodyState::wrap_theta(theta_primary);
    let theta_wrapped_secondary = RigidBodyState::wrap_theta(theta_secondary);

    let delta_raw = (theta_primary - theta_secondary).abs();
    let delta_wrapped = RigidBodyState::wrap_theta(delta_raw).abs();

    SimulationSample {
        time,
        x,
        y,
        vx,
        vy,
        radius,
        theta_primary,
        theta_wrapped_primary,
        omega_primary,
        theta_secondary,
        theta_wrapped_secondary,
        omega_secondary,
        delta_theta: delta_wrapped,
    }
}
