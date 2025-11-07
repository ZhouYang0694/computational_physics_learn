use serde::Serialize;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct RigidBodyState {
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub theta: f64,
    pub omega: f64,
}

impl RigidBodyState {
    pub fn radius(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    #[allow(dead_code)]
    pub fn speed(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy).sqrt()
    }

    pub fn wrap_theta(theta: f64) -> f64 {
        let two_pi = 2.0 * PI;
        let mut wrapped = (theta + PI) % two_pi;
        if wrapped < 0.0 {
            wrapped += two_pi;
        }
        wrapped - PI
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Serialize)]
pub struct DumbbellPairState {
    pub primary: RigidBodyState,
    pub secondary: RigidBodyState,
    pub delta_theta0: f64,
}

#[allow(dead_code)]
impl DumbbellPairState {
    pub fn new(primary: RigidBodyState, secondary: RigidBodyState, delta_theta0: f64) -> Self {
        Self {
            primary,
            secondary,
            delta_theta0,
        }
    }
}
