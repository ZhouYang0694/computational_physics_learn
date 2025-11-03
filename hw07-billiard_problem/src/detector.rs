use anyhow::{Result, anyhow};

use crate::boundary::{Boundary2D, Collision};
use crate::config::{DetectorConfig, DetectorMode};
use crate::math::Ray2D;
use crate::physics::State2D;
use std::f64;

#[derive(Debug)]
pub enum DetectorError {
    OutsideBoundary,
    TangentialImpact,
    IterationOverflow,
    ZeroVelocity,
    NumericalFailure(&'static str),
}

#[derive(Debug, Clone)]
pub struct CollisionEvent {
    pub time_of_impact: f64,
    pub collision: Collision,
}

pub trait CollisionDetector: Send {
    fn name(&self) -> &'static str;
    fn next_collision(
        &mut self,
        state: &State2D,
        boundary: &dyn Boundary2D,
        epsilon: f64,
    ) -> Result<CollisionEvent, DetectorError>;
}

pub fn build_detector(config: &DetectorConfig) -> Result<Box<dyn CollisionDetector>> {
    match config.mode {
        DetectorMode::StepBack => {
            let step = config
                .step_size
                .ok_or_else(|| anyhow!("step-back detector requires `step_size`"));
            Ok(Box::new(StepBackDetector::new(step?, config.tolerance)))
        }
        DetectorMode::Analytic => Ok(Box::new(AnalyticDetector::new(config.tolerance))),
    }
}

pub struct StepBackDetector {
    step_size: f64,
    tolerance: f64,
    max_refinements: u32,
}

impl StepBackDetector {
    pub fn new(step_size: f64, tolerance: f64) -> Self {
        Self {
            step_size,
            tolerance,
            max_refinements: 32,
        }
    }
}

impl CollisionDetector for StepBackDetector {
    fn name(&self) -> &'static str {
        "step-back"
    }

    fn next_collision(
        &mut self,
        state: &State2D,
        boundary: &dyn Boundary2D,
        epsilon: f64,
    ) -> Result<CollisionEvent, DetectorError> {
        let speed = state.velocity.norm();
        if speed <= f64::EPSILON {
            return Err(DetectorError::ZeroVelocity);
        }

        if state.time == 0.0 && !boundary.contains_point(&state.position, epsilon) {
            return Err(DetectorError::OutsideBoundary);
        }

        let direction = state.velocity / speed;
        let mut last_inside_distance = 0.0;
        let mut distance = 0.0;
        let mut steps = 0_u32;

        loop {
            if steps > 1_000_000 {
                return Err(DetectorError::IterationOverflow);
            }
            steps += 1;

            distance += self.step_size;
            let candidate_point = state.position + direction * distance;
            if boundary.contains_point(&candidate_point, epsilon) {
                last_inside_distance = distance;
                continue;
            }

            let mut low = last_inside_distance;
            let mut high = distance;
            let mut high_point = candidate_point;

            for _ in 0..self.max_refinements {
                let mid = 0.5 * (low + high);
                let mid_point = state.position + direction * mid;
                if boundary.contains_point(&mid_point, epsilon) {
                    low = mid;
                } else {
                    high = mid;
                    high_point = mid_point;
                }
                if (high - low) <= self.tolerance {
                    break;
                }
            }

            // Prefer the refined outside point as approximation.
            let approx_distance = high;
            let approx_point = high_point;

            // Refine collision using an exact ray intersection for a stable normal/time.
            let ray = state_to_ray(state).map_err(|_| {
                DetectorError::NumericalFailure("failed to normalize ray direction")
            })?;
            let collision = boundary
                .intersect_ray(&ray, epsilon)
                .ok_or(DetectorError::TangentialImpact)?;

            let time = collision.distance / speed;
            if time.is_nan() || !time.is_finite() {
                return Err(DetectorError::NumericalFailure(
                    "invalid collision time computed",
                ));
            }

            if collision.distance <= self.tolerance {
                return Err(DetectorError::TangentialImpact);
            }

            // Guard: ensure the refined result is close to the approximated distance.
            if (collision.distance - approx_distance).abs() > 10.0 * self.tolerance {
                let mut approx_collision = collision.clone();
                approx_collision.distance = approx_distance;
                approx_collision.point = approx_point;
                // If the analytic result deviates significantly from the stepping search,
                // fall back to the approximate point for distance.
                return Ok(CollisionEvent {
                    time_of_impact: approx_distance / speed,
                    collision: approx_collision,
                });
            }

            return Ok(CollisionEvent {
                time_of_impact: time,
                collision,
            });
        }
    }
}

pub struct AnalyticDetector {
    tolerance: f64,
}

impl AnalyticDetector {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }
}

impl CollisionDetector for AnalyticDetector {
    fn name(&self) -> &'static str {
        "analytic"
    }

    fn next_collision(
        &mut self,
        state: &State2D,
        boundary: &dyn Boundary2D,
        epsilon: f64,
    ) -> Result<CollisionEvent, DetectorError> {
        let speed = state.velocity.norm();
        if speed <= f64::EPSILON {
            return Err(DetectorError::ZeroVelocity);
        }

        let ray = state_to_ray(state)
            .map_err(|_| DetectorError::NumericalFailure("failed to normalize ray direction"))?;
        let collision = boundary
            .intersect_ray(&ray, epsilon)
            .ok_or(DetectorError::TangentialImpact)?;

        let time = collision.distance / speed;
        if !time.is_finite() {
            return Err(DetectorError::NumericalFailure(
                "invalid collision time computed",
            ));
        }

        Ok(CollisionEvent {
            time_of_impact: time,
            collision,
        })
    }
}

/// Helper used later to construct rays from state and velocity.
pub fn state_to_ray(state: &State2D) -> Result<Ray2D> {
    Ray2D::new(state.position, state.velocity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boundary::PolygonBoundary;
    use crate::physics::State2D;
    use nalgebra::{Point2, Vector2, point};

    fn square_boundary() -> PolygonBoundary {
        PolygonBoundary::new(vec![
            point![-1.0, -1.0],
            point![1.0, -1.0],
            point![1.0, 1.0],
            point![-1.0, 1.0],
        ])
        .expect("valid square")
    }

    fn make_state(position: Point2<f64>, velocity: Vector2<f64>) -> State2D {
        State2D {
            position,
            velocity,
            time: 0.0,
            bounce_id: 0,
        }
    }

    #[test]
    fn analytic_detector_hits_right_wall() {
        let boundary = square_boundary();
        let mut detector = AnalyticDetector::new(1e-9);
        let state = make_state(point![0.0, 0.0], Vector2::new(1.0, 0.2));
        let event = detector
            .next_collision(&state, &boundary, 1e-9)
            .expect("collision computed");
        assert!(event.time_of_impact > 0.0);
        assert!(event.collision.point.x > 0.99);
        // normal should point in +x (wall normal)
        assert!(event.collision.normal.x > 0.9);
    }

    #[test]
    fn step_back_matches_analytic_for_horizontal_hit() {
        let boundary = square_boundary();
        let state = make_state(point![-0.5, 0.0], Vector2::new(1.0, 0.0));
        let mut step_detector = StepBackDetector::new(0.05, 1e-6);
        let mut analytic = AnalyticDetector::new(1e-9);

        let step_event = step_detector
            .next_collision(&state, &boundary, 1e-9)
            .expect("step-back collision");
        let analytic_event = analytic
            .next_collision(&state, &boundary, 1e-9)
            .expect("analytic collision");

        let diff = (step_event.time_of_impact - analytic_event.time_of_impact).abs();
        assert!(diff < 1e-3);
        assert!((step_event.collision.point.x - 1.0).abs() < 1e-3);
        assert!(step_event.collision.normal.x > 0.9);
    }
}
