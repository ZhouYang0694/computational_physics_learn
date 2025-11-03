use anyhow::{Result, anyhow};
use nalgebra::{Vector2, point};

use crate::boundary::{self, Boundary2D};
use crate::config::SimulationConfig;
use crate::detector::{CollisionDetector, DetectorError, build_detector};
use crate::physics::State2D;
use crate::sampling::{SampleEvent, SimulationSamples, StateSnapshot};

pub struct SimulationContext<'a> {
    pub config: &'a SimulationConfig,
    pub boundary: Box<dyn Boundary2D>,
    pub detector: Box<dyn CollisionDetector>,
    pub samples: SimulationSamples,
}

impl<'a> SimulationContext<'a> {
    pub fn new(config: &'a SimulationConfig) -> Result<Self> {
        let boundary = boundary::build_boundary(&config.boundary)?;
        let detector = build_detector(&config.detector)?;
        Ok(Self {
            config,
            boundary,
            detector,
            samples: SimulationSamples::new(),
        })
    }

    pub fn run(&mut self) -> Result<()> {
        let mut state = initial_state(&self.config.initial_state);
        let detector_eps = self.config.detector.tolerance;
        let max_collisions = self.config.termination.max_collisions.unwrap_or(1_000_000);
        let max_time = self.config.termination.max_time.unwrap_or(f64::INFINITY);

        while state.time < max_time && state.bounce_id < max_collisions {
            let event = self
                .detector
                .next_collision(&state, self.boundary.as_ref(), detector_eps)
                .map_err(|e| describe_detector_error(e, &state))?;

            let dt = event.time_of_impact;
            let mut end_state = state.advance(dt);
            end_state = end_state.reflect(&event.collision);

            self.samples.record_segment(
                StateSnapshot {
                    time: state.time,
                    bounce_id: state.bounce_id,
                    position: state.position,
                    velocity: state.velocity,
                    event: Some(SampleEvent::SegmentStart),
                },
                StateSnapshot {
                    time: state.time + dt,
                    bounce_id: end_state.bounce_id,
                    position: event.collision.point,
                    velocity: state.velocity,
                    event: Some(SampleEvent::Collision),
                },
                StateSnapshot {
                    time: end_state.time,
                    bounce_id: end_state.bounce_id,
                    position: end_state.position,
                    velocity: end_state.velocity,
                    event: Some(SampleEvent::SegmentEnd),
                },
            );

            if let Some(poincare_y) = Some(self.config.sampling.poincare_plane_y) {
                if (state.position.y - poincare_y) * (event.collision.point.y - poincare_y) <= 0.0 {
                    let t_fraction = (poincare_y - state.position.y)
                        / (event.collision.point.y - state.position.y);
                    let cross_time = state.time + dt * t_fraction;
                    let cross_x = state.position.x
                        + (event.collision.point.x - state.position.x) * t_fraction;
                    self.samples
                        .record_poincare_point(crate::sampling::PoincarePoint {
                            time: cross_time,
                            x: cross_x,
                            vx: state.velocity.x,
                            bounce_id: state.bounce_id,
                        });
                }
            }

            state = end_state;
        }

        Ok(())
    }
}

fn initial_state(config: &crate::config::InitialStateConfig) -> State2D {
    State2D {
        position: point![config.position[0], config.position[1]],
        velocity: Vector2::new(config.velocity[0], config.velocity[1]),
        time: 0.0,
        bounce_id: 0,
    }
}

fn describe_detector_error(err: DetectorError, state: &State2D) -> anyhow::Error {
    match err {
        DetectorError::OutsideBoundary => anyhow!(
            "collision detection failed: initial state {:?} lies outside the boundary",
            state.position
        ),
        DetectorError::TangentialImpact => anyhow!(
            "collision detection failed: trajectory became tangential near the boundary (state {:?}, velocity {:?})",
            state.position,
            state.velocity
        ),
        DetectorError::IterationOverflow => anyhow!(
            "collision detection failed: step-back refinement exceeded iteration limit (state {:?}, velocity {:?})",
            state.position,
            state.velocity
        ),
        DetectorError::ZeroVelocity => anyhow!(
            "collision detection failed: velocity is too small {:?}",
            state.velocity
        ),
        DetectorError::NumericalFailure(msg) => anyhow!(
            "collision detection failed due to numerical issue: {} (state {:?}, velocity {:?})",
            msg,
            state.position,
            state.velocity
        ),
    }
}
