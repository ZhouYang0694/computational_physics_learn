use nalgebra::{Point2, Vector2};

use crate::boundary::Collision;

/// Dynamical state of the billiard particle.
#[derive(Debug, Clone)]
pub struct State2D {
    pub position: Point2<f64>,
    pub velocity: Vector2<f64>,
    pub time: f64,
    pub bounce_id: u64,
}

impl State2D {
    pub fn new(position: Point2<f64>, velocity: Vector2<f64>) -> Self {
        Self {
            position,
            velocity,
            time: 0.0,
            bounce_id: 0,
        }
    }

    pub fn advance(&self, dt: f64) -> Self {
        let mut next = self.clone();
        next.position += self.velocity * dt;
        next.time += dt;
        next
    }

    pub fn energy(&self) -> f64 {
        self.velocity.norm_squared() * 0.5
    }

    pub fn reflect(&self, collision: &Collision) -> State2D {
        let n = collision.normal;
        let v = self.velocity;
        let vn = v.dot(&n) * n;
        let vt = v - vn;
        let mut next = self.clone();
        next.position = collision.point;
        next.velocity = vt - vn;
        next.bounce_id += 1;
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boundary::{BoundaryFeature, Collision};
    use nalgebra::point;

    #[test]
    fn state_advances_linearly() {
        let state = State2D::new(point![0.0, 0.0], Vector2::new(1.0, 0.0));
        let next = state.advance(2.0);
        assert!((next.position.x - 2.0).abs() < 1e-12);
        assert_eq!(state.bounce_id, next.bounce_id);
    }

    #[test]
    fn reflection_flips_normal_component() {
        let state = State2D {
            position: point![0.0, 0.0],
            velocity: Vector2::new(1.0, 0.5),
            time: 1.0,
            bounce_id: 3,
        };
        let collision = Collision {
            distance: 0.0,
            point: point![1.0, 0.1],
            normal: Vector2::new(1.0, 0.0),
            feature: BoundaryFeature::Segment { index: 0 },
        };
        let next = state.reflect(&collision);
        assert!((next.velocity.x + 1.0).abs() < 1e-12);
        assert!((next.velocity.y - 0.5).abs() < 1e-12);
        assert_eq!(next.bounce_id, state.bounce_id + 1);
        assert_eq!(next.position, collision.point);
    }
}
