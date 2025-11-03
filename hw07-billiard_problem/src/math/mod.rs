use anyhow::{Result, anyhow};
use nalgebra::{Point2, Vector2};

pub type Point = Point2<f64>;
pub type Vector = Vector2<f64>;

/// Ray in 2D used for collision queries.
#[derive(Debug, Clone)]
pub struct Ray2D {
    pub origin: Point,
    pub direction: Vector,
}

impl Ray2D {
    pub fn new(origin: Point, direction: Vector) -> Result<Self> {
        let norm_sq = direction.norm_squared();
        if norm_sq <= f64::EPSILON {
            return Err(anyhow!(
                "ray direction must have non-zero magnitude, got ({}, {})",
                direction.x,
                direction.y
            ));
        }
        Ok(Self {
            origin,
            direction: direction / norm_sq.sqrt(),
        })
    }

    #[inline]
    pub fn point_at(&self, distance: f64) -> Point {
        self.origin + self.direction * distance
    }
}

#[inline]
pub fn cross(a: &Vector, b: &Vector) -> f64 {
    a.x * b.y - a.y * b.x
}

#[inline]
pub fn right_normal(edge: &Vector) -> Vector {
    Vector::new(edge.y, -edge.x)
}

const POLY_EPS: f64 = 1e-12;

fn solve_linear(a: f64, b: f64) -> Vec<f64> {
    if a.abs() <= POLY_EPS {
        Vec::new()
    } else {
        vec![-b / a]
    }
}

pub fn solve_quadratic_real(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() <= POLY_EPS {
        return solve_linear(b, c);
    }
    let discriminant = b * b - 4.0 * a * c;
    if discriminant.abs() <= POLY_EPS {
        vec![-b / (2.0 * a)]
    } else if discriminant < 0.0 {
        Vec::new()
    } else {
        let sqrt_disc = discriminant.sqrt();
        let r1 = (-b - sqrt_disc) / (2.0 * a);
        let r2 = (-b + sqrt_disc) / (2.0 * a);
        if r1 <= r2 { vec![r1, r2] } else { vec![r2, r1] }
    }
}

pub fn solve_cubic_real(a: f64, b: f64, c: f64, d: f64) -> Vec<f64> {
    if a.abs() <= POLY_EPS {
        return solve_quadratic_real(b, c, d);
    }

    let a_norm = b / a;
    let b_norm = c / a;
    let c_norm = d / a;

    let sq_a = a_norm * a_norm;
    let p = b_norm - sq_a / 3.0;
    let q = (2.0 * a_norm * sq_a) / 27.0 - (a_norm * b_norm) / 3.0 + c_norm;

    let half_q = q / 2.0;
    let third_p = p / 3.0;
    let discriminant = half_q * half_q + third_p * third_p * third_p;

    let mut roots = Vec::new();

    if discriminant.abs() <= POLY_EPS {
        if half_q.abs() <= POLY_EPS {
            roots.push(-a_norm / 3.0);
        } else {
            let u = (-half_q).cbrt();
            roots.push(2.0 * u - a_norm / 3.0);
            roots.push(-u - a_norm / 3.0);
        }
    } else if discriminant > 0.0 {
        let sqrt_disc = discriminant.sqrt();
        let u = (-half_q + sqrt_disc).cbrt();
        let v = (-half_q - sqrt_disc).cbrt();
        roots.push(u + v - a_norm / 3.0);
    } else {
        let r = (-third_p).sqrt();
        if r <= POLY_EPS {
            roots.push(-a_norm / 3.0);
        } else {
            let phi = (-half_q / (r * r * r)).acos();
            let two_r = 2.0 * r;
            roots.push(two_r * (phi / 3.0).cos() - a_norm / 3.0);
            roots.push(two_r * ((phi + 2.0 * std::f64::consts::PI) / 3.0).cos() - a_norm / 3.0);
            roots.push(two_r * ((phi + 4.0 * std::f64::consts::PI) / 3.0).cos() - a_norm / 3.0);
        }
    }

    dedup_real_roots(roots)
}

fn dedup_real_roots(mut roots: Vec<f64>) -> Vec<f64> {
    if roots.is_empty() {
        return roots;
    }
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut unique = Vec::with_capacity(roots.len());
    let mut last = roots[0];
    unique.push(last);
    for root in roots.into_iter().skip(1) {
        if (root - last).abs() > 1e-9 {
            unique.push(root);
            last = root;
        }
    }
    unique
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_normalizes_direction() {
        let ray = Ray2D::new(Point::new(0.0, 0.0), Vector::new(2.0, 0.0)).unwrap();
        assert!((ray.direction.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn cross_product_sign() {
        let a = Vector::new(1.0, 0.0);
        let b = Vector::new(0.0, 1.0);
        assert!(cross(&a, &b) > 0.0);
        assert!(cross(&b, &a) < 0.0);
    }
}
