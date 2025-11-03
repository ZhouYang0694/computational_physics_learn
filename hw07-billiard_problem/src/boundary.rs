use std::f64::consts::PI;

use anyhow::{Result, anyhow};
use nalgebra::{Point2, point};

use crate::config::{BezierSegmentConfig, BoundaryConfig};
use crate::math::{Ray2D, Vector, cross, right_normal, solve_cubic_real, solve_quadratic_real};

/// Collision information returned by boundary intersection queries.
#[derive(Debug, Clone)]
pub struct Collision {
    pub distance: f64,
    pub point: Point2<f64>,
    pub normal: Vector,
    pub feature: BoundaryFeature,
}

#[derive(Debug, Clone)]
pub enum BoundaryFeature {
    Segment { index: usize },
    Arc { index: usize },
    Bezier { index: usize },
}

#[derive(Debug, Clone)]
pub enum BoundarySegment {
    Line {
        start: Point2<f64>,
        end: Point2<f64>,
    },
    Arc {
        center: Point2<f64>,
        radius: f64,
        start_angle: f64,
        end_angle: f64,
        clockwise: bool,
    },
    Bezier {
        start: Point2<f64>,
        ctrl1: Point2<f64>,
        ctrl2: Point2<f64>,
        end: Point2<f64>,
    },
}

pub trait Boundary2D: Send + Sync {
    fn shape_label(&self) -> &'static str;
    fn intersect_ray(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision>;
    fn contains_point(&self, point: &Point2<f64>, epsilon: f64) -> bool;
    fn bounding_radius(&self) -> f64;
    fn segments(&self) -> Vec<BoundarySegment>;
}

pub fn build_boundary(config: &BoundaryConfig) -> Result<Box<dyn Boundary2D>> {
    match config {
        BoundaryConfig::Polygon { vertices } => {
            let points = vertices
                .iter()
                .map(|xy| point![xy[0], xy[1]])
                .collect::<Vec<_>>();
            Ok(Box::new(PolygonBoundary::new(points)?))
        }
        BoundaryConfig::Stadium { radius, alpha } => {
            Ok(Box::new(StadiumBoundary::new(*radius, *alpha)?))
        }
        BoundaryConfig::Bezier { segments } => Ok(Box::new(BezierBoundary::new(segments)?)),
    }
}

#[derive(Debug, Clone)]
pub struct PolygonBoundary {
    vertices: Vec<Point2<f64>>,
    edges: Vec<LineSegment>,
    bounding_radius: f64,
}

#[derive(Debug, Clone)]
struct LineSegment {
    start: Point2<f64>,
    end: Point2<f64>,
    normal: Vector,
}

impl LineSegment {
    fn new(start: Point2<f64>, end: Point2<f64>) -> Result<Self> {
        let edge = end - start;
        if edge.norm_squared() <= f64::EPSILON {
            return Err(anyhow!(
                "degenerate edge with zero length detected at ({}, {})",
                start.x,
                start.y
            ));
        }
        let normal = right_normal(&edge).normalize();
        Ok(Self { start, end, normal })
    }

    fn intersect_ray(&self, ray: &Ray2D, epsilon: f64) -> Option<(f64, f64)> {
        let p = ray.origin;
        let r = ray.direction;
        let q = self.start;
        let s = self.end - self.start;

        let denom = cross(&r, &s);
        if denom.abs() <= epsilon {
            return None;
        }

        let qp = q - p;
        let t = cross(&qp, &s) / denom;
        if t <= epsilon {
            return None;
        }
        let u = cross(&qp, &r) / denom;
        if u < -epsilon || u > 1.0 + epsilon {
            return None;
        }
        Some((t, u))
    }
}

impl PolygonBoundary {
    pub fn new(vertices: Vec<Point2<f64>>) -> Result<Self> {
        if vertices.len() < 3 {
            return Err(anyhow!(
                "polygon boundary requires at least 3 vertices, got {}",
                vertices.len()
            ));
        }

        let mut edges = Vec::with_capacity(vertices.len());
        let mut max_radius_sq: f64 = 0.0;

        for (idx, vertex) in vertices.iter().enumerate() {
            max_radius_sq = max_radius_sq.max(vertex.coords.norm_squared());
            let next = vertices[(idx + 1) % vertices.len()];
            edges.push(LineSegment::new(*vertex, next)?);
        }

        let signed_area = polygon_signed_area(&vertices);
        if signed_area <= 0.0 {
            return Err(anyhow!(
                "polygon vertices must be ordered counter-clockwise (positive area)"
            ));
        }

        Ok(Self {
            vertices,
            edges,
            bounding_radius: max_radius_sq.sqrt(),
        })
    }
}

impl Boundary2D for PolygonBoundary {
    fn shape_label(&self) -> &'static str {
        "polygon"
    }

    fn intersect_ray(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision> {
        let mut best: Option<Collision> = None;

        for (idx, edge) in self.edges.iter().enumerate() {
            if let Some((distance, _param)) = edge.intersect_ray(ray, epsilon) {
                if best
                    .as_ref()
                    .map(|hit| distance + epsilon < hit.distance)
                    .unwrap_or(true)
                {
                    let point = ray.point_at(distance);
                    best = Some(Collision {
                        distance,
                        point,
                        normal: edge.normal,
                        feature: BoundaryFeature::Segment { index: idx },
                    });
                }
            }
        }

        best
    }

    fn contains_point(&self, point: &Point2<f64>, _epsilon: f64) -> bool {
        ray_cast_contains(point, &self.vertices)
    }

    fn bounding_radius(&self) -> f64 {
        self.bounding_radius
    }

    fn segments(&self) -> Vec<BoundarySegment> {
        let mut segments = Vec::with_capacity(self.vertices.len());
        for (idx, start) in self.vertices.iter().enumerate() {
            let end = self.vertices[(idx + 1) % self.vertices.len()];
            segments.push(BoundarySegment::Line { start: *start, end });
        }
        segments
    }
}

#[derive(Debug, Clone)]
pub struct StadiumBoundary {
    radius: f64,
    alpha: f64,
    top_center: Point2<f64>,
    bottom_center: Point2<f64>,
    left_segment: Option<LineSegment>,
    right_segment: Option<LineSegment>,
    bounding_radius: f64,
}

#[derive(Debug, Clone)]
struct ArcSegment {
    center: Point2<f64>,
    radius: f64,
    angle_start: f64,
    angle_end: f64,
    index: usize,
}

impl ArcSegment {
    fn contains_angle(&self, angle: f64, epsilon: f64) -> bool {
        if self.angle_start <= self.angle_end {
            angle + epsilon >= self.angle_start && angle - epsilon <= self.angle_end
        } else {
            angle + epsilon >= self.angle_start || angle - epsilon <= self.angle_end
        }
    }
}

impl StadiumBoundary {
    pub fn new(radius: f64, alpha: f64) -> Result<Self> {
        if radius <= 0.0 {
            return Err(anyhow!("stadium radius must be positive"));
        }
        if alpha < 0.0 {
            return Err(anyhow!("stadium alpha must be non-negative"));
        }

        let half_height = alpha * radius;
        let top_center = point![0.0, half_height];
        let bottom_center = point![0.0, -half_height];

        let left_segment = if alpha > 0.0 {
            Some(LineSegment::new(
                point![-radius, half_height],
                point![-radius, -half_height],
            )?)
        } else {
            None
        };
        let right_segment = if alpha > 0.0 {
            Some(LineSegment::new(
                point![radius, -half_height],
                point![radius, half_height],
            )?)
        } else {
            None
        };

        let top_extreme = half_height + radius;
        let bounding_radius = (top_extreme * top_extreme + radius * radius).sqrt();

        Ok(Self {
            radius,
            alpha,
            top_center,
            bottom_center,
            left_segment,
            right_segment,
            bounding_radius,
        })
    }

    fn arc_segments(&self) -> [ArcSegment; 2] {
        [
            ArcSegment {
                center: self.top_center,
                radius: self.radius,
                angle_start: 0.0,
                angle_end: PI,
                index: 0,
            },
            ArcSegment {
                center: self.bottom_center,
                radius: self.radius,
                angle_start: PI,
                angle_end: 2.0 * PI,
                index: 1,
            },
        ]
    }

    fn intersect_segments(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision> {
        let mut best: Option<Collision> = None;
        if let Some(segment) = self.left_segment.as_ref() {
            if let Some((distance, _)) = segment.intersect_ray(ray, epsilon) {
                if best
                    .as_ref()
                    .map(|hit| distance + epsilon < hit.distance)
                    .unwrap_or(true)
                {
                    let point = ray.point_at(distance);
                    best = Some(Collision {
                        distance,
                        point,
                        normal: segment.normal,
                        feature: BoundaryFeature::Segment { index: 0 },
                    });
                }
            }
        }
        if let Some(segment) = self.right_segment.as_ref() {
            if let Some((distance, _)) = segment.intersect_ray(ray, epsilon) {
                if best
                    .as_ref()
                    .map(|hit| distance + epsilon < hit.distance)
                    .unwrap_or(true)
                {
                    let point = ray.point_at(distance);
                    best = Some(Collision {
                        distance,
                        point,
                        normal: segment.normal,
                        feature: BoundaryFeature::Segment { index: 1 },
                    });
                }
            }
        }
        best
    }

    fn intersect_arcs(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision> {
        let mut best: Option<Collision> = None;
        for arc in self.arc_segments() {
            let to_origin = ray.origin - arc.center;
            let a = ray.direction.dot(&ray.direction);
            let b = 2.0 * ray.direction.dot(&to_origin);
            let c = to_origin.dot(&to_origin) - arc.radius * arc.radius;
            let discriminant = b * b - 4.0 * a * c;
            if discriminant < 0.0 {
                continue;
            }
            let sqrt_disc = discriminant.sqrt();
            for root in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
                if root <= epsilon {
                    continue;
                }
                let hit_point = ray.point_at(root);
                let relative = hit_point - arc.center;
                let mut angle = relative.y.atan2(relative.x);
                if angle < 0.0 {
                    angle += 2.0 * PI;
                }
                if !arc.contains_angle(angle, epsilon) {
                    continue;
                }

                let normal = relative.normalize();
                if best
                    .as_ref()
                    .map(|hit| root + epsilon < hit.distance)
                    .unwrap_or(true)
                {
                    best = Some(Collision {
                        distance: root,
                        point: hit_point,
                        normal,
                        feature: BoundaryFeature::Arc { index: arc.index },
                    });
                }
            }
        }
        best
    }
}

impl Boundary2D for StadiumBoundary {
    fn shape_label(&self) -> &'static str {
        "stadium"
    }

    fn intersect_ray(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision> {
        let segment_hit = self.intersect_segments(ray, epsilon);
        let arc_hit = self.intersect_arcs(ray, epsilon);

        match (segment_hit, arc_hit) {
            (None, None) => None,
            (Some(hit), None) | (None, Some(hit)) => Some(hit),
            (Some(seg_hit), Some(arc_hit)) => {
                if seg_hit.distance + epsilon < arc_hit.distance {
                    Some(seg_hit)
                } else {
                    Some(arc_hit)
                }
            }
        }
    }

    fn contains_point(&self, point: &Point2<f64>, epsilon: f64) -> bool {
        if self.alpha <= f64::EPSILON {
            return point.coords.norm_squared() <= (self.radius + epsilon).powi(2);
        }
        let half_height = self.alpha * self.radius;

        if point.x.abs() <= self.radius + epsilon && point.y.abs() <= half_height + epsilon {
            return true;
        }

        if point.y > half_height {
            (point - self.top_center).norm_squared() <= (self.radius + epsilon).powi(2)
        } else if point.y < -half_height {
            (point - self.bottom_center).norm_squared() <= (self.radius + epsilon).powi(2)
        } else {
            false
        }
    }

    fn bounding_radius(&self) -> f64 {
        self.bounding_radius
    }

    fn segments(&self) -> Vec<BoundarySegment> {
        let half_height = self.alpha * self.radius;
        vec![
            BoundarySegment::Line {
                start: point![-self.radius, -half_height],
                end: point![-self.radius, half_height],
            },
            BoundarySegment::Arc {
                center: self.top_center,
                radius: self.radius,
                start_angle: PI,
                end_angle: 0.0,
                clockwise: true,
            },
            BoundarySegment::Line {
                start: point![self.radius, half_height],
                end: point![self.radius, -half_height],
            },
            BoundarySegment::Arc {
                center: self.bottom_center,
                radius: self.radius,
                start_angle: 0.0,
                end_angle: -PI,
                clockwise: true,
            },
        ]
    }
}

const BEZIER_CONTAINMENT_SAMPLES: usize = 72;
const BEZIER_INTERSECTION_EPS: f64 = 1e-8;

#[derive(Debug, Clone)]
pub struct BezierBoundary {
    segments: Vec<BezierSegment>,
    bounding_radius: f64,
    containment_polyline: Vec<Point2<f64>>,
}

impl BezierBoundary {
    pub fn new(config_segments: &[BezierSegmentConfig]) -> Result<Self> {
        if config_segments.is_empty() {
            return Err(anyhow!("bezier boundary requires at least one segment"));
        }

        let mut segments: Vec<BezierSegment> = Vec::with_capacity(config_segments.len());
        let mut max_radius_sq: f64 = 0.0;
        let mut first_start: Option<Point2<f64>> = None;

        for (idx, seg) in config_segments.iter().enumerate() {
            let segment = BezierSegment::new(idx, seg)?;
            if let Some(prev) = segments.last() {
                if (prev.end - segment.start).norm_squared() > 1e-9 {
                    return Err(anyhow!(
                        "bezier segments do not connect continuously between {} and {}",
                        idx - 1,
                        idx
                    ));
                }
            } else {
                first_start = Some(segment.start);
            }
            max_radius_sq = max_radius_sq
                .max(segment.start.coords.norm_squared())
                .max(segment.ctrl1.coords.norm_squared())
                .max(segment.ctrl2.coords.norm_squared())
                .max(segment.end.coords.norm_squared());
            segments.push(segment);
        }

        if let (Some(first), Some(last)) = (first_start, segments.last()) {
            if (last.end - first).norm_squared() > 1e-9 {
                return Err(anyhow!(
                    "bezier boundary is not closed: last end does not match first start"
                ));
            }
        }

        let mut warnings = Vec::new();
        for i in 0..segments.len() {
            let current = &segments[i];
            let next = &segments[(i + 1) % segments.len()];

            let tangent_out = current.outgoing_tangent();
            let tangent_in = next.incoming_tangent();
            let norm_product = tangent_out.norm() * tangent_in.norm();
            if norm_product <= 1e-9 {
                continue;
            }
            let cross_val = cross(&tangent_out, &tangent_in).abs();
            if cross_val / norm_product > 1e-3 {
                warnings.push(format!(
                    "Warning: bezier segments {} and {} are not C1-continuous (control points not colinear)",
                    i,
                    (i + 1) % segments.len()
                ));
            }
        }

        for warning in warnings {
            eprintln!("{warning}");
        }

        let mut polyline = Vec::new();
        for (idx, segment) in segments.iter().enumerate() {
            let samples = segment.sample_points(BEZIER_CONTAINMENT_SAMPLES);
            if idx == 0 {
                polyline.extend(samples);
            } else {
                polyline.extend(samples.into_iter().skip(1));
            }
        }

        if polyline.len() < 3 {
            return Err(anyhow!(
                "bezier boundary must enclose an area; insufficient unique sample points"
            ));
        }

        let area = polygon_signed_area(&polyline);
        if area <= 0.0 {
            return Err(anyhow!(
                "bezier boundary control points must be ordered counter-clockwise"
            ));
        }

        Ok(Self {
            segments,
            bounding_radius: max_radius_sq.sqrt(),
            containment_polyline: polyline,
        })
    }
}

impl Boundary2D for BezierBoundary {
    fn shape_label(&self) -> &'static str {
        "bezier"
    }

    fn intersect_ray(&self, ray: &Ray2D, epsilon: f64) -> Option<Collision> {
        let mut best: Option<Collision> = None;

        for (idx, segment) in self.segments.iter().enumerate() {
            for (distance, t) in segment.ray_intersections(ray, epsilon) {
                if distance <= epsilon {
                    continue;
                }
                let update = best
                    .as_ref()
                    .map(|hit| distance + epsilon < hit.distance)
                    .unwrap_or(true);
                if !update {
                    continue;
                }
                let point = segment.point_at(t);
                if !point.coords.iter().all(|v| v.is_finite()) {
                    continue;
                }
                if let Some(normal) = segment.normal_at(t) {
                    best = Some(Collision {
                        distance,
                        point,
                        normal,
                        feature: BoundaryFeature::Bezier { index: idx },
                    });
                }
            }
        }

        best
    }

    fn contains_point(&self, point: &Point2<f64>, _epsilon: f64) -> bool {
        ray_cast_contains(point, &self.containment_polyline)
    }

    fn bounding_radius(&self) -> f64 {
        self.bounding_radius
    }

    fn segments(&self) -> Vec<BoundarySegment> {
        self.segments
            .iter()
            .map(|seg| BoundarySegment::Bezier {
                start: seg.start,
                ctrl1: seg.ctrl1,
                ctrl2: seg.ctrl2,
                end: seg.end,
            })
            .collect()
    }
}

#[derive(Debug, Clone)]
struct BezierSegment {
    index: usize,
    start: Point2<f64>,
    ctrl1: Point2<f64>,
    ctrl2: Point2<f64>,
    end: Point2<f64>,
    coeff_a: Vector,
    coeff_b: Vector,
    coeff_c: Vector,
    coeff_d: Vector,
}

impl BezierSegment {
    fn new(index: usize, cfg: &BezierSegmentConfig) -> Result<Self> {
        let start = point![cfg.start[0], cfg.start[1]];
        let ctrl1 = point![cfg.ctrl1[0], cfg.ctrl1[1]];
        let ctrl2 = point![cfg.ctrl2[0], cfg.ctrl2[1]];
        let end = point![cfg.end[0], cfg.end[1]];

        let mut max_span_sq: f64 = 0.0;
        let points = [start, ctrl1, ctrl2, end];
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                let diff = points[i] - points[j];
                max_span_sq = max_span_sq.max(diff.norm_squared());
            }
        }
        if max_span_sq <= 1e-12 {
            return Err(anyhow!(
                "bezier segment {index} has degenerate control points (all identical)"
            ));
        }

        let c0 = start.coords;
        let c1 = ctrl1.coords;
        let c2 = ctrl2.coords;
        let c3 = end.coords;

        let coeff_a = -c0 + 3.0 * c1 - 3.0 * c2 + c3;
        let coeff_b = 3.0 * c0 - 6.0 * c1 + 3.0 * c2;
        let coeff_c = -3.0 * c0 + 3.0 * c1;
        let coeff_d = c0;

        Ok(Self {
            index,
            start,
            ctrl1,
            ctrl2,
            end,
            coeff_a,
            coeff_b,
            coeff_c,
            coeff_d,
        })
    }

    fn point_at(&self, t: f64) -> Point2<f64> {
        let t2 = t * t;
        let t3 = t2 * t;
        let coords = self.coeff_a * t3 + self.coeff_b * t2 + self.coeff_c * t + self.coeff_d;
        Point2::from(coords)
    }

    fn derivative_at(&self, t: f64) -> Vector {
        let t2 = t * t;
        self.coeff_a * (3.0 * t2) + self.coeff_b * (2.0 * t) + self.coeff_c
    }

    fn normal_at(&self, t: f64) -> Option<Vector> {
        let tangent = self.derivative_at(t);
        let norm = tangent.norm();
        if norm <= 1e-12 {
            None
        } else {
            Some(right_normal(&(tangent / norm)))
        }
    }

    fn outgoing_tangent(&self) -> Vector {
        self.end - self.ctrl2
    }

    fn incoming_tangent(&self) -> Vector {
        self.ctrl1 - self.start
    }

    fn ray_intersections(&self, ray: &Ray2D, epsilon: f64) -> Vec<(f64, f64)> {
        let dir = ray.direction;
        let normal = Vector::new(-dir.y, dir.x);

        let a = self.coeff_a.dot(&normal);
        let b = self.coeff_b.dot(&normal);
        let c = self.coeff_c.dot(&normal);
        let d = (self.coeff_d - ray.origin.coords).dot(&normal);

        let mut roots = if a.abs() <= 1e-10 {
            solve_quadratic_real(b, c, d)
        } else {
            solve_cubic_real(a, b, c, d)
        };

        roots.retain(|t| *t >= -BEZIER_INTERSECTION_EPS && *t <= 1.0 + BEZIER_INTERSECTION_EPS);

        let mut intersections = Vec::new();
        for mut t in roots {
            if t < 0.0 {
                t = 0.0;
            } else if t > 1.0 {
                t = 1.0;
            }
            let point = self.point_at(t);
            let diff = point - ray.origin;
            let distance = diff.dot(&dir);
            if distance <= epsilon {
                continue;
            }
            let off_axis = diff.dot(&normal).abs();
            if off_axis > 1e-6 {
                continue;
            }
            intersections.push((distance, t));
        }

        intersections
    }

    fn sample_points(&self, samples: usize) -> Vec<Point2<f64>> {
        let steps = samples.max(4);
        let mut points = Vec::with_capacity(steps + 1);
        for i in 0..=steps {
            let t = (i as f64) / (steps as f64);
            points.push(self.point_at(t));
        }
        points
    }
}

fn polygon_signed_area(vertices: &[Point2<f64>]) -> f64 {
    let mut area = 0.0;
    for i in 0..vertices.len() {
        let p = vertices[i];
        let q = vertices[(i + 1) % vertices.len()];
        area += p.x * q.y - q.x * p.y;
    }
    0.5 * area
}

fn ray_cast_contains(point: &Point2<f64>, vertices: &[Point2<f64>]) -> bool {
    if vertices.len() < 3 {
        return false;
    }
    let mut winding = 0i32;
    for i in 0..vertices.len() {
        let a = vertices[i];
        let b = vertices[(i + 1) % vertices.len()];
        if (b.y - a.y).abs() <= f64::EPSILON {
            continue;
        }
        let intersects = ((a.y <= point.y && point.y < b.y) || (b.y <= point.y && point.y < a.y))
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if intersects {
            winding ^= 1;
        }
    }
    winding == 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::BezierSegmentConfig;
    use crate::math::{Ray2D, Vector};
    use approx::assert_relative_eq;

    #[test]
    fn polygon_intersection_hits_edge() {
        let polygon = PolygonBoundary::new(vec![
            point![-1.0, -1.0],
            point![1.0, -1.0],
            point![1.0, 1.0],
            point![-1.0, 1.0],
        ])
        .unwrap();
        let ray = Ray2D::new(point![0.0, 0.0], Vector::new(1.0, 0.0)).unwrap();
        let hit = polygon.intersect_ray(&ray, 1e-9).unwrap();
        assert_relative_eq!(hit.point.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(hit.point.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(hit.normal.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(hit.normal.y, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn stadium_contains_points() {
        let stadium = StadiumBoundary::new(1.0, 1.5).unwrap();
        assert!(stadium.contains_point(&point![0.0, 0.0], 1e-9));
        assert!(stadium.contains_point(&point![0.0, 2.4], 1e-9));
        assert!(!stadium.contains_point(&point![2.0, 0.0], 1e-9));
    }

    #[test]
    fn stadium_intersects_arc() {
        let stadium = StadiumBoundary::new(1.0, 1.0).unwrap();
        let ray = Ray2D::new(point![0.0, 3.0], Vector::new(0.0, -1.0)).unwrap();
        let hit = stadium.intersect_ray(&ray, 1e-8).unwrap();
        assert!(matches!(hit.feature, BoundaryFeature::Arc { .. }));
        assert_relative_eq!(hit.point.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(hit.point.y, 2.0, epsilon = 1e-9);
    }

    #[test]
    fn bezier_boundary_contains_and_intersects() {
        let k = 0.552_284_749_830_793_6;
        let segments = vec![
            BezierSegmentConfig {
                start: [1.0, 0.0],
                ctrl1: [1.0, k],
                ctrl2: [k, 1.0],
                end: [0.0, 1.0],
            },
            BezierSegmentConfig {
                start: [0.0, 1.0],
                ctrl1: [-k, 1.0],
                ctrl2: [-1.0, k],
                end: [-1.0, 0.0],
            },
            BezierSegmentConfig {
                start: [-1.0, 0.0],
                ctrl1: [-1.0, -k],
                ctrl2: [-k, -1.0],
                end: [0.0, -1.0],
            },
            BezierSegmentConfig {
                start: [0.0, -1.0],
                ctrl1: [k, -1.0],
                ctrl2: [1.0, -k],
                end: [1.0, 0.0],
            },
        ];
        let boundary = BezierBoundary::new(&segments).unwrap();
        assert!(boundary.contains_point(&point![0.0, 0.0], 1e-9));
        assert!(!boundary.contains_point(&point![2.0, 0.0], 1e-9));

        let ray = Ray2D::new(point![0.0, 0.0], Vector::new(1.0, 0.0)).unwrap();
        let hit = boundary.intersect_ray(&ray, 1e-9).unwrap();
        assert!(matches!(hit.feature, BoundaryFeature::Bezier { .. }));
        assert_relative_eq!(hit.point.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(hit.point.y, 0.0, epsilon = 1e-6);
    }
}
