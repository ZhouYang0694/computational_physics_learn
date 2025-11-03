use std::f64;

use nalgebra::{Point2, Vector2};

/// Snapshot of the particle state at a given time.
#[derive(Debug, Clone)]
pub struct StateSnapshot {
    pub time: f64,
    pub bounce_id: u64,
    pub position: Point2<f64>,
    pub velocity: Vector2<f64>,
    pub event: Option<SampleEvent>,
}

impl StateSnapshot {
    #[allow(dead_code)]
    pub fn with_event(mut self, event: SampleEvent) -> Self {
        self.event = Some(event);
        self
    }
}

/// Events recorded during the simulation timeline.
#[derive(Debug, Clone, Copy)]
pub enum SampleEvent {
    SegmentStart,
    SegmentEnd,
    Collision,
    Poincare,
}

/// Straight-line trajectory segment between two consecutive bounces.
#[derive(Debug, Clone)]
pub struct TrajectorySegment {
    pub start: StateSnapshot,
    pub end: StateSnapshot,
}

/// Segment in the x-vx phase space. vx is constant along each segment.
#[derive(Debug, Clone)]
pub struct PhaseSegment {
    pub start_x: f64,
    pub end_x: f64,
    pub vx: f64,
}

/// Point in the y=0 Poincaré section.
#[derive(Debug, Clone)]
pub struct PoincarePoint {
    pub time: f64,
    pub x: f64,
    pub vx: f64,
    pub bounce_id: u64,
}

/// Row for CSV export capturing the full evolution history.
#[derive(Debug, Clone)]
pub struct HistoryRow {
    pub time: f64,
    pub bounce_id: u64,
    pub position: Point2<f64>,
    pub velocity: Vector2<f64>,
    pub event: Option<SampleEvent>,
}

/// Aggregator for all sampled series.
#[derive(Debug, Default)]
pub struct SimulationSamples {
    trajectory: Vec<TrajectorySegment>,
    phase_space: Vec<PhaseSegment>,
    poincare: Vec<PoincarePoint>,
    history: Vec<HistoryRow>,
}

impl SimulationSamples {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_segment(
        &mut self,
        start: StateSnapshot,
        impact: StateSnapshot,
        post: StateSnapshot,
    ) {
        self.history.push(HistoryRow {
            time: start.time,
            bounce_id: start.bounce_id,
            position: start.position,
            velocity: start.velocity,
            event: start.event,
        });
        self.history.push(HistoryRow {
            time: impact.time,
            bounce_id: impact.bounce_id,
            position: impact.position,
            velocity: impact.velocity,
            event: impact.event,
        });
        self.history.push(HistoryRow {
            time: post.time,
            bounce_id: post.bounce_id,
            position: post.position,
            velocity: post.velocity,
            event: post.event,
        });

        self.trajectory.push(TrajectorySegment {
            start: start.clone(),
            end: impact.clone(),
        });

        self.phase_space.push(PhaseSegment {
            start_x: start.position.x,
            end_x: impact.position.x,
            vx: start.velocity.x,
        });
    }

    pub fn record_poincare_point(&mut self, point: PoincarePoint) {
        self.history.push(HistoryRow {
            time: point.time,
            bounce_id: point.bounce_id,
            position: Point2::new(point.x, 0.0),
            velocity: Vector2::new(point.vx, f64::NAN),
            event: Some(SampleEvent::Poincare),
        });
        self.poincare.push(point);
    }

    pub fn trajectory(&self) -> &[TrajectorySegment] {
        &self.trajectory
    }

    pub fn phase_space(&self) -> &[PhaseSegment] {
        &self.phase_space
    }

    pub fn poincare(&self) -> &[PoincarePoint] {
        &self.poincare
    }

    pub fn history(&self) -> &[HistoryRow] {
        &self.history
    }

    /// Compute bounding radius that covers every recorded trajectory position.
    pub fn trajectory_extent(&self) -> f64 {
        let mut max_sq = 0.0_f64;
        for segment in &self.trajectory {
            max_sq = max_sq.max(segment.start.position.coords.norm_squared());
            max_sq = max_sq.max(segment.end.position.coords.norm_squared());
        }
        max_sq.sqrt()
    }

    pub fn phase_extent(&self) -> (RangeExt, RangeExt) {
        let mut x_range = RangeExt::default();
        let mut vx_range = RangeExt::default();
        for segment in &self.phase_space {
            x_range.record(segment.start_x);
            x_range.record(segment.end_x);
            vx_range.record(segment.vx);
        }
        (x_range, vx_range)
    }

    pub fn poincare_extent(&self) -> (RangeExt, RangeExt) {
        let mut x_range = RangeExt::default();
        let mut vx_range = RangeExt::default();
        for point in &self.poincare {
            x_range.record(point.x);
            vx_range.record(point.vx);
        }
        (x_range, vx_range)
    }
}

/// Utility for dynamic range tracking.
#[derive(Debug, Clone, Copy)]
pub struct RangeExt {
    pub min: f64,
    pub max: f64,
}

impl Default for RangeExt {
    fn default() -> Self {
        Self {
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
        }
    }
}

impl RangeExt {
    pub fn record(&mut self, value: f64) {
        if value.is_finite() {
            if value < self.min {
                self.min = value;
            }
            if value > self.max {
                self.max = value;
            }
        }
    }

    pub fn widen_symmetric(&self, fallback: f64) -> (f64, f64) {
        if !self.min.is_finite() || !self.max.is_finite() {
            return (-fallback, fallback);
        }
        let span = (self.max - self.min).abs();
        let base_span = span.max(fallback * 0.1);
        let padding = base_span * 0.1;
        let half_span = base_span / 2.0 + padding;
        let center = (self.max + self.min) / 2.0;
        (center - half_span, center + half_span)
    }
}
