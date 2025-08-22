use glam::Vec2;

/// User-defined opaque key carried through events/queries (e.g., pack your `VID`).
pub type ColKey = u64;

/// Bitmask-based filtering.
#[derive(Copy, Clone, Debug, Default)]
pub struct LayerMask {
    /// Layer(s) this collider belongs to.
    pub layer: u32,
    /// Layers this collider wants to collide with.
    pub collides_with: u32,
    /// Extra mask to exclude (applied after `collides_with`).
    pub exclude: u32,
}

impl LayerMask {
    /// Convenience constructor.
    pub fn simple(layer: u32, collides_with: u32) -> Self {
        Self {
            layer,
            collides_with,
            exclude: 0,
        }
    }

    /// Pair filtering rule (spec):
    /// A may hit B iff `(A.collides_with & B.layer) != 0` AND `(A.exclude & B.layer) == 0`.
    /// Engines SHOULD also check the symmetric predicate (B→A) to require mutual consent.
    pub fn allows(self, other: LayerMask) -> bool {
        let hit = (self.collides_with & other.layer) != 0;
        let blocked = (self.exclude & other.layer) != 0;
        hit && !blocked
    }
}

/// Supported collider shapes.
#[derive(Copy, Clone, Debug)]
pub enum ColliderKind {
    /// Centered axis-aligned box (half extents along X/Y).
    Aabb { half_extents: Vec2 },
    /// Centered circle.
    Circle { radius: f32 },
    /// Mathematical point.
    Point,
}

/// One collider instance to be considered for **this frame**.
#[derive(Copy, Clone, Debug)]
pub struct ColliderDesc {
    pub kind: ColliderKind,
    pub center: Vec2,
    pub mask: LayerMask,
    /// Optional user key echoed in events and query results.
    pub user_key: Option<ColKey>,
}

/// Per-frame motion used for continuous detection.
#[derive(Copy, Clone, Debug, Default)]
pub struct Motion {
    /// Velocity over the frame (units per frame). CCD uses relative velocity.
    pub vel: Vec2,
}

/// Overlap contact result (discrete).
#[derive(Copy, Clone, Debug)]
pub struct Overlap {
    /// Approximate separating normal (may be (0,0) for degenerate cases).
    pub normal: Vec2,
    /// Penetration depth (≥ 0).
    pub depth: f32,
    /// A representative contact point (approx for AABBs).
    pub contact: Vec2,
}

/// Sweep (time-of-impact) result for continuous detection.
#[derive(Copy, Clone, Debug)]
pub struct SweepHit {
    /// Fraction in [0,1] where first impact occurs within the frame.
    pub toi: f32,
    /// Normal at impact (points from B into A).
    pub normal: Vec2,
    /// Representative impact/contact point.
    pub contact: Vec2,
}

/// Event discriminator.
#[derive(Copy, Clone, Debug)]
pub enum EventKind {
    Overlap,
    Sweep,
}

/// Frame-local handle for colliders inserted this frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameId(pub u32);

/// Collision event emitted after generation.
#[derive(Copy, Clone, Debug)]
pub struct Event {
    pub kind: EventKind,
    pub a: FrameId,
    pub b: FrameId,
    pub a_key: Option<ColKey>,
    pub b_key: Option<ColKey>,
    pub overlap: Option<Overlap>,
    pub sweep: Option<SweepHit>,
}

/// World-level configuration for the ephemeral detector.
#[derive(Clone, Debug)]
pub struct WorldConfig {
    /// Grid cell size in world units (typ. 32–64 for Breakout tiles).
    pub cell_size: f32,
    /// Simulation step used when interpreting velocities (e.g., 1/60).
    pub dt: f32,
    /// If true, dynamic binning uses swept AABB over `center .. center + vel*dt`.
    pub tighten_swept_aabb: bool,
    /// Emit discrete overlap events for zero-relative-velocity pairs.
    pub enable_overlap_events: bool,
    /// Emit first-TOI sweep events when relative velocity is non-zero.
    pub enable_sweep_events: bool,
    /// Maximum number of events to emit per frame; extra are dropped.
    pub max_events: usize,
    /// Enable internal timing instrumentation (adds small overhead when true).
    pub enable_timing: bool,
}

/// Debug/performance statistics for a built frame.
#[derive(Copy, Clone, Debug, Default)]
pub struct WorldStats {
    pub entries: usize,
    pub cells: usize,
    /// Sum of per-cell pair counts (n*(n-1)/2), counts duplicates across cells.
    pub candidate_pairs: usize,
    /// Unique pairs encountered when deduplicated across cells.
    pub unique_pairs: usize,
}

/// Timing breakdown for the last completed frame operations.
#[derive(Copy, Clone, Debug, Default)]
pub struct WorldTiming {
    pub end_frame_ms: f64,
    pub end_frame_aabbs_ms: f64,
    pub end_frame_grid_ms: f64,

    pub generate_ms: f64,
    pub generate_scan_ms: f64,
    pub generate_narrowphase_ms: f64,

    pub events_emitted: usize,
}
