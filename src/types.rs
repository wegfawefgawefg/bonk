use glam::{UVec2, Vec2};

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

/// Resolution hint attached to hits (tiles & non-tiles).
#[derive(Copy, Clone, Debug, Default)]
pub struct ResolutionHint {
    pub safe_pos: Option<Vec2>,
    pub start_embedded: bool,
    pub fully_embedded: bool,
}

/// Overlap contact result (discrete).
#[derive(Copy, Clone, Debug)]
pub struct Overlap {
    pub normal: Vec2,
    pub depth: f32,
    pub contact: Vec2,
    pub hint: ResolutionHint,
}

/// Sweep (time-of-impact) result for continuous detection.
#[derive(Copy, Clone, Debug)]
pub struct SweepHit {
    pub toi: f32,
    pub normal: Vec2,
    pub contact: Vec2,
    pub hint: ResolutionHint,
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

/// Opaque handle to a registered tilemap layer.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct TileMapRef(pub u32);

/// Identifies a specific tile cell within a map.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct TileRef {
    pub map: TileMapRef,
    pub cell_xy: UVec2,
}

/// Reference to an event/query participant (collider or tile).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum BodyRef {
    Collider(FrameId),
    Tile(TileRef),
}

/// Collision event emitted after generation.
#[derive(Copy, Clone, Debug)]
pub struct Event {
    pub kind: EventKind,
    pub a: BodyRef,
    pub b: BodyRef,
    pub a_key: Option<ColKey>,
    pub b_key: Option<ColKey>,
    pub overlap: Option<Overlap>,
    pub sweep: Option<SweepHit>,
}

/// World-level configuration for the ephemeral detector.
#[derive(Clone, Debug)]
pub struct WorldConfig {
    pub cell_size: f32,
    pub dt: f32,
    pub tighten_swept_aabb: bool,
    pub enable_overlap_events: bool,
    pub enable_sweep_events: bool,
    pub max_events: usize,
    pub enable_timing: bool,
    /// Epsilon used when computing safe_pos for tile hits.
    pub tile_eps: f32,
    /// If true, require mutual consent for events/queries (colliders and tiles).
    pub require_mutual_consent: bool,
}

/// Description of a tilemap to attach to the world.
#[derive(Clone, Debug)]
pub struct TileMapDesc<'a> {
    pub origin: Vec2,
    pub cell: f32,
    pub width: u32,
    pub height: u32,
    pub solids: &'a [u8],
    pub mask: LayerMask,
    pub user_key: Option<ColKey>,
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
