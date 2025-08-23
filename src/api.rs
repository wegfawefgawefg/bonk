use glam::Vec2;

use crate::types::*;

/// Public API contract for the detection-only physics world.
pub trait PhysicsWorldApi {
    /// Construct a new world with the given configuration.
    fn new(cfg: WorldConfig) -> Self
    where
        Self: Sized;

    // --- Frame lifecycle ---------------------------------------------------

    /// Begin a new frame. Clears ephemeral storage used for the previous frame.
    fn begin_frame(&mut self);

    /// Insert a collider for this frame and return its frame-local handle.
    fn push(&mut self, desc: ColliderDesc, motion: Motion) -> FrameId;

    /// Convenience: push a circle collider.
    fn push_circle(
        &mut self,
        center: Vec2,
        radius: f32,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId;

    /// Convenience: push an AABB collider (center + half extents).
    fn push_aabb(
        &mut self,
        center: Vec2,
        half_extents: Vec2,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId;

    /// Convenience: push a point collider.
    fn push_point(
        &mut self,
        p: Vec2,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId;

    /// Finalize insertions and build the uniform grid.
    fn end_frame(&mut self);

    /// Run broadphase & narrowphase and fill the internal event buffer.
    fn generate_events(&mut self);

    /// Drain and return the accumulated events for this frame.
    fn drain_events(&mut self) -> Vec<Event>;

    // --- Queries -----------------------------------------------------------

    /// Raycast against the current frame's colliders. Returns closest hit.
    fn raycast(
        &self,
        origin: Vec2,
        dir: Vec2,
        mask: LayerMask,
        max_t: f32,
    ) -> Option<(FrameId, SweepHit, Option<ColKey>)>;

    /// Return all colliders whose shapes contain the point `p` (after masking).
    fn query_point(&self, p: Vec2, mask: LayerMask) -> Vec<(FrameId, Option<ColKey>)>;

    /// Return all colliders overlapping the given centered AABB.
    fn query_aabb(
        &self,
        center: Vec2,
        half_extents: Vec2,
        mask: LayerMask,
    ) -> Vec<(FrameId, Option<ColKey>)>;

    /// Return all colliders overlapping the given circle.
    fn query_circle(
        &self,
        center: Vec2,
        radius: f32,
        mask: LayerMask,
    ) -> Vec<(FrameId, Option<ColKey>)>;

    // --- Tilemap lifecycle --------------------------------------------------

    /// Attach a tilemap layer. Multiple tilemaps are allowed.
    fn attach_tilemap(&mut self, desc: TileMapDesc) -> TileMapRef;

    /// Update a rectangular region (x,y,w,h) of the tile buffer for `map`.
    /// `data.len()` must equal `w*h` (row-major).
    fn update_tiles(&mut self, map: TileMapRef, changed_rect: (u32, u32, u32, u32), data: &[u8]);

    /// Detach and free a tilemap.
    fn detach_tilemap(&mut self, map: TileMapRef);

    // --- Unified queries (colliders + tiles; closest or full set) ----------

    /// Raycast against colliders and tiles; returns the closest hit.
    fn raycast_all(
        &self,
        origin: Vec2,
        dir: Vec2,
        mask: LayerMask,
        max_t: f32,
    ) -> Option<(BodyRef, SweepHit, Option<ColKey>)>;

    /// Return all bodies (collider or tile) containing the point.
    fn query_point_all(&self, p: Vec2, mask: LayerMask) -> Vec<(BodyRef, Option<ColKey>)>;

    /// Return all bodies overlapping the AABB.
    fn query_aabb_all(
        &self,
        center: Vec2,
        half_extents: Vec2,
        mask: LayerMask,
    ) -> Vec<(BodyRef, Option<ColKey>)>;

    /// Return all bodies overlapping the circle.
    fn query_circle_all(
        &self,
        center: Vec2,
        radius: f32,
        mask: LayerMask,
    ) -> Vec<(BodyRef, Option<ColKey>)>;

    // --- Tile-only fast path (for profiling / direct control) ---------------

    /// Raycast against tiles only (closest hit across all tilemaps).
    fn raycast_tiles(
        &self,
        origin: Vec2,
        dir: Vec2,
        max_t: f32,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)>;

    /// Sweep AABB against tiles only (first hit).
    fn sweep_aabb_tiles(
        &self,
        center: Vec2,
        half_extents: Vec2,
        vel: Vec2,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)>;

    /// Sweep circle against tiles only (first hit).
    fn sweep_circle_tiles(
        &self,
        center: Vec2,
        radius: f32,
        vel: Vec2,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)>;

    // --- Pairwise checks ---------------------------------------------------

    /// Overlap test between two frame-local colliders (same-frame only).
    fn overlap_pair(&self, a: FrameId, b: FrameId) -> Option<Overlap>;

    /// Sweep test (relative velocity) between two frame-local colliders.
    fn sweep_pair(&self, a: FrameId, b: FrameId) -> Option<SweepHit>;

    /// Overlap test between two user keys (if unique keys were provided).
    fn overlap_by_key(&self, a: ColKey, b: ColKey) -> Option<Overlap>;

    /// Sweep test between two user keys.
    fn sweep_by_key(&self, a: ColKey, b: ColKey) -> Option<SweepHit>;
}

/// Narrowphase and primitive intersection signatures to be provided.
pub trait NarrowphaseApi {
    // Rays / segments -------------------------------------------------------

    fn ray_aabb(origin: Vec2, dir: Vec2, aabb_min: Vec2, aabb_max: Vec2) -> Option<SweepHit>;
    fn ray_circle(origin: Vec2, dir: Vec2, center: Vec2, r: f32) -> Option<SweepHit>;
    fn line_segment_aabb(a: Vec2, b: Vec2, aabb_min: Vec2, aabb_max: Vec2) -> Option<SweepHit>;
    fn line_segment_circle(a: Vec2, b: Vec2, center: Vec2, r: f32) -> Option<SweepHit>;

    // Overlaps --------------------------------------------------------------

    fn overlap_aabb_aabb(c0: Vec2, h0: Vec2, c1: Vec2, h1: Vec2) -> Option<Overlap>;
    fn overlap_circle_circle(c0: Vec2, r0: f32, c1: Vec2, r1: f32) -> Option<Overlap>;
    fn overlap_point_aabb(p: Vec2, c: Vec2, h: Vec2) -> bool;
    fn overlap_point_circle(p: Vec2, c: Vec2, r: f32) -> bool;

    // Sweeps (relative velocity variants expected in world impl) ------------

    fn sweep_aabb_aabb(
        c0: Vec2,
        h0: Vec2,
        v0: Vec2,
        c1: Vec2,
        h1: Vec2,
        v1: Vec2,
    ) -> Option<SweepHit>;

    fn sweep_circle_aabb(
        c: Vec2,
        r: f32,
        v: Vec2,
        box_c: Vec2,
        box_h: Vec2,
        box_v: Vec2,
    ) -> Option<SweepHit>;

    fn sweep_circle_circle(
        c0: Vec2,
        r0: f32,
        v0: Vec2,
        c1: Vec2,
        r1: f32,
        v1: Vec2,
    ) -> Option<SweepHit>;

    // Tile helpers -----------------------------------------------------------
    fn aabb_tile_pushout(c: Vec2, he: Vec2, tile_min: Vec2, cell: f32) -> (Vec2, f32, Vec2);
    fn circle_tile_pushout(c: Vec2, r: f32, tile_min: Vec2, cell: f32) -> (Vec2, f32, Vec2);
}
