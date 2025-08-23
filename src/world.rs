use glam::Vec2;

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use crate::api::{NarrowphaseApi, PhysicsWorldApi};
use crate::types::*;

/// Ephemeral detection-only world implementation (skeleton).
pub struct PhysicsWorld {
    pub cfg: WorldConfig,
    pub frame_counter: u32,

    // Frame-local storage
    entries: Vec<Entry>,
    aabbs: Vec<(Vec2, Vec2)>, // (min, max) per entry
    key_to_id: HashMap<ColKey, FrameId>,

    // Uniform grid: cell coord -> list of indices into `entries`
    grid: HashMap<(i32, i32), Vec<usize>>,

    // Tilemaps
    tilemaps: Vec<TileMap>,

    // Event buffer for this frame
    events: Vec<Event>,

    // Timing for last operations (optional)
    last_timing: Option<WorldTiming>,
}

struct Entry {
    desc: ColliderDesc,
    motion: Motion,
}

#[derive(Clone)]
struct TileMap {
    origin: Vec2,
    cell: f32,
    width: u32,
    height: u32,
    solids: Vec<u8>,
    mask: LayerMask,
    user_key: Option<ColKey>,
}

impl PhysicsWorldApi for PhysicsWorld {
    fn new(cfg: WorldConfig) -> Self {
        Self {
            cfg,
            frame_counter: 0,
            entries: Vec::new(),
            aabbs: Vec::new(),
            key_to_id: HashMap::new(),
            grid: HashMap::new(),
            tilemaps: Vec::new(),
            events: Vec::new(),
            last_timing: None,
        }
    }

    fn begin_frame(&mut self) {
        // Clear ephemeral state
        self.entries.clear();
        self.aabbs.clear();
        self.grid.clear();
        self.key_to_id.clear();
        self.events.clear();
        self.last_timing = None;
        self.frame_counter = self.frame_counter.wrapping_add(1);
    }

    fn push(&mut self, desc: ColliderDesc, motion: Motion) -> FrameId {
        let id = FrameId(self.entries.len() as u32);
        if let Some(k) = desc.user_key {
            debug_assert!(
                !self.key_to_id.contains_key(&k),
                "Duplicate user_key encountered within a frame"
            );
            self.key_to_id.insert(k, id);
        }
        self.entries.push(Entry { desc, motion });
        id
    }

    fn push_circle(
        &mut self,
        center: Vec2,
        radius: f32,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId {
        let desc = ColliderDesc {
            kind: ColliderKind::Circle { radius },
            center,
            mask,
            user_key,
        };
        let motion = Motion { vel };
        self.push(desc, motion)
    }

    fn push_aabb(
        &mut self,
        center: Vec2,
        half_extents: Vec2,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId {
        let desc = ColliderDesc {
            kind: ColliderKind::Aabb { half_extents },
            center,
            mask,
            user_key,
        };
        let motion = Motion { vel };
        self.push(desc, motion)
    }

    fn push_point(
        &mut self,
        p: Vec2,
        vel: Vec2,
        mask: LayerMask,
        user_key: Option<ColKey>,
    ) -> FrameId {
        let desc = ColliderDesc {
            kind: ColliderKind::Point,
            center: p,
            mask,
            user_key,
        };
        let motion = Motion { vel };
        self.push(desc, motion)
    }

    fn end_frame(&mut self) {
        // Build axis-aligned bounds for each entry and insert into uniform grid.
        let t_all = if self.cfg.enable_timing {
            Some(Instant::now())
        } else {
            None
        };
        let t0 = if self.cfg.enable_timing {
            Some(Instant::now())
        } else {
            None
        };
        self.aabbs
            .resize(self.entries.len(), (Vec2::ZERO, Vec2::ZERO));

        for (i, e) in self.entries.iter().enumerate() {
            let (min, max) = self.compute_entry_aabb(e);
            self.aabbs[i] = (min, max);
        }
        let aabb_ms = t0
            .map(|t| t.elapsed().as_secs_f64() * 1000.0)
            .unwrap_or(0.0);
        let t1 = if self.cfg.enable_timing {
            Some(Instant::now())
        } else {
            None
        };
        let aabbs_snapshot = self.aabbs.clone();
        for (i, (min, max)) in aabbs_snapshot.into_iter().enumerate() {
            self.insert_into_grid(i, min, max);
        }
        let grid_ms = t1
            .map(|t| t.elapsed().as_secs_f64() * 1000.0)
            .unwrap_or(0.0);
        if let Some(t_all) = t_all {
            self.last_timing = Some(WorldTiming {
                end_frame_ms: t_all.elapsed().as_secs_f64() * 1000.0,
                end_frame_aabbs_ms: aabb_ms,
                end_frame_grid_ms: grid_ms,
                ..Default::default()
            });
        }
    }

    fn generate_events(&mut self) {
        // Build candidate pairs from grid, deduplicate, then dispatch narrowphase
        let t_all = if self.cfg.enable_timing {
            Some(Instant::now())
        } else {
            None
        };
        let t_scan0 = if self.cfg.enable_timing {
            Some(Instant::now())
        } else {
            None
        };
        let mut seen_pairs: HashSet<(usize, usize)> = HashSet::new();
        let push_event = |ev: Event, buf: &mut Vec<Event>, max: usize| {
            if buf.len() < max {
                buf.push(ev);
            }
        };

        for indices in self.grid.values() {
            for i0 in 0..indices.len() {
                for i1 in (i0 + 1)..indices.len() {
                    let a = indices[i0];
                    let b = indices[i1];
                    let key = if a < b { (a, b) } else { (b, a) };
                    if !seen_pairs.insert(key) {
                        continue;
                    }
                    if self.events.len() >= self.cfg.max_events {
                        return;
                    }

                    let t_np0 = if self.cfg.enable_timing {
                        Some(Instant::now())
                    } else {
                        None
                    };
                    let ea = &self.entries[a];
                    let eb = &self.entries[b];
                    // Mask consent (possibly mutual based on config)
                    if !self.allows_pair(ea.desc.mask, eb.desc.mask) {
                        continue;
                    }

                    let rel = ea.motion.vel - eb.motion.vel;
                    let dynamic = rel.length_squared() > 1e-12;

                    if dynamic && self.cfg.enable_sweep_events {
                        if let Some(mut sweep) = self.sweep_pair_idx(a, b) {
                            sweep.hint = ResolutionHint::default();
                            let ev = Event {
                                kind: crate::types::EventKind::Sweep,
                                a: BodyRef::Collider(FrameId(a as u32)),
                                b: BodyRef::Collider(FrameId(b as u32)),
                                a_key: ea.desc.user_key,
                                b_key: eb.desc.user_key,
                                overlap: None,
                                sweep: Some(sweep),
                            };
                            push_event(ev, &mut self.events, self.cfg.max_events);
                        } else if self.cfg.enable_overlap_events
                            && let Some(mut ov) = self.overlap_pair_idx(a, b)
                        {
                            ov.hint = ResolutionHint::default();
                            let ev = Event {
                                kind: crate::types::EventKind::Overlap,
                                a: BodyRef::Collider(FrameId(a as u32)),
                                b: BodyRef::Collider(FrameId(b as u32)),
                                a_key: ea.desc.user_key,
                                b_key: eb.desc.user_key,
                                overlap: Some(ov),
                                sweep: None,
                            };
                            push_event(ev, &mut self.events, self.cfg.max_events);
                        }
                    } else if self.cfg.enable_overlap_events
                        && let Some(mut ov) = self.overlap_pair_idx(a, b)
                    {
                        ov.hint = ResolutionHint::default();
                        let ev = Event {
                            kind: crate::types::EventKind::Overlap,
                            a: BodyRef::Collider(FrameId(a as u32)),
                            b: BodyRef::Collider(FrameId(b as u32)),
                            a_key: ea.desc.user_key,
                            b_key: eb.desc.user_key,
                            overlap: Some(ov),
                            sweep: None,
                        };
                        push_event(ev, &mut self.events, self.cfg.max_events);
                    }
                    if let (Some(t_np0), Some(timing)) = (t_np0, self.last_timing.as_mut()) {
                        timing.generate_narrowphase_ms += t_np0.elapsed().as_secs_f64() * 1000.0;
                    }
                }
            }
        }
        if let Some(t_scan0) = t_scan0 {
            if self.last_timing.is_none() {
                self.last_timing = Some(WorldTiming::default());
            }
            if let Some(timing) = self.last_timing.as_mut() {
                timing.generate_scan_ms =
                    t_scan0.elapsed().as_secs_f64() * 1000.0 - timing.generate_narrowphase_ms;
            }
        }

        // Phase 2: collider ↔ tile events
        if self.events.len() < self.cfg.max_events {
            for (i, e) in self.entries.iter().enumerate() {
                let he = match e.desc.kind {
                    ColliderKind::Aabb { half_extents } => half_extents,
                    ColliderKind::Circle { radius } => Vec2::splat(radius),
                    ColliderKind::Point => Vec2::ZERO,
                };
                let mask_a = e.desc.mask;
                let v = e.motion.vel;
                let mut emitted = false;
                if v.length_squared() > 1e-12
                    && self.cfg.enable_sweep_events
                    && let Some((tref, mut hit, key_b)) =
                        self.sweep_shape_tiles(e.desc.center, he, v, mask_a)
                {
                    hit.hint.start_embedded = false;
                    let ev = Event {
                        kind: EventKind::Sweep,
                        a: BodyRef::Collider(FrameId(i as u32)),
                        b: BodyRef::Tile(tref),
                        a_key: e.desc.user_key,
                        b_key: key_b,
                        overlap: None,
                        sweep: Some(hit),
                    };
                    push_event(ev, &mut self.events, self.cfg.max_events);
                    emitted = true;
                }
                if !emitted && self.cfg.enable_overlap_events {
                    // Check start embedded
                    for (mi, m) in self.tilemaps.iter().enumerate() {
                        if !self.allows_pair(mask_a, m.mask) {
                            continue;
                        }
                        if let Some(tref) = self.any_tile_overlap_at(mi, m, e.desc.center, he) {
                            // Build overlap with pushout hint
                            let cell = m.cell.max(1e-5);
                            let tile_min = m.origin
                                + Vec2::new(
                                    tref.cell_xy.x as f32 * cell,
                                    tref.cell_xy.y as f32 * cell,
                                );
                            let (normal, depth, contact) = if he == Vec2::ZERO {
                                crate::narrowphase::Narrowphase::circle_tile_pushout(
                                    e.desc.center,
                                    0.0,
                                    tile_min,
                                    cell,
                                )
                            } else if he.x == he.y {
                                // treat as circle for simplicity when square
                                crate::narrowphase::Narrowphase::circle_tile_pushout(
                                    e.desc.center,
                                    he.x,
                                    tile_min,
                                    cell,
                                )
                            } else {
                                crate::narrowphase::Narrowphase::aabb_tile_pushout(
                                    e.desc.center,
                                    he,
                                    tile_min,
                                    cell,
                                )
                            };
                            let mut ov = Overlap {
                                normal,
                                depth,
                                contact,
                                hint: ResolutionHint::default(),
                            };
                            ov.hint.start_embedded = true;
                            let ev = Event {
                                kind: EventKind::Overlap,
                                a: BodyRef::Collider(FrameId(i as u32)),
                                b: BodyRef::Tile(tref),
                                a_key: e.desc.user_key,
                                b_key: m.user_key,
                                overlap: Some(ov),
                                sweep: None,
                            };
                            push_event(ev, &mut self.events, self.cfg.max_events);
                            break;
                        }
                    }
                }
                if self.events.len() >= self.cfg.max_events {
                    break;
                }
            }
        }
        if let Some(t_all) = t_all {
            if self.last_timing.is_none() {
                self.last_timing = Some(WorldTiming::default());
            }
            if let Some(timing) = self.last_timing.as_mut() {
                timing.generate_ms = t_all.elapsed().as_secs_f64() * 1000.0;
                timing.events_emitted = self.events.len();
            }
        }
    }

    fn drain_events(&mut self) -> Vec<Event> {
        let out = self.events.clone();
        self.events.clear();
        out
    }

    // --- Tilemap lifecycle --------------------------------------------------
    fn attach_tilemap(&mut self, desc: TileMapDesc) -> TileMapRef {
        let map = TileMap {
            origin: desc.origin,
            cell: desc.cell,
            width: desc.width,
            height: desc.height,
            solids: desc.solids.to_vec(),
            mask: desc.mask,
            user_key: desc.user_key,
        };
        self.tilemaps.push(map);
        TileMapRef((self.tilemaps.len() - 1) as u32)
    }

    fn update_tiles(&mut self, map: TileMapRef, changed_rect: (u32, u32, u32, u32), data: &[u8]) {
        if let Some(m) = self.tilemaps.get_mut(map.0 as usize) {
            let (x, y, w, h) = changed_rect;
            assert_eq!((w * h) as usize, data.len());
            for row in 0..h {
                let dst_y = y + row;
                if dst_y >= m.height {
                    break;
                }
                let dst_off = (dst_y * m.width + x) as usize;
                let src_off = (row * w) as usize;
                let len = w.min(m.width - x) as usize;
                m.solids[dst_off..dst_off + len].copy_from_slice(&data[src_off..src_off + len]);
            }
        }
    }

    fn detach_tilemap(&mut self, map: TileMapRef) {
        let idx = map.0 as usize;
        if idx < self.tilemaps.len() {
            self.tilemaps.remove(idx);
        }
    }

    fn raycast(
        &self,
        origin: Vec2,
        dir: Vec2,
        mask: LayerMask,
        max_t: f32,
    ) -> Option<(FrameId, SweepHit, Option<ColKey>)> {
        if dir.length_squared() == 0.0 {
            return None;
        }
        let cs = self.cfg.cell_size.max(1e-5);
        // Setup DDA
        let mut best: Option<(usize, SweepHit)> = None;
        let mut tested: HashSet<usize> = HashSet::new();

        let mut cell = self.world_to_cell(origin, cs);
        let step_x = if dir.x > 0.0 {
            1
        } else if dir.x < 0.0 {
            -1
        } else {
            0
        };
        let step_y = if dir.y > 0.0 {
            1
        } else if dir.y < 0.0 {
            -1
        } else {
            0
        };
        let next_boundary = |c: i32, step: i32| -> f32 {
            if step > 0 {
                (c as f32 + 1.0) * cs
            } else {
                c as f32 * cs
            }
        };
        let _cell_origin = Vec2::new(cell.0 as f32 * cs, cell.1 as f32 * cs);
        let mut t_max_x = if step_x != 0 {
            let nb = next_boundary(cell.0, step_x);
            (nb - origin.x) / dir.x
        } else {
            f32::INFINITY
        };
        let mut t_max_y = if step_y != 0 {
            let nb = next_boundary(cell.1, step_y);
            (nb - origin.y) / dir.y
        } else {
            f32::INFINITY
        };
        let t_delta_x = if step_x != 0 {
            cs / dir.x.abs()
        } else {
            f32::INFINITY
        };
        let t_delta_y = if step_y != 0 {
            cs / dir.y.abs()
        } else {
            f32::INFINITY
        };

        let mut t_curr = 0.0f32;
        // Visit cells until exceeding max_t
        for _ in 0..10_000 {
            // safety cap
            if t_curr > max_t {
                break;
            }
            if let Some(list) = self.grid.get(&cell) {
                for &idx in list {
                    if !tested.insert(idx) {
                        continue;
                    }
                    let e = &self.entries[idx];
                    // Mask mutual consent between ray mask and collider mask
                    if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) {
                        continue;
                    }
                    let hit = match e.desc.kind {
                        ColliderKind::Aabb { .. } => {
                            let (min, max) = self.aabbs[idx];
                            crate::narrowphase::Narrowphase::ray_aabb(origin, dir, min, max)
                        }
                        ColliderKind::Circle { radius } => {
                            crate::narrowphase::Narrowphase::ray_circle(
                                origin,
                                dir,
                                e.desc.center,
                                radius,
                            )
                        }
                        ColliderKind::Point => crate::narrowphase::Narrowphase::ray_circle(
                            origin,
                            dir,
                            e.desc.center,
                            0.0,
                        ),
                    };
                    if let Some(mut h) = hit {
                        if h.toi < 0.0 || h.toi > max_t {
                            continue;
                        }
                        h.hint = ResolutionHint::default();
                        match &mut best {
                            Some((_, bh)) if h.toi >= bh.toi => {}
                            _ => best = Some((idx, h)),
                        }
                    }
                }
            }

            // Step to next cell
            if t_max_x < t_max_y {
                cell.0 += step_x;
                t_curr = t_max_x;
                t_max_x += t_delta_x;
            } else {
                cell.1 += step_y;
                t_curr = t_max_y;
                t_max_y += t_delta_y;
            }
        }

        best.map(|(idx, h)| (FrameId(idx as u32), h, self.entries[idx].desc.user_key))
    }

    // --- Unified queries (colliders + tiles) --------------------------------
    fn raycast_all(
        &self,
        origin: Vec2,
        dir: Vec2,
        mask: LayerMask,
        max_t: f32,
    ) -> Option<(BodyRef, SweepHit, Option<ColKey>)> {
        let mut best: Option<(BodyRef, SweepHit, Option<ColKey>)> = None;
        if let Some((id, hit, key)) = self.raycast(origin, dir, mask, max_t) {
            best = Some((BodyRef::Collider(id), hit, key));
        }
        if let Some((tref, hit, key)) = self.raycast_tiles_internal(origin, dir, max_t, mask) {
            match &best {
                Some((_, bh, _)) if hit.toi >= bh.toi => {}
                _ => best = Some((BodyRef::Tile(tref), hit, key)),
            }
        }
        best
    }

    fn query_point_all(&self, p: Vec2, mask: LayerMask) -> Vec<(BodyRef, Option<ColKey>)> {
        let mut out: Vec<(BodyRef, Option<ColKey>)> = Vec::new();
        for (id, key) in self.query_point(p, mask) {
            out.push((BodyRef::Collider(id), key));
        }
        for (mi, m) in self.tilemaps.iter().enumerate() {
            if !self.allows_pair(mask, m.mask) {
                continue;
            }
            let local = p - m.origin;
            let cell = m.cell.max(1e-5);
            let cx = (local.x / cell).floor() as i32;
            let cy = (local.y / cell).floor() as i32;
            if cx >= 0 && cy >= 0 && (cx as u32) < m.width && (cy as u32) < m.height {
                let idx = cy as u32 * m.width + cx as u32;
                if m.solids[idx as usize] != 0 {
                    out.push((
                        BodyRef::Tile(TileRef {
                            map: TileMapRef(mi as u32),
                            cell_xy: glam::UVec2::new(cx as u32, cy as u32),
                        }),
                        m.user_key,
                    ));
                }
            }
        }
        out
    }

    fn query_aabb_all(
        &self,
        center: Vec2,
        half_extents: Vec2,
        mask: LayerMask,
    ) -> Vec<(BodyRef, Option<ColKey>)> {
        let mut out: Vec<(BodyRef, Option<ColKey>)> = Vec::new();
        for (id, key) in self.query_aabb(center, half_extents, mask) {
            out.push((BodyRef::Collider(id), key));
        }
        for (mi, m) in self.tilemaps.iter().enumerate() {
            if !self.allows_pair(mask, m.mask) {
                continue;
            }
            let cell = m.cell.max(1e-5);
            let min = center - half_extents - m.origin;
            let max = center + half_extents - m.origin;
            let ix0 = (min.x / cell).floor() as i32;
            let iy0 = (min.y / cell).floor() as i32;
            let ix1 = (max.x / cell).floor() as i32;
            let iy1 = (max.y / cell).floor() as i32;
            for iy in iy0..=iy1 {
                for ix in ix0..=ix1 {
                    if ix < 0 || iy < 0 {
                        continue;
                    }
                    let (ux, uy) = (ix as u32, iy as u32);
                    if ux >= m.width || uy >= m.height {
                        continue;
                    }
                    let idx = (uy * m.width + ux) as usize;
                    if m.solids[idx] == 0 {
                        continue;
                    }
                    let tile_min = m.origin + Vec2::new(ix as f32 * cell, iy as f32 * cell);
                    let tile_c = tile_min + Vec2::splat(cell * 0.5);
                    let tile_h = Vec2::splat(cell * 0.5);
                    if crate::narrowphase::Narrowphase::overlap_aabb_aabb(
                        center,
                        half_extents,
                        tile_c,
                        tile_h,
                    )
                    .is_some()
                    {
                        out.push((
                            BodyRef::Tile(TileRef {
                                map: TileMapRef(mi as u32),
                                cell_xy: glam::UVec2::new(ux, uy),
                            }),
                            m.user_key,
                        ));
                    }
                }
            }
        }
        out
    }

    fn query_circle_all(
        &self,
        center: Vec2,
        radius: f32,
        mask: LayerMask,
    ) -> Vec<(BodyRef, Option<ColKey>)> {
        let mut out: Vec<(BodyRef, Option<ColKey>)> = Vec::new();
        for (id, key) in self.query_circle(center, radius, mask) {
            out.push((BodyRef::Collider(id), key));
        }
        for (mi, m) in self.tilemaps.iter().enumerate() {
            if !self.allows_pair(mask, m.mask) {
                continue;
            }
            let cell = m.cell.max(1e-5);
            let min = center - Vec2::splat(radius) - m.origin;
            let max = center + Vec2::splat(radius) - m.origin;
            let ix0 = (min.x / cell).floor() as i32;
            let iy0 = (min.y / cell).floor() as i32;
            let ix1 = (max.x / cell).floor() as i32;
            let iy1 = (max.y / cell).floor() as i32;
            for iy in iy0..=iy1 {
                for ix in ix0..=ix1 {
                    if ix < 0 || iy < 0 {
                        continue;
                    }
                    let (ux, uy) = (ix as u32, iy as u32);
                    if ux >= m.width || uy >= m.height {
                        continue;
                    }
                    let idx = (uy * m.width + ux) as usize;
                    if m.solids[idx] == 0 {
                        continue;
                    }
                    let tile_min = m.origin + Vec2::new(ix as f32 * cell, iy as f32 * cell);
                    let tile_c = tile_min + Vec2::splat(cell * 0.5);
                    let tile_h = Vec2::splat(cell * 0.5);
                    if Self::overlap_circle_aabb_bool(center, radius, tile_c, tile_h) {
                        out.push((
                            BodyRef::Tile(TileRef {
                                map: TileMapRef(mi as u32),
                                cell_xy: glam::UVec2::new(ux, uy),
                            }),
                            m.user_key,
                        ));
                    }
                }
            }
        }
        out
    }

    // --- Tile-only fast paths ----------------------------------------------
    fn raycast_tiles(
        &self,
        origin: Vec2,
        dir: Vec2,
        max_t: f32,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)> {
        self.raycast_tiles_internal(origin, dir, max_t, mask)
    }

    fn sweep_aabb_tiles(
        &self,
        center: Vec2,
        half_extents: Vec2,
        vel: Vec2,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)> {
        self.sweep_shape_tiles(center, half_extents, vel, mask)
    }

    fn sweep_circle_tiles(
        &self,
        center: Vec2,
        radius: f32,
        vel: Vec2,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)> {
        self.sweep_shape_tiles(center, Vec2::splat(radius), vel, mask)
    }

    fn query_point(&self, p: Vec2, mask: LayerMask) -> Vec<(FrameId, Option<ColKey>)> {
        let cs = self.cfg.cell_size.max(1e-5);
        let cell = self.world_to_cell(p, cs);
        let mut out = Vec::new();
        if let Some(list) = self.grid.get(&cell) {
            for &idx in list {
                let e = &self.entries[idx];
                if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) {
                    continue;
                }
                let hit = match e.desc.kind {
                    ColliderKind::Aabb { .. } => {
                        crate::narrowphase::Narrowphase::overlap_point_aabb(
                            p,
                            e.desc.center,
                            self.half_extents_of(idx),
                        )
                    }
                    ColliderKind::Circle { radius } => {
                        crate::narrowphase::Narrowphase::overlap_point_circle(
                            p,
                            e.desc.center,
                            radius,
                        )
                    }
                    ColliderKind::Point => p == e.desc.center,
                };
                if hit {
                    out.push((FrameId(idx as u32), e.desc.user_key));
                }
            }
        }
        out
    }

    fn query_aabb(
        &self,
        center: Vec2,
        half_extents: Vec2,
        mask: LayerMask,
    ) -> Vec<(FrameId, Option<ColKey>)> {
        let cs = self.cfg.cell_size.max(1e-5);
        let min = center - half_extents;
        let max = center + half_extents;
        let (ix0, iy0) = self.world_to_cell(min, cs);
        let (ix1, iy1) = self.world_to_cell(max, cs);
        let mut out = Vec::new();
        let mut seen = HashSet::new();
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                if let Some(list) = self.grid.get(&(ix, iy)) {
                    for &idx in list {
                        if !seen.insert(idx) {
                            continue;
                        }
                        let e = &self.entries[idx];
                        if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) {
                            continue;
                        }
                        let ov = match e.desc.kind {
                            ColliderKind::Aabb { .. } => {
                                crate::narrowphase::Narrowphase::overlap_aabb_aabb(
                                    e.desc.center,
                                    self.half_extents_of(idx),
                                    center,
                                    half_extents,
                                )
                                .is_some()
                            }
                            ColliderKind::Circle { radius } => Self::overlap_circle_aabb_bool(
                                e.desc.center,
                                radius,
                                center,
                                half_extents,
                            ),
                            ColliderKind::Point => {
                                crate::narrowphase::Narrowphase::overlap_point_aabb(
                                    e.desc.center,
                                    center,
                                    half_extents,
                                )
                            }
                        };
                        if ov {
                            out.push((FrameId(idx as u32), e.desc.user_key));
                        }
                    }
                }
            }
        }
        out
    }

    fn query_circle(
        &self,
        center: Vec2,
        radius: f32,
        mask: LayerMask,
    ) -> Vec<(FrameId, Option<ColKey>)> {
        let cs = self.cfg.cell_size.max(1e-5);
        let min = center - Vec2::splat(radius);
        let max = center + Vec2::splat(radius);
        let (ix0, iy0) = self.world_to_cell(min, cs);
        let (ix1, iy1) = self.world_to_cell(max, cs);
        let mut out = Vec::new();
        let mut seen = HashSet::new();
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                if let Some(list) = self.grid.get(&(ix, iy)) {
                    for &idx in list {
                        if !seen.insert(idx) {
                            continue;
                        }
                        let e = &self.entries[idx];
                        if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) {
                            continue;
                        }
                        let ov = match e.desc.kind {
                            ColliderKind::Aabb { .. } => Self::overlap_circle_aabb_bool(
                                center,
                                radius,
                                e.desc.center,
                                self.half_extents_of(idx),
                            ),
                            ColliderKind::Circle { radius: r1 } => {
                                crate::narrowphase::Narrowphase::overlap_circle_circle(
                                    center,
                                    radius,
                                    e.desc.center,
                                    r1,
                                )
                                .is_some()
                            }
                            ColliderKind::Point => {
                                crate::narrowphase::Narrowphase::overlap_point_circle(
                                    e.desc.center,
                                    center,
                                    radius,
                                )
                            }
                        };
                        if ov {
                            out.push((FrameId(idx as u32), e.desc.user_key));
                        }
                    }
                }
            }
        }
        out
    }

    fn overlap_pair(&self, a: FrameId, b: FrameId) -> Option<Overlap> {
        self.overlap_pair_idx(a.0 as usize, b.0 as usize)
    }

    fn sweep_pair(&self, a: FrameId, b: FrameId) -> Option<SweepHit> {
        self.sweep_pair_idx(a.0 as usize, b.0 as usize)
    }

    fn overlap_by_key(&self, a: ColKey, b: ColKey) -> Option<Overlap> {
        let ia = self.key_to_id.get(&a)?.0 as usize;
        let ib = self.key_to_id.get(&b)?.0 as usize;
        self.overlap_pair_idx(ia, ib)
    }

    fn sweep_by_key(&self, a: ColKey, b: ColKey) -> Option<SweepHit> {
        let ia = self.key_to_id.get(&a)?.0 as usize;
        let ib = self.key_to_id.get(&b)?.0 as usize;
        self.sweep_pair_idx(ia, ib)
    }
}

impl PhysicsWorld {
    fn compute_entry_aabb(&self, e: &Entry) -> (Vec2, Vec2) {
        // Base extents by kind
        let half = match e.desc.kind {
            ColliderKind::Aabb { half_extents } => half_extents,
            ColliderKind::Circle { radius } => Vec2::splat(radius),
            ColliderKind::Point => Vec2::ZERO,
        };

        if self.cfg.tighten_swept_aabb {
            let p0 = e.desc.center;
            let p1 = e.desc.center + e.motion.vel * self.cfg.dt;
            let min_c = p0.min(p1) - half;
            let max_c = p0.max(p1) + half;
            (min_c, max_c)
        } else {
            let min_c = e.desc.center - half;
            let max_c = e.desc.center + half;
            (min_c, max_c)
        }
    }

    fn insert_into_grid(&mut self, idx: usize, min: Vec2, max: Vec2) {
        let cs = self.cfg.cell_size.max(1e-5);
        let ix0 = (min.x / cs).floor() as i32;
        let iy0 = (min.y / cs).floor() as i32;
        let ix1 = (max.x / cs).floor() as i32;
        let iy1 = (max.y / cs).floor() as i32;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                self.grid.entry((ix, iy)).or_default().push(idx);
            }
        }
    }

    fn world_to_cell(&self, p: Vec2, cs: f32) -> (i32, i32) {
        ((p.x / cs).floor() as i32, (p.y / cs).floor() as i32)
    }

    fn half_extents_of(&self, idx: usize) -> Vec2 {
        match self.entries[idx].desc.kind {
            ColliderKind::Aabb { half_extents } => half_extents,
            ColliderKind::Circle { radius } => Vec2::splat(radius),
            ColliderKind::Point => Vec2::ZERO,
        }
    }

    fn overlap_circle_aabb_bool(circle_c: Vec2, r: f32, box_c: Vec2, box_h: Vec2) -> bool {
        let min = box_c - box_h;
        let max = box_c + box_h;
        let clamp = |v: f32, lo: f32, hi: f32| v.max(lo).min(hi);
        let closest = Vec2::new(
            clamp(circle_c.x, min.x, max.x),
            clamp(circle_c.y, min.y, max.y),
        );
        (closest - circle_c).length_squared() <= r * r
    }

    fn overlap_pair_idx(&self, ai: usize, bi: usize) -> Option<Overlap> {
        use crate::api::NarrowphaseApi;
        use crate::narrowphase::Narrowphase;
        let a = &self.entries[ai];
        let b = &self.entries[bi];
        match (a.desc.kind, b.desc.kind) {
            (ColliderKind::Aabb { .. }, ColliderKind::Aabb { .. }) => {
                Narrowphase::overlap_aabb_aabb(
                    a.desc.center,
                    self.half_extents_of(ai),
                    b.desc.center,
                    self.half_extents_of(bi),
                )
            }
            (ColliderKind::Circle { radius: r0 }, ColliderKind::Circle { radius: r1 }) => {
                Narrowphase::overlap_circle_circle(a.desc.center, r0, b.desc.center, r1)
            }
            (ColliderKind::Point, ColliderKind::Aabb { .. }) => {
                if Narrowphase::overlap_point_aabb(
                    a.desc.center,
                    b.desc.center,
                    self.half_extents_of(bi),
                ) {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: a.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Point) => {
                if Narrowphase::overlap_point_aabb(
                    b.desc.center,
                    a.desc.center,
                    self.half_extents_of(ai),
                ) {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: b.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Point, ColliderKind::Circle { radius: r }) => {
                if Narrowphase::overlap_point_circle(a.desc.center, b.desc.center, r) {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: a.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Point) => {
                if Narrowphase::overlap_point_circle(b.desc.center, a.desc.center, r) {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: b.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Circle { radius }, ColliderKind::Aabb { .. }) => {
                if Self::overlap_circle_aabb_bool(
                    a.desc.center,
                    radius,
                    b.desc.center,
                    self.half_extents_of(bi),
                ) {
                    // Approximate normal/contact
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: a.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Circle { radius }) => {
                if Self::overlap_circle_aabb_bool(
                    b.desc.center,
                    radius,
                    a.desc.center,
                    self.half_extents_of(ai),
                ) {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: b.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
            (ColliderKind::Point, ColliderKind::Point) => {
                if a.desc.center == b.desc.center {
                    Some(Overlap {
                        normal: Vec2::ZERO,
                        depth: 0.0,
                        contact: a.desc.center,
                        hint: ResolutionHint::default(),
                    })
                } else {
                    None
                }
            }
        }
    }

    fn sweep_pair_idx(&self, ai: usize, bi: usize) -> Option<SweepHit> {
        use crate::api::NarrowphaseApi;
        use crate::narrowphase::Narrowphase;
        let a = &self.entries[ai];
        let b = &self.entries[bi];
        match (a.desc.kind, b.desc.kind) {
            (ColliderKind::Aabb { .. }, ColliderKind::Aabb { .. }) => Narrowphase::sweep_aabb_aabb(
                a.desc.center,
                self.half_extents_of(ai),
                a.motion.vel * self.cfg.dt,
                b.desc.center,
                self.half_extents_of(bi),
                b.motion.vel * self.cfg.dt,
            ),
            (ColliderKind::Circle { radius: r0 }, ColliderKind::Circle { radius: r1 }) => {
                Narrowphase::sweep_circle_circle(
                    a.desc.center,
                    r0,
                    a.motion.vel * self.cfg.dt,
                    b.desc.center,
                    r1,
                    b.motion.vel * self.cfg.dt,
                )
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Aabb { .. }) => {
                Narrowphase::sweep_circle_aabb(
                    a.desc.center,
                    r,
                    a.motion.vel * self.cfg.dt,
                    b.desc.center,
                    self.half_extents_of(bi),
                    b.motion.vel * self.cfg.dt,
                )
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Circle { radius: r }) => {
                // Swap and invert normal later
                let hit = Narrowphase::sweep_circle_aabb(
                    b.desc.center,
                    r,
                    b.motion.vel * self.cfg.dt,
                    a.desc.center,
                    self.half_extents_of(ai),
                    a.motion.vel * self.cfg.dt,
                )?;
                Some(SweepHit {
                    toi: hit.toi,
                    normal: -hit.normal,
                    contact: hit.contact,
                    hint: ResolutionHint::default(),
                })
            }
            (ColliderKind::Point, ColliderKind::Aabb { .. }) => Narrowphase::sweep_circle_aabb(
                a.desc.center,
                0.0,
                a.motion.vel * self.cfg.dt,
                b.desc.center,
                self.half_extents_of(bi),
                b.motion.vel * self.cfg.dt,
            ),
            (ColliderKind::Aabb { .. }, ColliderKind::Point) => {
                let hit = Narrowphase::sweep_circle_aabb(
                    b.desc.center,
                    0.0,
                    b.motion.vel * self.cfg.dt,
                    a.desc.center,
                    self.half_extents_of(ai),
                    a.motion.vel * self.cfg.dt,
                )?;
                Some(SweepHit {
                    toi: hit.toi,
                    normal: -hit.normal,
                    contact: hit.contact,
                    hint: ResolutionHint::default(),
                })
            }
            (ColliderKind::Point, ColliderKind::Circle { radius: r }) => {
                Narrowphase::sweep_circle_circle(
                    a.desc.center,
                    0.0,
                    a.motion.vel * self.cfg.dt,
                    b.desc.center,
                    r,
                    b.motion.vel * self.cfg.dt,
                )
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Point) => {
                let hit = Narrowphase::sweep_circle_circle(
                    b.desc.center,
                    0.0,
                    b.motion.vel * self.cfg.dt,
                    a.desc.center,
                    r,
                    a.motion.vel * self.cfg.dt,
                )?;
                Some(SweepHit {
                    toi: hit.toi,
                    normal: -hit.normal,
                    contact: hit.contact,
                    hint: ResolutionHint::default(),
                })
            }
            (ColliderKind::Point, ColliderKind::Point) => None,
        }
    }

    /// Return debug/perf stats for the current built frame.
    pub fn debug_stats(&self) -> WorldStats {
        use std::collections::HashSet;
        let entries = self.entries.len();
        let cells = self.grid.len();
        let mut candidate_pairs: usize = 0;
        let mut seen: HashSet<(usize, usize)> = HashSet::new();
        for v in self.grid.values() {
            let n = v.len();
            if n >= 2 {
                candidate_pairs += n * (n - 1) / 2;
            }
            for i in 0..n {
                for j in (i + 1)..n {
                    let a = v[i];
                    let b = v[j];
                    let key = if a < b { (a, b) } else { (b, a) };
                    seen.insert(key);
                }
            }
        }
        WorldStats {
            entries,
            cells,
            candidate_pairs,
            unique_pairs: seen.len(),
        }
    }

    /// Return timing breakdown for the last `end_frame`/`generate_events` runs.
    pub fn timing(&self) -> Option<WorldTiming> {
        self.last_timing
    }

    fn allows_pair(&self, a: LayerMask, b: LayerMask) -> bool {
        if self.cfg.require_mutual_consent {
            a.allows(b) && b.allows(a)
        } else {
            a.allows(b) || b.allows(a)
        }
    }

    fn tile_at(m: &TileMap, ix: i32, iy: i32) -> Option<usize> {
        if ix < 0 || iy < 0 {
            return None;
        }
        let ux = ix as u32;
        let uy = iy as u32;
        if ux >= m.width || uy >= m.height {
            return None;
        }
        Some((uy * m.width + ux) as usize)
    }

    fn any_tile_overlap_at(&self, mi: usize, m: &TileMap, center: Vec2, he: Vec2) -> Option<TileRef> {
        let cell = m.cell.max(1e-5);
        let min = center - he - m.origin;
        let max = center + he - m.origin;
        let ix0 = (min.x / cell).floor() as i32;
        let iy0 = (min.y / cell).floor() as i32;
        let ix1 = (max.x / cell).floor() as i32;
        let iy1 = (max.y / cell).floor() as i32;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                if let Some(idx) = Self::tile_at(m, ix, iy)
                    && m.solids[idx] != 0
                {
                    let tile_min = m.origin + Vec2::new(ix as f32 * cell, iy as f32 * cell);
                    // quick overlap check: AABB vs tile AABB
                    let tile_c = tile_min + Vec2::splat(cell * 0.5);
                    let tile_h = Vec2::splat(cell * 0.5);
                    if crate::narrowphase::Narrowphase::overlap_aabb_aabb(
                        center, he, tile_c, tile_h,
                    )
                    .is_some()
                    {
                        return Some(TileRef {
                            map: TileMapRef(mi as u32),
                            cell_xy: glam::UVec2::new(ix as u32, iy as u32),
                        });
                    }
                }
            }
        }
        None
    }

    fn sweep_shape_tiles(
        &self,
        center: Vec2,
        he: Vec2,
        vel: Vec2,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)> {
        let mut best: Option<(TileRef, SweepHit, Option<ColKey>)> = None;
        let eps = self.cfg.tile_eps.max(1e-6);
        let p0 = center;
        let d = vel * self.cfg.dt;
        for (mi, m) in self.tilemaps.iter().enumerate() {
            if !self.allows_pair(mask, m.mask) {
                continue;
            }
            let cell = m.cell.max(1e-5);
            let len = d.length();
            let steps_f = ((len / cell).ceil().max(1.0)) * 2.0;
            let steps = steps_f as i32;
            let mut t_prev = 0.0f32;
            let mut prev_free = p0;
            let tref_hit: Option<TileRef>;
            for i in 1..=steps {
                let t = (i as f32 / steps_f).min(1.0);
                let p = p0 + d * t;
                if let Some(tref) = self.any_tile_overlap_at(mi, m, p, he) {
                    tref_hit = Some(tref);
                    // binary search refine
                    let mut lo = t_prev;
                    let mut hi = t;
                    for _ in 0..14 {
                        let mid = 0.5 * (lo + hi);
                        let q = p0 + d * mid;
                        if self.any_tile_overlap_at(mi, m, q, he).is_some() {
                            hi = mid;
                        } else {
                            lo = mid;
                            prev_free = q;
                        }
                    }
                    let toi = hi;
                    let p_hit = p0 + d * toi;
                    let tr = tref_hit.unwrap();
                    let tile_min = m.origin
                        + Vec2::new(tr.cell_xy.x as f32 * cell, tr.cell_xy.y as f32 * cell);
                    let (n, _depth, contact) = crate::narrowphase::Narrowphase::aabb_tile_pushout(
                        p_hit, he, tile_min, cell,
                    );
                    let mut hit = SweepHit {
                        toi,
                        normal: if n.length_squared() > 0.0 {
                            n
                        } else {
                            (p_hit - prev_free).normalize_or_zero()
                        },
                        contact,
                        hint: ResolutionHint::default(),
                    };
                    hit.hint.safe_pos = Some(p0 + d * (toi - eps));
                    best = Some((tr, hit, m.user_key));
                    break;
                } else {
                    t_prev = t;
                    prev_free = p;
                }
            }
            if best.is_some() {
                break;
            }
        }
        best
    }

    // Tile raycast helper
    fn raycast_tiles_internal(
        &self,
        origin: Vec2,
        dir: Vec2,
        max_t: f32,
        mask: LayerMask,
    ) -> Option<(TileRef, SweepHit, Option<ColKey>)> {
        if dir.length_squared() == 0.0 {
            return None;
        }
        let mut best: Option<(TileRef, SweepHit, Option<ColKey>)> = None;
        let eps = self.cfg.tile_eps.max(1e-6);

        for (mi, m) in self.tilemaps.iter().enumerate() {
            let cell = m.cell.max(1e-5);
            let local = origin - m.origin;
            let mut cx = (local.x / cell).floor() as i32;
            let mut cy = (local.y / cell).floor() as i32;

            let step_x = if dir.x > 0.0 {
                1
            } else if dir.x < 0.0 {
                -1
            } else {
                0
            };
            let step_y = if dir.y > 0.0 {
                1
            } else if dir.y < 0.0 {
                -1
            } else {
                0
            };

            let next_boundary = |c: i32, step: i32| {
                if step > 0 {
                    (c as f32 + 1.0) * cell
                } else {
                    c as f32 * cell
                }
            };

            let mut t_max_x = if step_x != 0 {
                let nb = m.origin.x + next_boundary(cx, step_x);
                (nb - origin.x) / dir.x
            } else {
                f32::INFINITY
            };

            let mut t_max_y = if step_y != 0 {
                let nb = m.origin.y + next_boundary(cy, step_y);
                (nb - origin.y) / dir.y
            } else {
                f32::INFINITY
            };

            let t_delta_x = if step_x != 0 {
                cell / dir.x.abs()
            } else {
                f32::INFINITY
            };
            let t_delta_y = if step_y != 0 {
                cell / dir.y.abs()
            } else {
                f32::INFINITY
            };

            let mut t_curr = 0.0f32;
            let mut last_axis_x: Option<bool> = None; // None => starting cell

            for _ in 0..20_000 {
                if t_curr > max_t {
                    break;
                }

                if cx >= 0 && cy >= 0 && (cx as u32) < m.width && (cy as u32) < m.height {
                    let idx = cy as u32 * m.width + cx as u32;
                    if m.solids[idx as usize] != 0 && self.allows_pair(mask, m.mask) {
                        // hit the NEAR face: we entered this cell at t_curr
                        let toi = t_curr.max(0.0);
                        let normal = match last_axis_x {
                            Some(true) => Vec2::new(-(step_x as f32), 0.0),
                            Some(false) => Vec2::new(0.0, -(step_y as f32)),
                            None => Vec2::ZERO, // started inside a solid tile
                        };
                        let mut hit = SweepHit {
                            toi,
                            normal,
                            contact: origin + dir * toi,
                            hint: ResolutionHint::default(),
                        };
                        hit.hint.safe_pos = Some(origin + dir * (toi - eps));
                        let tr = TileRef {
                            map: TileMapRef(mi as u32),
                            cell_xy: glam::UVec2::new(cx as u32, cy as u32),
                        };
                        let key = m.user_key;

                        match &best {
                            Some((_, bh, _)) if hit.toi >= bh.toi => {}
                            _ => best = Some((tr, hit, key)),
                        }
                        break;
                    }
                }

                // step to next cell; update entry time & axis
                if t_max_x < t_max_y {
                    cx += step_x;
                    t_curr = t_max_x;
                    t_max_x += t_delta_x;
                    last_axis_x = Some(true);
                } else {
                    cy += step_y;
                    t_curr = t_max_y;
                    t_max_y += t_delta_y;
                    last_axis_x = Some(false);
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> WorldConfig {
        WorldConfig {
            cell_size: 1.0,
            dt: 1.0,
            tighten_swept_aabb: true,
            enable_overlap_events: true,
            enable_sweep_events: true,
            max_events: 1024,
            enable_timing: false,
            tile_eps: 1e-4,
            require_mutual_consent: true,
        }
    }

    #[test]
    fn test_push_and_end_frame_grid_coverage() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        // AABB covering from (-0.5,-0.5) to (0.5,0.5)
        let mask = LayerMask::simple(1, 1);
        w.push_aabb(Vec2::ZERO, Vec2::splat(0.5), Vec2::ZERO, mask, None);
        w.end_frame();
        // With floor indexing, bounds straddling origin cover 4 cells
        assert_eq!(w.grid.len(), 4);
        for k in [(-1, -1), (-1, 0), (0, -1), (0, 0)] {
            assert!(w.grid.contains_key(&k));
            assert_eq!(w.grid[&k].len(), 1);
        }
    }

    #[test]
    fn test_mask_mutual_consent() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let a_mask = LayerMask {
            layer: 1,
            collides_with: 2,
            exclude: 0,
        };
        let b_mask = LayerMask {
            layer: 2,
            collides_with: 0,
            exclude: 0,
        };
        w.push_aabb(
            Vec2::new(-0.5, 0.0),
            Vec2::splat(0.5),
            Vec2::new(1.0, 0.0),
            a_mask,
            None,
        );
        w.push_aabb(
            Vec2::new(0.5, 0.0),
            Vec2::splat(0.5),
            Vec2::ZERO,
            b_mask,
            None,
        );
        w.end_frame();
        w.generate_events();
        assert_eq!(w.drain_events().len(), 0);
    }

    #[test]
    fn test_generate_sweep_event_and_drain() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let mask = LayerMask::simple(1, 1);
        let a = w.push_circle(
            Vec2::new(-2.0, 0.0),
            0.5,
            Vec2::new(4.0, 0.0),
            mask,
            Some(11),
        );
        let b = w.push_circle(Vec2::new(0.0, 0.0), 0.5, Vec2::ZERO, mask, Some(22));
        w.end_frame();
        w.generate_events();
        let evs = w.drain_events();
        assert_eq!(evs.len(), 1);
        let ev = evs[0];
        assert!(matches!(ev.kind, crate::types::EventKind::Sweep));
        match ev.a {
            BodyRef::Collider(id) => assert_eq!(id, a),
            _ => panic!("expected collider A"),
        }
        match ev.b {
            BodyRef::Collider(id) => assert_eq!(id, b),
            _ => panic!("expected collider B"),
        }
        assert!(ev.sweep.is_some());
        // Drained; buffer should be empty now
        assert!(w.drain_events().is_empty());
    }

    #[test]
    fn test_queries_and_pairwise() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let mask = LayerMask::simple(1, 1);
        let id_a = w.push_aabb(
            Vec2::new(0.0, 0.0),
            Vec2::splat(1.0),
            Vec2::ZERO,
            mask,
            Some(100),
        );
        let id_b = w.push_circle(Vec2::new(3.0, 0.0), 1.0, Vec2::ZERO, mask, Some(200));
        w.end_frame();
        // point inside AABB
        let q1 = w.query_point(Vec2::new(0.5, 0.5), mask);
        assert!(q1.iter().any(|(id, _)| *id == id_a));
        // aabb overlaps a
        let q2 = w.query_aabb(Vec2::new(0.0, 0.0), Vec2::splat(0.5), mask);
        assert!(q2.iter().any(|(id, _)| *id == id_a));
        // circle query hits circle b
        let q3 = w.query_circle(Vec2::new(3.0, 0.0), 1.0, mask);
        assert!(q3.iter().any(|(id, _)| *id == id_b));
        // pairwise overlap between aabb and circle should be false
        assert!(w.overlap_pair(id_a, id_b).is_none());
        // by key lookup
        assert!(w.overlap_by_key(100, 200).is_none());
    }

    #[test]
    fn test_raycast_hits_closest() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let mask = LayerMask::simple(1, 1);
        let id_a = w.push_aabb(
            Vec2::new(2.0, 0.0),
            Vec2::splat(0.5),
            Vec2::ZERO,
            mask,
            Some(1),
        );
        let _id_b = w.push_aabb(
            Vec2::new(4.0, 0.0),
            Vec2::splat(0.5),
            Vec2::ZERO,
            mask,
            Some(2),
        );
        w.end_frame();
        let hit = w
            .raycast(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), mask, 10.0)
            .unwrap();
        assert_eq!(hit.0, id_a);
        let hit2 = w.raycast(Vec2::new(0.0, 0.0), Vec2::new(-1.0, 0.0), mask, 10.0);
        assert!(hit2.is_none());
    }

    // --- Tile tests ---------------------------------------------------------

    fn simple_map_bits() -> Vec<u8> {
        // 3x1 with middle solid
        vec![0, 1, 0]
    }

    #[test]
    fn test_tile_raycast_basic() {
        let mut w = PhysicsWorld::new(cfg());
        let map = TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 3,
            height: 1,
            solids: &simple_map_bits(),
            mask: LayerMask::simple(2, 1),
            user_key: Some(77),
        };
        w.attach_tilemap(map);
        // ray from left hits middle cell at x=1 boundary
        let origin = Vec2::new(-0.5, 0.5);
        let dir = Vec2::new(1.0, 0.0);
        let mask = LayerMask::simple(1, 2);
        let hit = w.raycast_all(origin, dir, mask, 10.0).unwrap();
        match hit.0 {
            BodyRef::Tile(t) => {
                assert_eq!(t.cell_xy.x, 1);
            }
            _ => panic!("expected tile hit"),
        }
    }

    #[test]
    fn test_query_aabb_all_tiles() {
        let mut w = PhysicsWorld::new(cfg());
        let map = TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 3,
            height: 1,
            solids: &simple_map_bits(),
            mask: LayerMask::simple(2, 1),
            user_key: None,
        };
        w.attach_tilemap(map);
        let res = w.query_aabb_all(
            Vec2::new(1.0, 0.5),
            Vec2::splat(0.6),
            LayerMask::simple(1, 2),
        );
        assert!(
            res.iter().any(
                |(b, _)| matches!(b, BodyRef::Tile(TileRef { cell_xy, .. }) if cell_xy.x == 1)
            )
        );
    }

    #[test]
    fn test_sweep_aabb_tiles_basic() {
        let mut w = PhysicsWorld::new(cfg());
        let solids = vec![0, 1, 0, 0, 1, 0, 0, 1, 0]; // 3x3 column in middle
        let map = TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 3,
            height: 3,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        };
        w.attach_tilemap(map);
        let start = Vec2::new(0.2, 1.5);
        let he = Vec2::splat(0.3);
        let vel = Vec2::new(2.0, 0.0);
        let res = w
            .sweep_aabb_tiles(start, he, vel, LayerMask::simple(1, 2))
            .unwrap();
        assert!(res.1.toi > 0.0 && res.1.toi <= 1.0);
        // normal should be -X (hitting vertical face)
        assert!(res.1.normal.x < -0.5);
        assert!(res.1.hint.safe_pos.is_some());
    }

    #[test]
    fn test_tile_raycast_monotonicity() {
        let mut w = PhysicsWorld::new(cfg());
        let solids = vec![0, 1, 0]; // 3x1, solid at x=1
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 3,
            height: 1,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let origin = Vec2::new(0.1, 0.5);
        let dir = Vec2::new(1.0, 0.0);
        let mask = LayerMask::simple(1, 2);
        let h1 = w.raycast_tiles(origin, dir, 0.8, mask);
        assert!(h1.is_none());
        let h2 = w.raycast_tiles(origin, dir, 10.0, mask).unwrap();
        let t2 = h2.1.toi;
        assert!(t2 > 0.8);
        let h3 = w.raycast_tiles(origin, dir, t2, mask).unwrap();
        assert!((h3.1.toi - t2).abs() < 1e-5);
    }

    #[test]
    fn test_safe_pos_no_overlap_after_sweep() {
        let mut w = PhysicsWorld::new(cfg());
        let solids = vec![0, 1, 0, 0, 1, 0, 0, 1, 0];
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 3,
            height: 3,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let start = Vec2::new(0.2, 1.5);
        let he = Vec2::splat(0.4);
        let vel = Vec2::new(3.0, 0.0);
        let (_tref, hit, _key) = w
            .sweep_aabb_tiles(start, he, vel, LayerMask::simple(1, 2))
            .unwrap();
        let p = hit.hint.safe_pos.expect("safe_pos should exist");
        let hits = w.query_aabb_all(p, he, LayerMask::simple(1, 2));
        assert!(!hits.iter().any(|(b, _)| matches!(b, BodyRef::Tile(_))));
    }

    #[test]
    fn test_start_embedded_emits_overlap_event() {
        let mut w = PhysicsWorld::new(cfg());
        let solids = vec![1]; // 1x1 solid at origin cell [0,0]
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width: 1,
            height: 1,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: Some(42),
        });
        w.begin_frame();
        // AABB entirely inside the tile, no motion
        let mask = LayerMask::simple(1, 2);
        w.push_aabb(
            Vec2::new(0.5, 0.5),
            Vec2::splat(0.1),
            Vec2::ZERO,
            mask,
            Some(7),
        );
        w.end_frame();
        w.generate_events();
        let evs = w.drain_events();
        assert!(evs.iter().any(|e| matches!(e.kind, EventKind::Overlap)
            && matches!(e.b, BodyRef::Tile(_))
            && e.overlap.unwrap().hint.start_embedded));
    }

    #[test]
    fn test_tile_raycast_monotonicity_random() {
        let mut w = PhysicsWorld::new(cfg());
        // map with a single solid column at x=10
        let width = 32u32;
        let height = 16u32;
        let mut solids = vec![0u8; (width * height) as usize];
        for y in 0..height {
            solids[(y * width + 10) as usize] = 1;
        }
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width,
            height,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let mask = LayerMask::simple(1, 2);
        let mut seed = 1234567u32;
        let lcg = |s: &mut u32| {
            *s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            *s
        };
        for _ in 0..50 {
            let ry = (lcg(&mut seed) as f32 / u32::MAX as f32) * (height as f32 - 1.0) + 0.5;
            let ox = (lcg(&mut seed) as f32 / u32::MAX as f32) * 5.0; // start in [0,5)
            let origin = Vec2::new(ox, ry);
            let dir = Vec2::new(1.0, 0.0);
            let small = 1.0; // < distance to column at x=10
            let big = 100.0;
            let h_small = w.raycast_tiles(origin, dir, small, mask);
            let h_big = w.raycast_tiles(origin, dir, big, mask);
            if let Some((_tref_s, hs, _)) = h_small {
                let (_tref_b, hb, _) = h_big.expect("big max_t should retain hit");
                assert!((hs.toi - hb.toi).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn test_safe_pos_invariant_random() {
        let mut w = PhysicsWorld::new(cfg());
        // vertical wall at x=5 across all rows
        let width = 16u32;
        let height = 16u32;
        let mut solids = vec![0u8; (width * height) as usize];
        for y in 0..height {
            solids[(y * width + 5) as usize] = 1;
        }
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width,
            height,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let mask = LayerMask::simple(1, 2);
        let mut seed = 42u32;
        let lcg = |s: &mut u32| {
            *s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            *s
        };
        for _ in 0..40 {
            let y = (lcg(&mut seed) as f32 / u32::MAX as f32) * 10.0 + 2.0;
            let start_x = (lcg(&mut seed) as f32 / u32::MAX as f32) * 3.0;
            let start = Vec2::new(start_x, y);
            let he = Vec2::new(0.2, 0.3);
            let vel = Vec2::new(4.0 + (lcg(&mut seed) as f32 / u32::MAX as f32) * 2.0, 0.0);
            if let Some((_tref, hit, _)) = w.sweep_aabb_tiles(start, he, vel, mask)
                && let Some(p) = hit.hint.safe_pos
            {
                let hits = w.query_aabb_all(p, he, mask);
                assert!(!hits.iter().any(|(b, _)| matches!(b, BodyRef::Tile(_))));
            }
        }
    }

    #[test]
    fn test_tile_raycast_diagonal_hits_correct_cell() {
        let mut w = PhysicsWorld::new(cfg());
        // 16x16 map with a single solid at (5,5)
        let width = 16u32;
        let height = 16u32;
        let mut solids = vec![0u8; (width * height) as usize];
        solids[(5 * width + 5) as usize] = 1;
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width,
            height,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let mask = LayerMask::simple(1, 2);
        let origin = Vec2::new(0.25, 0.25);
        let dir = Vec2::new(1.0, 1.0).normalize();
        let (_tref, hit, _key) = w
            .raycast_tiles(origin, dir, 100.0, mask)
            .expect("expected tile hit");
        // The cell index should be (5,5)
        if let Some((TileRef { cell_xy, .. }, _, _)) = w.raycast_tiles(origin, dir, 100.0, mask) {
            assert_eq!(cell_xy.x, 5);
            assert_eq!(cell_xy.y, 5);
            // Contact must lie on one of the tile boundaries [1.0 tolerance]
            let cx = hit.contact.x;
            let cy = hit.contact.y;
            let on_vert = (cx - 5.0).abs() < 1e-3 || (cx - 6.0).abs() < 1e-3;
            let on_horz = (cy - 5.0).abs() < 1e-3 || (cy - 6.0).abs() < 1e-3;
            assert!(on_vert || on_horz);
        } else {
            panic!("no tile hit");
        }
    }

    #[test]
    fn test_circle_sweep_minkowski_equivalence() {
        let mut w = PhysicsWorld::new(cfg());
        // vertical wall at x=5 across all rows
        let width = 16u32;
        let height = 16u32;
        let mut solids = vec![0u8; (width * height) as usize];
        for y in 0..height {
            solids[(y * width + 5) as usize] = 1;
        }
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width,
            height,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let mask = LayerMask::simple(1, 2);
        let c = Vec2::new(1.5, 3.5);
        let r = 0.4;
        let vel = Vec2::new(6.0, 0.0);
        let (_t_aabb, hit_aabb, _) = w.sweep_aabb_tiles(c, Vec2::splat(r), vel, mask).unwrap();
        let (_t_circ, hit_circ, _) = w.sweep_circle_tiles(c, r, vel, mask).unwrap();
        assert!((hit_aabb.toi - hit_circ.toi).abs() < 5e-3);
        // Normals should closely match
        let dn = (hit_aabb.normal - hit_circ.normal).length();
        assert!(dn < 1e-3);
    }

    // Note: diagonal raycast octants are covered by test_tile_raycast_diagonal_hits_correct_cell.

    #[test]
    fn test_circle_sweep_diagonal_vel_and_radii() {
        let mut w = PhysicsWorld::new(cfg());
        // 32x32 map with vertical wall at x=16
        let width = 32u32;
        let height = 32u32;
        let mut solids = vec![0u8; (width * height) as usize];
        for y in 0..height {
            solids[(y * width + 16) as usize] = 1;
        }
        w.attach_tilemap(TileMapDesc {
            origin: Vec2::new(0.0, 0.0),
            cell: 1.0,
            width,
            height,
            solids: &solids,
            mask: LayerMask::simple(2, 1),
            user_key: None,
        });
        let mask = LayerMask::simple(1, 2);
        let center = Vec2::new(12.5, 10.5);
        let radii = [0.1f32, 0.25, 0.5, 0.9];
        let vels = [
            Vec2::new(6.0, 3.0),
            Vec2::new(12.0, -6.0),
            Vec2::new(8.0, 4.0),
        ];
        for &r in &radii {
            for &v in &vels {
                let (_tr1, hit_c, _k1) = w
                    .sweep_circle_tiles(center, r, v, mask)
                    .expect("circle sweep should hit");
                let (_tr2, hit_a, _k2) = w
                    .sweep_aabb_tiles(center, Vec2::splat(r), v, mask)
                    .expect("aabb(r) sweep should hit");
                assert!(
                    (hit_c.toi - hit_a.toi).abs() < 5e-3,
                    "toi mismatch r={} v=({},{})",
                    r,
                    v.x,
                    v.y
                );
                let dn = (hit_c.normal - hit_a.normal).length();
                assert!(dn < 1e-2, "normal mismatch r={} v=({},{})", r, v.x, v.y);
                assert!(hit_c.hint.safe_pos.is_some());
            }
        }
    }
}
