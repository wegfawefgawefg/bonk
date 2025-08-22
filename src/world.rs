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

    // Event buffer for this frame
    events: Vec<Event>,

    // Timing for last operations (optional)
    last_timing: Option<WorldTiming>,
}

struct Entry {
    desc: ColliderDesc,
    motion: Motion,
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
        let desc = ColliderDesc { kind: ColliderKind::Circle { radius }, center, mask, user_key };
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
        let desc = ColliderDesc { kind: ColliderKind::Aabb { half_extents }, center, mask, user_key };
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
        let desc = ColliderDesc { kind: ColliderKind::Point, center: p, mask, user_key };
        let motion = Motion { vel };
        self.push(desc, motion)
    }

    fn end_frame(&mut self) {
        // Build axis-aligned bounds for each entry and insert into uniform grid.
        let t_all = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
        let t0 = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
        self.aabbs.resize(self.entries.len(), (Vec2::ZERO, Vec2::ZERO));

        for (i, e) in self.entries.iter().enumerate() {
            let (min, max) = self.compute_entry_aabb(e);
            self.aabbs[i] = (min, max);
        }
        let aabb_ms = t0.map(|t| t.elapsed().as_secs_f64() * 1000.0).unwrap_or(0.0);
        let t1 = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
        let aabbs_snapshot = self.aabbs.clone();
        for (i, (min, max)) in aabbs_snapshot.into_iter().enumerate() {
            self.insert_into_grid(i, min, max);
        }
        let grid_ms = t1.map(|t| t.elapsed().as_secs_f64() * 1000.0).unwrap_or(0.0);
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
        let t_all = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
        let t_scan0 = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
        let mut seen_pairs: HashSet<(usize, usize)> = HashSet::new();
        let mut push_event = |ev: Event, buf: &mut Vec<Event>, max: usize| {
            if buf.len() < max { buf.push(ev); }
        };

        for indices in self.grid.values() {
            for i0 in 0..indices.len() {
                for i1 in (i0 + 1)..indices.len() {
                    let a = indices[i0];
                    let b = indices[i1];
                    let key = if a < b { (a, b) } else { (b, a) };
                    if !seen_pairs.insert(key) { continue; }
                    if self.events.len() >= self.cfg.max_events { return; }

                    let t_np0 = if self.cfg.enable_timing { Some(Instant::now()) } else { None };
                    let ea = &self.entries[a];
                    let eb = &self.entries[b];
                    // Mask mutual consent
                    if !(ea.desc.mask.allows(eb.desc.mask) && eb.desc.mask.allows(ea.desc.mask)) {
                        continue;
                    }

                    let rel = ea.motion.vel - eb.motion.vel;
                    let dynamic = rel.length_squared() > 1e-12;

                    if dynamic && self.cfg.enable_sweep_events {
                        if let Some(sweep) = self.sweep_pair_idx(a, b) {
                            let ev = Event {
                                kind: crate::types::EventKind::Sweep,
                                a: FrameId(a as u32),
                                b: FrameId(b as u32),
                                a_key: ea.desc.user_key,
                                b_key: eb.desc.user_key,
                                overlap: None,
                                sweep: Some(sweep),
                            };
                            push_event(ev, &mut self.events, self.cfg.max_events);
                        } else if self.cfg.enable_overlap_events {
                            if let Some(ov) = self.overlap_pair_idx(a, b) {
                                let ev = Event {
                                    kind: crate::types::EventKind::Overlap,
                                    a: FrameId(a as u32),
                                    b: FrameId(b as u32),
                                    a_key: ea.desc.user_key,
                                    b_key: eb.desc.user_key,
                                    overlap: Some(ov),
                                    sweep: None,
                                };
                                push_event(ev, &mut self.events, self.cfg.max_events);
                            }
                        }
                    } else if self.cfg.enable_overlap_events {
                        if let Some(ov) = self.overlap_pair_idx(a, b) {
                            let ev = Event {
                                kind: crate::types::EventKind::Overlap,
                                a: FrameId(a as u32),
                                b: FrameId(b as u32),
                                a_key: ea.desc.user_key,
                                b_key: eb.desc.user_key,
                                overlap: Some(ov),
                                sweep: None,
                            };
                            push_event(ev, &mut self.events, self.cfg.max_events);
                        }
                    }
                    if let (Some(t_np0), Some(timing)) = (t_np0, self.last_timing.as_mut()) {
                        timing.generate_narrowphase_ms += t_np0.elapsed().as_secs_f64() * 1000.0;
                    }
                }
            }
        }
        if let Some(t_scan0) = t_scan0 {
            if self.last_timing.is_none() { self.last_timing = Some(WorldTiming::default()); }
            if let Some(timing) = self.last_timing.as_mut() {
                timing.generate_scan_ms = t_scan0.elapsed().as_secs_f64() * 1000.0 - timing.generate_narrowphase_ms;
            }
        }
        if let Some(t_all) = t_all {
            if self.last_timing.is_none() { self.last_timing = Some(WorldTiming::default()); }
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

    fn raycast(
        &self,
        origin: Vec2,
        dir: Vec2,
        mask: LayerMask,
        max_t: f32,
    ) -> Option<(FrameId, SweepHit, Option<ColKey>)> {
        if dir.length_squared() == 0.0 { return None; }
        let cs = self.cfg.cell_size.max(1e-5);
        // Setup DDA
        let mut best: Option<(usize, SweepHit)> = None;
        let mut tested: HashSet<usize> = HashSet::new();

        let mut cell = self.world_to_cell(origin, cs);
        let step_x = if dir.x > 0.0 { 1 } else if dir.x < 0.0 { -1 } else { 0 };
        let step_y = if dir.y > 0.0 { 1 } else if dir.y < 0.0 { -1 } else { 0 };
        let next_boundary = |c: i32, step: i32| -> f32 {
            if step > 0 { (c as f32 + 1.0) * cs } else { c as f32 * cs }
        };
        let cell_origin = Vec2::new(cell.0 as f32 * cs, cell.1 as f32 * cs);
        let mut t_max_x = if step_x != 0 {
            let nb = next_boundary(cell.0, step_x);
            (nb - origin.x) / dir.x
        } else { f32::INFINITY };
        let mut t_max_y = if step_y != 0 {
            let nb = next_boundary(cell.1, step_y);
            (nb - origin.y) / dir.y
        } else { f32::INFINITY };
        let t_delta_x = if step_x != 0 { cs / dir.x.abs() } else { f32::INFINITY };
        let t_delta_y = if step_y != 0 { cs / dir.y.abs() } else { f32::INFINITY };

        let mut t_curr = 0.0f32;
        // Visit cells until exceeding max_t
        for _ in 0..10_000 { // safety cap
            if t_curr > max_t { break; }
            if let Some(list) = self.grid.get(&cell) {
                for &idx in list {
                    if !tested.insert(idx) { continue; }
                    let e = &self.entries[idx];
                    // Mask mutual consent between ray mask and collider mask
                    if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) { continue; }
                    let hit = match e.desc.kind {
                        ColliderKind::Aabb { .. } => {
                            let (min, max) = self.aabbs[idx];
                            crate::narrowphase::Narrowphase::ray_aabb(origin, dir, min, max)
                        }
                        ColliderKind::Circle { radius } => {
                            crate::narrowphase::Narrowphase::ray_circle(origin, dir, e.desc.center, radius)
                        }
                        ColliderKind::Point => {
                            crate::narrowphase::Narrowphase::ray_circle(origin, dir, e.desc.center, 0.0)
                        }
                    };
                    if let Some(h) = hit {
                        if h.toi < 0.0 || h.toi > max_t { continue; }
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

    fn query_point(&self, p: Vec2, mask: LayerMask) -> Vec<(FrameId, Option<ColKey>)> {
        let cs = self.cfg.cell_size.max(1e-5);
        let cell = self.world_to_cell(p, cs);
        let mut out = Vec::new();
        if let Some(list) = self.grid.get(&cell) {
            for &idx in list {
                let e = &self.entries[idx];
                if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) { continue; }
                let hit = match e.desc.kind {
                    ColliderKind::Aabb { .. } => crate::narrowphase::Narrowphase::overlap_point_aabb(p, e.desc.center, self.half_extents_of(idx)),
                    ColliderKind::Circle { radius } => crate::narrowphase::Narrowphase::overlap_point_circle(p, e.desc.center, radius),
                    ColliderKind::Point => p == e.desc.center,
                };
                if hit { out.push((FrameId(idx as u32), e.desc.user_key)); }
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
        for iy in iy0..=iy1 { for ix in ix0..=ix1 {
            if let Some(list) = self.grid.get(&(ix, iy)) {
                for &idx in list {
                    if !seen.insert(idx) { continue; }
                    let e = &self.entries[idx];
                    if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) { continue; }
                    let ov = match e.desc.kind {
                        ColliderKind::Aabb { .. } => crate::narrowphase::Narrowphase::overlap_aabb_aabb(e.desc.center, self.half_extents_of(idx), center, half_extents).is_some(),
                        ColliderKind::Circle { radius } => Self::overlap_circle_aabb_bool(e.desc.center, radius, center, half_extents),
                        ColliderKind::Point => crate::narrowphase::Narrowphase::overlap_point_aabb(e.desc.center, center, half_extents),
                    };
                    if ov { out.push((FrameId(idx as u32), e.desc.user_key)); }
                }
            }
        }}
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
        for iy in iy0..=iy1 { for ix in ix0..=ix1 {
            if let Some(list) = self.grid.get(&(ix, iy)) {
                for &idx in list {
                    if !seen.insert(idx) { continue; }
                    let e = &self.entries[idx];
                    if !(mask.allows(e.desc.mask) && e.desc.mask.allows(mask)) { continue; }
                    let ov = match e.desc.kind {
                        ColliderKind::Aabb { .. } => Self::overlap_circle_aabb_bool(center, radius, e.desc.center, self.half_extents_of(idx)),
                        ColliderKind::Circle { radius: r1 } => crate::narrowphase::Narrowphase::overlap_circle_circle(center, radius, e.desc.center, r1).is_some(),
                        ColliderKind::Point => crate::narrowphase::Narrowphase::overlap_point_circle(e.desc.center, center, radius),
                    };
                    if ov { out.push((FrameId(idx as u32), e.desc.user_key)); }
                }
            }
        }}
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
                Narrowphase::overlap_aabb_aabb(a.desc.center, self.half_extents_of(ai), b.desc.center, self.half_extents_of(bi))
            }
            (ColliderKind::Circle { radius: r0 }, ColliderKind::Circle { radius: r1 }) => {
                Narrowphase::overlap_circle_circle(a.desc.center, r0, b.desc.center, r1)
            }
            (ColliderKind::Point, ColliderKind::Aabb { .. }) => {
                if Narrowphase::overlap_point_aabb(a.desc.center, b.desc.center, self.half_extents_of(bi)) {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: a.desc.center })
                } else { None }
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Point) => {
                if Narrowphase::overlap_point_aabb(b.desc.center, a.desc.center, self.half_extents_of(ai)) {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: b.desc.center })
                } else { None }
            }
            (ColliderKind::Point, ColliderKind::Circle { radius: r }) => {
                if Narrowphase::overlap_point_circle(a.desc.center, b.desc.center, r) {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: a.desc.center })
                } else { None }
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Point) => {
                if Narrowphase::overlap_point_circle(b.desc.center, a.desc.center, r) {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: b.desc.center })
                } else { None }
            }
            (ColliderKind::Circle { radius }, ColliderKind::Aabb { .. }) => {
                if Self::overlap_circle_aabb_bool(a.desc.center, radius, b.desc.center, self.half_extents_of(bi)) {
                    // Approximate normal/contact
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: a.desc.center })
                } else { None }
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Circle { radius }) => {
                if Self::overlap_circle_aabb_bool(b.desc.center, radius, a.desc.center, self.half_extents_of(ai)) {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: b.desc.center })
                } else { None }
            }
            (ColliderKind::Point, ColliderKind::Point) => {
                if a.desc.center == b.desc.center {
                    Some(Overlap { normal: Vec2::ZERO, depth: 0.0, contact: a.desc.center })
                } else { None }
            }
        }
    }

    fn sweep_pair_idx(&self, ai: usize, bi: usize) -> Option<SweepHit> {
        use crate::api::NarrowphaseApi;
        use crate::narrowphase::Narrowphase;
        let a = &self.entries[ai];
        let b = &self.entries[bi];
        match (a.desc.kind, b.desc.kind) {
            (ColliderKind::Aabb { .. }, ColliderKind::Aabb { .. }) => {
                Narrowphase::sweep_aabb_aabb(a.desc.center, self.half_extents_of(ai), a.motion.vel * self.cfg.dt,
                                             b.desc.center, self.half_extents_of(bi), b.motion.vel * self.cfg.dt)
            }
            (ColliderKind::Circle { radius: r0 }, ColliderKind::Circle { radius: r1 }) => {
                Narrowphase::sweep_circle_circle(a.desc.center, r0, a.motion.vel * self.cfg.dt,
                                                 b.desc.center, r1, b.motion.vel * self.cfg.dt)
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Aabb { .. }) => {
                Narrowphase::sweep_circle_aabb(a.desc.center, r, a.motion.vel * self.cfg.dt,
                                               b.desc.center, self.half_extents_of(bi), b.motion.vel * self.cfg.dt)
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Circle { radius: r }) => {
                // Swap and invert normal later
                let hit = Narrowphase::sweep_circle_aabb(b.desc.center, r, b.motion.vel * self.cfg.dt,
                                                         a.desc.center, self.half_extents_of(ai), a.motion.vel * self.cfg.dt)?;
                Some(SweepHit { toi: hit.toi, normal: -hit.normal, contact: hit.contact })
            }
            (ColliderKind::Point, ColliderKind::Aabb { .. }) => {
                Narrowphase::sweep_circle_aabb(a.desc.center, 0.0, a.motion.vel * self.cfg.dt,
                                               b.desc.center, self.half_extents_of(bi), b.motion.vel * self.cfg.dt)
            }
            (ColliderKind::Aabb { .. }, ColliderKind::Point) => {
                let hit = Narrowphase::sweep_circle_aabb(b.desc.center, 0.0, b.motion.vel * self.cfg.dt,
                                                         a.desc.center, self.half_extents_of(ai), a.motion.vel * self.cfg.dt)?;
                Some(SweepHit { toi: hit.toi, normal: -hit.normal, contact: hit.contact })
            }
            (ColliderKind::Point, ColliderKind::Circle { radius: r }) => {
                Narrowphase::sweep_circle_circle(a.desc.center, 0.0, a.motion.vel * self.cfg.dt,
                                                 b.desc.center, r, b.motion.vel * self.cfg.dt)
            }
            (ColliderKind::Circle { radius: r }, ColliderKind::Point) => {
                let hit = Narrowphase::sweep_circle_circle(b.desc.center, 0.0, b.motion.vel * self.cfg.dt,
                                                           a.desc.center, r, a.motion.vel * self.cfg.dt)?;
                Some(SweepHit { toi: hit.toi, normal: -hit.normal, contact: hit.contact })
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
            if n >= 2 { candidate_pairs += n * (n - 1) / 2; }
            for i in 0..n {
                for j in (i + 1)..n {
                    let a = v[i];
                    let b = v[j];
                    let key = if a < b { (a, b) } else { (b, a) };
                    seen.insert(key);
                }
            }
        }
        WorldStats { entries, cells, candidate_pairs, unique_pairs: seen.len() }
    }

    /// Return timing breakdown for the last `end_frame`/`generate_events` runs.
    pub fn timing(&self) -> Option<WorldTiming> { self.last_timing }
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
        let a_mask = LayerMask { layer: 1, collides_with: 2, exclude: 0 };
        let b_mask = LayerMask { layer: 2, collides_with: 0, exclude: 0 };
        w.push_aabb(Vec2::new(-0.5, 0.0), Vec2::splat(0.5), Vec2::new(1.0, 0.0), a_mask, None);
        w.push_aabb(Vec2::new(0.5, 0.0), Vec2::splat(0.5), Vec2::ZERO, b_mask, None);
        w.end_frame();
        w.generate_events();
        assert_eq!(w.drain_events().len(), 0);
    }

    #[test]
    fn test_generate_sweep_event_and_drain() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let mask = LayerMask::simple(1, 1);
        let a = w.push_circle(Vec2::new(-2.0, 0.0), 0.5, Vec2::new(4.0, 0.0), mask, Some(11));
        let b = w.push_circle(Vec2::new(0.0, 0.0), 0.5, Vec2::ZERO, mask, Some(22));
        w.end_frame();
        w.generate_events();
        let evs = w.drain_events();
        assert_eq!(evs.len(), 1);
        let ev = evs[0];
        assert!(matches!(ev.kind, crate::types::EventKind::Sweep));
        assert_eq!(ev.a, a);
        assert_eq!(ev.b, b);
        assert!(ev.sweep.is_some());
        // Drained; buffer should be empty now
        assert!(w.drain_events().is_empty());
    }

    #[test]
    fn test_queries_and_pairwise() {
        let mut w = PhysicsWorld::new(cfg());
        w.begin_frame();
        let mask = LayerMask::simple(1, 1);
        let id_a = w.push_aabb(Vec2::new(0.0, 0.0), Vec2::splat(1.0), Vec2::ZERO, mask, Some(100));
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
        let id_a = w.push_aabb(Vec2::new(2.0, 0.0), Vec2::splat(0.5), Vec2::ZERO, mask, Some(1));
        let id_b = w.push_aabb(Vec2::new(4.0, 0.0), Vec2::splat(0.5), Vec2::ZERO, mask, Some(2));
        w.end_frame();
        let hit = w.raycast(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), mask, 10.0).unwrap();
        assert_eq!(hit.0, id_a);
        let hit2 = w.raycast(Vec2::new(0.0, 0.0), Vec2::new(-1.0, 0.0), mask, 10.0);
        assert!(hit2.is_none());
    }
}
