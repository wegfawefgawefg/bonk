bonk — detection-only 2D physics (ephemeral)

Overview
- Ephemeral world: rebuild every frame, no persistent state.
- Uniform grid broadphase; narrowphase for AABB, Circle, Point; rays/segments; CCD via relative velocity (TOI).
- Event emission only; you resolve responses yourself.
- Layer masks with mutual-consent filtering; optional user keys echoed back.

Install
- Add to your workspace and `use bonk::*;`.

Quick Start
```
use bonk::*;
use glam::Vec2;

let mut world = PhysicsWorld::new(WorldConfig {
    cell_size: 1.0,
    dt: 1.0/60.0,
    tighten_swept_aabb: true,
    enable_overlap_events: true,
    enable_sweep_events: true,
    max_events: 1024,
    enable_timing: false,
});

world.begin_frame();
let mask_dyn = LayerMask::simple(1, 2);
let mask_sta = LayerMask::simple(2, 1);
world.push_circle(Vec2::new(-3.0, 0.0), 0.5, Vec2::new(5.0, 0.0), mask_dyn, Some(1));
world.push_aabb(Vec2::new(0.0, 0.0), Vec2::splat(1.0), Vec2::ZERO, mask_sta, Some(2));
world.end_frame();
world.generate_events();
for ev in world.drain_events() { /* handle */ }
```

Basic Usage Pattern
- Frame loop order:
- 1) `begin_frame()`
- 2) `push_*` colliders for this frame
- 3) `end_frame()` builds the grid
- 4) `generate_events()` produces overlap/sweep events
- 5) `drain_events()` to consume events

More Examples
- Overlaps only: set `enable_sweep_events=false` (still supports queries).
- CCD only: set `enable_overlap_events=false` for pure TOI events.
- Points: use `push_point(...)` (CCD treats them as radius=0 circles).
- Keys: pass `Some(key)` to mirror app entity IDs in events/queries.

Queries
- `raycast(origin, dir, mask, max_t) -> Option<(FrameId, SweepHit, Option<ColKey>)>`
- `query_point(p, mask) -> Vec<(FrameId, Option<ColKey>)>`
- `query_aabb(center, half_extents, mask)` / `query_circle(center, radius, mask)`

Pairwise
- `overlap_pair/sweep_pair` by `FrameId` and by `ColKey`.

Design Notes
- Overlap normals for AABB/AABB and Circle/Circle point from B into A.
- Circle↔AABB overlap returns a representative result with zero normal/depth.
- Points are treated as zero-radius circles for CCD.
- Grid binning can include multiple cells when bounds straddle cell edges.

Run Examples
- `cargo run --example breakout_events`
- `cargo run --example raycast_demo`
- `cargo run --example bench_narrowphase`
- `cargo run --example perf_world`
- `cargo run --example perf_sweep` (CSV)

Performance
- Use reasonable `cell_size` (e.g., 32–64 in tile units) for broadphase efficiency.
- `tighten_swept_aabb` reduces false positives when velocities are large relative to cells.
- CCD reduces to ray-vs-expanded shapes for speed.

Limitations
- No contact resolution; detection-only.
- No persistent contacts or manifold caching.
- Not a general-purpose physics engine; focused on arcade-style needs.

Non-Panicking API
- Library functions never unwrap/expect/panic. Query and pairwise paths return `Option` and simply return `None` on miss.
- Event payloads are optional: check `Event.kind` and then the corresponding `Option` payload.
- Duplicate `user_key` insertions are guarded by `debug_assert!` in debug builds only; release builds accept last-write-wins.

Performance Tuning
- Cell size: choose `cell_size` close to average collider diameter to balance grid sparsity vs. per-cell candidate counts.
- Tightened swept AABBs: set `tighten_swept_aabb=true` when velocities are large relative to cells to reduce false positives.
- Event limits: cap `max_events` to avoid worst-case bursts.
- Masks: use `layer/collides_with/exclude` to prune early.

Perf Utilities
- `cargo run --example perf_world` prints a quick single-config timing.
- `cargo run --example perf_sweep` sweeps `n`, `cell_size`, and `tighten_swept_aabb` and prints CSV with:
  `n,cell_size,tighten,end_frame_ms,generate_ms,cells,candidate_pairs,unique_pairs,events`.
  You can redirect to a file and graph the results.

Timing & Perf Counters
- Enable instrumentation via `WorldConfig { enable_timing: true, .. }`.
- After `end_frame()` + `generate_events()`, call `world.timing()` to fetch `WorldTiming`:
  - `end_frame_ms`, `end_frame_aabbs_ms`, `end_frame_grid_ms`
  - `generate_ms`, `generate_scan_ms`, `generate_narrowphase_ms`, and `events_emitted`
- `world.debug_stats()` returns `WorldStats` (entries, cells, candidate_pairs, unique_pairs).

Running Tests & Perf
- Correctness tests: `cargo test`
- Quick perf sample: `cargo run --example perf_world`
- CSV sweep: `cargo run --example perf_sweep > sweep.csv`
