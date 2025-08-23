# NoBonk

<p align="center">
  <img src="assets/no_bonk_logo.png" alt="nobonk logo" width="800" />
</p>

## What

Nobonk is a 2D physics engine that doesn't have physics. It does not resolve collisions. It just gives you events that collisions happened.
You do with that information what you will.

## Why

Box2D and Rapier are big. 90% of the time I don't need or want to spend hours tweaking restitution, adding springs, or deal with some huge constructor. I just want to know if two things are touching, and maybe do some raycasts. This is a simple library for a simple mind.

## Overview

- No persistent state. (No looking up objects to move them. Readd your objects every frame.)
- Event emission only. (resolve events yourself.)
- Layer masks.
- Optional user ids.
- AABB, Circle, Point colliders.
- Raycast, Point, AABB, and Circle queries.
- Events come with Time of Impact (TOI) (via CCD of swept AABBs)
- No penetration.
- Tiles: dense 0/1 tilemaps with fast DDA ray/sweep and unified queries.

## Install

- Add to your workspace and `use nobonk::*;`.

## Quick Start

```rust

use nobonk::*;
use glam::Vec2;

let mut world = PhysicsWorld::new(WorldConfig {
    cell_size: 1.0,
    dt: 1.0/60.0,
    tighten_swept_aabb: true,
    enable_overlap_events: true,
    enable_sweep_events: true,
    max_events: 1024,
    enable_timing: false, // this is for performance. leave it off unless ur trying to figure out the cell size or something.
    tile_eps: 1e-4,
    require_mutual_consent: true,
});

// Define layer masks. Assume this is like breakout/arkanoid or something.
let ball_mask = LayerMask::simple(1, 2);
let blocks_mask = LayerMask::simple(2, 1);


// your game loop
loop {
    world.begin_frame();
    world.push_circle(
        Vec2::new(-3.0, 0.0), // center
        0.5, // radius
        Vec2::new(5.0, 0.0), // velocity
        ball_mask, // collision mask
        Some(1) // user key for the ball
    );
    world.push_aabb(
        Vec2::new(0.0, 0.0), // center
        Vec2::splat(1.0),  // half extents
        Vec2::ZERO, // velocity
        blocks_mask, // collision mask
        Some(2) // user key for the block (use it for looking up the block or back referencing some shit in a hashmap somewhere)
    );
    world.end_frame();
    world.generate_events();
    for ev in world.drain_events() { 
      /* match on the events or something */ 
    }
}

```

<br>

## Basic Usage Pattern

- `begin_frame()`
- `push_*` colliders for this frame
- `end_frame()` builds the grid
- `generate_events()` produces overlap/sweep events
- `drain_events()` to consume events

## Extras

- Overlaps only: set `enable_sweep_events=false` (still supports queries).
- CCD only: set `enable_overlap_events=false` for pure TOI events.
- Points: use `push_point(...)` (CCD treats them as radius=0 circles).
- Keys: pass `Some(key)` to mirror app entity IDs in events/queries.

## Queries

### Shape Based

- `raycast(origin, dir, mask, max_t) -> Option<(FrameId, SweepHit, Option<ColKey>)>`
- `query_point(p, mask) -> Vec<(FrameId, Option<ColKey>)>`
- `query_aabb(center, half_extents, mask)`
- `query_circle(center, radius, mask)`

### Tile + Unified (v0.2)

- `attach_tilemap(TileMapDesc) -> TileMapRef`
- `update_tiles(map, (x,y,w,h), data)` and `detach_tilemap(map)`
- `raycast_all(origin, dir, mask, max_t) -> Option<(BodyRef, SweepHit, Option<ColKey>)>`
- `query_point_all(...) -> Vec<(BodyRef, Option<ColKey>)>`
- `query_aabb_all(...) -> Vec<(BodyRef, Option<ColKey>)>`
- `query_circle_all(...) -> Vec<(BodyRef, Option<ColKey>)>`
- Tile-only helpers: `raycast_tiles`, `sweep_aabb_tiles`, `sweep_circle_tiles`

`BodyRef` identifies either a collider (`BodyRef::Collider(FrameId)`) or a specific tile cell (`BodyRef::Tile(TileRef)`). `SweepHit` and `Overlap` now include a `hint: ResolutionHint` with:
- `safe_pos`: suggested non-penetrating center position
- `start_embedded`: true if the shape started inside a hit
- `fully_embedded`: true if no push-out was possible

## Tile Usage (v0.2)

```rust
use nobonk::*;
use glam::Vec2;

let mut world = PhysicsWorld::new(WorldConfig {
    cell_size: 1.0,
    dt: 1.0/60.0,
    tighten_swept_aabb: true,
    enable_overlap_events: true,
    enable_sweep_events: true,
    max_events: 10_000,
    enable_timing: false,
    tile_eps: 1e-4,
    require_mutual_consent: true,
});

let bits = vec![0,1,0]; // 3x1 map with middle solid
let _map = world.attach_tilemap(TileMapDesc {
    origin: Vec2::new(0.0, 0.0),
    cell: 1.0,
    width: 3,
    height: 1,
    solids: &bits,
    mask: LayerMask::simple(2, 1),
    user_key: Some(0xT1LES),
});

let mask = LayerMask::simple(1, 2);
if let Some((_who, hit, _)) = world.raycast_all(Vec2::new(-0.5, 0.5), Vec2::new(1.0, 0.0), mask, 100.0) {
    if let Some(p) = hit.hint.safe_pos { /* candidate stop position */ }
}

if let Some((_tile, hit, _)) = world.sweep_aabb_tiles(Vec2::new(0.2,0.5), Vec2::splat(0.3), Vec2::new(2.0,0.0), mask) {
    // reflect or clamp using hit.normal and hit.hint.safe_pos
}
```

### Overlap

- `overlap_pair(a: FrameId, b: FrameId) -> Option<Overlap>`
- `sweep_pair(a: FrameId, b: FrameId) -> Option<SweepHit>`
- `overlap_by_key(a: ColKey, b: ColKey) -> Option<Overlap>`
- `sweep_by_key(a: ColKey, b: ColKey) -> Option<SweepHit>`

<br>

## Design Notes

- Overlap normals for AABB/AABB and Circle/Circle point from B into A.
- Circle↔AABB overlap returns a representative result with zero normal/depth.
- Points are treated as zero-radius circles for CCD.
- Grid binning can include multiple cells when bounds straddle cell edges.
- Tiles use top-left `origin` and square `cell` size. Consent masking applies between collider masks and tilemap mask, honoring `WorldConfig { require_mutual_consent }`.
- `WorldConfig { tile_eps }` controls the backoff used to compute `hint.safe_pos` in tile sweeps.

### Signed Depth (Tiles)

- Tile pushouts return signed depth:
  - depth >= 0: overlapping or tangent (0 = just touching)
  - depth < 0: separated; |depth| is the gap distance (axis metric for AABB, radial for circle)
- Tip (skin): treat as contact if `depth >= -skin` (e.g., `skin = 0.02`).

## Run Examples

- `cargo run --example breakout_events`
- `cargo run --example raycast_demo`
- `cargo run --example bench_narrowphase`
- `cargo run --example perf_world`
- `cargo run --example perf_sweep` (CSV)
- `cargo run --example perf_tiles` (tile ray/sweep throughput)

<br>

## Quirks

- Use reasonable `cell_size`  for broadphase efficiency.
- CCD reduces to ray-vs-expanded shapes for speed. (bla bla Minkowski trick)
- Library functions never unwrap/expect/panic. Query and pairwise paths return `Option` and simply return `None` on miss.
- Event payloads are optional: check `Event.kind` and then the corresponding `Option` payload.
- Duplicate `user_key` insertions are guarded by `debug_assert!` in debug builds only; release builds accept last-write-wins.

## Performance Tuning

- Cell size: choose `cell_size` close to average collider diameter to balance grid sparsity vs. per-cell candidate counts. (e.g., 32–64 in tile units)
- Tightened swept AABBs: set `tighten_swept_aabb=true` when velocities are large relative to cells to reduce false positives.
- Event limits: cap `max_events` to avoid worst-case bursts.
- Masks: use `layer/collides_with/exclude` to prune early.

## Perf Utilities

- `cargo run --example perf_world` prints a quick single-config timing.
- `cargo run --example perf_sweep` sweeps `n`, `cell_size`, and `tighten_swept_aabb` and prints CSV with:
  `n,cell_size,tighten,end_frame_ms,generate_ms,cells,candidate_pairs,unique_pairs,events`.
  You can redirect to a file and graph the results.

## Timing & Perf Counters

- Enable instrumentation via `WorldConfig { enable_timing: true, .. }`.
- After `end_frame()` + `generate_events()`, call `world.timing()` to fetch `WorldTiming`:
  - `end_frame_ms`, `end_frame_aabbs_ms`, `end_frame_grid_ms`
  - `generate_ms`, `generate_scan_ms`, `generate_narrowphase_ms`, and `events_emitted`
- `world.debug_stats()` returns `WorldStats` (entries, cells, candidate_pairs, unique_pairs).

## Running Tests & Perf

- Correctness tests: `cargo test`
- Quick perf sample: `cargo run --example perf_world`
- CSV sweep: `cargo run --example perf_sweep > sweep.csv`

## WTF

```bash
# 1) Install nightly (minimal components are enough)
rustup toolchain install nightly --profile minimal

# 2) Run from your crate/workspace root
cargo public-api --simplified > public-api.txt

```
