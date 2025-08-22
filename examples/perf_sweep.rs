use glam::Vec2;
use nobonk::*;

fn lcg(seed: &mut u32) -> u32 {
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    *seed
}

fn build_world(n: usize, cs: f32, tighten: bool, seed0: u32) -> (PhysicsWorld, usize) {
    let mut world = PhysicsWorld::new(WorldConfig {
        cell_size: cs,
        dt: 1.0 / 60.0,
        tighten_swept_aabb: tighten,
        enable_overlap_events: false,
        enable_sweep_events: true,
        max_events: usize::MAX / 4,
        enable_timing: true,
    });
    let mut seed = seed0;
    let mask = LayerMask::simple(1, 1);
    world.begin_frame();
    for i in 0..n {
        let rx = (lcg(&mut seed) as f32 / u32::MAX as f32) * 200.0 - 100.0;
        let ry = (lcg(&mut seed) as f32 / u32::MAX as f32) * 200.0 - 100.0;
        let vx = (lcg(&mut seed) as f32 / u32::MAX as f32) * 6.0 - 3.0;
        let vy = (lcg(&mut seed) as f32 / u32::MAX as f32) * 6.0 - 3.0;
        if i % 2 == 0 {
            world.push_aabb(
                Vec2::new(rx, ry),
                Vec2::splat(0.5),
                Vec2::new(vx, vy),
                mask,
                None,
            );
        } else {
            world.push_circle(Vec2::new(rx, ry), 0.5, Vec2::new(vx, vy), mask, None);
        }
    }
    (world, seed as usize)
}

fn main() {
    let sizes = [0.5f32, 1.0, 2.0, 4.0, 8.0];
    let tighten_opts = [false, true];
    let n_vals = [5_000usize, 10_000, 20_000];
    println!(
        "n,cell_size,tighten,end_frame_ms,generate_ms,cells,candidate_pairs,unique_pairs,events"
    );
    for &n in &n_vals {
        for &cs in &sizes {
            for &tighten in &tighten_opts {
                let (mut world, _) = build_world(n, cs, tighten, 1);
                world.end_frame();
                world.generate_events();
                let t = world.timing().unwrap_or_default();
                let stats = world.debug_stats();
                let events = world.drain_events().len();
                println!(
                    "{},{},{},{:.3},{:.3},{},{},{},{}",
                    n,
                    cs,
                    tighten as u8,
                    t.end_frame_ms,
                    t.generate_ms,
                    stats.cells,
                    stats.candidate_pairs,
                    stats.unique_pairs,
                    events
                );
            }
        }
    }
}
