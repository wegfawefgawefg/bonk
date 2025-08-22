use glam::Vec2;
use nobonk::*;
use std::time::Instant;

fn lcg(seed: &mut u32) -> u32 {
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    *seed
}

fn main() {
    let mut world = PhysicsWorld::new(WorldConfig {
        cell_size: 2.0,
        dt: 1.0 / 60.0,
        tighten_swept_aabb: true,
        enable_overlap_events: false,
        enable_sweep_events: true,
        max_events: 1_000_000,
        enable_timing: true,
    });

    let n = 20_000usize; // number of colliders
    let mut seed = 1u32;
    let mask = LayerMask::simple(1, 1);
    world.begin_frame();
    for i in 0..n {
        let rx = (lcg(&mut seed) as f32 / u32::MAX as f32) * 200.0 - 100.0;
        let ry = (lcg(&mut seed) as f32 / u32::MAX as f32) * 200.0 - 100.0;
        let vx = (lcg(&mut seed) as f32 / u32::MAX as f32) * 4.0 - 2.0;
        let vy = (lcg(&mut seed) as f32 / u32::MAX as f32) * 4.0 - 2.0;
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
    let t0 = Instant::now();
    world.end_frame();
    let t_end = t0.elapsed();
    let t1 = Instant::now();
    world.generate_events();
    let t_gen = t1.elapsed();
    let n_events = world.drain_events().len();
    if let Some(t) = world.timing() {
        println!(
            "N={} cell_size={} tighten={} end={:.3}ms gen={:.3}ms (scan={:.3}ms narrow={:.3}ms) events={}",
            n,
            world.cfg.cell_size,
            world.cfg.tighten_swept_aabb,
            t.end_frame_ms,
            t.generate_ms,
            t.generate_scan_ms,
            t.generate_narrowphase_ms,
            n_events
        );
    } else {
        println!(
            "N={} cell_size={} tighten={} end_frame={:?} generate={:?} events={}",
            n, world.cfg.cell_size, world.cfg.tighten_swept_aabb, t_end, t_gen, n_events
        );
    }
}
