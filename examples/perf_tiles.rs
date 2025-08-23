use glam::Vec2;
use nobonk::*;
use std::time::Instant;

fn main() {
    let mut world = PhysicsWorld::new(WorldConfig {
        cell_size: 1.0,
        dt: 1.0 / 60.0,
        tighten_swept_aabb: true,
        enable_overlap_events: false,
        enable_sweep_events: false,
        max_events: 0,
        enable_timing: false,
        tile_eps: 1e-4,
        require_mutual_consent: true,
    });

    // Build a 256x256 map with ~25% solids in a checkerboard-ish pattern
    let w = 256u32; let h = 256u32;
    let mut solids = vec![0u8; (w*h) as usize];
    for y in 0..h { for x in 0..w { if (x ^ y) & 0x3 == 0 { solids[(y*w+x) as usize] = 1; } }}
    world.attach_tilemap(TileMapDesc { origin: Vec2::new(0.0,0.0), cell: 1.0, width: w, height: h, solids: &solids, mask: LayerMask::simple(2,1), user_key: None });

    // Ray throughput
    let origin = Vec2::new(-10.0, 100.5);
    let dir = Vec2::new(1.0, 0.3).normalize();
    let mask = LayerMask::simple(1,2);
    let n_rays = 200_000;
    let t0 = Instant::now();
    let mut acc = 0.0f32;
    for i in 0..n_rays { let max_t = 1000.0 + (i % 10) as f32; if let Some((_tref, hit, _)) = world.raycast_tiles(origin, dir, max_t, mask) { acc += hit.toi; } }
    let dt = t0.elapsed().as_secs_f64();
    println!("tile_raycast: rays={} secs={:.3} throughput={:.0} rays/s checksum={:.3}", n_rays, dt, (n_rays as f64 / dt), acc);

    // Sweep throughput
    let center0 = Vec2::new(20.0, 20.0);
    let he = Vec2::splat(0.4);
    let n_sweeps = 100_000;
    let t1 = Instant::now();
    let mut acc2 = 0.0f32;
    for i in 0..n_sweeps {
        let a = (i as f32 * 0.01).sin();
        let vel = Vec2::new(6.0 * a.abs() + 1.0, 2.0 * a);
        if let Some((_tref, hit, _)) = world.sweep_aabb_tiles(center0, he, vel, mask) { acc2 += hit.toi; }
    }
    let dt2 = t1.elapsed().as_secs_f64();
    println!("tile_sweep: sweeps={} secs={:.3} throughput={:.0} sweeps/s checksum={:.3}", n_sweeps, dt2, (n_sweeps as f64 / dt2), acc2);
}

