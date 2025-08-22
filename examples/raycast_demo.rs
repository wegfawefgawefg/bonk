use nobonk::*;
use glam::Vec2;

fn main() {
    let mut world = PhysicsWorld::new(WorldConfig {
        cell_size: 1.0,
        dt: 1.0 / 60.0,
        tighten_swept_aabb: true,
        enable_overlap_events: false,
        enable_sweep_events: false,
        max_events: 0,
        enable_timing: false,
    });

    world.begin_frame();
    let mask = LayerMask::simple(1, 1);
    world.push_aabb(Vec2::new(2.0, 0.0), Vec2::splat(0.5), Vec2::ZERO, mask, Some(10));
    world.push_circle(Vec2::new(4.0, 0.0), 0.5, Vec2::ZERO, mask, Some(20));
    world.end_frame();

    if let Some((id, hit, key)) = world.raycast(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), mask, 100.0) {
        println!("Ray hit id={:?} key={:?} t={:.3} n=({:.2},{:.2})", id, key, hit.toi, hit.normal.x, hit.normal.y);
    } else {
        println!("No hit");
    }
}
