use glam::Vec2;
use nobonk::*;

fn main() {
    let mut world = PhysicsWorld::new(WorldConfig {
        cell_size: 1.0,
        dt: 1.0 / 60.0,
        tighten_swept_aabb: true,
        enable_overlap_events: true,
        enable_sweep_events: true,
        max_events: 1024,
        enable_timing: true,
        tile_eps: 1e-4,
        require_mutual_consent: true,
    });

    world.begin_frame();
    let mask_dyn = LayerMask::simple(1, 2);
    let mask_sta = LayerMask::simple(2, 1);

    let ball = world.push_circle(
        Vec2::new(-3.0, 0.0),
        0.5,
        Vec2::new(5.0, 0.0),
        mask_dyn,
        Some(1),
    );
    let wall = world.push_aabb(
        Vec2::new(0.0, 0.0),
        Vec2::splat(1.0),
        Vec2::ZERO,
        mask_sta,
        Some(2),
    );

    println!("Inserted ball={:?} wall={:?}", ball, wall);

    world.end_frame();
    world.generate_events();
    if let Some(t) = world.timing() {
        println!(
            "timing: end_frame={:.3}ms (aabbs={:.3}ms grid={:.3}ms) generate={:.3}ms scan={:.3}ms narrow={:.3}ms",
            t.end_frame_ms,
            t.end_frame_aabbs_ms,
            t.end_frame_grid_ms,
            t.generate_ms,
            t.generate_scan_ms,
            t.generate_narrowphase_ms
        );
    }
    for ev in world.drain_events() {
        match ev.kind {
            EventKind::Sweep => {
                if let Some(s) = ev.sweep {
                    println!(
                        "Sweep: {:?} vs {:?} toi={:.3} n=({:.2},{:.2})",
                        ev.a, ev.b, s.toi, s.normal.x, s.normal.y
                    );
                }
            }
            EventKind::Overlap => {
                if let Some(o) = ev.overlap {
                    println!(
                        "Overlap: {:?} vs {:?} depth={:.3} n=({:.2},{:.2})",
                        ev.a, ev.b, o.depth, o.normal.x, o.normal.y
                    );
                }
            }
        }
    }
}
