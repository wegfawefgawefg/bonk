use bonk::*;
use glam::Vec2;
use std::time::Instant;

fn main() {
    let n = 1_000_000u32;
    let mut acc = 0.0f32;
    let start = Instant::now();
    for i in 0..n {
        let t = (i as f32) * 0.001;
        let c0 = Vec2::new(-3.0 + t.sin(), 0.0);
        let c1 = Vec2::new(0.0, 0.0);
        if let Some(hit) = bonk::narrowphase::Narrowphase::ray_circle(c0, Vec2::new(5.0, 0.0), c1, 1.0) {
            acc += hit.toi;
        }
    }
    let dt = start.elapsed();
    println!("ray_circle {} iters in {:?} acc={}", n, dt, acc);
}

