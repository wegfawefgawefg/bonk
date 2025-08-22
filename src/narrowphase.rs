use glam::Vec2;

use crate::api::NarrowphaseApi;
use crate::types::*;

/// Narrowphase primitive tests (skeleton; to be implemented).
pub struct Narrowphase;

impl NarrowphaseApi for Narrowphase {
    fn ray_aabb(origin: Vec2, dir: Vec2, aabb_min: Vec2, aabb_max: Vec2) -> Option<SweepHit> {
        // Slab method with normal tracking; returns earliest t >= 0
        let mut tmin = f32::NEG_INFINITY;
        let mut tmax = f32::INFINITY;
        let mut n_enter = Vec2::ZERO;

        // X axis
        if dir.x.abs() < f32::EPSILON {
            if origin.x < aabb_min.x || origin.x > aabb_max.x {
                return None;
            }
        } else {
            let inv = 1.0 / dir.x;
            let mut t1 = (aabb_min.x - origin.x) * inv;
            let mut t2 = (aabb_max.x - origin.x) * inv;
            let mut nx = -1.0;
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
                nx = 1.0;
            }
            if t1 > tmin {
                tmin = t1;
                n_enter = Vec2::new(nx, 0.0);
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }

        // Y axis
        if dir.y.abs() < f32::EPSILON {
            if origin.y < aabb_min.y || origin.y > aabb_max.y {
                return None;
            }
        } else {
            let inv = 1.0 / dir.y;
            let mut t1 = (aabb_min.y - origin.y) * inv;
            let mut t2 = (aabb_max.y - origin.y) * inv;
            let mut ny = -1.0;
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
                ny = 1.0;
            }
            if t1 > tmin {
                tmin = t1;
                n_enter = Vec2::new(0.0, ny);
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }

        // If origin inside, tmin < 0; treat as immediate hit
        let toi = if tmin < 0.0 { 0.0 } else { tmin };
        let contact = origin + dir * toi;
        let normal = if tmin < 0.0 { Vec2::ZERO } else { n_enter };
        Some(SweepHit {
            toi,
            normal,
            contact,
        })
    }

    fn ray_circle(origin: Vec2, dir: Vec2, center: Vec2, r: f32) -> Option<SweepHit> {
        // Solve ||origin + t*dir - center||^2 = r^2 for t >= 0
        let m = origin - center;
        let a = dir.length_squared();
        if a == 0.0 {
            return None;
        }
        let b = 2.0 * m.dot(dir);
        let c = m.length_squared() - r * r;
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sqrt_disc = disc.sqrt();
        let t0 = (-b - sqrt_disc) / (2.0 * a);
        let t1 = (-b + sqrt_disc) / (2.0 * a);
        let t = if t0 >= 0.0 { t0 } else { t1 };
        if t < 0.0 {
            return None;
        }
        let contact = origin + dir * t;
        let n = contact - center;
        let len = n.length();
        let normal = if len > 0.0 { n / len } else { Vec2::ZERO };
        Some(SweepHit {
            toi: t,
            normal,
            contact,
        })
    }

    fn line_segment_aabb(a: Vec2, b: Vec2, aabb_min: Vec2, aabb_max: Vec2) -> Option<SweepHit> {
        let d = b - a;
        // Reuse slab, but clamp to segment [0,1]
        let mut tmin = 0.0;
        let mut tmax = 1.0;
        let mut n_enter = Vec2::ZERO;

        // X axis
        if d.x.abs() < f32::EPSILON {
            if a.x < aabb_min.x || a.x > aabb_max.x {
                return None;
            }
        } else {
            let inv = 1.0 / d.x;
            let mut t1 = (aabb_min.x - a.x) * inv;
            let mut t2 = (aabb_max.x - a.x) * inv;
            let mut nx = -1.0;
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
                nx = 1.0;
            }
            if t1 > tmin {
                tmin = t1;
                n_enter = Vec2::new(nx, 0.0);
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }

        // Y axis
        if d.y.abs() < f32::EPSILON {
            if a.y < aabb_min.y || a.y > aabb_max.y {
                return None;
            }
        } else {
            let inv = 1.0 / d.y;
            let mut t1 = (aabb_min.y - a.y) * inv;
            let mut t2 = (aabb_max.y - a.y) * inv;
            let mut ny = -1.0;
            if t1 > t2 {
                core::mem::swap(&mut t1, &mut t2);
                ny = 1.0;
            }
            if t1 > tmin {
                tmin = t1;
                n_enter = Vec2::new(0.0, ny);
            }
            if t2 < tmax {
                tmax = t2;
            }
            if tmin > tmax {
                return None;
            }
        }

        if tmin < 0.0 || tmin > 1.0 {
            return None;
        }
        let toi = tmin.max(0.0).min(1.0);
        let contact = a + d * toi;
        let normal = if toi == 0.0
            && (a.x >= aabb_min.x && a.x <= aabb_max.x && a.y >= aabb_min.y && a.y <= aabb_max.y)
        {
            Vec2::ZERO
        } else {
            n_enter
        };
        Some(SweepHit {
            toi,
            normal,
            contact,
        })
    }

    fn line_segment_circle(a: Vec2, b: Vec2, center: Vec2, r: f32) -> Option<SweepHit> {
        // Solve |a + t d - c|^2 = r^2, t in [0,1]
        let d = b - a;
        let m = a - center;
        let acoef = d.length_squared();
        if acoef == 0.0 {
            return None;
        }
        let bcoef = 2.0 * m.dot(d);
        let ccoef = m.length_squared() - r * r;
        let disc = bcoef * bcoef - 4.0 * acoef * ccoef;
        if disc < 0.0 {
            return None;
        }
        let sqrt_disc = disc.sqrt();
        let t0 = (-bcoef - sqrt_disc) / (2.0 * acoef);
        let t1 = (-bcoef + sqrt_disc) / (2.0 * acoef);
        let mut t = f32::INFINITY;
        for cand in [t0, t1] {
            if cand >= 0.0 && cand <= 1.0 {
                t = t.min(cand);
            }
        }
        if !t.is_finite() {
            return None;
        }
        let contact = a + d * t;
        let n = contact - center;
        let len = n.length();
        let normal = if len > 0.0 { n / len } else { Vec2::ZERO };
        Some(SweepHit {
            toi: t,
            normal,
            contact,
        })
    }

    fn overlap_aabb_aabb(c0: Vec2, h0: Vec2, c1: Vec2, h1: Vec2) -> Option<Overlap> {
        // Compute overlap extents along axes
        let d = c1 - c0;
        let ox = (h0.x + h1.x) - d.x.abs();
        let oy = (h0.y + h1.y) - d.y.abs();
        if ox < 0.0 || oy < 0.0 {
            return None;
        }

        // Choose axis of minimum penetration
        let (depth, mut normal, axis_h) = if ox <= oy {
            let nx = if d.x >= 0.0 { -1.0 } else { 1.0 }; // from B into A
            (ox.max(0.0), Vec2::new(nx, 0.0), h0.x)
        } else {
            let ny = if d.y >= 0.0 { -1.0 } else { 1.0 };
            (oy.max(0.0), Vec2::new(0.0, ny), h0.y)
        };

        if depth == 0.0 {
            // Degenerate: edge-touch; keep normal axis-aligned as above
        } else if normal.length_squared() == 0.0 {
            // Shouldn't happen, but guard against NaN
            normal = Vec2::ZERO;
        }

        // Contact point: project A's center onto B's box then move to A's surface along normal
        let bmin = c1 - h1;
        let bmax = c1 + h1;
        let clamp = |v: f32, lo: f32, hi: f32| v.max(lo).min(hi);
        let mut contact = Vec2::new(clamp(c0.x, bmin.x, bmax.x), clamp(c0.y, bmin.y, bmax.y));
        // Move to A's surface along the chosen axis
        contact -= normal * axis_h;

        Some(Overlap {
            normal,
            depth,
            contact,
        })
    }

    fn overlap_circle_circle(c0: Vec2, r0: f32, c1: Vec2, r1: f32) -> Option<Overlap> {
        let delta = c0 - c1;
        let dist2 = delta.length_squared();
        let rsum = r0 + r1;
        let rsum2 = rsum * rsum;
        if dist2 > rsum2 {
            return None;
        }
        if dist2 == 0.0 {
            // Coincident centers; undefined normal.
            let normal = Vec2::ZERO;
            let depth = rsum; // maximal, but depth is >= 0; choosing r0+r1
            let contact = c0; // arbitrary representative
            return Some(Overlap {
                normal,
                depth,
                contact,
            });
        }
        let dist = dist2.sqrt();
        let normal = delta / dist; // from B into A
        let depth = (rsum - dist).max(0.0);
        let contact = c0 - normal * r0;
        Some(Overlap {
            normal,
            depth,
            contact,
        })
    }

    fn overlap_point_aabb(p: Vec2, c: Vec2, h: Vec2) -> bool {
        let min = c - h;
        let max = c + h;
        p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y
    }

    fn overlap_point_circle(p: Vec2, c: Vec2, r: f32) -> bool {
        let d = p - c;
        d.length_squared() <= r * r
    }

    fn sweep_aabb_aabb(
        c0: Vec2,
        h0: Vec2,
        v0: Vec2,
        c1: Vec2,
        h1: Vec2,
        v1: Vec2,
    ) -> Option<SweepHit> {
        let vrel = v0 - v1;
        if vrel.length_squared() <= f32::EPSILON {
            return None;
        }
        let expand = h0 + h1;
        let min = c1 - expand;
        let max = c1 + expand;
        let hit = Self::ray_aabb(c0, vrel, min, max)?;
        if hit.toi < 0.0 || hit.toi > 1.0 {
            return None;
        }
        let center_at_hit = c0 + vrel * hit.toi;
        let normal = hit.normal;
        let contact = center_at_hit - normal * h0;
        Some(SweepHit {
            toi: hit.toi,
            normal,
            contact,
        })
    }

    fn sweep_circle_aabb(
        c: Vec2,
        r: f32,
        v: Vec2,
        box_c: Vec2,
        box_h: Vec2,
        box_v: Vec2,
    ) -> Option<SweepHit> {
        let vrel = v - box_v;
        if vrel.length_squared() <= f32::EPSILON {
            return None;
        }
        let rvec = Vec2::splat(r);
        let min = box_c - box_h - rvec;
        let max = box_c + box_h + rvec;
        let hit = Self::ray_aabb(c, vrel, min, max)?;
        if hit.toi < 0.0 || hit.toi > 1.0 {
            return None;
        }
        let center_at_hit = c + vrel * hit.toi;
        let normal = hit.normal;
        let contact = center_at_hit - normal * r;
        Some(SweepHit {
            toi: hit.toi,
            normal,
            contact,
        })
    }

    fn sweep_circle_circle(
        c0: Vec2,
        r0: f32,
        v0: Vec2,
        c1: Vec2,
        r1: f32,
        v1: Vec2,
    ) -> Option<SweepHit> {
        let vrel = v0 - v1;
        if vrel.length_squared() <= f32::EPSILON {
            return None;
        }
        let rsum = r0 + r1;
        let hit = Self::ray_circle(c0, vrel, c1, rsum)?;
        if hit.toi < 0.0 || hit.toi > 1.0 {
            return None;
        }
        let center_at_hit = c0 + vrel * hit.toi;
        let normal = hit.normal; // outward from expanded circle => from B to A
        let contact = center_at_hit - normal * r0;
        Some(SweepHit {
            toi: hit.toi,
            normal,
            contact,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_overlap_aabb_aabb_basic() {
        let c0 = Vec2::new(0.0, 0.0);
        let h0 = Vec2::new(1.0, 1.0);
        let c1 = Vec2::new(1.5, 0.0);
        let h1 = Vec2::new(1.0, 1.0);
        let o = Narrowphase::overlap_aabb_aabb(c0, h0, c1, h1).unwrap();
        assert!(o.depth >= 0.0);
        // Expect overlap along X predominantly
        assert!(o.normal.x.abs() > 0.0 || o.normal.y.abs() > 0.0);
    }

    #[test]
    fn test_overlap_aabb_aabb_separated() {
        let c0 = Vec2::new(0.0, 0.0);
        let h0 = Vec2::new(1.0, 1.0);
        let c1 = Vec2::new(3.1, 0.0);
        let h1 = Vec2::new(1.0, 1.0);
        assert!(Narrowphase::overlap_aabb_aabb(c0, h0, c1, h1).is_none());
    }

    #[test]
    fn test_overlap_circle_circle_basic() {
        let c0 = Vec2::new(0.0, 0.0);
        let c1 = Vec2::new(1.0, 0.0);
        let r0 = 1.0;
        let r1 = 1.0;
        let o = Narrowphase::overlap_circle_circle(c0, r0, c1, r1).unwrap();
        assert!((o.depth - 1.0).abs() < 1e-5);
        // Normal points from B (c1) into A (c0): (-1, 0)
        assert!((o.normal.x + 1.0).abs() < 1e-5);
        assert!((o.normal.y).abs() < 1e-5);
    }

    #[test]
    fn test_overlap_circle_circle_tangent() {
        let c0 = Vec2::new(0.0, 0.0);
        let c1 = Vec2::new(2.0, 0.0);
        let r0 = 1.0;
        let r1 = 1.0;
        let o = Narrowphase::overlap_circle_circle(c0, r0, c1, r1).unwrap();
        assert!((o.depth).abs() < 1e-5);
    }

    #[test]
    fn test_overlap_point_aabb() {
        let c = Vec2::new(0.0, 0.0);
        let h = Vec2::new(1.0, 2.0);
        assert!(Narrowphase::overlap_point_aabb(Vec2::new(0.0, 0.0), c, h));
        assert!(Narrowphase::overlap_point_aabb(Vec2::new(1.0, 2.0), c, h));
        assert!(!Narrowphase::overlap_point_aabb(Vec2::new(1.1, 0.0), c, h));
    }

    #[test]
    fn test_overlap_point_circle() {
        let c = Vec2::new(1.0, -1.0);
        let r = 2.0;
        assert!(Narrowphase::overlap_point_circle(
            Vec2::new(1.0, -1.0),
            c,
            r
        ));
        assert!(Narrowphase::overlap_point_circle(
            Vec2::new(3.0, -1.0),
            c,
            r
        ));
        assert!(!Narrowphase::overlap_point_circle(
            Vec2::new(3.1, -1.0),
            c,
            r
        ));
    }

    // --- Rays / segments ---------------------------------------------------

    #[test]
    fn test_ray_aabb_hit() {
        let o = Vec2::new(-5.0, 0.0);
        let d = Vec2::new(1.0, 0.0);
        let min = Vec2::new(-1.0, -1.0);
        let max = Vec2::new(1.0, 1.0);
        let hit = Narrowphase::ray_aabb(o, d, min, max).unwrap();
        assert!(hit.toi > 0.0);
        assert!((hit.normal.x + 1.0).abs() < 1e-5);
        assert!(hit.contact.x <= min.x + 1e-5);
    }

    #[test]
    fn test_ray_aabb_parallel_miss() {
        let o = Vec2::new(-5.0, 2.0);
        let d = Vec2::new(1.0, 0.0);
        let min = Vec2::new(-1.0, -1.0);
        let max = Vec2::new(1.0, 1.0);
        assert!(Narrowphase::ray_aabb(o, d, min, max).is_none());
    }

    #[test]
    fn test_ray_circle_hit() {
        let o = Vec2::new(-3.0, 0.0);
        let d = Vec2::new(1.0, 0.0);
        let c = Vec2::new(0.0, 0.0);
        let r = 1.0;
        let hit = Narrowphase::ray_circle(o, d, c, r).unwrap();
        assert!((hit.contact.x - (-1.0)).abs() < 1e-5);
        assert!((hit.normal.x - (-1.0)).abs() < 1e-5);
    }

    #[test]
    fn test_segment_aabb_hit_and_miss() {
        let a = Vec2::new(-2.0, 0.0);
        let b = Vec2::new(2.0, 0.0);
        let min = Vec2::new(-1.0, -1.0);
        let max = Vec2::new(1.0, 1.0);
        let hit = Narrowphase::line_segment_aabb(a, b, min, max).unwrap();
        assert!(hit.toi >= 0.0 && hit.toi <= 1.0);
        assert!((hit.normal.x + 1.0).abs() < 1e-5);
        let a2 = Vec2::new(-2.0, 2.0);
        let b2 = Vec2::new(2.0, 2.0);
        assert!(Narrowphase::line_segment_aabb(a2, b2, min, max).is_none());
    }

    #[test]
    fn test_segment_circle_basic() {
        let a = Vec2::new(-2.0, 0.0);
        let b = Vec2::new(2.0, 0.0);
        let c = Vec2::new(0.0, 0.0);
        let r = 1.0;
        let hit = Narrowphase::line_segment_circle(a, b, c, r).unwrap();
        assert!(hit.toi >= 0.0 && hit.toi <= 1.0);
        assert!((hit.contact.x - (-1.0)).abs() < 1e-5);
        assert!((hit.normal.x - (-1.0)).abs() < 1e-5);
    }

    // --- Sweeps (CCD) -----------------------------------------------------

    #[test]
    fn test_sweep_aabb_aabb_head_on() {
        let c0 = Vec2::new(-3.0, 0.0);
        let h0 = Vec2::new(1.0, 1.0);
        let v0 = Vec2::new(5.0, 0.0);
        let c1 = Vec2::new(0.0, 0.0);
        let h1 = Vec2::new(1.0, 1.0);
        let v1 = Vec2::new(0.0, 0.0);
        let hit = Narrowphase::sweep_aabb_aabb(c0, h0, v0, c1, h1, v1).unwrap();
        assert!((hit.toi - 0.2).abs() < 1e-5);
        assert!((hit.normal.x + 1.0).abs() < 1e-5);
        assert!((hit.contact.x - (-1.0)).abs() < 1e-5);
    }

    #[test]
    fn test_sweep_circle_circle_head_on() {
        let c0 = Vec2::new(-3.0, 0.0);
        let r0 = 1.0;
        let v0 = Vec2::new(5.0, 0.0);
        let c1 = Vec2::new(0.0, 0.0);
        let r1 = 1.0;
        let v1 = Vec2::new(0.0, 0.0);
        let hit = Narrowphase::sweep_circle_circle(c0, r0, v0, c1, r1, v1).unwrap();
        assert!((hit.toi - 0.2).abs() < 1e-5);
        assert!((hit.normal.x + 1.0).abs() < 1e-5);
        assert!((hit.contact.x - (-1.0)).abs() < 1e-5);
    }

    #[test]
    fn test_sweep_circle_aabb_head_on() {
        let c = Vec2::new(-3.0, 0.0);
        let r = 1.0;
        let v = Vec2::new(5.0, 0.0);
        let bc = Vec2::new(0.0, 0.0);
        let bh = Vec2::new(1.0, 1.0);
        let bv = Vec2::new(0.0, 0.0);
        let hit = Narrowphase::sweep_circle_aabb(c, r, v, bc, bh, bv).unwrap();
        assert!((hit.toi - 0.2).abs() < 1e-5);
        assert!((hit.normal.x + 1.0).abs() < 1e-5);
        assert!((hit.contact.x - (-1.0)).abs() < 1e-5);
    }
}
