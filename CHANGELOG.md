# Changelog

All notable changes to this project will be documented in this file.

## 0.1.0 — Initial scaffolding
- Ephemeral world with uniform grid broadphase.
- Narrowphase overlaps, rays/segments, and sweeps (AABB, Circle, Point).
- Event emission (overlap/sweep), queries, and pairwise helpers.
- Examples and basic CI.

## 0.9.0 — FLESHED OUT
- Everything is fleshed out.

## Unreleased
- Tile pushouts now return signed depth:
  - Overlap/tangent: depth >= 0 (0 at tangency)
  - Separation: depth < 0 with magnitude equal to gap distance
- AABB separation normal is axis-aligned toward the shape; circle separation normal remains zero.
- `query_circle_all` uses a boolean overlap gate for tiles (no pushout in query path).
