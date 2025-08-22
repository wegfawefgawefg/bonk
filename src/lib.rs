//! bonk: detection-only physics engine (ephemeral world, no resolution)

pub mod types;
pub mod api;
pub mod world;
pub mod narrowphase;

pub use crate::types::*;
pub use crate::api::*;
pub use crate::world::PhysicsWorld;
