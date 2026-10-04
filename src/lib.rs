//! Safe Bevy integration for Jolt Physics.
//!
//! Raw FFI lives in `jolt_sys`. This crate owns world lifetime, the safe
//! body API, and the Bevy schedule wiring. New subsystems (character,
//! vehicle, soft body, ...) get their own modules.

mod debug_draw;
mod physics_world;
mod plugin;

pub use crate::debug_draw::{JoltDebugDraw, JoltDebugPlugin};
pub use crate::physics_world::{BodySnapshot, JoltWorld, PhysicsShape, RayHit};
pub use crate::plugin::{JoltPhysicsWorld, JoltPlugin};
pub use jolt_sys::{OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING};

