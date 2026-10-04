//! Safe Bevy integration for Jolt Physics.
//!
//! Raw FFI lives in `jolt_sys`. This crate owns world lifetime, the safe
//! body API, and the Bevy schedule wiring. New subsystems (character,
//! vehicle, soft body, ...) get their own modules.

mod body_forces;
mod body_sync;
mod debug_draw;
mod joint_sync;
mod physics_world;
mod plugin;

pub use crate::body_forces::{JoltForce, JoltImpulse, JoltSetVelocity};
pub use crate::body_sync::{JoltBody, JoltBodyId, JoltMotion, JoltShape, PreviousBodyTransform};
pub use crate::debug_draw::JoltDebugPlugin;
pub use crate::joint_sync::{JointKind, JoltJoint, JoltJointId};
pub use crate::physics_world::{BodySnapshot, CollisionLayers, JoltWorld, PhysicsShape, RayHit};
pub use crate::plugin::{JoltCollisionLayers, JoltPhysicsWorld, JoltPlugin, JoltStepConfig, step_physics_world};
pub use jolt_sys::MAX_OBJECT_LAYERS;

