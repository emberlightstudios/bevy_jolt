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
mod vehicle;
pub use crate::body_forces::{
    JoltAngularForce, JoltAngularVelocity, JoltImpulse, JoltLinearForce, JoltLinearVelocity,
    JoltSetVelocity,
};
pub use crate::body_sync::{JoltBody, JoltBodyId, JoltMotion, JoltShape, PreviousBodyTransform};
pub use crate::debug_draw::JoltDebugPlugin;
pub use crate::joint_sync::{JointKind, JointMotor, JoltJoint, JoltJointId, JoltMotorDrive};
pub use crate::joint_sync::{SixDofAxis, SixDofFrame, SixDofLimits};
pub use crate::physics_world::{BodySnapshot, CollisionLayers, JointSpace, JoltWorld, PhysicsShape, RayHit};
pub use crate::plugin::{JoltCollisionLayers, JoltPhysicsWorld, JoltPlugin, JoltStartupGravity, JoltStepConfig, step_physics_world};
pub use crate::vehicle::{
    JoltTrackedDrive, JoltVehicle, JoltVehicleDrive, JoltVehicleId, JoltVehicleShift, VehicleDifferential,
    VehicleEngine, VehicleKind, VehicleLean, VehicleRollBar, VehicleSpec, VehicleTrack,
    VehicleTransmission, VehicleWheel, VEHICLE_NO_FLIP_LIMIT,
};
pub use jolt_sys::MAX_OBJECT_LAYERS;

