//! Safe Bevy integration for Jolt Physics.
//!
//! Raw FFI lives in `jolt_sys`. This crate owns world lifetime, the safe
//! body API, and the Bevy schedule wiring. New subsystems (character,
//! vehicle, soft body, ...) get their own modules.

mod body_forces;
mod body_sync;
mod buoyancy;
mod character;
mod contact_events;
mod debug_draw;
mod joint_sync;
mod physics_world;
mod plugin;
mod ragdoll;
mod soft_body;
mod spatial_queries;
mod vehicle;
pub use crate::body_forces::{
    JoltAngularForce, JoltAngularVelocity, JoltImpulse, JoltKinematicTarget, JoltLinearForce,
    JoltLinearVelocity, JoltSetVelocity, JoltTeleport,
};
pub use crate::body_sync::{JoltBody, JoltBodyId, JoltMotion, JoltShape, PreviousBodyTransform};
pub use crate::buoyancy::{JoltBuoyant, JoltWater};
pub use crate::character::{
    CharacterGround, JoltCharacter, JoltCharacterGround, JoltCharacterId, JoltCharacterStep,
    JoltCharacterTeleport, JoltCharacterVelocity, JoltRigidCharacter, JoltRigidCharacterId,
    JoltRigidCharacterImpulse, JoltRigidCharacterPush, JoltRigidCharacterTeleport,
    JoltRigidCharacterVelocity,
};
pub use crate::contact_events::{
    JoltContactAdded, JoltContactRemoved, JoltSensor, MAX_CONTACT_EVENTS,
};
pub use crate::debug_draw::JoltDebugPlugin;
pub use crate::joint_sync::{JointKind, JointMotor, JoltJoint, JoltJointId, JoltMotorDrive};
pub use crate::joint_sync::{
    MAX_PATH_KNOTS, PathKnot, PathRotation, SixDofAxis, SixDofFrame, SixDofLimits,
    path_knots_from_waypoints,
};
pub use crate::physics_world::{
    BodySnapshot, CharacterDofs, CollisionLayers, CompoundGeometry, CompoundPart, JointSpace,
    JoltWorld, PhysicsShape, SoftBendType, SoftBodyConfig,
};
pub use crate::plugin::{JoltCollisionLayers, JoltPhysicsWorld, JoltPlugin, JoltStartupGravity, JoltStepConfig, step_physics_world};
pub use crate::ragdoll::{JoltRagdoll, JoltRagdollParts, RagdollPart};
pub use crate::soft_body::{
    JoltSoftBodyConfig, JoltSoftBodyId, JoltSoftBodyMesh, JoltSoftSharedSettings,
};
pub use crate::spatial_queries::{MAX_QUERY_HITS, OverlapHit, QueryProbe, RayHit};
pub use crate::vehicle::{
    CurveKnot, JoltTrackedDrive, JoltVehicleDrive, JoltVehicleId, JoltVehicleShift,
    VehicleDifferential, VehicleEngine, VehicleKind, VehicleLean, VehicleRollBar, VehicleSpec,
    VehicleTrack, VehicleTransmission, VehicleWheel, STOCK_ENGINE_TORQUE, STOCK_TIRE_LATERAL, STOCK_TIRE_LONGITUDINAL,
};
pub use jolt_sys::MAX_OBJECT_LAYERS;
