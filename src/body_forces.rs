//! One-shot force events and persistent drive components.
//!
//! One-shots are [`EntityEvent`]s so firing them never moves the target
//! entity between archetypes (see code-quality rule 18): trigger
//! `JoltImpulse`/`JoltSetVelocity` on the body entity and the observer
//! applies it to the Jolt body immediately. Held drives live as components
//! on the entity itself: the two archetype moves happen once at
//! attach/detach, steady-state reads are free, and despawn cleans up
//! with no extra work.
//!
//! Linear and angular halves are separate components so driving movement
//! never wipes out spin (and vice versa): most bodies only need the linear
//! half, and each half is added/removed on its own.

use bevy::prelude::*;

use crate::body_sync::JoltBodyId;
use crate::plugin::JoltPhysicsWorld;

/// One-shot linear + angular impulse at center of mass. Zero halves are
/// skipped, so one trigger covers a pure shove, a pure spin, or both.
/// Trigger on the body entity: `commands.trigger(JoltImpulse::linear(id, v))`.
/// Missing bodies (not baked yet, despawned) are skipped: one-shot means
/// fire-and-forget, never retry.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltImpulse {
    #[event_target]
    pub body_entity: Entity,
    pub linear_impulse: Vec3,
    pub angular_impulse: Vec3,
}

impl JoltImpulse {
    pub fn linear(body_entity: Entity, linear_impulse: Vec3) -> Self {
        Self {
            body_entity,
            linear_impulse,
            angular_impulse: Vec3::ZERO,
        }
    }

    pub fn angular(body_entity: Entity, angular_impulse: Vec3) -> Self {
        Self {
            body_entity,
            linear_impulse: Vec3::ZERO,
            angular_impulse,
        }
    }

    /// Combined shove + spin in one trigger. Zero halves are skipped, so
    /// either side can be zero for a pure shove or a pure spin.
    pub fn swinging(body_entity: Entity, linear_impulse: Vec3, angular_impulse: Vec3) -> Self {
        Self {
            body_entity,
            linear_impulse,
            angular_impulse,
        }
    }
}

/// One-shot velocity overwrite (not a kick): zero halves stop that axis
/// instead of leaving it alone. Same fire-and-forget rule as impulses.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltSetVelocity {
    #[event_target]
    pub body_entity: Entity,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl JoltSetVelocity {
    pub fn linear(body_entity: Entity, linear_velocity: Vec3) -> Self {
        Self {
            body_entity,
            linear_velocity,
            angular_velocity: Vec3::ZERO,
        }
    }

    pub fn stop(body_entity: Entity) -> Self {
        Self {
            body_entity,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

/// One-shot pose teleport: moves the body inside Jolt (position + rotation
/// atomically), wakes it, and the sync carries the pose out to Bevy. Zero
/// velocities on arrival unless told otherwise: a teleported body keeps its
/// old momentum by default, pass `JoltSetVelocity::stop` after for a dead
/// stop. Missing bodies are skipped, same as impulses.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltTeleport {
    #[event_target]
    pub body_entity: Entity,
    pub target_position: Vec3,
    pub target_rotation: Quat,
}
/// clears accumulated forces each step, so the system re-adds it before
/// the physics step. Remove the component to stop pushing.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltLinearForce {
    pub linear_force: Vec3,
}

/// Persistent angular torque, re-applied every tick while present.
/// Same re-add rule as [`JoltLinearForce`].
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltAngularForce {
    pub angular_torque: Vec3,
}

/// Persistent linear velocity, overwritten every tick while present.
/// Overrules gravity/friction/drag on this axis while attached: for
/// directly driven bodies (player, platform, hover), not for bodies that
/// should react naturally. Remove the component to release the body.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltLinearVelocity {
    pub linear_velocity: Vec3,
}

/// Persistent angular velocity, overwritten every tick while present.
/// Same overrule warning as [`JoltLinearVelocity`].
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltAngularVelocity {
    pub angular_velocity: Vec3,
}

/// Measured linear velocity, written by the crate every tick after the
/// physics step. Read-only for game code: never write it, and never drive
/// from it (the pre-step systems ignore it). Tells what the body is
/// actually doing, unlike [`JoltLinearVelocity`] which says what you asked
/// for. Inserted at bake for every non-static body.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltMeasuredLinearVelocity {
    pub measured_linear_velocity: Vec3,
}

/// Measured spin, same read-only rule as [`JoltMeasuredLinearVelocity`].
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltMeasuredAngularVelocity {
    pub measured_angular_velocity: Vec3,
}

/// Target pose for a kinematic body, driven every tick while present.
/// `MoveKinematic` derives velocity from the delta, so the body shoves
/// dynamics aside instead of teleporting through them. Set the fields each
/// frame (sine wave, elevator, patrol) and remove the component to stop.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltKinematicTarget {
    pub target_position: Vec3,
    pub target_rotation: Quat,
}

/// Applies a triggered [`JoltImpulse`] to the target entity's Jolt body.
pub fn apply_jolt_impulse(
    trigger: On<JoltImpulse>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let impulse = trigger.event();
    let Ok(body_id) = body_ids.get(impulse.body_entity) else {
        return;
    };
    physics_world.apply_impulse(
        body_id.body_id_raw,
        impulse.linear_impulse,
        impulse.angular_impulse,
    );
}

/// Applies a triggered [`JoltSetVelocity`] to the target entity's Jolt body.
pub fn apply_jolt_set_velocity(
    trigger: On<JoltSetVelocity>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let velocity = trigger.event();
    let Ok(body_id) = body_ids.get(velocity.body_entity) else {
        return;
    };
    physics_world.set_body_velocity(
        body_id.body_id_raw,
        velocity.linear_velocity,
        velocity.angular_velocity,
    );
}
/// Applies a triggered [`JoltTeleport`] to the target entity's Jolt body.
pub fn apply_jolt_teleport(
    trigger: On<JoltTeleport>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let teleport = trigger.event();
    let Ok(body_id) = body_ids.get(teleport.body_entity) else {
        return;
    };
    physics_world.teleport_body(
        body_id.body_id_raw,
        teleport.target_position,
        teleport.target_rotation,
    );
}

/// Re-adds every [`JoltLinearForce`] and [`JoltAngularForce`] before the
/// physics step. Bodies missing their id (not baked yet) are skipped for
/// the tick, not despawned: unlike joints, a force has no endpoint to go
/// stale on. One pass with `Option` reads: no extra loop cost for the
/// split, and entities carrying only one half skip the other FFI call
/// (zero halves are skipped inside the FFI anyway).
pub fn apply_jolt_forces(
    force_query: Query<
        (
            &JoltBodyId,
            Option<&JoltLinearForce>,
            Option<&JoltAngularForce>,
        ),
        Or<(With<JoltLinearForce>, With<JoltAngularForce>)>,
    >,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, linear_push, angular_push) in &force_query {
        let linear_force = linear_push
            .map(|push| push.linear_force)
            .unwrap_or(Vec3::ZERO);
        let angular_torque = angular_push
            .map(|push| push.angular_torque)
            .unwrap_or(Vec3::ZERO);
        physics_world.apply_force(body_id.body_id_raw, linear_force, angular_torque);
    }
}


/// Overwrites velocity every tick for entities carrying
/// [`JoltLinearVelocity`] and/or [`JoltAngularVelocity`]. Each half only
/// touches its own axis, so a driven move never wipes out spin. Runs
/// before the physics step, same as forces.
pub fn apply_jolt_driven_velocities(
    velocity_query: Query<
        (
            &JoltBodyId,
            Option<&JoltLinearVelocity>,
            Option<&JoltAngularVelocity>,
        ),
        Or<(With<JoltLinearVelocity>, With<JoltAngularVelocity>)>,
    >,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, linear_drive, angular_drive) in &velocity_query {
        if let Some(linear_drive) = linear_drive {
            physics_world.set_linear_velocity(body_id.body_id_raw, linear_drive.linear_velocity);
        }
        if let Some(angular_drive) = angular_drive {
            physics_world.set_angular_velocity(body_id.body_id_raw, angular_drive.angular_velocity);
        }
    }
}

/// Drives every [`JoltKinematicTarget`] toward its pose before the physics
/// step. The query needs `JoltBodyId`, so not-yet-baked bodies simply don't
/// match. Game code just writes the target fields; never touches the world.
pub fn apply_jolt_kinematic_targets(
    target_query: Query<(&JoltBodyId, &JoltKinematicTarget)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    fixed_time: Res<Time<Fixed>>,
) {
    let tick_delta = fixed_time.delta().as_secs_f32();
    for (body_id, kinematic_target) in &target_query {
        physics_world.move_kinematic(
            body_id.body_id_raw,
            kinematic_target.target_position,
            kinematic_target.target_rotation,
            tick_delta,
        );
    }
}
