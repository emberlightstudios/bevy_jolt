//! One-shot force events and persistent force components.
//!
//! One-shots are [`EntityEvent`]s so firing them never moves the target
//! entity between archetypes (see code-quality rule 18): trigger
//! `JoltImpulse`/`JoltSetVelocity` on the body entity and the observer
//! applies it to the Jolt body immediately. A held push lives as
//! [`JoltForce`] on the entity itself: the two archetype moves happen once
//! at attach/detach, steady-state reads are free, and despawn cleans up
//! with no extra work.

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

/// Persistent force + torque, re-applied every tick while present. Jolt
/// clears accumulated forces each step, so the system re-adds them before
/// the physics step. Remove the component to stop pushing.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltForce {
    pub push_force: Vec3,
    pub push_torque: Vec3,
}

impl JoltForce {
    pub fn force(push_force: Vec3) -> Self {
        Self {
            push_force,
            push_torque: Vec3::ZERO,
        }
    }

    pub fn torque(push_torque: Vec3) -> Self {
        Self {
            push_force: Vec3::ZERO,
            push_torque,
        }
    }
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

/// Re-adds every [`JoltForce`] before the physics step. Bodies missing
/// their id (not baked yet) are skipped for the tick, not despawned:
/// unlike joints, a force has no endpoint to go stale on.
pub fn apply_jolt_forces(
    force_query: Query<(&JoltForce, &JoltBodyId)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (push, body_id) in &force_query {
        physics_world.apply_force(body_id.body_id_raw, push.push_force, push.push_torque);
    }
}
