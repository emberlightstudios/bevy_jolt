//! One-shot force events and persistent drive components.
//!
//! One-shots are [`EntityEvent`]s so firing them never moves the target
//! entity between archetypes (see code-quality rule 18): trigger
//! [`JoltImpulse`] on the body entity and the observer applies it to the
//! Jolt body immediately. Velocity needs no trigger: write
//! [`JoltLinearVelocity`] / [`JoltAngularVelocity`] and change detection
//! pushes it before the next step. Held drives live as components
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

/// One-shot pose teleport: moves the body inside Jolt (position + rotation
/// atomically), wakes it, and the sync carries the pose out to Bevy. A
/// teleported body keeps its old momentum by default: write
/// [`JoltLinearVelocity`] / [`JoltAngularVelocity`] zero after for a dead
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

/// Linear velocity, read and written as one: game code writes a drive
/// request, the crate pushes `Changed` values to Jolt before the step, and
/// the post-step sync writes the measured result back into the same
/// component (bypassing change detection, so the writeback never re-drives
/// itself). The value you read is always the truth: what the sim says the
/// body is doing right now. A blocked request is forgotten, not retried:
/// write again (or hold with `set_if_neq`) to keep pushing. Present on
/// every non-static body from bake, zeroed.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltLinearVelocity {
    pub linear_velocity: Vec3,
}

/// Angular velocity, same unified read/write rule as [`JoltLinearVelocity`].
/// Kept separate so driving movement never wipes out spin (and vice versa):
/// each half is added/removed on its own. Present on every non-static body
/// from bake, zeroed.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltAngularVelocity {
    pub angular_velocity: Vec3,
}

/// Per-tick destination for a kinematic body (elevator, moving platform,
/// sliding door, patrol): any entity with `JoltBody::kinematic`, not just
/// characters. Write where the body should be this tick and the crate moves
/// it there with `MoveKinematic`, which derives the velocity from the delta
/// — so the platform shoves dynamic bodies aside instead of teleporting
/// through them. Not for teleports (`JoltTeleport` does that) and not for
/// dynamics (write [`JoltLinearVelocity`] there). Set the fields each frame
/// while the platform runs; remove the component to stop driving it.
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

/// One-shot sleep: freezes the body where it stands. Still solid, wakes on
/// contact. For dormant crowds. Missing bodies skipped, same as impulses.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltSleep {
    #[event_target]
    pub body_entity: Entity,
}

/// One-shot wake: sleeping body rejoins next step, velocities intact.
/// Harmless on awake bodies. Missing bodies skipped, same as impulses.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltWake {
    #[event_target]
    pub body_entity: Entity,
}

/// One-shot motion-type setter: changes how a body moves while the game runs
/// (frozen statue to falling rock, platform to solid ground) without
/// despawning it. Setting static also sleeps the body permanently: only
/// setting back to kinematic/dynamic rejoins the sim, `JoltWake` alone won't
/// do it. Updates the entity's [`JoltBody`] in the same call, so Bevy-side
/// reads never disagree with Jolt.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltSetMotion {
    #[event_target]
    pub body_entity: Entity,
    pub motion: crate::body_sync::JoltMotion,
}

/// Applies a triggered [`JoltSleep`] to the target entity's Jolt body.
pub fn apply_jolt_sleep(
    trigger: On<JoltSleep>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let sleep = trigger.event();
    let Ok(body_id) = body_ids.get(sleep.body_entity) else {
        return;
    };
    physics_world.sleep_body(body_id.body_id_raw);
}

/// Applies a triggered [`JoltWake`] to the target entity's Jolt body.
pub fn apply_jolt_wake(
    trigger: On<JoltWake>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let wake = trigger.event();
    let Ok(body_id) = body_ids.get(wake.body_entity) else {
        return;
    };
    physics_world.wake_body(body_id.body_id_raw);
}

/// Applies a triggered [`JoltSetMotion`] to the target entity's Jolt body
/// and its [`JoltBody`]: one call, never half-synced.
pub fn apply_jolt_set_motion(
    trigger: On<JoltSetMotion>,
    mut body_query: Query<(&JoltBodyId, &mut crate::body_sync::JoltBody)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let set_motion = trigger.event();
    let Ok((body_id, mut body)) = body_query.get_mut(set_motion.body_entity) else {
        return;
    };
    physics_world.set_body_motion(body_id.body_id_raw, &mut body, set_motion.motion);
}

/// Marker on sleeping bodies, added/removed by the activation drain. The
/// per-tick sync filters these out (no FFI for frozen bodies) and the debug
/// drawer reads the marker for its sleep color instead of polling Jolt.
/// Managed by the crate: never add or remove it by hand. Game code reacts
/// with `Added<JoltSleeping>` / `Removed<JoltSleeping>` queries: no polling,
/// no callbacks in the public API.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct JoltSleeping;

/// Drains sleep/wake transitions after the step: slept bodies gain
/// [`JoltSleeping`], woken bodies lose it. Unknown ids (despawned mid-step)
/// are skipped.
pub fn sync_sleep_markers(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    body_entities: Query<(Entity, &JoltBodyId)>,
) {
    let mut slept_ids = [0u32; crate::physics_world::JoltWorld::MAX_ACTIVATION_EVENTS];
    let mut woke_ids = [0u32; crate::physics_world::JoltWorld::MAX_ACTIVATION_EVENTS];
    let slept_kept = physics_world.drain_slept(&mut slept_ids);
    let woke_kept = physics_world.drain_woke(&mut woke_ids);
    let find_body_entity = |body_id_raw: u32| {
        body_entities
            .iter()
            .find(|(_, body_id)| body_id.body_id_raw == body_id_raw)
            .map(|(body_entity, _)| body_entity)
    };
    for slept_index in 0..slept_kept as usize {
        let Some(slept_entity) = find_body_entity(slept_ids[slept_index]) else {
            continue;
        };
        commands.entity(slept_entity).insert(JoltSleeping);
    }
    for woke_index in 0..woke_kept as usize {
        let Some(woke_entity) = find_body_entity(woke_ids[woke_index]) else {
            continue;
        };
        commands.entity(woke_entity).remove::<JoltSleeping>();
    }
}
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


/// Pushes `Changed` [`JoltLinearVelocity`] / [`JoltAngularVelocity`] values
/// to Jolt before the physics step. Change detection is the on-switch:
/// untouched bodies are never written, so sleep survives and natural motion
/// stays natural. Drive with `set_if_neq` (or write only when the target
/// changes): a plain write every frame re-drives every frame, which is the
/// correct pattern for a motor and the wrong one for a nudge.
pub fn apply_jolt_driven_velocities(
    velocity_query: Query<
        (
            &JoltBodyId,
            Option<Ref<JoltLinearVelocity>>,
            Option<Ref<JoltAngularVelocity>>,
        ),
        Or<(
            Changed<JoltLinearVelocity>,
            Changed<JoltAngularVelocity>,
        )>,
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
