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

use crate::body_sync::{JoltBody, JoltBodyId};
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

/// Linear + angular velocity bleed, read by the pre-step push like sleep and
/// motion: add or rewrite the component and `Changed` detection carries it
/// into Jolt before the next step. 0 is Jolt's default glide; small values
/// (0.1) calm jitter, large ones (3+) still joint crawl in seconds. Ragdolls
/// hold still this way instead of micro-sliding on joint motion.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq)]
pub struct JoltDamping {
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl JoltDamping {
    pub const fn new(linear_damping: f32, angular_damping: f32) -> Self {
        Self {
            linear_damping,
            angular_damping,
        }
    }
}

/// Pushes `Changed` [`JoltDamping`] values to Jolt. Write the fields (or add
/// the component to a live body) and the pre-step push applies them before
/// the next step — same `Changed` pattern as [`sync_jolt_sleep`].
pub fn sync_jolt_damping(
    damping_query: Query<(&JoltBodyId, Ref<JoltDamping>), Changed<JoltDamping>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, damping) in &damping_query {
        physics_world.set_body_damping(
            body_id.body_id_raw,
            damping.linear_damping,
            damping.angular_damping,
        );
    }
}

/// Mass density in kg/m³ (water ≈ 1000). Bake reads it for the spawn mass;
/// later writes rescale mass + inertia live (density × shape volume) before
/// the next step. Must be positive and finite.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct JoltDensity {
    pub density_kg_per_m3: f32,
}

impl JoltDensity {
    pub const fn new(density_kg_per_m3: f32) -> Self {
        Self { density_kg_per_m3 }
    }
}

/// Pushes `Changed` [`JoltDensity`] values to Jolt. Same pre-step pattern
/// as [`sync_jolt_damping`].
pub fn sync_jolt_density(
    density_query: Query<(&JoltBodyId, Ref<JoltDensity>), Changed<JoltDensity>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, density) in &density_query {
        physics_world.set_body_density(body_id.body_id_raw, density.density_kg_per_m3);
    }
}

/// Gravity multiplier: 0 floats, 1 is normal, 2 pulls twice as hard. Bake
/// reads it for the spawn body; later writes land before the next step.
/// Must be finite and non-negative.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct JoltGravity {
    pub gravity_factor: f32,
}

impl JoltGravity {
    pub const fn new(gravity_factor: f32) -> Self {
        Self { gravity_factor }
    }
}

/// Pushes `Changed` [`JoltGravity`] values to Jolt. Same pre-step pattern
/// as [`sync_jolt_damping`].
pub fn sync_jolt_gravity(
    gravity_query: Query<(&JoltBodyId, Ref<JoltGravity>), Changed<JoltGravity>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, gravity) in &gravity_query {
        physics_world.set_body_gravity_factor(body_id.body_id_raw, gravity.gravity_factor);
    }
}

/// Surface grip 0 (ice) to 1+ (rubber). Bake reads it for the spawn body;
/// later writes land before the next step. Must be finite, non-negative.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct JoltFriction {
    pub friction: f32,
}

impl JoltFriction {
    pub const fn new(friction: f32) -> Self {
        Self { friction }
    }
}

/// Pushes `Changed` [`JoltFriction`] values to Jolt. Same pre-step pattern
/// as [`sync_jolt_damping`].
pub fn sync_jolt_friction(
    friction_query: Query<(&JoltBodyId, Ref<JoltFriction>), Changed<JoltFriction>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, friction) in &friction_query {
        physics_world.set_body_friction(body_id.body_id_raw, friction.friction);
    }
}

/// Bounciness 0 (dead) to 1 (superball). Bake reads it for the spawn body;
/// later writes land before the next step. Must be finite, 0..=1.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct JoltRestitution {
    pub restitution: f32,
}

impl JoltRestitution {
    pub const fn new(restitution: f32) -> Self {
        Self { restitution }
    }
}

/// Pushes `Changed` [`JoltRestitution`] values to Jolt. Same pre-step
/// pattern as [`sync_jolt_damping`].
pub fn sync_jolt_restitution(
    restitution_query: Query<(&JoltBodyId, Ref<JoltRestitution>), Changed<JoltRestitution>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, restitution) in &restitution_query {
        physics_world.set_body_restitution(body_id.body_id_raw, restitution.restitution);
    }
}

/// Sweep the shape between steps (LinearCast) so a fast body stops at the
/// first hit instead of tunneling. Costs more per tick; reserve for bodies
/// that outrun their own size in one step. Bake reads it; later writes
/// flip the motion quality live.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JoltCcd {
    pub use_ccd: bool,
}

impl JoltCcd {
    pub const fn enabled() -> Self {
        Self { use_ccd: true }
    }
}

/// Pushes `Changed` [`JoltCcd`] values to Jolt. Same pre-step pattern as
/// [`sync_jolt_damping`].
pub fn sync_jolt_ccd(
    ccd_query: Query<(&JoltBodyId, Ref<JoltCcd>), Changed<JoltCcd>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, ccd) in &ccd_query {
        physics_world.set_body_ccd(body_id.body_id_raw, ccd.use_ccd);
    }
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

/// Pushes `Changed` [`JoltSleeping`] values to Jolt. Write `sleeping = true`
/// to freeze a body where it stands (still solid, wakes on contact), `false`
/// to rejoin next step with velocities intact. Harmless on bodies already in
/// that state. One feedback echo per flip: the post-step drain writes Jolt's
/// transition back into this same field, which re-fires `Changed` once — the
/// second push is a no-op (Jolt reports nothing new) and the loop stops.
pub fn sync_jolt_sleep(
    sleep_query: Query<(&JoltBodyId, Ref<JoltSleeping>), Changed<JoltSleeping>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, sleeping) in &sleep_query {
        if **sleeping {
            physics_world.sleep_body(body_id.body_id_raw);
        } else {
            physics_world.wake_body(body_id.body_id_raw);
        }
    }
}

/// Pushes `Changed` [`JoltBody`] motion values to Jolt. Runs first in
/// `JoltStep`, before the sim steps: every `FixedUpdate` write has landed by
/// then, so a direct `body.motion` write takes effect the same tick — no
/// frame delay, no trigger, no ordering needed. Setting static also sleeps
/// the body permanently: only setting back to kinematic/dynamic rejoins the
/// sim, writing `sleeping = false` alone won't do it.
pub fn sync_jolt_motion(
    motion_query: Query<(&JoltBodyId, Ref<JoltBody>), Changed<JoltBody>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (body_id, body) in &motion_query {
        physics_world.set_body_motion(body_id.body_id_raw, body.motion);
    }
}

/// Sleep state, written by both sides. Game code writes requests (`sleeping
/// = true` freezes, `false` rejoins); the pre-step sync pushes `Changed`
/// values to Jolt; the post-step drain writes Jolt's own transitions back
/// here. Both operations are idempotent, so the drain's echo costs one extra
/// no-op push per flip, then stops — it cannot loop. Derefs to bool, so
/// `if **sleeping` reads the state directly. Never moves the entity between
/// archetypes, no matter how often bodies nod off and wake.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JoltSleeping {
    pub sleeping: bool,
}

impl std::ops::Deref for JoltSleeping {
    type Target = bool;

    fn deref(&self) -> &Self::Target {
        &self.sleeping
    }
}

impl std::ops::DerefMut for JoltSleeping {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.sleeping
    }
}

/// Drains sleep/wake transitions after the step: slept bodies read sleeping,
/// woken bodies read awake. Unknown ids (despawned mid-step) are skipped.
pub fn sync_sleep_states(
    mut sleep_query: Query<&mut JoltSleeping>,
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
        let Some(sleeping_entity) = find_body_entity(slept_ids[slept_index]) else {
            continue;
        };
        let Ok(mut sleeping) = sleep_query.get_mut(sleeping_entity) else {
            continue;
        };
        sleeping.sleeping = true;
    }
    for woke_index in 0..woke_kept as usize {
        let Some(waking_entity) = find_body_entity(woke_ids[woke_index]) else {
            continue;
        };
        let Ok(mut sleeping) = sleep_query.get_mut(waking_entity) else {
            continue;
        };
        sleeping.sleeping = false;
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
/// to Jolt. Runs first in `JoltStep`, before the sim steps, so every
/// `FixedUpdate` write has landed no matter what order game code ran in.
/// Change detection is the on-switch: untouched bodies are never written, so
/// sleep survives and natural motion stays natural. Drive with `set_if_neq`
/// (or write only when the target changes): a plain write every frame
/// re-drives every frame, which is the correct pattern for a motor and the
/// wrong one for a nudge.
pub fn apply_jolt_driven_velocities(
    velocity_query: Query<
        (
            &JoltBodyId,
            Option<Ref<JoltLinearVelocity>>,
            Option<Ref<JoltAngularVelocity>>,
        ),
        Or<(Changed<JoltLinearVelocity>, Changed<JoltAngularVelocity>)>,
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
    let tick_delta = physics_world.sim_tick_delta(fixed_time.delta().as_secs_f32());
    for (body_id, kinematic_target) in &target_query {
        physics_world.move_kinematic(
            body_id.body_id_raw,
            kinematic_target.target_position,
            kinematic_target.target_rotation,
            tick_delta,
        );
    }
}
