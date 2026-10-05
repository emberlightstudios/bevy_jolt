//! Virtual character controller: a kinematic capsule steered by velocity.
//!
//! A [`JoltCharacter`] is NOT a rigid body — Jolt moves it with
//! `ExtendedUpdate` (collide + stick-to-floor + walk-stairs) instead of
//! forces. The game sets [`JoltCharacterVelocity`] every tick (gravity
//! included); [`step_jolt_characters`] runs the move before the physics
//! step and writes the result back to `Transform`. Grounded state lands in
//! [`JoltCharacterGround`] for jumps and footsteps.

use bevy::prelude::*;

use crate::plugin::JoltPhysicsWorld;

/// Character capsule: bottom of the capsule spawns at the entity's
/// `Transform`. Creation reads the entity once, then owns the Jolt
/// character until despawn.
#[derive(Component, Clone, Debug)]
pub struct JoltCharacter {
    pub capsule_half_height: f32,
    pub capsule_radius: f32,
    pub object_layer: u16,
    /// Push strength against dynamic bodies (Newtons-ish, Jolt units).
    pub mass_kg: f32,
    pub max_strength: f32,
    /// Steepest walkable slope in degrees; steeper slides.
    pub max_slope_degrees: f32,
    /// Skin gap Jolt keeps from geometry (meters).
    pub character_padding: f32,
    /// How fast penetration resolves (0 = stuck, 1 = instant).
    pub penetration_recovery: f32,
}

impl JoltCharacter {
    /// Human-sized default on a layer: 0.5 m cylinder + 0.3 m radius caps.
    pub fn new(object_layer: u16) -> Self {
        Self {
            capsule_half_height: 0.5,
            capsule_radius: 0.3,
            object_layer,
            mass_kg: 70.0,
            max_strength: 100.0,
            max_slope_degrees: 50.0,
            character_padding: 0.02,
            penetration_recovery: 1.0,
        }
    }
}

/// Velocity the game wants this tick, gravity included. Set it every frame
/// from input + jump + `gravity * dt`; the stepper consumes it.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct JoltCharacterVelocity {
    pub velocity: Vec3,
}

/// How far up a step the character may climb, and how far down it sticks to
/// descending ground. Zero either to turn that half off.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltCharacterStep {
    pub step_up_height: f32,
    pub stick_to_floor_distance: f32,
}

impl Default for JoltCharacterStep {
    fn default() -> Self {
        Self {
            step_up_height: 0.4,
            stick_to_floor_distance: 0.5,
        }
    }
}

/// Jolt-assigned character id. Inserted at bake; the character dies with
/// the entity.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltCharacterId {
    pub character_id_raw: u32,
}

/// Ground reading from the last move: state (air / ground / steep), surface
/// normal, and whether the character stands supported.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct JoltCharacterGround {
    pub ground_state: CharacterGround,
    pub ground_normal: Vec3,
    pub is_supported: bool,
}

/// Where the character stands: open air, walkable ground, or a slope too
/// steep to hold (slides).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CharacterGround {
    #[default]
    InAir,
    OnGround,
    OnSteepGround,
}

impl CharacterGround {
    /// Jolt `EGroundState` order: OnGround = 0, OnSteepGround = 1,
    /// NotSupported = 2, InAir = 3.
    pub fn from_raw(ground_state: u32) -> Self {
        match ground_state {
            0 => CharacterGround::OnGround,
            1 => CharacterGround::OnSteepGround,
            _ => CharacterGround::InAir,
        }
    }
}

/// Teleports the character to a position with a velocity. Fire-and-forget
/// like the impulse trigger: missing characters (not baked yet, despawned)
/// are skipped.
#[derive(EntityEvent, Clone, Copy, Debug)]
pub struct JoltCharacterTeleport {
    #[event_target]
    pub character_entity: Entity,
    pub target_position: Vec3,
    pub target_velocity: Vec3,
}

/// Creates the Jolt character from the spawn `Transform` and files the id.
/// Bodies missing nothing: a character needs only its spec + pose.
pub fn bake_jolt_character(
    trigger: On<Add, JoltCharacter>,
    character_query: Query<(&JoltCharacter, &Transform)>,
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let character_entity = trigger.event().entity;
    let Ok((character, character_transform)) = character_query.get(character_entity) else {
        panic!("JoltCharacter gone on {character_entity:?} before bake ran");
    };
    let character_position = character_transform.translation;
    let character_id_raw = physics_world.character_create(
        character_position,
        character.capsule_half_height,
        character.capsule_radius,
        character.object_layer,
        character.mass_kg,
        character.max_strength,
        character.max_slope_degrees,
        character.character_padding,
        character.penetration_recovery,
    );
    assert_ne!(character_id_raw, 0, "Jolt rejected the character capsule");
    commands.entity(character_entity).insert((
        JoltCharacterId { character_id_raw },
        JoltCharacterGround::default(),
    ));
}

/// Destroys the Jolt character when its entity goes. Order-independent:
/// the character registry never touches bodies.
pub fn despawn_jolt_character(
    trigger: On<Remove, JoltCharacterId>,
    character_query: Query<&JoltCharacterId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(character_id) = character_query.get(trigger_entity) else {
        panic!("JoltCharacterId gone on {trigger_entity:?} before despawn ran");
    };
    physics_world.character_destroy(character_id.character_id_raw);
}

/// Applies a triggered teleport to the target entity's Jolt character.
pub fn apply_jolt_character_teleport(
    trigger: On<JoltCharacterTeleport>,
    character_ids: Query<&JoltCharacterId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let teleport = trigger.event();
    let Ok(character_id) = character_ids.get(teleport.character_entity) else {
        return;
    };
    physics_world.character_teleport(
        character_id.character_id_raw,
        teleport.target_position,
        teleport.target_velocity,
    );
}

/// Runs every character's move (ExtendedUpdate) before the physics step.
/// Characters missing velocity or step config stand still but still settle
/// (velocity defaults to zero, step config to its default).
#[allow(clippy::too_many_arguments)]
pub fn step_jolt_characters(
    character_query: Query<(
        &JoltCharacterId,
        Option<&JoltCharacterVelocity>,
        Option<&JoltCharacterStep>,
    )>,
    fixed_time: Res<Time<Fixed>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let tick_delta = fixed_time.timestep().as_secs_f32();
    let world_gravity = physics_world.world_gravity();
    for (character_id, character_velocity, character_step) in &character_query {
        let wanted_velocity = character_velocity
            .map(|character_velocity| character_velocity.velocity)
            .unwrap_or(Vec3::ZERO);
        let step_config = character_step.copied().unwrap_or_default();
        physics_world.character_move(
            character_id.character_id_raw,
            tick_delta,
            wanted_velocity,
            world_gravity,
            step_config.step_up_height,
            step_config.stick_to_floor_distance,
        );
    }
}

/// Copies each character's Jolt pose + ground reading into Bevy after the
/// step. Characters missing their id (not baked yet) are skipped.
pub fn sync_character_transforms(
    mut character_query: Query<(&JoltCharacterId, &mut Transform, &mut JoltCharacterGround)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (character_id, mut character_transform, mut character_ground) in &mut character_query {
        let (character_position, ground_reading) =
            physics_world.character_pose(character_id.character_id_raw);
        character_transform.translation = character_position;
        *character_ground = ground_reading;
    }
}
