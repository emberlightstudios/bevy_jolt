//! Bevy schedule wiring for the Jolt physics world.
//!
//! Two fixed schedules, one rule: everything in [`FixedUpdate`] runs before
//! the sim steps, everything in [`JoltStep`] runs after. Game code writes
//! drives in `FixedUpdate` and reads results in `FixedPostUpdate` (or later)
//! without ever wondering whether the sim has run: the schedules themselves
//! run in that order, so no per-system ordering can get it wrong.

use bevy::app::FixedMainScheduleOrder;
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;
use crate::physics_world::{CollisionLayers, JoltWorld};

/// Step schedule: runs once per fixed tick, right after [`FixedUpdate`] and
/// before [`FixedPostUpdate`]. Owns the sim step plus every readback (pose
/// sync, measured velocities, contact drain). The plugin inserts it into the
/// fixed loop; game code never schedules here, it only relies on the rule
/// that by `FixedPostUpdate` the sim has stepped.
#[derive(ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
pub struct JoltStep;

/// Collision layer table applied when the physics world is created.
/// Inserted by [`JoltPlugin`]; read it to inspect the active teams.
#[derive(Resource, Clone, Debug, Default)]
pub struct JoltCollisionLayers {
    pub collision_layers: CollisionLayers,
}

 /// Startup gravity carried from the [`JoltPlugin`] builder to world
#[derive(Resource, Clone, Copy, Debug)]
pub struct JoltStartupGravity {
    pub world_gravity: Vec3,
}

/// Bevy resource owning the Jolt physics world. Derefs to [`JoltWorld`] so
/// systems call `physics_world.create_box(...)` directly; the field stays
/// for the rare case the resource wrapper itself matters.
#[derive(Resource, Deref, DerefMut)]
pub struct JoltPhysicsWorld {
    #[deref]
    pub physics_world: JoltWorld,
}
impl FromWorld for JoltPhysicsWorld {
    fn from_world(world: &mut World) -> Self {
        let collision_layers = world
            .get_resource::<JoltCollisionLayers>()
            .map(|layers_resource| layers_resource.collision_layers.clone())
            .unwrap_or_default();
        let mut physics_world = JoltWorld::with_layers(collision_layers);
        // The plugin builder owns the startup gravity: Jolt itself always
        // starts at Earth-like (0, -9.81, 0), so overwrite with whatever the
        // builder carries (default = same value, no visible change).
        let startup_gravity = world
            .get_resource::<JoltStartupGravity>()
            .map(|gravity_resource| gravity_resource.world_gravity)
            .unwrap_or(JoltPlugin::DEFAULT_GRAVITY);
        physics_world.set_world_gravity(startup_gravity);
        Self { physics_world }
    }
}

/// Adds the Jolt physics world resource to the app.
/// Carries the collision layer table and the fixed-step rate so game code
/// never inserts resources or steps the world manually.
pub struct JoltPlugin {
    collision_layers: CollisionLayers,
    physics_hz: f64,
    max_sub_steps: u32,
    world_gravity: Vec3,
}

impl JoltPlugin {
    /// Physics tick rate in Hertz. Defaults to 60.
    pub const DEFAULT_PHYSICS_HZ: f64 = 60.0;

    /// collision_steps handed to Jolt per Fixed tick. Defaults to 1: one
    /// Jolt collision step per tick. Raise for fast bodies at low tick rates.
    pub const DEFAULT_MAX_SUB_STEPS: u32 = 1;

    /// World gravity every body feels, scaled per body by its gravity
    /// factor. Jolt default: Earth-like (0, -9.81, 0).
    pub const DEFAULT_GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

    pub fn new() -> Self {
        Self {
            collision_layers: CollisionLayers::default(),
            physics_hz: Self::DEFAULT_PHYSICS_HZ,
            max_sub_steps: Self::DEFAULT_MAX_SUB_STEPS,
            world_gravity: Self::DEFAULT_GRAVITY,
        }
    }

    pub fn with_collision_layers(mut self, collision_layers: CollisionLayers) -> Self {
        self.collision_layers = collision_layers;
        self
    }

    /// Physics ticks per second. Must be positive and finite.
    pub fn with_physics_hz(mut self, physics_hz: f64) -> Self {
        assert!(
            physics_hz.is_finite() && physics_hz > 0.0,
            "physics rate must be positive, got {}",
            physics_hz
        );
        self.physics_hz = physics_hz;
        self
    }

    /// Jolt collision steps per Fixed tick. Must be at least 1.
    pub fn with_max_sub_steps(mut self, max_sub_steps: u32) -> Self {
        assert!(max_sub_steps >= 1, "need at least 1 collision step");
        self.max_sub_steps = max_sub_steps;
        self
    }

    /// World gravity every body feels, scaled per body by its gravity
    /// factor. Must be finite (zero vector allowed: no pull anywhere).
    pub fn with_world_gravity(mut self, world_gravity: Vec3) -> Self {
        assert!(
            world_gravity.is_finite(),
            "gravity must be finite, got {}",
            world_gravity
        );
        self.world_gravity = world_gravity;
        self
    }
}

impl Default for JoltPlugin {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-step rate and collision steps, inserted by [`JoltPlugin`].
#[derive(Resource, Clone, Copy, Debug)]
pub struct JoltStepConfig {
    pub physics_hz: f64,
    pub max_sub_steps: u32,
}

impl Plugin for JoltPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(JoltCollisionLayers {
            collision_layers: self.collision_layers.clone(),
        });
        // Before the world resource: FromWorld reads this for startup gravity.
        app.insert_resource(JoltStartupGravity {
            world_gravity: self.world_gravity,
        });
        app.init_resource::<JoltPhysicsWorld>();
        app.insert_resource(Time::<Fixed>::from_hz(self.physics_hz));
        app.insert_resource(JoltStepConfig {
            physics_hz: self.physics_hz,
            max_sub_steps: self.max_sub_steps,
        });
        app.add_observer(crate::body_sync::spawn_jolt_body);
        app.add_observer(crate::character::bake_jolt_character);
        app.add_observer(crate::character::bake_jolt_rigid_character);
        app.add_observer(crate::soft_body::bake_jolt_soft_body);
        app.add_observer(crate::ragdoll::bake_jolt_ragdoll);
        app.add_observer(crate::body_forces::apply_jolt_impulse);
        app.add_observer(crate::body_forces::apply_jolt_teleport);
        app.add_observer(crate::character::apply_jolt_character_teleport);
        app.add_observer(crate::character::apply_jolt_rigid_character_impulse);
        app.add_observer(crate::character::apply_jolt_rigid_character_push);
        app.add_observer(crate::character::apply_jolt_rigid_character_teleport);
        app.add_observer(crate::contact_events::bake_jolt_sensor);
        app.add_observer(crate::joint_sync::despawn_jolt_joint);
        // Joint cascade before body destroy: constraint removals here are
        // synchronous, so the solver never sees a constraint on a dead body.
        app.add_observer(crate::joint_sync::cascade_body_remove_to_joints);
        app.add_observer(crate::body_sync::despawn_jolt_body);
        app.add_observer(crate::character::despawn_jolt_character);
        app.add_observer(crate::character::despawn_jolt_rigid_character);
        app.add_observer(crate::vehicle::despawn_jolt_vehicle);
        app.add_observer(crate::soft_body::despawn_jolt_soft_body);
        // Pre-step bakes: joints/vehicles/ragdoll links resolve before any
        // drive writes. Everything in FixedUpdate precedes the JoltStep
        // schedule by construction, so no .before() needed.
        app.add_systems(
            FixedUpdate,
            (
                crate::ragdoll::bake_ragdoll_links,
                crate::joint_sync::create_jolt_joints,
                crate::vehicle::create_jolt_vehicles,
            ),
        );
        // Pre-step drives: forces, kinematic targets, buoyancy, characters,
        // sensors. One-shot impulses and teleports need no scheduling: their
        // observers fire the moment the trigger fires.
        app.add_systems(
            FixedUpdate,
            (
                crate::body_forces::apply_jolt_forces,
                crate::body_forces::apply_jolt_kinematic_targets,
                crate::buoyancy::apply_buoyancy,
                crate::character::step_jolt_characters,
                crate::character::step_jolt_rigid_characters,
                crate::contact_events::apply_pending_sensors,
                crate::joint_sync::apply_jolt_motor_drives,
                crate::vehicle::apply_jolt_vehicle_drives,
            ),
        );
        // The step runs in its own schedule after FixedUpdate: by the time
        // FixedPostUpdate runs, the sim has always stepped.
        app.init_schedule(JoltStep);
        app.world_mut()
            .resource_mut::<FixedMainScheduleOrder>()
            .insert_after(FixedUpdate, JoltStep);
        app.add_systems(JoltStep, step_physics_world);
        // Pre-step pushes read `Changed` components, so they run first in
        // `JoltStep`: every `FixedUpdate` write has landed by then, no matter
        // what order game code ran in.
        app.add_systems(
            JoltStep,
            (
                crate::body_forces::sync_jolt_motion,
                crate::body_forces::sync_jolt_sleep,
                crate::body_forces::apply_jolt_driven_velocities,
            )
                .before(step_physics_world),
        );
        // Post-step readbacks share the schedule, ordered after the step.
        app.add_systems(
            JoltStep,
            (
                crate::body_sync::sync_body_transforms,
                crate::character::sync_character_transforms,
                crate::character::sync_rigid_character_transforms,
                crate::soft_body::sync_soft_body_meshes,
                crate::contact_events::drain_contact_events,
                crate::body_forces::sync_sleep_states,
            )
                .after(step_physics_world),
        );
        app.add_systems(Update, crate::body_sync::interpolate_body_transforms);
    }
}

/// Advances the Jolt world once per Fixed tick using the tick's delta.
pub fn step_physics_world(
    mut physics_world: ResMut<JoltPhysicsWorld>,
    step_config: Res<JoltStepConfig>,
    fixed_time: Res<Time<Fixed>>,
) {
    physics_world.update(
        fixed_time.delta().as_secs_f32(),
        step_config.max_sub_steps as i32,
    );
}
