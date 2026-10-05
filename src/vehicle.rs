//! Wheeled-vehicle support: spawn spec, drive input, and sync.
//!
//! Game code spawns an entity with [`JoltVehicle`] (collision layer plus a
//! spawn `Transform`); the plugin creates the Jolt car body plus ray-cast
//! wheels once, stores the resulting [`JoltVehicleId`], and destroys both
//! when the entity leaves. Held throttle/steer/brake lives as
//! [`JoltVehicleDrive`] on the same entity: attach once, then change its
//! fields to drive. The sync system pushes it into Jolt before the physics
//! step, so gameplay never touches the world directly.

use bevy::prelude::*;

use crate::plugin::JoltPhysicsWorld;

/// Vehicle spec: which collision layer the chassis simulates on. Creation
/// reads the entity's `Transform` as the spawn pose and stores the resulting
/// [`JoltVehicleId`].
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicle {
    pub object_layer: u16,
}

impl JoltVehicle {
    pub fn new(object_layer: u16) -> Self {
        Self { object_layer }
    }
}

/// Jolt car body + vehicle constraint owned by an entity. Inserted by the
/// creation system; game code reads it but never writes it.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicleId {
    pub body_id_raw: u32,
    pub constraint_id_raw: u32,
}

/// Held vehicle input: throttle, steering, and brake, each roughly in
/// [-1, 1]. Pushed into Jolt every tick while present. Change the fields to
/// drive, remove the component to coast on the last input.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltVehicleDrive {
    /// Engine throttle: 1 full gas, -0.6 reverse-ish, 0 coast.
    pub forward: f32,
    /// Steering: -0.6 left, 0.6 right, 0 straight.
    pub steer: f32,
    /// Handbrake: 1 locked, 0 free.
    pub brake: f32,
}

/// Creates the Jolt car body plus wheels for each [`JoltVehicle`] missing a
/// [`JoltVehicleId`]. Polls instead of using `On<Add>`: like joints, creation
/// waits a flush and must run before the physics step so the car exists for
/// its first tick.
pub fn create_jolt_vehicles(
    mut commands: Commands,
    pending: Query<(Entity, &JoltVehicle, &Transform), Without<JoltVehicleId>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (vehicle_entity, vehicle, spawn_transform) in &pending {
        let Some((body_id_raw, constraint_id_raw)) = physics_world
            .create_demo_car(vehicle.object_layer, spawn_transform.translation)
        else {
            panic!(
                "Jolt rejected vehicle creation on {vehicle_entity:?}: {vehicle:?}"
            );
        };
        commands.entity(vehicle_entity).insert((
            JoltVehicleId {
                body_id_raw,
                constraint_id_raw,
            },
            // Joins the shared transform sync + debug draw: the chassis is a
            // 1.8 x 1.2 x 4.4 box matching the C++ shape, so the car entity
            // carries its own pose like any body.
            crate::body_sync::JoltBodyId { body_id_raw },
            crate::body_sync::PreviousBodyTransform {
                previous_position: spawn_transform.translation,
                previous_rotation: spawn_transform.rotation,
            },
        ));
        physics_world.register_shape(
            body_id_raw,
            crate::physics_world::PhysicsShape::Box {
                half_extents: bevy::prelude::Vec3::new(0.9, 0.6, 2.2),
            },
        );
    }
}

/// Pushes every [`JoltVehicleDrive`] into its vehicle constraint before the
/// physics step. Vehicles missing their id (not baked yet) are skipped for
/// the tick.
pub fn apply_jolt_vehicle_drives(
    drive_query: Query<(&JoltVehicleId, &JoltVehicleDrive)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (vehicle_id, drive) in &drive_query {
        physics_world.vehicle_drive(
            vehicle_id.constraint_id_raw,
            drive.forward,
            drive.steer,
            drive.brake,
        );
    }
}

/// Destroys the Jolt car body when its entity is despawned.
pub fn despawn_jolt_vehicle(
    trigger: On<Remove, JoltVehicleId>,
    vehicle_query: Query<&JoltVehicleId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(vehicle_id) = vehicle_query.get(trigger_entity) else {
        panic!("JoltVehicleId gone on {trigger_entity:?} before despawn ran");
    };
    physics_world.remove_constraint(vehicle_id.constraint_id_raw);
    physics_world.remove_and_destroy_body(vehicle_id.body_id_raw);
}
