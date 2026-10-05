//! Vehicle: a default `VehicleSpec` car with WASD driving (W gas,
//! S brake/reverse, A/D steer). No keys: sits still.

use bevy::prelude::*;
use bevy_jolt::{
JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape, JoltVehicleDrive,
    VehicleSpec,
};

const CAR_SPAWN: Vec3 = Vec3::new(0.0, 1.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, drive_car)
        .run();
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 18.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(300.0, 2.0, 300.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.35, 0.35, 0.38))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(150.0, 1.0, 150.0)),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.8, 1.2, 4.4))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.15, 0.2))),
        Transform::from_translation(CAR_SPAWN),
        VehicleSpec::new(0),
        JoltVehicleDrive {
            forward: 0.0,
            steer: 0.0,
            brake: 0.0,
            hand_brake: 0.0,
        },
    ));
}

/// WASD driving: W gas, S brake/reverse, A/D steer. Car sits still
/// with no keys held.
fn drive_car(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut drive_query: Query<&mut JoltVehicleDrive>,
) {
    for mut drive in &mut drive_query {
        drive.forward = if keyboard.pressed(KeyCode::KeyW) {
            1.0
        } else if keyboard.pressed(KeyCode::KeyS) {
            -0.6
        } else {
            0.0
        };
        drive.steer = if keyboard.pressed(KeyCode::KeyA) {
            -0.6
        } else if keyboard.pressed(KeyCode::KeyD) {
            0.6
        } else {
            0.0
        };
    }
}
