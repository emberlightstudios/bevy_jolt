use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_physics_scene)
        .run();
}

fn spawn_physics_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-6.0, 5.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::plane(Vec3::Y, 0.0),
    ));
    commands.spawn((
        Transform::from_xyz(-1.0, 3.0, 0.0),
        JoltBody::dynamic(CollisionLayers::MOVING),
        JoltShape::box_shape(Vec3::splat(0.5)),
    ));
    commands.spawn((
        Transform::from_xyz(1.0, 4.0, 0.0),
        JoltBody::dynamic(CollisionLayers::MOVING),
        JoltShape::capsule(0.5, 0.3),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 5.0, 0.0),
        JoltBody::dynamic(CollisionLayers::MOVING),
        JoltShape::sphere(0.5),
    ));
}
