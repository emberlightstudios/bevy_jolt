use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltDebugPlugin, JoltMotion, JoltPhysicsWorld, JoltPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_physics_scene)
        .run();
}

fn spawn_physics_scene(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
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

    physics_world
        .create_plane(Vec3::Y, 0.0, 50.0, CollisionLayers::NON_MOVING);
    physics_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(-1.0, 3.0, 0.0),
        CollisionLayers::MOVING,
        JoltMotion::Dynamic,
    );
    physics_world.create_capsule(
        0.5,
        0.3,
        Vec3::new(1.0, 4.0, 0.0),
        CollisionLayers::MOVING,
    );
    physics_world.create_sphere(
        0.5,
        Vec3::new(0.0, 5.0, 0.0),
        CollisionLayers::MOVING,
    );
}
