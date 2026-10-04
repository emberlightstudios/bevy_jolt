use bevy::prelude::*;
use bevy_jolt::{JoltDebugDraw, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin, OBJECT_LAYER_MOVING};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin)
        .add_plugins(JoltDebugPlugin)
        .insert_resource(JoltDebugDraw)
        .add_systems(Startup, spawn_physics_scene)
        .add_systems(Update, step_physics_world)
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
        .physics_world
        .create_plane(Vec3::Y, 0.0, 50.0, OBJECT_LAYER_MOVING);
    physics_world.physics_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(-1.0, 3.0, 0.0),
        OBJECT_LAYER_MOVING,
        false,
    );
    physics_world.physics_world.create_capsule(
        0.5,
        0.3,
        Vec3::new(1.0, 4.0, 0.0),
        OBJECT_LAYER_MOVING,
    );
    physics_world
        .physics_world
        .create_sphere(0.5, 5.0);
}

fn step_physics_world(mut physics_world: ResMut<JoltPhysicsWorld>) {
    physics_world.physics_world.update(1.0 / 60.0, 1);
}
