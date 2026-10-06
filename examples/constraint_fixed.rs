//! Fixed (weld) joint: two stacked boxes fall and rest as one rigid unit.
//! Every 10s the pair gets kicked again to prove the weld holds.

use bevy::prelude::*;
use bevy_jolt::{
    JointSpace, JoltBody, JoltDebugPlugin, JoltImpulse, JoltJoint, JoltPlugin, JoltShape,
};

const KICK_EVERY_N_TICKS: u32 = 300;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, rekick_pair)
        .run();
}

#[derive(Resource)]
struct Demo {
    bottom: Entity,
    kick_in: u32,
}
fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 9.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Dir3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(200.0, 2.0, 200.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.35, 0.35, 0.38))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("Fixed Constraint"),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let bottom = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.3))),
            Transform::from_xyz(0.0, 1.5, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    let top = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.7))),
            Transform::from_xyz(0.0, 2.6, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    commands.spawn(JoltJoint::fixed(bottom, top, JointSpace::World));
    commands.insert_resource(Demo {
        bottom,
        kick_in: KICK_EVERY_N_TICKS,
    });
}

fn rekick_pair(mut demo: ResMut<Demo>, mut commands: Commands) {
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        commands.trigger(JoltImpulse::linear(
            demo.bottom,
            Vec3::new(0.0, 2000.0, 0.0),
        ));
        demo.kick_in = KICK_EVERY_N_TICKS;
        println!("kicked the bottom object");
    }
}
