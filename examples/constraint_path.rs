//! Path (track) joint: a cart glues to a straight track and shuttles along
//! it. A velocity motor ping-pongs every 5s so the cart loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltMotorDrive,
    JoltPlugin, JoltShape, JointSpace,
};

const TRACK_FROM: Vec3 = Vec3::new(-1.5, 3.5, 0.0);
const TRACK_TO: Vec3 = Vec3::new(1.5, 3.5, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(PostUpdate, draw_track)
        .run();
}

#[derive(Resource)]
struct Demo {
    joint: Entity,
    flip_in: u32,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Dir3::Y),
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
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("path (track cart)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let anchor = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 0.4, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(-1.5, 4.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();
    let cart = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.7, 0.7, 0.7))),
            MeshMaterial3d(materials.add(Color::srgb(0.7, 0.3, 0.9))),
            Transform::from_translation(TRACK_FROM),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.35)),
        ))
        .id();
    let joint = commands
        .spawn((
            JoltJoint::path_shuttle(anchor, cart, TRACK_FROM, TRACK_TO, JointSpace::World),
            JoltMotorDrive {
                target_velocity: 1.0,
            },
        ))
        .id();
    commands.insert_resource(Demo { joint, flip_in: 300 });
}

fn pingpong_motor(
    mut demo: ResMut<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut drive_query: Query<&mut JoltMotorDrive>,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    let Ok(mut drive) = drive_query.get_mut(demo.joint) else {
        return;
    };
    demo.flip_in = demo.flip_in.saturating_sub(1);
    if demo.flip_in == 0 {
        demo.flip_in = 300;
        drive.target_velocity = -drive.target_velocity;
    }
}

fn draw_track(
    demo: Res<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    gizmos.line(TRACK_FROM, TRACK_TO, Color::srgb(0.8, 0.5, 1.0));
}
