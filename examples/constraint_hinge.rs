//! Hinge joint: a panel swings on a pin driven by a velocity motor.
//! The motor ping-pongs every 5s so the flap keeps swinging.

use bevy::prelude::*;
use bevy_jolt::{
JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltMotorDrive,
    JoltPlugin, JoltShape, JointSpace,
};

const HINGE_POINT: Vec3 = Vec3::new(0.0, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(PostUpdate, draw_pin)
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
        Transform::from_xyz(0.0, 3.5, 10.0).looking_at(Vec3::new(0.5, 2.5, 0.0), Dir3::Y),
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
        Text::new("hinge (flap)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let post = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 3.5, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.45, 0.45, 0.5))),
            Transform::from_xyz(0.0, 1.75, -0.6),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(0.2, 1.75, 0.2)),
        ))
        .id();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.1, 0.1, 0.6))),
        MeshMaterial3d(materials.add(Color::srgb(0.45, 0.45, 0.5))),
        Transform::from_xyz(0.0, 3.2, -0.3),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(0.05, 0.05, 0.3)),
    ));
    let panel = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.6, 2.4, 0.2))),
            MeshMaterial3d(materials.add(Color::srgb(0.6, 0.3, 0.8))),
            Transform::from_xyz(1.1, 3.2, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::new(0.8, 1.2, 0.1)),
        ))
        .id();
    let joint = commands
        .spawn((
            JoltJoint::hinge_limited(
                post,
                panel,
                HINGE_POINT,
                Dir3::Z,
                Dir3::X,
                Dir3::Z,
                Dir3::X,
                -1.2,
                1.2,
                JointSpace::World,
            ),
            JoltMotorDrive(1.2),
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
        drive.0 = -drive.0;
    }
}

fn draw_pin(
    demo: Res<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    gizmos.line(
        HINGE_POINT + Vec3::new(0.0, 0.0, -0.35),
        HINGE_POINT + Vec3::new(0.0, 0.0, 0.35),
        Color::WHITE,
    );
}
