//! Gear Joint: two round discs hang off pivot posts, rotations coupled 2:1.
//! Disc1 is driven directly with an angular velocity; the gear drags disc2
//! along at twice the rate, counter-rotating. Both discs also get a hinge
//! each, which the gear needs as reference.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltAngularVelocity, JoltBody, JoltDebugPlugin, JoltJoint,
    JoltPlugin, JoltShape, JointSpace,
};

const DISC1_POS: Vec3 = Vec3::new(-0.65, 3.0, 0.0);
const DISC2_POS: Vec3 = Vec3::new(0.65, 3.0, 0.0);
const DISC1_RADIUS: f32 = 0.8;
const DISC2_RADIUS: f32 = 0.4;
const DRIVE_RATE: f32 = 2.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .run();
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
    let floor = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(200.0, 2.0, 200.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.35, 0.35, 0.38))),
            Transform::from_xyz(0.0, -1.0, 0.0),
            JoltBody::fixed(CollisionLayers::NON_MOVING),
            JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
        ))
        .id();
    commands.spawn((
        Text::new("Gear Constraint"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    // No posts: each disc hinges directly to the floor (the fixed frame),
    // so each hinge still has a static side and a spinning side. The visible
    // disc is a child mesh: body sync overwrites the body's own rotation
    // every tick, so a tip applied on the body would be wiped.
    let disc1_mesh = meshes.add(Cylinder::new(DISC1_RADIUS, 0.15));
    let disc1_face = materials.add(Color::srgb(0.8, 0.7, 0.2));
    let disc1 = commands
        .spawn((
            Transform::from_translation(DISC1_POS)
                .with_rotation(Quat::from_rotation_x(
                    core::f32::consts::FRAC_PI_2,
                )),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::sphere(DISC1_RADIUS),
            JoltAngularVelocity {
                angular_velocity: Vec3::Z * DRIVE_RATE,
            },
            children![(
                Mesh3d(disc1_mesh),
                MeshMaterial3d(disc1_face),
                Transform::from_rotation(Quat::from_rotation_x(
                    core::f32::consts::FRAC_PI_2,
                )),
            )]
        ))
        .id();
    let disc2_mesh = meshes.add(Cylinder::new(DISC2_RADIUS, 0.15));
    let disc2_face = materials.add(Color::srgb(0.2, 0.7, 0.8));
    let disc2 = commands
        .spawn((
            Transform::from_translation(DISC2_POS),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::sphere(DISC2_RADIUS),
            children![(
                Mesh3d(disc2_mesh),
                MeshMaterial3d(disc2_face),
                Transform::from_rotation(Quat::from_rotation_x(
                    core::f32::consts::FRAC_PI_2,
                )),
            )]
        ))
        .id();
    let hinge1 = commands
        .spawn(JoltJoint::hinge(
            floor,
            disc1,
            DISC1_POS,
            Dir3::Z,
            Dir3::X,
            JointSpace::World,
        ))
        .id();
    let hinge2 = commands
        .spawn(JoltJoint::hinge(
            floor,
            disc2,
            DISC2_POS,
            Dir3::Z,
            Dir3::X,
            JointSpace::World,
        ))
        .id();
    commands.spawn(JoltJoint::gear(
        disc1,
        disc2,
        Dir3::Z,
        2.0,
        hinge1,
        hinge2,
        JointSpace::World,
    ));
}