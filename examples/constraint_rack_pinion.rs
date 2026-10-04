//! Rack-and-pinion (jack) joint: a spinning pinion drives a sliding rack.
//! Gravity drags the rack down so the pinion keeps turning.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltPlugin, JoltShape,
    JointSpace,
};

const PINION_HINGE: Vec3 = Vec3::new(-1.0, 4.0, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    rack: Entity,
    joint: Entity,
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
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("rack-and-pinion (jack)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let post = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.0, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(-1.0, 5.0, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.5, 0.2)),
        ))
        .id();
    let pinion = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.6, 0.2))),
            Transform::from_xyz(-1.0, 3.5, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.5, 0.5, 0.15)),
        ))
        .id();
    let rail = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(1.0, 4.2, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let rack = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.6, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.6, 0.9))),
            Transform::from_xyz(1.0, 2.6, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.8, 0.2)),
        ))
        .id();
    let hinge = commands
        .spawn(JoltJoint::hinge(post, pinion, PINION_HINGE, Dir3::Z, Dir3::X, JointSpace::World))
        .id();
    let slider = commands
        .spawn(JoltJoint::slider(rail, rack, Dir3::Y, Dir3::X, -1.5, 0.5, JointSpace::World))
        .id();
    let joint = commands
        .spawn(JoltJoint::rack_pinion(
            pinion, rack, Dir3::Z, Dir3::Y, 1.0, hinge, slider, JointSpace::World,
        ))
        .id();
    commands.insert_resource(Demo { rack, joint });
}

fn draw_link(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    let Ok(rack) = transform_query.get(demo.rack) else {
        return;
    };
    gizmos.line(
        rack.translation + Vec3::new(0.0, 0.8, 0.0),
        rack.translation - Vec3::new(0.0, 0.8, 0.0),
        Color::srgb(0.6, 0.9, 1.0),
    );
}
