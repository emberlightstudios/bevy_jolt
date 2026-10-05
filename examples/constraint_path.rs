//! Path (track) joint: a cart rides a looping oval Hermite spline, driven
//! by a velocity motor. Waypoints become knots via
//! `path_knots_from_waypoints` (central-difference tangents, up normals).

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltMotorDrive,
    JoltPlugin, JoltShape, JointMotor, JointSpace,
};

/// Oval waypoints (x/z plane) lifted to cart height: the Hermite builder
/// turns these into spline knots. `looping` closes the track.
const WAYPOINTS: [Vec3; 6] = [
    Vec3::new(-3.0, 3.5, 0.0),
    Vec3::new(-1.5, 3.5, 1.5),
    Vec3::new(1.5, 3.5, 1.5),
    Vec3::new(3.0, 3.5, 0.0),
    Vec3::new(1.5, 3.5, -1.5),
    Vec3::new(-1.5, 3.5, -1.5),
];
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PostUpdate, draw_track)
        .run();
}

#[derive(Resource)]
struct Demo {
    joint: Entity,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.5, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Dir3::Y),
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
        Text::new("Path Constraint"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    // Anchor defines the path-local frame but must not sit on the track:
    // the cart would collide with it at the seam. Park it inside the oval.
    let anchor = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 0.4, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(0.0, 3.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();
    let cart = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.7, 0.7, 0.7))),
            MeshMaterial3d(materials.add(Color::srgb(0.7, 0.3, 0.9))),
            Transform::from_translation(WAYPOINTS[0]),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.35)),
        ))
        .id();
    let joint = commands
        .spawn((
            JoltJoint::path_waypoints(
                anchor,
                cart,
                &WAYPOINTS,
                true,
                JointMotor {
                    frequency_hz: 8.0,
                    damping: 1.0,
                    force_limit: 1.0e6,
                },
                JointSpace::World,
            ),
            JoltMotorDrive(2.0),
        ))
        .id();
    commands.insert_resource(Demo { joint });
}

fn draw_track(
    demo: Res<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    for i in 0..WAYPOINTS.len() {
        let from = WAYPOINTS[i];
        let to = WAYPOINTS[(i + 1) % WAYPOINTS.len()];
        gizmos.line(from, to, Color::srgb(0.8, 0.5, 1.0));
    }
}
