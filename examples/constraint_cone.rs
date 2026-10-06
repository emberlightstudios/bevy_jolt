//! Cone Joint: the ball's spin is driven directly with an
//! angular velocity, swinging freely inside the cone.
use bevy::prelude::*;
use bevy_jolt::{
    JointSpace, JoltAngularVelocity, JoltBody, JoltDebugPlugin, JoltJoint, JoltPlugin, JoltShape,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.5, 0.0);
const BALL_SPAWN: Vec3 = Vec3::new(0.8, 3.2, 0.0);

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
    anchor: Entity,
    ball: Entity,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
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
        Text::new("Cone constraint"),
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
            Transform::from_translation(ANCHOR),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();

    let dir = Dir3::new(ANCHOR - BALL_SPAWN).expect("cone anchor and ball must differ");
    let ball = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.8, 0.6))),
            Transform::from_translation(BALL_SPAWN)
                .with_rotation(Quat::from_rotation_arc(dir.as_vec3(), Vec3::Y)),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.3),
            JoltAngularVelocity {
                angular_velocity: Vec3::ZERO,
            },
        ))
        .id();
    let _joint = commands
        .spawn(JoltJoint::cone(
            anchor,
            ball,
            ANCHOR,
            Dir3::Y,
            dir,
            0.35,
            JointSpace::World,
        ))
        .id();
    commands.insert_resource(Demo { anchor, ball });
}

fn draw_link(demo: Res<Demo>, transform_query: Query<&Transform>, mut gizmos: Gizmos) {
    let (Ok(anchor), Ok(ball)) = (
        transform_query.get(demo.anchor),
        transform_query.get(demo.ball),
    ) else {
        return;
    };
    gizmos.line(
        anchor.translation,
        ball.translation,
        Color::srgb(0.4, 1.0, 0.7),
    );
}
