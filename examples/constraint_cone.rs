//! Cone (shoulder) joint: the mount cube sways slowly side to side while the
//! ball hangs off it, swinging freely inside the cone. No kicks: the ball
//! starts displaced and gravity plus the swaying mount do the rest.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltBodyId, JoltDebugPlugin, JoltJoint, JoltJointId, JoltPhysicsWorld,
    JoltPlugin, JoltShape,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.5, 0.0);
const BALL_SPAWN: Vec3 = Vec3::new(0.8, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, sway_anchor.before(bevy_jolt::step_physics_world))
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    anchor: Entity,
    ball: Entity,
    joint: Entity,
    sway_ticks: u32,
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
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("cone (shoulder)"),
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
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();

    let dir = (ANCHOR - BALL_SPAWN).normalize();
    let ball = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.8, 0.6))),
            Transform::from_translation(BALL_SPAWN)
                .with_rotation(Quat::from_rotation_arc(dir, Vec3::Y)),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::sphere(0.3),
        ))
        .id();
    let joint = commands
        .spawn(JoltJoint::cone(
            anchor,
            ball,
            ANCHOR,
            Vec3::Y,
            dir,
            0.35,
        ))
        .id();
    commands.insert_resource(Demo {
        anchor,
        ball,
        joint,
        sway_ticks: 0,
    });
}

/// Slow twist: oscillating torque around the ball's own up winds and unwinds
/// the hanging ball.
fn sway_anchor(
    mut demo: ResMut<Demo>,
    body_query: Query<(&JoltBodyId, &Transform)>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    fixed_time: Res<Time<Fixed>>,
) {
    let Ok((ball_id, ball_transform)) = body_query.get(demo.ball) else {
        return;
    };
    demo.sway_ticks += 1;
    let elapsed = demo.sway_ticks as f32 * fixed_time.delta().as_secs_f32();
    let twist_torque = 2.5 * (0.942 * elapsed).cos();
    physics_world.apply_force(ball_id.body_id_raw, Vec3::ZERO, ball_transform.up() * twist_torque);
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
