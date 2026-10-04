//! Cone (shoulder) joint: the ball's spin is driven directly with an
//! oscillating angular velocity, swinging freely inside the cone. No kicks:
//! the joint plus the driven spin do the rest.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltAngularVelocity, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId,
    JoltPlugin, JoltShape, JointSpace,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.5, 0.0);
const BALL_SPAWN: Vec3 = Vec3::new(0.8, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(Update, sway_anchor)
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

    let dir = Dir3::new(ANCHOR - BALL_SPAWN).expect("cone anchor and ball must differ");
    let ball = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.8, 0.6))),
            Transform::from_translation(BALL_SPAWN)
                .with_rotation(Quat::from_rotation_arc(dir.as_vec3(), Vec3::Y)),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::sphere(0.3),
            JoltAngularVelocity {
                angular_velocity: Vec3::ZERO,
            },
        ))
        .id();
    let joint = commands
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
    commands.insert_resource(Demo {
        anchor,
        ball,
        joint,
        sway_ticks: 0,
    });
}

/// Slow twist: oscillating angular velocity around the ball's own up winds
/// and unwinds the hanging ball. Direct velocity drive (not torque): the
/// cone still caps the swing while the spin rate is exact.
fn sway_anchor(
    mut demo: ResMut<Demo>,
    mut body_query: Query<(&Transform, &mut JoltAngularVelocity)>,
    render_time: Res<Time>,
) {
    let Ok((ball_transform, mut spin)) = body_query.get_mut(demo.ball) else {
        return;
    };
    demo.sway_ticks += 1;
    // Fixed-hz phase would be nicer, but Update has no fixed delta: advance
    // by wall clock so the oscillation rate is framerate-independent.
    let elapsed = render_time.elapsed_secs();
    let twist_rate = 2.5 * (0.942 * elapsed).cos();
    spin.angular_velocity = ball_transform.up() * twist_rate;
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
