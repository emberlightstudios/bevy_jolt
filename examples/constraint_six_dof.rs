//! Six-DOF joint used as a piston: every axis fixed except free Y travel.
//! This rebuilds what the dedicated slider joint does, proving the general
//! joint covers it. Prefer [`JoltJoint::slider`] for real pistons: one axis,
//! no frame/limit boilerplate. A held [`JoltMotorDrive`] ping-pongs every
//! ~3.3s so motion loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JointMotor, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId,
    JoltMotorDrive, JoltPlugin, JoltShape, JointSpace, SixDofAxis, SixDofFrame, SixDofLimits,
};
/// Fixed axis band: min > max pins the axis at zero (Jolt convention).
const FIXED: f32 = 1.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(PostUpdate, draw_rail)
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
        Transform::from_xyz(3.0, 3.0, 10.0).looking_at(Vec3::new(0.0, 2.5, 0.0), Dir3::Y),
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
        Text::new("six-dof (as slider)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    // Static anchor: every Jolt joint is a two-body constraint, so the slide
    // needs a partner even though nothing moves but the block.
    let rail = commands
        .spawn((
            Transform::from_xyz(0.0, 4.2, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let block = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.5, 0.9))),
            Transform::from_xyz(0.0, 2.6, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.3)),
        ))
        .id();
    // Same piston as the slider example, spelled out in full: identity
    // frames on both bodies, Y limited to the travel band, everything else
    // fixed, motor armed on Y only. Compare with the four-arg slider call.
    let joint = commands
        .spawn((
            JoltJoint::six_dof(
                rail,
                block,
                SixDofFrame {
                    position1: Vec3::new(0.0, 2.6, 0.0),
                    axis_x1: Dir3::X,
                    axis_y1: Dir3::Y,
                    position2: Vec3::new(0.0, 2.6, 0.0),
                    axis_x2: Dir3::X,
                    axis_y2: Dir3::Y,
                },
                SixDofLimits {
                    translation_min: Vec3::new(-FIXED, -1.5, -FIXED),
                    translation_max: Vec3::new(FIXED, 0.5, FIXED),
                    rotation_min: Vec3::new(-FIXED, -FIXED, -FIXED),
                    rotation_max: Vec3::new(FIXED, FIXED, FIXED),
                },
                SixDofAxis::TranslationY.bit(),
                JointMotor::default(),
                JointSpace::World,
            ),
            JoltMotorDrive {
                target_velocity: 3.0,
            },
        ))
        .id();
    commands.insert_resource(Demo { joint, flip_in: 200 });
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
        demo.flip_in = 200;
        drive.target_velocity = -drive.target_velocity;
    }
}

fn draw_rail(
    demo: Res<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    // Legal travel: spawn pose plus the Y limits along the slide axis.
    gizmos.line(
        Vec3::new(0.0, 2.6 - 1.5, 0.0),
        Vec3::new(0.0, 2.6 + 0.5, 0.0),
        Color::srgb(0.4, 0.7, 1.0),
    );
}
