//! Swing-twist (shoulder) joint: a capsule arm hangs off a shoulder anchor
//! with separate swing and twist limits. Alternating sideways and twist
//! kicks show both freedoms: the arm sweeps its cone, then spins in place.

use bevy::prelude::*;
use bevy_jolt::{
JoltBody, JoltDebugPlugin, JoltImpulse, JoltJoint, JoltJointId, JoltPlugin,
    JoltShape, JointSpace,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.2, 0.0);
const ARM_SPAWN: Vec3 = Vec3::new(0.6, 2.8, 0.0);
const KICK_EVERY_N_TICKS: u32 = 200;
const SWING_KICK: Vec3 = Vec3::new(2000.0, 0.0, 800.0);
const TWIST_SPIN: Vec3 = Vec3::new(0.0, 60.0, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, kick_arm)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    shoulder: Entity,
    arm: Entity,
    joint: Entity,
    kick_in: u32,
    swing_turn: bool,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 3.5, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Dir3::Y),
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
        Text::new("swing-twist (arm)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    // Static anchor: every Jolt joint is a two-body constraint, so the arm
    // needs a partner even though only it moves. Keeps its cube so the
    // shoulder reads in the scene.
    let shoulder = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_translation(ANCHOR),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let arm = commands
        .spawn((
            //Mesh3d(meshes.add(Capsule3d::new(0.2, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.85, 0.6, 0.2))),
            Transform::from_translation(ARM_SPAWN),
            JoltBody::dynamic(0),
            JoltShape::capsule(0.5, 0.2),
        ))
        .id();
    // Body1 (anchor) defines the cone center: straight up, the middle of the
    // shoulder's range. Body2 (arm) carries its own twist axis along the limb,
    // so the creation pose sits off-center inside the cone: the arm starts
    // partway toward one wall, with more travel the other way. Same vector
    // on both sides would center rest; differing vectors offset it.
    let arm_axis = (ARM_SPAWN - ANCHOR).normalize();
    let plane_axis = arm_axis.cross(Vec3::Z).normalize();
    let joint = commands
        .spawn(JoltJoint::swing_twist(
            shoulder,
            arm,
            ANCHOR,
            Dir3::NEG_Y,
            Dir3::X,
            Dir3::new(arm_axis).unwrap(),
            Dir3::new(plane_axis).unwrap(),
            0.4,
            0.4,
            -0.5,
            0.5,
            JointSpace::World,
        ))
        .id();
    commands.insert_resource(Demo {
        shoulder,
        arm,
        joint,
        kick_in: 60,
        swing_turn: true,
    });
}

fn kick_arm(
    mut demo: ResMut<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut commands: Commands,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        // Swing shove and twist spin alternate: one sweeps the cone, the
        // next spins the arm in place, showing each limit in turn.
        if demo.swing_turn {
            commands.trigger(JoltImpulse::linear(demo.arm, SWING_KICK));
        } else {
            commands.trigger(JoltImpulse::swinging(demo.arm, SWING_KICK, TWIST_SPIN));
        }
        demo.swing_turn = !demo.swing_turn;
        demo.kick_in = KICK_EVERY_N_TICKS;
    }
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
    let (Ok(shoulder), Ok(arm)) = (
        transform_query.get(demo.shoulder),
        transform_query.get(demo.arm),
    ) else {
        return;
    };
    gizmos.line(
        shoulder.translation,
        arm.translation,
        Color::srgb(1.0, 0.7, 0.3),
    );
}
