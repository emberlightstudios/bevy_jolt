//! Pulley (elevator) joint: two weights share one rope, so one rises as the
//! other falls. Alternating kicks keep the elevator shuttling.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltImpulse, JoltJoint, JoltJointId, JoltPlugin,
    JoltShape, JointSpace,
};

const FIXED1: Vec3 = Vec3::new(-1.0, 6.0, 0.0);
const FIXED2: Vec3 = Vec3::new(1.0, 6.0, 0.0);
const BODY1_SPAWN: Vec3 = Vec3::new(-1.0, 4.0, 0.0);
const BODY2_SPAWN: Vec3 = Vec3::new(1.0, 4.0, 0.0);
const KICK_EVERY_N_TICKS: u32 = 200;
const KICK_IMPULSE: f32 = 800.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, kick_weights)
        .add_systems(PostUpdate, draw_ropes)
        .run();
}

#[derive(Resource)]
struct Demo {
    weight1: Entity,
    weight2: Entity,
    joint: Entity,
    kick_in: u32,
    pull_first_down: bool,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 11.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Dir3::Y),
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
        Text::new("pulley (elevator)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let weight1 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
            MeshMaterial3d(materials.add(Color::srgb(0.9, 0.4, 0.2))),
            Transform::from_translation(BODY1_SPAWN),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.3)),
        ))
        .id();
    let weight2 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.4, 0.9))),
            Transform::from_translation(BODY2_SPAWN),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.3)),
        ))
        .id();
    let joint = commands
        .spawn(JoltJoint::pulley(
            weight1,
            weight2,
            BODY1_SPAWN,
            FIXED1,
            BODY2_SPAWN,
            FIXED2,
            1.0,
            3.0,
            4.5,
            JointSpace::World,
        ))
        .id();
    commands.insert_resource(Demo {
        weight1,
        weight2,
        joint,
        kick_in: 60,
        pull_first_down: true,
    });
}

fn kick_weights(mut demo: ResMut<Demo>, joint_query: Query<(), With<JoltJointId>>, mut commands: Commands) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        let (target, push) = if demo.pull_first_down {
            (demo.weight1, Vec3::new(0.0, -KICK_IMPULSE, 0.0))
        } else {
            (demo.weight2, Vec3::new(0.0, -KICK_IMPULSE, 0.0))
        };
        commands.trigger(JoltImpulse::linear(target, push));
        demo.pull_first_down = !demo.pull_first_down;
        demo.kick_in = KICK_EVERY_N_TICKS;
    }
}


fn draw_ropes(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    let (Ok(weight1), Ok(weight2)) = (
        transform_query.get(demo.weight1),
        transform_query.get(demo.weight2),
    ) else {
        return;
    };
    gizmos.line(FIXED1, weight1.translation, Color::srgb(1.0, 0.6, 0.2));
    gizmos.line(FIXED2, weight2.translation, Color::srgb(0.4, 0.6, 1.0));
}
