//! Pulley (elevator) joint: two weights share one rope, so one rises as the
//! other falls. Every 15s the first weight gets hoisted back up to loop.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltBodyId, JoltDebugPlugin, JoltJoint, JoltJointId,
    JoltPhysicsWorld, JoltPlugin, JoltShape,
};

const FIXED1: Vec3 = Vec3::new(-1.0, 6.0, 0.0);
const FIXED2: Vec3 = Vec3::new(1.0, 6.0, 0.0);
const BODY1_SPAWN: Vec3 = Vec3::new(-1.0, 4.0, 0.0);
const BODY2_SPAWN: Vec3 = Vec3::new(1.0, 4.0, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(Startup, kick_weight_once)
        .add_systems(FixedUpdate, (apply_kick_once, hoist_weight).chain())
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_ropes)
        .run();
}

#[derive(Resource)]
struct Demo {
    weight1: Entity,
    weight2: Entity,
    joint: Entity,
    hoist_in: u32,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 11.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
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
        ))
        .id();
    commands.insert_resource(Demo {
        weight1,
        weight2,
        joint,
        hoist_in: 900,
    });
}

fn kick_weight_once(demo: Res<Demo>, mut commands: Commands) {
    commands.entity(demo.weight2).insert(KickOnce);
}

#[derive(Component)]
struct KickOnce;

fn apply_kick_once(
    mut commands: Commands,
    kick_query: Query<(Entity, &JoltBodyId), With<KickOnce>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    for (kick_entity, body_id) in &kick_query {
        // Unbalance the pair so the elevator starts moving immediately.
        physics_world.kick_body(body_id.body_id_raw, Vec3::new(0.0, -2.0, 0.0));
        commands.entity(kick_entity).remove::<KickOnce>();
    }
}

fn hoist_weight(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    joint_query: Query<(), With<JoltJointId>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    demo.hoist_in = demo.hoist_in.saturating_sub(1);
    if demo.hoist_in == 0 {
        if let Ok(weight1) = body_query.get(demo.weight1) {
            physics_world.reset_body_to(weight1.body_id_raw, BODY1_SPAWN);
        }
        demo.hoist_in = 900;
        println!("hoisted the elevator weight");
    }
}

fn report(
    mut tick: Local<u32>,
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let (Ok(weight1), Ok(weight2)) = (
        transform_query.get(demo.weight1),
        transform_query.get(demo.weight2),
    ) else {
        return;
    };
    let total_rope = (weight1.translation - FIXED1).length()
        + (weight2.translation - FIXED2).length();
    println!("tick {}: elevator rope {:.3}.", *tick, total_rope);
    assert!(
        (2.8..4.7).contains(&total_rope),
        "pulley rope should stay in its band"
    );
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
