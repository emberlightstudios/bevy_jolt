//! Swing-twist (arm) joint: a capsule hangs off a shoulder with separate
//! swing and twist limits. Re-kicked every 10s so the swing loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.2, 0.0);
const ARM_SPAWN: Vec3 = Vec3::new(0.6, 2.8, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, rekick_arm)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    shoulder: Entity,
    arm: Entity,
    joint: u32,
    kick_in: u32,
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
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
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

    let shoulder = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_translation(ANCHOR),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let arm = commands
        .spawn((
            Mesh3d(meshes.add(Capsule3d::new(0.2, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.85, 0.6, 0.2))),
            Transform::from_translation(ARM_SPAWN),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::capsule(0.5, 0.2),
        ))
        .id();
    commands.insert_resource(Demo {
        shoulder,
        arm,
        joint: 0,
        kick_in: 600,
    });
}

fn create_joint_once(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.joint != 0 {
        return;
    }
    let Ok(shoulder) = body_query.get(demo.shoulder) else {
        return;
    };
    let Ok(arm) = body_query.get(demo.arm) else {
        return;
    };
    let joint = physics_world.create_swing_twist_constraint(
        shoulder.body_id_raw,
        arm.body_id_raw,
        ANCHOR,
        Vec3::NEG_Y,
        Vec3::X,
        0.4,
        0.4,
        -0.5,
        0.5,
    );
    assert!(joint != 0, "swing-twist creation failed");
    demo.joint = joint;
    physics_world
        .kick_body(arm.body_id_raw, Vec3::new(3.0, 0.5, 1.5));
    println!("swing-twist joint id {joint}");
}

fn rekick_arm(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.joint == 0 {
        return;
    }
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        if let Ok(arm) = body_query.get(demo.arm) {
            physics_world
                .kick_body(arm.body_id_raw, Vec3::new(3.0, 0.5, 1.5));
        }
        demo.kick_in = 600;
        println!("re-kicked the arm");
    }
}

fn report(
    mut tick: Local<u32>,
    demo: Res<Demo>,
    body_query: Query<&JoltBodyId>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    if demo.joint == 0 {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let position = body_query
        .get(demo.arm)
        .map(|id| {
            physics_world
                .body_full_transform(id.body_id_raw)
                .0
        })
        .expect("arm should own a Jolt body");
    println!("tick {}: arm height {:.3}.", *tick, position.y);
    assert!(
        position.y > 0.3,
        "swing arm should not fall through the floor"
    );
}

fn draw_link(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    mut gizmos: Gizmos,
) {
    if demo.joint == 0 {
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
