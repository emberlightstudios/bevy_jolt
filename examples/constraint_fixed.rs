//! Fixed (weld) joint: two stacked boxes fall and rest as one rigid unit.
//! Every 10s the pair gets kicked again to prove the weld holds.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltBodyId, JoltDebugPlugin, JoltJoint, JoltPhysicsWorld,
    JoltPlugin, JoltShape,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, rekick_pair)
        .add_systems(FixedPostUpdate, report)
        .run();
}

#[derive(Resource)]
struct Demo {
    bottom: Entity,
    top: Entity,
    kick_in: u32,
}
fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 9.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
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
        Text::new("fixed (weld)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let bottom = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.3))),
            Transform::from_xyz(0.0, 1.5, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    let top = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.6, 0.7))),
            Transform::from_xyz(0.0, 2.6, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    commands.spawn(JoltJoint::fixed(bottom, top));
    commands.insert_resource(Demo {
        bottom,
        top,
        kick_in: 600,
    });
}

fn rekick_pair(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        if let Ok(bottom) = body_query.get(demo.bottom) {
            physics_world
                .kick_body(bottom.body_id_raw, Vec3::new(2.0, 3.0, 0.5));
        }
        demo.kick_in = 600;
        println!("kicked the welded pair");
    }
}

fn report(
    mut tick: Local<u32>,
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
) {
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let (Ok(bottom), Ok(top)) = (
        transform_query.get(demo.bottom),
        transform_query.get(demo.top),
    ) else {
        return;
    };
    let gap = (top.translation - bottom.translation).length();
    println!("tick {}: weld gap {:.3}.", *tick, gap);
    assert!(
        (gap - 1.1).abs() < 0.15,
        "welded boxes should keep their spacing"
    );
}
