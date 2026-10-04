//! Slider (piston) joint: a block shuttles along a vertical rail.
//! A velocity motor ping-pongs every 5s so motion loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_rail)
        .run();
}

#[derive(Resource)]
struct Demo {
    rail: Entity,
    block: Entity,
    joint: u32,
    flip_in: u32,
    forward: bool,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 10.0).looking_at(Vec3::new(0.0, 2.5, 0.0), Vec3::Y),
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
        Text::new("slider (piston)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let rail = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(0.0, 4.2, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let block = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.6, 0.6, 0.6))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.5, 0.9))),
            Transform::from_xyz(0.0, 2.6, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.3)),
        ))
        .id();
    commands.insert_resource(Demo {
        rail,
        block,
        joint: 0,
        flip_in: 300,
        forward: true,
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
    let Ok(rail) = body_query.get(demo.rail) else {
        return;
    };
    let Ok(block) = body_query.get(demo.block) else {
        return;
    };
    let joint = physics_world.create_slider_constraint(
        rail.body_id_raw,
        block.body_id_raw,
        Vec3::Y,
        Vec3::X,
        -1.5,
        0.5,
    );
    assert!(joint != 0, "slider creation failed");
    demo.joint = joint;
    physics_world.constraint_drive_at(joint, 1.5);
    println!("slider joint id {joint}");
}

fn pingpong_motor(mut demo: ResMut<Demo>, mut physics_world: ResMut<JoltPhysicsWorld>) {
    if demo.joint == 0 {
        return;
    }
    demo.flip_in = demo.flip_in.saturating_sub(1);
    if demo.flip_in == 0 {
        demo.forward = !demo.forward;
        demo.flip_in = 300;
        let speed = if demo.forward { 1.5 } else { -1.5 };
        physics_world
            .constraint_drive_at(demo.joint, speed);
        println!("slider reversed ({}).", if demo.forward { "up" } else { "down" });
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
        .get(demo.block)
        .map(|id| {
            physics_world
                .body_full_transform(id.body_id_raw)
                .0
        })
        .expect("block should own a Jolt body");
    println!("tick {}: piston height {:.3}.", *tick, position.y);
    assert!(
        (position.x - 0.0).abs() < 0.2,
        "piston block should stay on its rail"
    );
}

fn draw_rail(demo: Res<Demo>, mut gizmos: Gizmos) {
    if demo.joint == 0 {
        return;
    }
    gizmos.line(
        Vec3::new(0.0, 0.2, 0.0),
        Vec3::new(0.0, 4.2, 0.0),
        Color::srgb(0.4, 0.7, 1.0),
    );
}
