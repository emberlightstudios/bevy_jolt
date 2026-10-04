//! Hinge joint: a panel swings on a pin driven by a velocity motor.
//! The motor ping-pongs every 5s so the flap keeps swinging.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltPhysicsWorld,
    JoltPlugin, JoltShape,
};

const HINGE_POINT: Vec3 = Vec3::new(0.0, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, (start_motor_once, pingpong_motor).chain())
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_pin)
        .run();
}

#[derive(Resource)]
struct Demo {
    panel: Entity,
    joint: Entity,
    flip_in: u32,
    forward: bool,
    started: bool,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 10.0).looking_at(Vec3::new(0.5, 2.5, 0.0), Vec3::Y),
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
        Text::new("hinge (flap)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let post = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 3.5, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.45, 0.45, 0.5))),
            Transform::from_xyz(0.0, 1.75, -0.6),
            JoltBody::fixed(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::new(0.2, 1.75, 0.2)),
        ))
        .id();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.1, 0.1, 0.6))),
        MeshMaterial3d(materials.add(Color::srgb(0.45, 0.45, 0.5))),
        Transform::from_xyz(0.0, 3.2, -0.3),
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::box_shape(Vec3::new(0.05, 0.05, 0.3)),
    ));
    let panel = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.6, 2.4, 0.2))),
            MeshMaterial3d(materials.add(Color::srgb(0.6, 0.3, 0.8))),
            Transform::from_xyz(1.1, 3.2, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::new(0.8, 1.2, 0.1)),
        ))
        .id();
    let joint = commands
        .spawn(JoltJoint::hinge(post, panel, HINGE_POINT, Vec3::Z, Vec3::X))
        .id();
    commands.insert_resource(Demo {
        panel,
        joint,
        flip_in: 300,
        forward: true,
        started: false,
    });
}

fn start_motor_once(
    mut demo: ResMut<Demo>,
    joint_query: Query<&JoltJointId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.started {
        return;
    }
    let Ok(joint_id) = joint_query.get(demo.joint) else {
        return;
    };
    physics_world.constraint_drive_at(joint_id.constraint_id_raw, 1.2);
    demo.started = true;
    println!("hinge joint id {}", joint_id.constraint_id_raw);
}

fn pingpong_motor(
    mut demo: ResMut<Demo>,
    joint_query: Query<&JoltJointId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if !demo.started {
        return;
    }
    let Ok(joint_id) = joint_query.get(demo.joint) else {
        return;
    };
    demo.flip_in = demo.flip_in.saturating_sub(1);
    if demo.flip_in == 0 {
        demo.forward = !demo.forward;
        demo.flip_in = 300;
        let speed = if demo.forward { 1.2 } else { -1.2 };
        physics_world.constraint_drive_at(joint_id.constraint_id_raw, speed);
        println!(
            "hinge reversed ({}).",
            if demo.forward { "forward" } else { "back" }
        );
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
    let Ok(panel) = transform_query.get(demo.panel) else {
        return;
    };
    let position = panel.translation;
    let radius = (position - HINGE_POINT).length();
    println!("tick {}: hinge radius {:.3}.", *tick, radius);
    assert!(
        (radius - 1.1).abs() < 0.2,
        "hinged panel should stay on its pin"
    );
    assert!(
        position.y > 0.3,
        "hinged panel should not fall through the floor"
    );
}

fn draw_pin(
    demo: Res<Demo>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.joint).is_err() {
        return;
    }
    gizmos.line(
        HINGE_POINT + Vec3::new(0.0, 0.0, -0.35),
        HINGE_POINT + Vec3::new(0.0, 0.0, 0.35),
        Color::WHITE,
    );
}
