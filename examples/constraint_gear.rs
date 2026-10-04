//! Gear joint: two discs hang off pivot posts, rotations coupled 2:1.
//! A disc gets spun every 5s; watch the other counter-rotate twice as fast.
//! Both discs also get a hinge each, which the gear needs as reference.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltJoint, JoltJointId, JoltPhysicsWorld,
    JoltPlugin, JoltShape,
    JointSpace,
};

const HINGE1: Vec3 = Vec3::new(-1.0, 3.5, 0.0);
const HINGE2: Vec3 = Vec3::new(1.0, 3.5, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, start_motor_once)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    disc1: Entity,
    disc2: Entity,
    hinge1: Entity,
    gear: Entity,
    started: bool,
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
        Text::new("gear (2:1 discs)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let post1 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.0, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(-1.0, 4.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.5, 0.2)),
        ))
        .id();
    let disc1 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.2, 1.2, 0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.2))),
            Transform::from_xyz(-1.0, 3.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.6, 0.6, 0.15)),
        ))
        .id();
    let post2 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.0, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(1.0, 4.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.5, 0.2)),
        ))
        .id();
    let disc2 = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.2, 1.2, 0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.8))),
            Transform::from_xyz(1.0, 3.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.6, 0.6, 0.15)),
        ))
        .id();
    let hinge1 = commands
        .spawn(JoltJoint::hinge(post1, disc1, HINGE1, Vec3::Z, Vec3::X, JointSpace::World))
        .id();
    let hinge2 = commands
        .spawn(JoltJoint::hinge(post2, disc2, HINGE2, Vec3::Z, Vec3::X, JointSpace::World))
        .id();
    let gear = commands
        .spawn(JoltJoint::gear(disc1, disc2, Vec3::Z, 2.0, hinge1, hinge2, JointSpace::World))
        .id();
    commands.insert_resource(Demo {
        disc1,
        disc2,
        hinge1,
        gear,
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
    let (Ok(hinge_id), Ok(gear_id)) = (
        joint_query.get(demo.hinge1),
        joint_query.get(demo.gear),
    ) else {
        return;
    };
    // Steady spin: the hinge motor drives disc1, the gear drags disc2 along.
    physics_world.constraint_drive_at(hinge_id.constraint_id_raw, 2.0);
    demo.started = true;
    println!("gear joint id {}", gear_id.constraint_id_raw);
}

fn report(
    mut tick: Local<u32>,
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
) {
    if !demo.started || joint_query.get(demo.gear).is_err() {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let (Ok(disc1), Ok(disc2)) = (
        transform_query.get(demo.disc1),
        transform_query.get(demo.disc2),
    ) else {
        return;
    };
    // Twist around Z shows the coupling: equal and opposite, scaled by ratio.
    let (axis, angle1) = disc1.rotation.to_axis_angle();
    let signed = angle1 * axis.z.signum();
    println!(
        "tick {}: discs at y={:.3}, y={:.3}, disc1 twist {:.2} rad.",
        *tick, disc1.translation.y, disc2.translation.y, signed
    );
    assert!(
        disc1.translation.y > 1.5 && disc2.translation.y > 1.5,
        "gear discs should hang on"
    );
}

fn draw_link(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    joint_query: Query<(), With<JoltJointId>>,
    mut gizmos: Gizmos,
) {
    if joint_query.get(demo.gear).is_err() {
        return;
    }
    let (Ok(disc1), Ok(disc2)) = (
        transform_query.get(demo.disc1),
        transform_query.get(demo.disc2),
    ) else {
        return;
    };
    gizmos.line(
        disc1.translation,
        disc2.translation,
        Color::srgb(1.0, 0.9, 0.3),
    );
}
