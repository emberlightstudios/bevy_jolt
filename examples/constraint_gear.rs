//! Gear joint: two discs hang off pivot posts, rotations coupled 2:1.
//! A disc gets spun every 5s; watch the other counter-rotate twice as fast.
//! Both discs also get a hinge each, which the gear needs as reference.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const HINGE1: Vec3 = Vec3::new(-1.0, 3.5, 0.0);
const HINGE2: Vec3 = Vec3::new(1.0, 3.5, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    post1: Entity,
    disc1: Entity,
    post2: Entity,
    disc2: Entity,
    gear: u32,
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
    commands.insert_resource(Demo {
        post1,
        disc1,
        post2,
        disc2,
        gear: 0,
    });
}

fn create_joint_once(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.gear != 0 {
        return;
    }
    let Ok(post1) = body_query.get(demo.post1) else {
        return;
    };
    let Ok(disc1) = body_query.get(demo.disc1) else {
        return;
    };
    let Ok(post2) = body_query.get(demo.post2) else {
        return;
    };
    let Ok(disc2) = body_query.get(demo.disc2) else {
        return;
    };
    let world = &mut **physics_world;
    let hinge1 = world.create_hinge_constraint(post1.body_id_raw, disc1.body_id_raw, HINGE1, Vec3::Z, Vec3::X);
    let hinge2 = world.create_hinge_constraint(post2.body_id_raw, disc2.body_id_raw, HINGE2, Vec3::Z, Vec3::X);
    assert!(hinge1 != 0 && hinge2 != 0, "gear hinge failed");
    let gear = world.create_gear_constraint(
        disc1.body_id_raw,
        disc2.body_id_raw,
        Vec3::Z,
        2.0,
        hinge1,
        hinge2,
    );
    assert!(gear != 0, "gear creation failed");
    demo.gear = gear;
    // Steady spin: the hinge motor drives disc1, the gear drags disc2 along.
    world.constraint_drive_at(hinge1, 2.0);
    println!("gear joint id {gear} (hinges {hinge1}, {hinge2})");
}

fn report(
    mut tick: Local<u32>,
    demo: Res<Demo>,
    body_query: Query<&JoltBodyId>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    if demo.gear == 0 {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let position = |entity: Entity| {
        body_query
            .get(entity)
            .map(|id| {
                physics_world
                    .body_full_transform(id.body_id_raw)
            })
            .expect("demo entity should own a Jolt body")
    };
    let (pos1, rot1) = position(demo.disc1);
    let (pos2, _) = position(demo.disc2);
    // Twist around Z shows the coupling: equal and opposite, scaled by ratio.
    let (axis, angle1) = rot1.to_axis_angle();
    let signed = angle1 * axis.z.signum();
    println!(
        "tick {}: discs at y={:.3}, y={:.3}, disc1 twist {:.2} rad.",
        *tick, pos1.y, pos2.y, signed
    );
    assert!(pos1.y > 1.5 && pos2.y > 1.5, "gear discs should hang on");
}

fn draw_link(
    demo: Res<Demo>,
    transform_query: Query<&Transform>,
    mut gizmos: Gizmos,
) {
    if demo.gear == 0 {
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
