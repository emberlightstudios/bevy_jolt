//! Hinge joint: a panel swings on a pin driven by a velocity motor.
//! The motor ping-pongs every 5s so the flap keeps swinging.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const HINGE_POINT: Vec3 = Vec3::new(0.0, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_pin)
        .run();
}

#[derive(Resource)]
struct Demo {
    post: Entity,
    panel: Entity,
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
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
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
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 1.75, 0.2)),
        ))
        .id();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.1, 0.1, 0.6))),
        MeshMaterial3d(materials.add(Color::srgb(0.45, 0.45, 0.5))),
        Transform::from_xyz(0.0, 3.2, -0.3),
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(0.05, 0.05, 0.3)),
    ));
    let panel = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.6, 2.4, 0.2))),
            MeshMaterial3d(materials.add(Color::srgb(0.6, 0.3, 0.8))),
            Transform::from_xyz(1.1, 3.2, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.8, 1.2, 0.1)),
        ))
        .id();
    commands.insert_resource(Demo {
        post,
        panel,
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
    let Ok(post) = body_query.get(demo.post) else {
        return;
    };
    let Ok(panel) = body_query.get(demo.panel) else {
        return;
    };
    let joint = physics_world.create_hinge_constraint(
        post.body_id_raw,
        panel.body_id_raw,
        HINGE_POINT,
        Vec3::Z,
        Vec3::X,
    );
    assert!(joint != 0, "hinge creation failed");
    demo.joint = joint;
    physics_world.constraint_drive_at(joint, 1.2);
    println!("hinge joint id {joint}");
}

fn pingpong_motor(mut demo: ResMut<Demo>, mut physics_world: ResMut<JoltPhysicsWorld>) {
    if demo.joint == 0 {
        return;
    }
    demo.flip_in = demo.flip_in.saturating_sub(1);
    if demo.flip_in == 0 {
        demo.forward = !demo.forward;
        demo.flip_in = 300;
        let speed = if demo.forward { 1.2 } else { -1.2 };
        physics_world
            .constraint_drive_at(demo.joint, speed);
        println!("hinge reversed ({}).", if demo.forward { "forward" } else { "back" });
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
        .get(demo.panel)
        .map(|id| {
            physics_world
                .body_full_transform(id.body_id_raw)
                .0
        })
        .expect("panel should own a Jolt body");
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

fn draw_pin(demo: Res<Demo>, mut gizmos: Gizmos) {
    if demo.joint == 0 {
        return;
    }
    gizmos.line(
        HINGE_POINT + Vec3::new(0.0, 0.0, -0.35),
        HINGE_POINT + Vec3::new(0.0, 0.0, 0.35),
        Color::WHITE,
    );
}
