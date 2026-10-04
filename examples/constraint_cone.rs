//! Cone (shoulder) joint: a ball swings inside a cone around the twist
//! axis. Re-kicked every 10s so the swing loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const ANCHOR: Vec3 = Vec3::new(0.0, 4.5, 0.0);
const BALL_SPAWN: Vec3 = Vec3::new(0.8, 3.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, rekick_ball)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    anchor: Entity,
    ball: Entity,
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
        Text::new("cone (shoulder)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let anchor = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 0.4, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_translation(ANCHOR),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();
    let ball = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.8, 0.6))),
            Transform::from_translation(BALL_SPAWN),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::sphere(0.3),
        ))
        .id();
    commands.insert_resource(Demo {
        anchor,
        ball,
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
    let Ok(anchor) = body_query.get(demo.anchor) else {
        return;
    };
    let Ok(ball) = body_query.get(demo.ball) else {
        return;
    };
    let joint = physics_world.create_cone_constraint(
        anchor.body_id_raw,
        ball.body_id_raw,
        ANCHOR,
        Vec3::Y,
        0.35,
    );
    assert!(joint != 0, "cone creation failed");
    demo.joint = joint;
    physics_world
        .kick_body(ball.body_id_raw, Vec3::new(3.0, 0.5, 1.5));
    println!("cone joint id {joint}");
}

fn rekick_ball(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.joint == 0 {
        return;
    }
    demo.kick_in = demo.kick_in.saturating_sub(1);
    if demo.kick_in == 0 {
        if let Ok(ball) = body_query.get(demo.ball) {
            physics_world
                .kick_body(ball.body_id_raw, Vec3::new(3.0, 0.5, 1.5));
        }
        demo.kick_in = 600;
        println!("re-kicked the shoulder ball");
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
    let position = |entity: Entity| {
        body_query
            .get(entity)
            .map(|id| {
                physics_world
                    .body_full_transform(id.body_id_raw)
                    .0
            })
            .expect("demo entity should own a Jolt body")
    };
    let radius = (position(demo.ball) - position(demo.anchor)).length();
    println!("tick {}: shoulder radius {:.3}.", *tick, radius);
    assert!(
        (radius - 1.6).abs() < 0.25,
        "shoulder ball should hold its radius"
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
    let (Ok(anchor), Ok(ball)) = (
        transform_query.get(demo.anchor),
        transform_query.get(demo.ball),
    ) else {
        return;
    };
    gizmos.line(
        anchor.translation,
        ball.translation,
        Color::srgb(0.4, 1.0, 0.7),
    );
}
