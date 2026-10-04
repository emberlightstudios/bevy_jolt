//! Path (track) joint: a cart glues to a straight track and shuttles along
//! it. A velocity motor ping-pongs every 5s so the cart loops.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const TRACK_FROM: Vec3 = Vec3::new(-1.5, 3.5, 0.0);
const TRACK_TO: Vec3 = Vec3::new(1.5, 3.5, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, pingpong_motor)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_track)
        .run();
}

#[derive(Resource)]
struct Demo {
    anchor: Entity,
    cart: Entity,
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
        Text::new("path (track cart)"),
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
            Transform::from_xyz(-1.5, 4.5, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.2)),
        ))
        .id();
    let cart = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.7, 0.7, 0.7))),
            MeshMaterial3d(materials.add(Color::srgb(0.7, 0.3, 0.9))),
            Transform::from_translation(TRACK_FROM),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.35)),
        ))
        .id();
    commands.insert_resource(Demo {
        anchor,
        cart,
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
    let Ok(anchor) = body_query.get(demo.anchor) else {
        return;
    };
    let Ok(cart) = body_query.get(demo.cart) else {
        return;
    };
    let joint = physics_world.create_path_cart(
        anchor.body_id_raw,
        cart.body_id_raw,
        TRACK_FROM,
        TRACK_TO,
    );
    assert!(joint != 0, "path cart creation failed");
    demo.joint = joint;
    physics_world.constraint_drive_at(joint, 1.0);
    println!("path joint id {joint}");
}

fn pingpong_motor(mut demo: ResMut<Demo>, mut physics_world: ResMut<JoltPhysicsWorld>) {
    if demo.joint == 0 {
        return;
    }
    demo.flip_in = demo.flip_in.saturating_sub(1);
    if demo.flip_in == 0 {
        demo.forward = !demo.forward;
        demo.flip_in = 300;
        let speed = if demo.forward { 1.0 } else { -1.0 };
        physics_world
            .constraint_drive_at(demo.joint, speed);
        println!("cart reversed ({}).", if demo.forward { "forward" } else { "back" });
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
        .get(demo.cart)
        .map(|id| {
            physics_world
                .body_full_transform(id.body_id_raw)
                .0
        })
        .expect("cart should own a Jolt body");
    let track = TRACK_TO - TRACK_FROM;
    let progress = ((position - TRACK_FROM).dot(track) / track.length_squared()).clamp(0.0, 1.0);
    println!("tick {}: cart progress {:.2}.", *tick, progress);
    let off_track = ((position - TRACK_FROM) - track * progress).length();
    assert!(off_track < 0.3, "cart should stay glued to its track");
}

fn draw_track(demo: Res<Demo>, mut gizmos: Gizmos) {
    if demo.joint == 0 {
        return;
    }
    gizmos.line(TRACK_FROM, TRACK_TO, Color::srgb(0.8, 0.5, 1.0));
}
