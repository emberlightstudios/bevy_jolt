//! Rack-and-pinion (jack) joint: a spinning pinion drives a sliding rack.
//! Gravity drags the rack down so the pinion keeps turning; every 15s the
//! rack gets hoisted back up to loop.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin,
};

const PINION_HINGE: Vec3 = Vec3::new(-1.0, 4.0, 0.0);
const RACK_SPAWN: Vec3 = Vec3::new(1.0, 2.6, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_joint_once)
        .add_systems(FixedUpdate, hoist_rack)
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, draw_link)
        .run();
}

#[derive(Resource)]
struct Demo {
    post: Entity,
    pinion: Entity,
    rail: Entity,
    rack: Entity,
    joint: u32,
    hoist_in: u32,
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
        Text::new("rack-and-pinion (jack)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let post = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.0, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(-1.0, 5.0, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.5, 0.2)),
        ))
        .id();
    let pinion = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 0.3))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.6, 0.2))),
            Transform::from_xyz(-1.0, 3.5, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.5, 0.5, 0.15)),
        ))
        .id();
    let rail = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 0.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.5, 0.5, 0.55))),
            Transform::from_xyz(1.0, 4.2, 0.0),
            JoltBody::fixed(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.25)),
        ))
        .id();
    let rack = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 1.6, 0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.6, 0.9))),
            Transform::from_translation(RACK_SPAWN),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::new(0.2, 0.8, 0.2)),
        ))
        .id();
    commands.insert_resource(Demo {
        post,
        pinion,
        rail,
        rack,
        joint: 0,
        hoist_in: 900,
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
    let Ok(pinion) = body_query.get(demo.pinion) else {
        return;
    };
    let Ok(rail) = body_query.get(demo.rail) else {
        return;
    };
    let Ok(rack) = body_query.get(demo.rack) else {
        return;
    };
    let world = &mut **physics_world;
    let hinge = world.create_hinge_constraint(
        post.body_id_raw,
        pinion.body_id_raw,
        PINION_HINGE,
        Vec3::Z,
        Vec3::X,
    );
    let slider = world.create_slider_constraint(
        rail.body_id_raw,
        rack.body_id_raw,
        Vec3::Y,
        Vec3::X,
        -1.5,
        0.5,
    );
    assert!(hinge != 0 && slider != 0, "rack sub-joints failed");
    let joint = world.create_rack_pinion_constraint(
        pinion.body_id_raw,
        rack.body_id_raw,
        Vec3::Z,
        Vec3::Y,
        1.0,
        hinge,
        slider,
    );
    assert!(joint != 0, "rack-pinion creation failed");
    demo.joint = joint;
    println!("rack-pinion joint id {joint} (hinge {hinge}, slider {slider})");
}

fn hoist_rack(
    mut demo: ResMut<Demo>,
    body_query: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.joint == 0 {
        return;
    }
    demo.hoist_in = demo.hoist_in.saturating_sub(1);
    if demo.hoist_in == 0 {
        if let Ok(rack) = body_query.get(demo.rack) {
            physics_world
                .reset_body_to(rack.body_id_raw, RACK_SPAWN);
        }
        demo.hoist_in = 900;
        println!("hoisted the rack");
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
    let bodies = |entity: Entity| {
        body_query
            .get(entity)
            .map(|id| {
                physics_world
                    .body_full_transform(id.body_id_raw)
            })
            .expect("demo entity should own a Jolt body")
    };
    let (rack_pos, _) = bodies(demo.rack);
    let (_, pinion_rot) = bodies(demo.pinion);
    let (_, angle) = pinion_rot.to_axis_angle();
    println!(
        "tick {}: jack height {:.3}, pinion turned {:.2} rad.",
        *tick, rack_pos.y, angle
    );
    assert!(
        (rack_pos.x - 1.0).abs() < 0.25,
        "rack should stay on its rail"
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
    let (Ok(pinion), Ok(rack)) = (
        transform_query.get(demo.pinion),
        transform_query.get(demo.rack),
    ) else {
        return;
    };
    gizmos.line(
        pinion.translation,
        rack.translation,
        Color::srgb(0.6, 0.9, 1.0),
    );
}
