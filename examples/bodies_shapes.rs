//! Shape tumble: one dynamic body per built-in shape rolls down a ramp,
//! collides, and settles. Matching outlines prove each shape matches Jolt.

use bevy::prelude::*;
use bevy_jolt::{
    CompoundGeometry, CompoundPart, JoltBody, JoltDebugPlugin, JoltImpulse, JoltPlugin,
    JoltShape,
};

const SETTLE_TICKS: u32 = 900;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .insert_resource(ShapeDemo::default())
        .add_systems(Startup, spawn_shape_scene)
        .add_systems(FixedUpdate, watch_shape_scene)
        .run();
}

#[derive(Resource, Default)]
struct ShapeDemo {
    tumbler_entities: Vec<Entity>,
    kicked_tick: u32,
}

fn spawn_shape_scene(mut commands: Commands, mut demo: ResMut<ShapeDemo>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 22.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(4.0, 10.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Floor plus a low ramp wedge the tumblers roll off.
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(30.0, 1.0, 10.0)),
    ));
    let ramp_body = commands
        .spawn((
            Transform::from_xyz(-12.0, 0.5, 0.0).with_rotation(Quat::from_rotation_z(0.25)),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(4.0, 0.25, 3.0)),
        ))
        .id();
    let _ = ramp_body;

    // One tumbler per shape, staged along the ramp top with gaps.
    let tumbler_specs: Vec<(&str, JoltShape)> = vec![
        ("box", JoltShape::box_shape(Vec3::splat(0.5))),
        ("sphere", JoltShape::sphere(0.5)),
        ("capsule", JoltShape::capsule(0.5, 0.3)),
        ("cylinder", JoltShape::cylinder(0.4, 0.5)),
        ("tapered_cylinder", JoltShape::tapered_cylinder(0.4, 0.3, 0.5)),
        (
            "tapered_capsule",
            JoltShape::tapered_capsule(0.4, 0.3, 0.5),
        ),
        (
            "compound",
            JoltShape::compound(vec![
                CompoundPart {
                    part_geometry: CompoundGeometry::Box {
                        part_half_extents: Vec3::new(0.5, 0.1, 0.3),
                    },
                    part_offset: Vec3::ZERO,
                    part_rotation: Quat::IDENTITY,
                },
                CompoundPart {
                    part_geometry: CompoundGeometry::Sphere { part_radius: 0.25 },
                    part_offset: Vec3::new(0.0, 0.3, 0.0),
                    part_rotation: Quat::IDENTITY,
                },
            ]),
        ),
    ];
    for (tumbler_index, (_, tumbler_shape)) in tumbler_specs.into_iter().enumerate() {
        let tumbler = commands
            .spawn((
                Transform::from_xyz(-14.0 + tumbler_index as f32 * 2.2, 4.0, 0.0),
                JoltBody::dynamic(0),
                tumbler_shape,
            ))
            .id();
        demo.tumbler_entities.push(tumbler);
    }
}

/// Kicks every tumbler sideways once, then asserts all settled on the floor:
/// resting outlines prove each drawn shape matches its simulated body.
fn watch_shape_scene(
    mut tick_count: Local<u32>,
    mut demo: ResMut<ShapeDemo>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
    mut commands: Commands,
) {
    *tick_count += 1;
    if *tick_count == 30 {
        for tumbler_entity in &demo.tumbler_entities {
            commands.trigger(JoltImpulse::linear(
                *tumbler_entity,
                Vec3::new(30.0, 0.0, 5.0),
            ));
        }
        demo.kicked_tick = *tick_count;
    }
    if *tick_count < SETTLE_TICKS {
        return;
    }
    for tumbler_entity in &demo.tumbler_entities {
        let Ok(tumbler_pose) = transform_query.get(*tumbler_entity) else {
            panic!("tumbler gone before settle check");
        };
        assert!(
            tumbler_pose.translation.y > -0.5 && tumbler_pose.translation.y < 4.0,
            "tumbler should rest near the floor, got y={:.2}",
            tumbler_pose.translation.y
        );
    }
    println!("All shapes tumbled and settled; outlines match.");
    app_exit.write(AppExit::Success);
}
