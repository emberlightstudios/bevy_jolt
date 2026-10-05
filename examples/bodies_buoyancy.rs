//! Buoyancy: a buoyant box floats at the surface, a plain box sinks to
//! the floor. Prints both fates and exits.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltBuoyant, JoltPlugin, JoltShape, JoltWater};

const SETTLE_TICKS: u32 = 600;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .insert_resource(JoltWater {
            surface_height: 0.0,
            buoyancy: 1.0,
            linear_drag: 1.5,
            angular_drag: 1.0,
            current_velocity: Vec3::ZERO,
        })
        .insert_resource(FloatDemo::default())
        .add_systems(Startup, spawn_float_scene)
        .add_systems(FixedUpdate, watch_float_scene)
        .run();
}

#[derive(Resource, Default)]
struct FloatDemo {
    floater: Option<Entity>,
    sinker: Option<Entity>,
}

fn spawn_float_scene(mut commands: Commands, mut demo: ResMut<FloatDemo>) {
    commands.spawn((
        Transform::from_xyz(0.0, -6.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let floater = commands
        .spawn((
            Transform::from_xyz(-2.0, -3.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::new(0.5, 0.5, 0.5)),
            JoltBuoyant { buoyancy_scale: 1.2 },
        ))
        .id();
    let sinker = commands
        .spawn((
            Transform::from_xyz(2.0, -3.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::new(0.5, 0.5, 0.5)),
        ))
        .id();
    demo.floater = Some(floater);
    demo.sinker = Some(sinker);
}

fn watch_float_scene(
    mut tick_count: Local<u32>,
    demo: Res<FloatDemo>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let (Some(floater), Some(sinker)) = (demo.floater, demo.sinker) else {
        return;
    };
    let Ok(floater_pose) = transform_query.get(floater) else {
        return;
    };
    let Ok(sinker_pose) = transform_query.get(sinker) else {
        return;
    };
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    println!(
        "floater y={:.3}, sinker y={:.3} (surface 0, floor -5)",
        floater_pose.translation.y, sinker_pose.translation.y
    );
    assert!(
        floater_pose.translation.y > -1.0,
        "floater should ride near the surface, got y={:.3}",
        floater_pose.translation.y
    );
    assert!(
        sinker_pose.translation.y < -4.0,
        "sinker should rest on the floor, got y={:.3}",
        sinker_pose.translation.y
    );
    println!("Floater floats; sinker sinks.");
    app_exit.write(AppExit::Success);
}
