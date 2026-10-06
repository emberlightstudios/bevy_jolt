//! Materials: a superball bounces, a dead ball thuds. Outlines show both
//! drops; the console prints peak rebound heights, then it exits.
use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltPlugin, JoltShape};

const SETTLE_TICKS: u32 = 400;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(bevy_jolt::JoltDebugPlugin)
        .insert_resource(BounceDemo::default())
        .add_systems(Startup, spawn_bounce_scene)
        .add_systems(FixedUpdate, watch_bounce_scene)
        .run();
}

#[derive(Resource, Default)]
struct BounceDemo {
    superball: Option<Entity>,
    deadball: Option<Entity>,
    superball_peak: f32,
    deadball_peak: f32,
    dropped: bool,
}

fn spawn_bounce_scene(mut commands: Commands, mut demo: ResMut<BounceDemo>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 12.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(4.0, 8.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let superball = commands
        .spawn((
            Transform::from_xyz(-2.0, 5.0, 0.0),
            JoltBody::dynamic(0).with_restitution(0.9),
            JoltShape::sphere(0.5),
        ))
        .id();
    let deadball = commands
        .spawn((
            Transform::from_xyz(2.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    demo.superball = Some(superball);
    demo.deadball = Some(deadball);
}

fn watch_bounce_scene(
    mut tick_count: Local<u32>,
    mut demo: ResMut<BounceDemo>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let (Some(superball), Some(deadball)) = (demo.superball, demo.deadball) else {
        return;
    };
    let Ok(superball_pose) = transform_query.get(superball) else {
        return;
    };
    let Ok(deadball_pose) = transform_query.get(deadball) else {
        return;
    };
    *tick_count += 1;
    // Arm rebound tracking only once both balls reach the floor (y < 1):
    // anything earlier is still the drop, not a rebound.
    if !demo.dropped && superball_pose.translation.y < 1.0 && deadball_pose.translation.y < 1.0 {
        demo.dropped = true;
    }
    if demo.dropped {
        demo.superball_peak = demo.superball_peak.max(superball_pose.translation.y);
        demo.deadball_peak = demo.deadball_peak.max(deadball_pose.translation.y);
    }
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let _ = transform_query.get(deadball);
    let _ = demo.deadball_peak;
    let _ = demo.superball_peak;
    println!(
        "superball rebound peak y={:.3}, dead ball peak y={:.3}",
        demo.superball_peak, demo.deadball_peak
    );
    assert!(
        demo.superball_peak > 2.0,
        "superball (restitution 0.9) should rebound high, got y={:.3}",
        demo.superball_peak
    );
    assert!(
        demo.deadball_peak < 1.0,
        "dead ball (restitution 0) should thud, got y={:.3}",
        demo.deadball_peak
    );
    println!("Bouncy bounces; dead thuds.");
    app_exit.write(AppExit::Success);
}
