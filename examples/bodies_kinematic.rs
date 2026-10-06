//! Kinematic bodies: move by velocity, push dynamics, ignore forces.
//! A kinematic platform shuttles on a sine wave while a dynamic ball rides
//! on top. The demo writes a target pose component; the crate drives it.
//! Watch the platform carry the ball side to side.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltDebugPlugin, JoltKinematicTarget, JoltPlugin, JoltShape};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(FixedUpdate, drive_platform)
        .add_systems(FixedPostUpdate, report);
}

#[derive(Resource)]
struct Demo {
    platform: Entity,
    ball: Entity,
    drive_ticks: u32,
    ready: bool,
}
fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.0, 11.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
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
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    commands.spawn((
        Text::new("kinematic (moving platform)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let platform = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(6.0, 0.4, 3.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.3, 0.6, 0.9))),
            Transform::from_xyz(0.0, 1.5, 0.0),
            JoltBody::kinematic(0),
            JoltShape::box_shape(Vec3::new(3.0, 0.2, 1.5)),
            JoltKinematicTarget {
                target_position: Vec3::new(0.0, 1.5, 0.0),
                target_rotation: Quat::IDENTITY,
            },
        ))
        .id();
    let ball = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.9, 0.4, 0.2))),
            Transform::from_xyz(0.0, 2.5, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.4),
        ))
        .id();
    commands.insert_resource(Demo {
        platform,
        ball,
        drive_ticks: 0,
        ready: false,
    });
}

/// Sine-wave drive: the platform glides ±2.5 on X with a 10s period. Just
/// writes the target pose; the crate's pre-step system moves it with
/// `MoveKinematic`, which derives velocity from the delta so the ball rides
/// along.
fn drive_platform(
    mut demo: ResMut<Demo>,
    mut target_query: Query<&mut JoltKinematicTarget>,
    fixed_time: Res<Time<Fixed>>,
) {
    let Ok(mut kinematic_target) = target_query.get_mut(demo.platform) else {
        return;
    };
    demo.ready = true;
    // Tick-counted time: advances exactly once per Fixed tick, immune to
    // Fixed-time elapsed quirks under load.
    demo.drive_ticks += 1;
    let elapsed = demo.drive_ticks as f32 * fixed_time.delta().as_secs_f32();
    kinematic_target.target_position = Vec3::new(2.0 * (0.314 * elapsed).sin(), 1.5, 0.0);
}

fn report(mut tick: Local<u32>, demo: Res<Demo>, transform_query: Query<&Transform>) {
    if !demo.ready {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let (Ok(platform), Ok(ball)) = (
        transform_query.get(demo.platform),
        transform_query.get(demo.ball),
    ) else {
        return;
    };
    println!(
        "tick {}: platform x={:.2}, ball at ({:.2}, {:.2}).",
        *tick, platform.translation.x, ball.translation.x, ball.translation.y
    );
    assert!(
        platform.translation.x.abs() < 2.2,
        "platform should stay on its sine track"
    );
    assert!(
        (ball.translation.y - 2.1).abs() < 0.5
            && (ball.translation.x - platform.translation.x).abs() < 3.2,
        "ball should ride the platform"
    );
}
