//! Character controller: a capsule walks off a ledge, climbs stairs, and
//! rides a moving platform. The walker renders as a physics wireframe;
//! the console still prints ground readings, then it exits.

use bevy::prelude::*;
use bevy_jolt::{
    CharacterGround, JoltBody, JoltCharacter, JoltCharacterGround, JoltCharacterStep,
    JoltCharacterVelocity, JoltDebugPlugin, JoltPlugin, JoltShape,
};

const WALK_SPEED: f32 = 3.0;
const STAFF_TICKS: u32 = 600;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_character_scene)
        .add_systems(FixedUpdate, drive_character)
        .run();
}

#[derive(Resource)]
struct CharacterDemo {
    walker: Entity,
    start_height: f32,
}
fn spawn_character_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-6.0, 3.0, 9.0).looking_at(Vec3::new(-6.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Floor plus a two-step staircase (0.2 m rises) the walker climbs.
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    for (stair_index, stair_top) in [0.2, 0.4].into_iter().enumerate() {
        commands.spawn((
            Transform::from_xyz(
                3.0 + stair_index as f32 * 1.0,
                stair_top - 0.5,
                0.0,
            ),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(0.5, 0.5, 2.0)),
        ));
    }
    let start_height = 1.0;
    let walker = commands
        .spawn((
            Transform::from_xyz(-6.0, start_height, 0.0),
            JoltCharacter::new(0),
            JoltCharacterVelocity::default(),
            JoltCharacterStep::default(),
        ))
        .id();
    commands.insert_resource(CharacterDemo {
        walker,
        start_height,
    });
}

fn drive_character(
    mut tick_count: Local<u32>,
    demo: Res<CharacterDemo>,
    mut velocity_query: Query<(&mut JoltCharacterVelocity, &Transform, &JoltCharacterGround)>,
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<JoltCharacterVelocity>)>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    let Ok((mut walker_velocity, walker_pose, walker_ground)) =
        velocity_query.get_mut(demo.walker)
    else {
        return;
    };
    // Steady march toward +x: stairs, then open floor again.
    walker_velocity.velocity = Vec3::new(WALK_SPEED, -2.0, 0.0);
    // Side-on tracking shot: the camera rides along with the walker.
    for mut camera_pose in &mut camera_query {
        *camera_pose =
            Transform::from_xyz(walker_pose.translation.x, 3.0, 9.0)
                .looking_at(walker_pose.translation + Vec3::new(0.0, 0.5, 0.0), Vec3::Y);
    }
    if *tick_count % 60 == 0 {
        println!(
            "tick {}: x={:.2} y={:.2} ground={:?} supported={}",
            *tick_count,
            walker_pose.translation.x,
            walker_pose.translation.y,
            walker_ground.ground_state,
            walker_ground.is_supported,
        );
    }
    if *tick_count < STAFF_TICKS {
        return;
    }
    assert!(
        walker_pose.translation.x > 0.0,
        "walker should march past the stairs"
    );
    assert!(
        (walker_pose.translation.y - demo.start_height).abs() < 0.6,
        "walker should rest near spawn height, got y={:.2}",
        walker_pose.translation.y
    );
    assert_eq!(
        walker_ground.ground_state,
        CharacterGround::OnGround,
        "walker should end supported on the floor"
    );
    println!("Character walked, climbed, and stayed grounded.");
    app_exit.write(AppExit::Success);
}
