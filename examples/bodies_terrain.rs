//! Rolling heightfield terrain: sine hills with balls settling into the
//! valleys. Grid wireframe shows the sampled ground; console prints resting
//! heights, then it exits.

use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape};

const GRID_WIDTH: u32 = 16;
const CELL_SIZE: f32 = 1.0;
const SETTLE_TICKS: u32 = 600;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .insert_resource(TerrainDemo::default())
        .add_systems(Startup, spawn_terrain_scene)
        .add_systems(FixedUpdate, watch_terrain_scene)
        .run();
}

#[derive(Resource, Default)]
struct TerrainDemo {
    balls: Vec<Entity>,
}

fn spawn_terrain_scene(mut commands: Commands, mut demo: ResMut<TerrainDemo>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 14.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(6.0, 12.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // 16x16 sine hills, 1m cells, centered on the origin.
    let mut field_heights = Vec::with_capacity((GRID_WIDTH * GRID_WIDTH) as usize);
    for grid_z in 0..GRID_WIDTH {
        for grid_x in 0..GRID_WIDTH {
            field_heights.push((grid_x as f32 * 0.9).sin() + (grid_z as f32 * 1.1).cos());
        }
    }
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::heightfield(field_heights, GRID_WIDTH, CELL_SIZE),
    ));
    // Three balls dropped from different heights: all should end up resting
    // on the hills, none through the world.
    for (ball_index, drop_height) in [5.0, 7.0, 9.0].iter().enumerate() {
        let ball = commands
            .spawn((
                Transform::from_xyz(-3.0 + ball_index as f32 * 3.0, *drop_height, 0.0),
                JoltBody::dynamic(0),
                JoltShape::sphere(0.5),
            ))
            .id();
        demo.balls.push(ball);
    }
}

fn watch_terrain_scene(
    mut tick_count: Local<u32>,
    demo: Res<TerrainDemo>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    for ball in &demo.balls {
        let Ok(ball_pose) = transform_query.get(*ball) else {
            return;
        };
        println!("ball resting at y={:.3}", ball_pose.translation.y);
        assert!(
            ball_pose.translation.y > -2.5 && ball_pose.translation.y < 4.0,
            "ball should rest on the hills, got y={:.3}",
            ball_pose.translation.y
        );
    }
    println!("All balls settled into the valleys.");
    app_exit.write(AppExit::Success);
}
