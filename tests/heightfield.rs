// Heightfield terrain: rolling hills hold a dropped ball in a valley.
// Run single-threaded (`--test-threads=1`): parallel `JoltWorld`s crash on
// the shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltPlugin, JoltShape, JoltStep};

fn tick(app: &mut App) {
    app.update();
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(JoltStep);
}

/// 16x16 sine hills, 1m cells, centered on the spawn: wide enough that a
/// dropped ball settles in a valley instead of rolling off the edge.
fn hill_heights() -> Vec<f32> {
    let grid_width = 16;
    let mut field_heights = Vec::with_capacity(grid_width * grid_width);
    for grid_z in 0..grid_width {
        for grid_x in 0..grid_width {
            let hill_height = (grid_x as f32 * 0.9).sin() + (grid_z as f32 * 1.1).cos();
            field_heights.push(hill_height);
        }
    }
    field_heights
}

#[test]
fn ball_rests_in_terrain_valley() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.world_mut().spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::heightfield(hill_heights(), 16, 1.0),
    ));
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 6.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    for _ in 0..240 {
        tick(&mut app);
    }
    let ball_pose = app.world().get::<Transform>(ball).expect("ball exists");
    assert!(
        ball_pose.translation.y > -2.5 && ball_pose.translation.y < 4.0,
        "ball should rest on the hills, got y={:.3}",
        ball_pose.translation.y
    );
}

#[test]
fn dynamic_heightfield_rejected() {
    // Mesh-style fail-loud: heightfields cannot simulate, so a dynamic
    // bake must panic rather than silently cook a dead shape.
    let result = std::panic::catch_unwind(|| {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, JoltPlugin::new()));
        app.world_mut().spawn((
            Transform::IDENTITY,
            JoltBody::dynamic(0),
            JoltShape::heightfield(hill_heights(), 8, 1.0),
        ));
        tick(&mut app);
    });
    assert!(result.is_err(), "dynamic heightfield should panic at bake");
}
