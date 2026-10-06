// Hull + mesh shapes: a hull rock tumbles off a ramp, a mesh arch holds a
// dropped ball on its span. Run single-threaded (`--test-threads=1`):
// parallel `JoltWorld`s crash on the shared job plumbing (see
// joint_lifecycle.rs).
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

fn rock_points() -> Vec<Vec3> {
    vec![
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::new(-0.4, 0.3, 0.2),
        Vec3::new(0.0, -0.3, 0.4),
        Vec3::new(0.1, 0.4, -0.3),
        Vec3::new(-0.2, -0.1, -0.4),
        Vec3::new(0.0, 0.5, 0.1),
        Vec3::new(0.3, -0.4, -0.1),
        Vec3::new(-0.5, -0.2, 0.3),
    ]
}

fn arch_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    // Flat span: two triangles forming a 4x0.5 plate at y=2, held by two
    // legs. A ball dropped above the span should rest on it.
    let mesh_vertices = vec![
        Vec3::new(-2.0, 2.0, -0.5), // 0 span corners
        Vec3::new(2.0, 2.0, -0.5),  // 1
        Vec3::new(2.0, 2.0, 0.5),   // 2
        Vec3::new(-2.0, 2.0, 0.5),  // 3
        Vec3::new(-2.0, 0.0, -0.5), // 4 left leg
        Vec3::new(-1.5, 0.0, 0.5),  // 5
        Vec3::new(1.5, 0.0, -0.5),  // 6 right leg
        Vec3::new(2.0, 0.0, 0.5),   // 7
    ];
    let mesh_triangles = vec![
        [0, 2, 1],
        [0, 3, 2], // span top (up-facing)
        [4, 0, 3],
        [4, 3, 5], // left leg
        [6, 7, 2],
        [6, 2, 1], // right leg
    ];
    (mesh_vertices, mesh_triangles)
}

#[test]
fn hull_rock_tumbles_down_ramp() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.world_mut().spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(10.0, 1.0, 10.0)),
    ));
    let rock = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::hull(rock_points()),
        ))
        .id();
    for _ in 0..120 {
        tick(&mut app);
    }
    let rock_pose = app.world().get::<Transform>(rock).expect("rock exists");
    assert!(
        rock_pose.translation.y < 4.0,
        "rock should have fallen, got y={:.3}",
        rock_pose.translation.y
    );
    assert!(
        rock_pose.translation.y > -0.5,
        "rock should rest on the floor, got y={:.3}",
        rock_pose.translation.y
    );
}

#[test]
fn mesh_arch_holds_dropped_ball() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (mesh_vertices, mesh_triangles) = arch_mesh();
    app.world_mut().spawn((
        Transform::IDENTITY,
        JoltBody::fixed(0),
        JoltShape::mesh(mesh_vertices, mesh_triangles),
    ));
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    for _ in 0..180 {
        tick(&mut app);
    }
    let ball_pose = app.world().get::<Transform>(ball).expect("ball exists");
    assert!(
        ball_pose.translation.y > 1.5,
        "ball should rest on the arch span, got y={:.3}",
        ball_pose.translation.y
    );
    assert!(
        ball_pose.translation.y < 3.5,
        "ball should not fall through the span, got y={:.3}",
        ball_pose.translation.y
    );
}
