// Unified velocity: a dropped ball reports falling speed in the same
// component game code writes to, a static floor carries nothing. Run
// single-threaded (`--test-threads=1`): parallel `JoltWorld`s crash on the
// shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltAngularVelocity, JoltBody, JoltLinearVelocity, JoltPlugin, JoltShape,
};

fn tick(app: &mut App) {
    app.update();
    // FixedUpdate alone doesn't advance its own clock: push it one timestep
    // so the step integrates with a real delta instead of zero. JoltStep
    // runs explicitly too: tests drive schedules by hand, so the fixed-loop
    // order doesn't apply.
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(bevy_jolt::JoltStep);
}

#[test]
fn falling_body_reports_velocity() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    // Bake + a few ticks of falling.
    for _ in 0..30 {
        tick(&mut app);
    }
    let unified_linear = app
        .world()
        .get::<JoltLinearVelocity>(ball)
        .expect("dynamic body should carry unified linear velocity");
    assert!(
        unified_linear.linear_velocity.y < -2.0,
        "falling ball should report downward speed, got {:?}",
        unified_linear.linear_velocity
    );
    let unified_angular = app
        .world()
        .get::<JoltAngularVelocity>(ball)
        .expect("dynamic body should carry unified angular velocity");
    assert!(
        unified_angular.angular_velocity.length() < 0.5,
        "straight drop should barely spin, got {:?}",
        unified_angular.angular_velocity
    );
}

#[test]
fn static_body_carries_no_velocity() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let floor = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, -1.0, 0.0),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(10.0, 1.0, 10.0)),
        ))
        .id();
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltLinearVelocity>(floor).is_none(),
        "static floor should skip the velocity component"
    );
}

#[test]
fn drive_write_lands_once_and_writeback_does_not_redrive() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    // Bake so the unified components exist.
    for _ in 0..5 {
        tick(&mut app);
    }
    // One drive write: the body should move at roughly the requested speed
    // on the next tick...
    app.world_mut()
        .get_mut::<JoltLinearVelocity>(ball)
        .expect("baked body should carry velocity")
        .linear_velocity = Vec3::new(3.0, 0.0, 0.0);
    tick(&mut app);
    let after_drive = app
        .world()
        .get::<JoltLinearVelocity>(ball)
        .expect("body should still carry velocity")
        .linear_velocity;
    assert!(
        (after_drive.x - 3.0).abs() < 0.6,
        "drive write should land near 3 m/s, got {after_drive:?}"
    );
    // ...and the writeback must not count as a new drive: clearing change
    // detection state means the next tick integrates naturally instead of
    // re-applying the readback as a request. Gravity pulls y down while x
    // coasts (no drag), so x holds and y falls.
    tick(&mut app);
    let coasted = app
        .world()
        .get::<JoltLinearVelocity>(ball)
        .expect("body should still carry velocity")
        .linear_velocity;
    assert!(
        (coasted.x - 3.0).abs() < 0.6 && coasted.y < -0.1,
        "writeback should coast, not re-drive: got {coasted:?}"
    );
}
