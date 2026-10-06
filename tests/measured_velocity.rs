// Measured velocity readback: a dropped ball reports falling speed, a
// static floor reports nothing. Run single-threaded (`--test-threads=1`):
// parallel `JoltWorld`s crash on the shared job plumbing (see
// joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltMeasuredAngularVelocity, JoltMeasuredLinearVelocity, JoltPlugin, JoltShape,
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
    let measured_linear = app
        .world()
        .get::<JoltMeasuredLinearVelocity>(ball)
        .expect("dynamic body should carry measured linear velocity");
    assert!(
        measured_linear.measured_linear_velocity.y < -2.0,
        "falling ball should report downward speed, got {:?}",
        measured_linear.measured_linear_velocity
    );
    let measured_angular = app
        .world()
        .get::<JoltMeasuredAngularVelocity>(ball)
        .expect("dynamic body should carry measured angular velocity");
    assert!(
        measured_angular.measured_angular_velocity.length() < 0.5,
        "straight drop should barely spin, got {:?}",
        measured_angular.measured_angular_velocity
    );
}

#[test]
fn static_body_carries_no_measured_velocity() {
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
        app.world()
            .get::<JoltMeasuredLinearVelocity>(floor)
            .is_none(),
        "static floor should skip the measured component"
    );
}
