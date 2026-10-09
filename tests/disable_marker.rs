// Disable marker: adding `JoltDisabled` makes a body sensor-quiet (no
// collision response), removing it re-enables response. A pre-bake add
// parks on `PendingDisable` and still lands. Run single-threaded
// (`--test-threads=1`): parallel `JoltWorld`s crash on the shared job
// plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltBodyId, JoltDisabled, JoltPhysicsWorld, JoltPlugin, JoltShape, JoltStep,
};

fn tick(app: &mut App) {
    app.update();
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(JoltStep);
}

fn spawn_ball(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id()
}

#[test]
fn disable_add_remove_roundtrip() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = spawn_ball(&mut app);
    for _ in 0..5 {
        tick(&mut app);
    }
    let body_id = app
        .world()
        .get::<JoltBodyId>(ball)
        .expect("ball should be baked")
        .body_id_raw;
    app.world_mut().entity_mut(ball).insert(JoltDisabled);
    app.update();
    for _ in 0..3 {
        tick(&mut app);
    }
    let world = app.world().resource::<JoltPhysicsWorld>();
    assert!(
        !world.body_collides(body_id),
        "disabled ball should not collide"
    );
    app.world_mut().entity_mut(ball).remove::<JoltDisabled>();
    app.update();
}

#[test]
fn disable_before_bake_still_lands() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 10.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
            JoltDisabled,
        ))
        .id();
    for _ in 0..5 {
        tick(&mut app);
    }
    let body_id = app
        .world()
        .get::<JoltBodyId>(ball)
        .expect("ball should be baked")
        .body_id_raw;
    let world = app.world().resource::<JoltPhysicsWorld>();
    assert!(
        !world.body_collides(body_id),
        "pre-bake disabled ball should not collide"
    );
    app.world_mut().entity_mut(ball).remove::<JoltDisabled>();
    app.update();
}
