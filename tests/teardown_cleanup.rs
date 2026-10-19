// Teardown cleanup: what releasing an entity or a spec component has to give
// back — and the guard that keeps a stale id out of Jolt's destroy path.
// Soft bodies have their own target (`soft_body_lifecycle.rs`): theirs only
// crashes when the body was actually simulated.
//
// Run single-threaded (`--test-threads=1`): parallel `JoltWorld`s crash on the
// shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltBodyId, JoltPhysicsWorld, JoltPlugin, JoltRigidCharacter, JoltRigidCharacterId,
    JoltSensor, JoltShape,
};

fn tick(app: &mut App) {
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(bevy_jolt::JoltStep);
}

/// Runs one tick with a real fixed timestep. `tick` alone leaves the step delta
/// at ~0 (running the schedules by hand accumulates no `Time<Real>`), so a test
/// that needs bodies to actually move advances the clock itself — the same
/// helper `sleep_motion.rs` uses.
fn moving_tick(app: &mut App) {
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    tick(app);
}

/// A stale id must never reach Jolt's destroy path: `DestroyBodies` indexes
/// `mBodies[id.GetIndex()]` with no bounds check and no sequence check, and its
/// only guards are asserts the Distribution build compiles out — so it would
/// free whatever body owns that slot by then.
#[test]
#[should_panic(expected = "stale id or double destroy")]
fn destroying_an_already_destroyed_id_is_refused() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));

    let body_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    for _ in 0..5 {
        tick(&mut app);
    }
    let body_id_raw = app
        .world()
        .get::<JoltBodyId>(body_entity)
        .expect("body should bake")
        .body_id_raw;

    app.world_mut().entity_mut(body_entity).despawn();
    for _ in 0..3 {
        tick(&mut app);
    }
    app.world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .remove_and_destroy_body(body_id_raw);
}

/// `JoltBody` owns the Jolt body, so removing it has to free the body and drop
/// the id — not only despawning the entity. Before this, removal fired no
/// observer, a re-add overwrote the id (an insert is not a removal), and the
/// first body stayed in the world colliding with nothing able to reach it.
#[test]
fn removing_jolt_body_frees_the_body_and_a_re_add_bakes_one() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = app
        .world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .body_stats()
        .0;
    let baseline_shapes = app.world().resource::<JoltPhysicsWorld>().body_shapes().len();

    let body_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    for _ in 0..5 {
        tick(&mut app);
    }
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .body_stats()
            .0,
        baseline_bodies + 1,
        "bake should add exactly one live body"
    );

    app.world_mut().entity_mut(body_entity).remove::<JoltBody>();
    for _ in 0..5 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltBodyId>(body_entity).is_none(),
        "the id should leave with the spec that owns it"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .body_stats()
            .0,
        baseline_bodies,
        "removing JoltBody leaked its Jolt body"
    );
    assert_eq!(
        app.world().resource::<JoltPhysicsWorld>().body_shapes().len(),
        baseline_shapes,
        "the shape table kept the retired body"
    );

    app.world_mut()
        .entity_mut(body_entity)
        .insert((JoltBody::dynamic(0), JoltShape::sphere(0.5)));
    for _ in 0..5 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltBodyId>(body_entity).is_some(),
        "re-adding JoltBody should bake a new body"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .body_stats()
            .0,
        baseline_bodies + 1,
        "the re-add baked a second body: the first one leaked"
    );
}

/// `JoltRigidCharacter` owns its body the same way `JoltBody` does.
#[test]
fn removing_jolt_rigid_character_frees_its_body() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = app
        .world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .body_stats()
        .0;

    let character_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.0, 0.0),
            JoltRigidCharacter::new(0),
        ))
        .id();
    for _ in 0..5 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltRigidCharacterId>(character_entity).is_some(),
        "rigid character should bake"
    );

    app.world_mut()
        .entity_mut(character_entity)
        .remove::<JoltRigidCharacter>();
    for _ in 0..5 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltRigidCharacterId>(character_entity).is_none(),
        "the id should leave with the spec that owns it"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .body_stats()
            .0,
        baseline_bodies,
        "removing JoltRigidCharacter leaked its body"
    );
}

/// The sensor flag lives on the Jolt body, not on the marker: removing
/// `JoltSensor` has to clear it, or the body stays non-colliding for good.
/// Observable without reaching into Jolt — a sensor ball falls straight through
/// the floor and keeps going, while one whose marker left rests on it.
#[test]
fn removing_jolt_sensor_restores_collision_response() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));

    app.world_mut().spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(10.0, 0.5, 10.0)),
    ));
    let ball_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.8, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
            JoltSensor,
        ))
        .id();
    for _ in 0..2 {
        moving_tick(&mut app);
    }

    app.world_mut().entity_mut(ball_entity).remove::<JoltSensor>();
    for _ in 0..90 {
        moving_tick(&mut app);
    }
    let ball_height = app
        .world()
        .get::<Transform>(ball_entity)
        .expect("ball should still exist")
        .translation
        .y;
    assert!(
        ball_height > 0.4,
        "the ball fell through the floor ({ball_height} m): the sensor flag was never cleared"
    );
    assert!(
        ball_height < 0.7,
        "the ball should rest on the floor, not float ({ball_height} m)"
    );
}
