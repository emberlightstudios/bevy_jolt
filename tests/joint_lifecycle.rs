// NOTE: `#[serial]` (serial_test) is intentionally not used here. Parallel
// `JoltWorld`s crash each other today: every world builds its own
// `JobSystemThreadPool`, and Jolt's global job plumbing (plus a shared
// g_jolt_initialized / Factory gate) doesn't survive two worlds stepping on
// worker threads at once. Run this file single-threaded
// (`--test-threads=1`) until worlds share one pool or tests share one App.
use bevy::prelude::*;
use bevy_jolt::{JointSpace, JoltBody, JoltBodyId, JoltJoint, JoltJointId, JoltPlugin, JoltShape};

fn spawn_pair(app: &mut App) -> (Entity, Entity, Entity) {
    let body_a = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.5, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    let body_b = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 2.6, 0.0),
            JoltBody::dynamic(0),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    let joint = app
        .world_mut()
        .spawn(JoltJoint::fixed(body_a, body_b, JointSpace::World))
        .id();
    (body_a, body_b, joint)
}

fn tick(app: &mut App) {
    // `app.update()` alone may not accumulate enough `Time<Real>` to trip
    // the fixed accumulator, so drive the fixed schedules explicitly. The
    // step lives in JoltStep now: tests run it by hand since the fixed-loop
    // order only applies inside a real App run.
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(bevy_jolt::JoltStep);
}

/// One tick with a real fixed timestep: running the schedules by hand leaves the
/// step delta at ~0, so a test that needs bodies to move advances the clock
/// first. Same helper as `sleep_motion.rs`.
fn moving_tick(app: &mut App) {
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    tick(app);
}
#[test]
fn joint_gains_id_once_bodies_bake() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (_, _, joint) = spawn_pair(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    let joint_id = app
        .world()
        .get::<JoltJointId>(joint)
        .expect("joint should own a Jolt constraint id");
    assert_ne!(joint_id.constraint_id_raw, 0, "Jolt rejected the weld");
}

#[test]
fn body_despawn_cascades_to_joint() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (body_a, _, joint) = spawn_pair(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltJointId>(joint).is_some(),
        "joint should exist before the cascade"
    );
    app.world_mut().despawn(body_a);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get_entity(joint).is_err(),
        "joint on a dead body should be despawned"
    );
    // Surviving body keeps simulating.
    let mut body_ids = app.world_mut().query::<&JoltBodyId>();
    assert_eq!(
        body_ids.iter(app.world()).count(),
        1,
        "only the dead body should be gone"
    );
}

/// `JoltJoint` owns the constraint, so removing the spec has to remove the
/// constraint — not only despawning the entity. The body index is unlinked in
/// that path too, because the id observer runs after the endpoints are gone.
#[test]
fn joint_spec_removal_removes_the_constraint() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (_, body_b, joint) = spawn_pair(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltJointId>(joint).is_some(),
        "joint should own a constraint before the spec leaves"
    );
    let welded_height = app
        .world()
        .get::<Transform>(body_b)
        .expect("body_b should exist")
        .translation
        .y;

    app.world_mut().entity_mut(joint).remove::<JoltJoint>();
    for _ in 0..30 {
        moving_tick(&mut app);
    }
    assert!(
        app.world().get::<JoltJointId>(joint).is_none(),
        "the id should leave with the spec that owns it"
    );
    let unwelded_height = app
        .world()
        .get::<Transform>(body_b)
        .expect("body_b should exist")
        .translation
        .y;
    assert!(
        unwelded_height < welded_height - 0.5,
        "the joint still holds the bodies together ({welded_height} m -> {unwelded_height} m): \
         removing JoltJoint did not remove the constraint"
    );
}
