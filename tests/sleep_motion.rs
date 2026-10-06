// Sleep/wake triggers + motion component: a falling ball freezes on sleep,
// resumes on wake, goes unwakeable on a static motion write, and rejoins on
// writing back. Run single-threaded (`--test-threads=1`): parallel
// `JoltWorld`s crash on the shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltMotion, JoltPhysicsWorld, JoltPlugin, JoltShape, JoltSleep,
    JoltSleeping, JoltStep, JoltWake,
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

fn body_speed(app: &App, ball: Entity) -> f32 {
    let world = app.world().resource::<JoltPhysicsWorld>();
    let body_id = app
        .world()
        .get::<bevy_jolt::JoltBodyId>(ball)
        .expect("ball should be baked");
    world.body_snapshot(body_id.body_id_raw).body_velocity.y
}

#[test]
fn sleep_freezes_and_wake_resumes() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = spawn_ball(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        body_speed(&app, ball) < -1.0,
        "ball should be falling before sleep"
    );
    app.world_mut().trigger(JoltSleep { body_entity: ball });
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(
        body_speed(&app, ball),
        0.0,
        "sleeping ball should report zero velocity"
    );
    assert!(
        app.world().get::<JoltSleeping>(ball).is_some(),
        "slept ball should carry the sleeping marker"
    );
    app.world_mut().trigger(JoltWake { body_entity: ball });
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltSleeping>(ball).is_none(),
        "woken ball should lose the sleeping marker"
    );
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        body_speed(&app, ball) < -1.0,
        "woken ball should fall again"
    );
}

#[test]
fn static_flip_sticks_and_flips_back() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let ball = spawn_ball(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    app.world_mut()
        .get_mut::<JoltBody>(ball)
        .expect("ball exists")
        .motion = JoltMotion::Static;
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(
        body_speed(&app, ball),
        0.0,
        "static-flipped ball should not move"
    );
    assert_eq!(
        app.world()
            .get::<JoltBody>(ball)
            .expect("ball exists")
            .motion,
        JoltMotion::Static,
        "entity motion should agree with Jolt after the flip"
    );
    app.world_mut()
        .get_mut::<JoltBody>(ball)
        .expect("ball exists")
        .motion = JoltMotion::Dynamic;
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        body_speed(&app, ball) < -1.0,
        "flipped-back ball should fall again"
    );
}
