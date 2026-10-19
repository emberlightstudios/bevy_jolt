// Soft-body lifecycle: a soft body that has actually been simulated must survive
// teardown. Regression: `bjolt_soft_destroy` called `DestroyBody` without the
// `RemoveBody` every other destroy path makes first, so an *active* soft body
// stayed in Jolt's active-soft-body list after its memory was freed and the next
// `PhysicsSystem::Update` walked it — STATUS_ACCESS_VIOLATION. The earlier
// teardown tests never advanced `Time<Fixed>`, so no soft body ever activated
// and nothing caught this. Run single-threaded (`--test-threads=1`): parallel
// `JoltWorld`s crash on the shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltPhysicsWorld, JoltPlugin, JoltSoftBodyConfig, JoltSoftBodyId, JoltSoftSharedSettings,
    JoltStep, SoftBodyConfig,
};

/// One tick with a real fixed timestep. Running the fixed schedules by hand
/// accumulates no `Time<Real>`, so the step delta stays ~0 and the body never
/// activates — the same helper `sleep_motion.rs` uses.
fn moving_tick(app: &mut App) {
    app.update();
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(JoltStep);
}

fn spawn_cloth(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((
            JoltSoftSharedSettings::cloth(4, 4, 0.25, 1),
            JoltSoftBodyConfig::default(),
        ))
        .id()
}

fn live_bodies(app: &mut App) -> u32 {
    app.world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .body_stats()
        .0
}

fn shared_handle(app: &App, cloth_entity: Entity) -> u64 {
    app.world()
        .get::<JoltSoftBodyId>(cloth_entity)
        .expect("cloth should bake")
        .shared_handle
}

/// Despawning a simulated cloth must not leave a stale entry in Jolt's active
/// soft bodies: the step *after* the despawn is the crash site.
#[test]
fn simulated_cloth_despawn_survives_the_next_step() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let cloth_entity = spawn_cloth(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    let cloth_shared_handle = shared_handle(&app, cloth_entity);
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies + 1,
        "the cloth should be a live body before the despawn"
    );

    app.world_mut().entity_mut(cloth_entity).despawn();
    for _ in 0..10 {
        moving_tick(&mut app);
    }

    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "the soft body outlived its entity"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .create_soft_body(cloth_shared_handle, &SoftBodyConfig::default()),
        0,
        "the freed shared-settings handle still builds bodies: the settings leaked"
    );
}

/// Removing the settings that own a simulated soft body takes the body and the
/// process-global settings handle with it — and must likewise survive the next
/// step. This is the removal path the retire observer adds.
#[test]
fn simulated_cloth_settings_removal_survives_the_next_step() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let cloth_entity = spawn_cloth(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    let cloth_shared_handle = shared_handle(&app, cloth_entity);

    app.world_mut()
        .entity_mut(cloth_entity)
        .remove::<JoltSoftSharedSettings>();
    for _ in 0..10 {
        moving_tick(&mut app);
    }

    assert!(
        app.world().get::<JoltSoftBodyId>(cloth_entity).is_none(),
        "the id should leave with the settings that own it"
    );
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "the soft body outlived the settings that own it"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .create_soft_body(cloth_shared_handle, &SoftBodyConfig::default()),
        0,
        "the freed shared-settings handle still builds bodies: the settings leaked"
    );
}
