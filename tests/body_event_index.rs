// Body-id -> entity lookups on the post-step event paths.
//
// The three systems under test (`sync_sleep_states`, `drain_contact_events`,
// and the debug-draw sleep tint) used to resolve a Jolt body id by scanning
// `Query<(Entity, &JoltBodyId)>` once per event. These tests pin the behavior
// a per-invocation index has to preserve, because a lookup table is only
// correct if it maps exactly what the scan mapped -- including the case where
// the right answer is "no entity".
//
// The perf win is O(events x bodies) -> O(events + bodies); these tests cover
// the correctness of that swap, which is where a regression would hide.
//
// Run single-threaded (`--test-threads=1`): parallel `JoltWorld`s crash on
// the shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltContactAdded, JoltMotion, JoltPlugin, JoltShape, JoltSleeping, JoltStep,
    MAX_CONTACT_EVENTS,
};
use std::sync::atomic::{AtomicUsize, Ordering};

fn tick(app: &mut App) {
    app.update();
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(JoltStep);
}

/// A wide floor plus `count` dynamic boxes dropped just above it, spread far
/// enough apart that each settles on its own patch of floor.
fn spawn_scene(app: &mut App, count: usize) -> (Entity, Vec<Entity>) {
    let floor = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, -0.5, 0.0),
            JoltBody::fixed(0),
            JoltShape::box_shape(Vec3::new(400.0, 0.5, 400.0)),
        ))
        .id();
    let boxes = (0..count)
        .map(|i| {
            let x = i as f32 * 4.0;
            app.world_mut()
                .spawn((
                    Transform::from_xyz(x, 2.0, 0.0),
                    JoltBody::dynamic(0),
                    JoltShape::box_shape(Vec3::splat(0.5)),
                ))
                .id()
        })
        .collect();
    (floor, boxes)
}

#[test]
fn contact_events_name_the_two_bodies_in_the_pair() {
    let mut app = App::new();
    let reported: std::sync::Arc<std::sync::Mutex<Vec<(Entity, Entity)>>> =
        std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let sink = reported.clone();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.add_observer(move |trigger: On<JoltContactAdded>| {
        let event = trigger.event();
        sink.lock()
            .expect("contact sink poisoned")
            .push((event.first_entity, event.second_entity));
    });

    let (floor, boxes) = spawn_scene(&mut app, 2);
    for _ in 0..40 {
        tick(&mut app);
    }

    for box_entity in &boxes {
        assert!(
            app.world()
                .get::<bevy_jolt::JoltBodyId>(*box_entity)
                .is_some(),
            "every dynamic box should bake a JoltBodyId"
        );
    }

    let events = reported.lock().expect("contact sink poisoned").clone();
    assert!(
        !events.is_empty(),
        "boxes dropped onto a floor should generate contact events"
    );

    for (first, second) in events {
        // A wrong-but-present mapping would still fire the event, so check the
        // ids actually resolve to live bodies rather than merely existing.
        assert!(
            app.world().get::<bevy_jolt::JoltBodyId>(first).is_some(),
            "reported first entity {first:?} carries no JoltBodyId"
        );
        assert!(
            app.world().get::<bevy_jolt::JoltBodyId>(second).is_some(),
            "reported second entity {second:?} carries no JoltBodyId"
        );
        assert!(
            first != second,
            "a contact pair must name two distinct bodies"
        );
        // With one floor and well-separated boxes, every pair is box+floor.
        // This is what catches an index that maps ids to the wrong entity:
        // two boxes or two floors would both fail it.
        assert!(
            first == floor || second == floor,
            "with separated boxes every pair should include the floor body \
             ({floor:?}), got {first:?}/{second:?}"
        );
    }
}

#[test]
fn sleep_state_reaches_the_right_body_with_many_bodies_present() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (_floor, boxes) = spawn_scene(&mut app, 24);
    for _ in 0..40 {
        tick(&mut app);
    }

    // Force a sleep write on one specific mid-list body. The drain has to
    // translate that body's id back to that exact entity.
    let target = boxes[12];
    app.world_mut()
        .get_mut::<JoltSleeping>(target)
        .expect("box carries sleep state after baking")
        .sleeping = true;
    for _ in 0..5 {
        tick(&mut app);
    }

    let asleep_indices: Vec<usize> = boxes
        .iter()
        .enumerate()
        .filter(|(_, entity)| {
            app.world()
                .get::<JoltSleeping>(**entity)
                .map(|sleeping| sleeping.sleeping)
                .unwrap_or(false)
        })
        .map(|(index, _)| index)
        .collect();

    assert!(
        asleep_indices.contains(&12),
        "the body told to sleep should report asleep, got {asleep_indices:?}"
    );
    assert!(
        asleep_indices.len() <= 2,
        "only the requested body (and at most one genuinely settled \
         neighbour) should be asleep, got {asleep_indices:?}"
    );
}

#[test]
fn contact_drain_stays_within_the_queue_cap_under_load() {
    let mut app = App::new();
    let total = std::sync::Arc::new(AtomicUsize::new(0));
    let sink = total.clone();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.add_observer(move |_trigger: On<JoltContactAdded>| {
        sink.fetch_add(1, Ordering::Relaxed);
    });

    // Many bodies is where the old per-id scan did the most work.
    spawn_scene(&mut app, 40);
    for _ in 0..20 {
        tick(&mut app);
    }

    let reported = total.load(Ordering::Relaxed);
    assert!(
        reported <= 2 * MAX_CONTACT_EVENTS,
        "drain must report at most one added-queue plus one removed-queue \
         worth of pairs per tick, saw {reported} over 20 ticks"
    );
}

#[test]
fn despawned_bodies_are_dropped_from_pairs_rather_than_misfired() {
    let mut app = App::new();
    let reported: std::sync::Arc<std::sync::Mutex<Vec<(Entity, Entity)>>> =
        std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let sink = reported.clone();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.add_observer(move |trigger: On<JoltContactAdded>| {
        let event = trigger.event();
        sink.lock()
            .expect("contact sink poisoned")
            .push((event.first_entity, event.second_entity));
    });

    let (floor, boxes) = spawn_scene(&mut app, 2);
    // Despawn one box BEFORE any contact settles, so the drain meets an id
    // that has no live entity. The pair must be skipped, not reported with a
    // stale or wrong entity.
    app.world_mut().despawn(boxes[1]);
    for _ in 0..40 {
        tick(&mut app);
    }

    assert!(
        !app.world().get_entity(boxes[1]).is_ok(),
        "despawned box should be gone"
    );
    for (first, second) in reported.lock().expect("contact sink poisoned").clone() {
        assert!(
            app.world().get::<bevy_jolt::JoltBodyId>(first).is_some(),
            "reported {first:?} has no JoltBodyId; a despawned body must be \
             skipped, not resolved to something else"
        );
        assert!(
            app.world().get::<bevy_jolt::JoltBodyId>(second).is_some(),
            "reported {second:?} has no JoltBodyId"
        );
        assert!(
            first != second,
            "a contact pair must name two distinct bodies"
        );
        let _ = floor;
    }
}

#[test]
fn floor_body_is_static_so_pair_checks_have_a_stable_anchor() {
    // Guards the fixture itself: the pair assertions above assume exactly one
    // static body in the scene.
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let (floor, boxes) = spawn_scene(&mut app, 2);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert_eq!(
        app.world()
            .get::<JoltBody>(floor)
            .expect("floor carries JoltBody")
            .motion,
        JoltMotion::Static
    );
    for box_entity in boxes {
        assert_eq!(
            app.world()
                .get::<JoltBody>(box_entity)
                .expect("box carries JoltBody")
                .motion,
            JoltMotion::Dynamic
        );
    }
}
