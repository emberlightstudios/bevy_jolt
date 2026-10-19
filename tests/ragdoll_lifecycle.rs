// Ragdoll lifecycle: a ragdoll owns its part bodies, so nothing else may
// destroy one. That ownership is a ledger in the world, not a marker on the
// entity: removing `RagdollPartBody` must not turn a teardown into a per-entity
// destroy followed by the bulk destroy (which crashes). Run single-threaded
// (`--test-threads=1`): parallel `JoltWorld`s crash on the shared job plumbing
// (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltMotion, JoltPhysicsWorld, JoltPlugin, JoltRagdoll, JoltRagdollParts, JoltShape,
    JoltStep, RagdollJoint, RagdollPart, RagdollPartBody, RagdollShape,
};

/// One tick with a real fixed timestep, so the parts actually simulate.
fn moving_tick(app: &mut App) {
    app.update();
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(JoltStep);
}

/// Two-part ragdoll: a root capsule with a capsule hanging off it.
fn pair_ragdoll(object_layer: u16) -> JoltRagdoll {
    let root_position = Vec3::new(0.0, 2.0, 0.0);
    let child_position = Vec3::new(0.0, 2.5, 0.0);
    let child_anchor = (root_position + child_position) * 0.5;
    let part_shape = RagdollShape::Capsule {
        cylinder_half_height: 0.2,
        radius: 0.1,
    };
    JoltRagdoll {
        parts: vec![
            RagdollPart {
                shape: part_shape,
                part_position: root_position,
                part_rotation: Quat::IDENTITY,
                parent_part: None,
                // Root joint is ignored by the bake.
                joint: RagdollJoint::SwingTwist {
                    anchor: child_anchor,
                    normal_half_cone: 0.5,
                    plane_half_cone: 0.5,
                    twist_min: 0.0,
                    twist_max: 0.0,
                },
            },
            RagdollPart {
                shape: part_shape,
                part_position: child_position,
                part_rotation: Quat::IDENTITY,
                parent_part: Some(0),
                joint: RagdollJoint::SwingTwist {
                    anchor: child_anchor,
                    normal_half_cone: 0.5,
                    plane_half_cone: 0.5,
                    twist_min: -0.5,
                    twist_max: 0.5,
                },
            },
        ],
        object_layer,
        density_kg_per_m3: 1000.0,
        motion: JoltMotion::Dynamic,
    }
}

fn spawn_ragdoll(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((Transform::from_xyz(0.0, 2.0, 0.0), pair_ragdoll(0)))
        .id()
}

fn live_bodies(app: &mut App) -> u32 {
    app.world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .body_stats()
        .0
}

fn part_entities(app: &App, ragdoll_entity: Entity) -> Vec<Entity> {
    app.world()
        .get::<JoltRagdollParts>(ragdoll_entity)
        .expect("ragdoll should bake")
        .part_entities
        .clone()
}

#[test]
fn ragdoll_despawn_releases_its_bodies() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let ragdoll_entity = spawn_ragdoll(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(part_entities(&app, ragdoll_entity).len(), 2, "spec has two parts");
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies + 2,
        "both part bodies should be live after the bake"
    );

    app.world_mut().entity_mut(ragdoll_entity).despawn();
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "the ragdoll's bodies outlived it"
    );
}

/// Despawning a part on its own must not destroy its body: the ragdoll owns it,
/// and the teardown destroys the whole set (a second destroy crashes). This is
/// the case the old marker guard could not survive — it read the entity, so a
/// part whose marker was gone looked like an ordinary body.
#[test]
fn part_without_its_marker_is_still_owned_by_the_ragdoll() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let ragdoll_entity = spawn_ragdoll(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    let first_part = part_entities(&app, ragdoll_entity)[0];

    app.world_mut()
        .entity_mut(first_part)
        .remove::<RagdollPartBody>();
    app.world_mut().entity_mut(first_part).despawn();
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies + 2,
        "the part's body should still be alive: the ragdoll owns it, not the entity"
    );

    app.world_mut().entity_mut(ragdoll_entity).despawn();
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "the teardown should free every part body, marker or not"
    );
}

/// Ids get recycled, so ownership has to die with the ragdoll. A stale entry
/// would make the destroy path skip whatever body lands on that id — a silent
/// leak, and a body the game can no longer remove.
#[test]
fn ownership_does_not_outlive_the_ragdoll() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let ragdoll_entity = spawn_ragdoll(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    app.world_mut().entity_mut(ragdoll_entity).despawn();
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "teardown should free both part bodies"
    );

    // Fresh bodies most likely reuse the ids just freed.
    let body_entity = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltBody::dynamic(0),
            JoltShape::sphere(0.5),
        ))
        .id();
    for _ in 0..5 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies + 1,
        "the fresh body should be live"
    );

    app.world_mut().entity_mut(body_entity).despawn();
    for _ in 0..5 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "a body was left behind: a stale ownership entry made the destroy path skip it"
    );
}
