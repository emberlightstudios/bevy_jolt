// Ragdoll lifecycle: a ragdoll owns its part bodies, so nothing else may free
// one. There are two supported ways to remove a ragdoll — despawn its entity, or
// remove `JoltRagdoll` — and each cleans up bodies, constraints and part
// entities at once. Run single-threaded (`--test-threads=1`): parallel
// `JoltWorld`s crash on the shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltMotion, JoltPhysicsWorld, JoltPlugin, JoltRagdoll, JoltRagdollParts, JoltShape,
    JoltStep, RagdollJoint, RagdollPart, RagdollShape,
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

/// Despawning the ragdoll entity frees its bodies and takes the part entities
/// with it.
#[test]
fn ragdoll_despawn_releases_bodies_and_parts() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let ragdoll_entity = spawn_ragdoll(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    let parts = part_entities(&app, ragdoll_entity);
    assert_eq!(parts.len(), 2, "spec has two parts");
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
    for part_entity in parts {
        assert!(
            app.world().get_entity(part_entity).is_err(),
            "the parts should be despawned with the ragdoll"
        );
    }
}

/// Removing the spec component on a live entity is the other supported trigger,
/// and it has to leave nothing behind either: the parts are children of the
/// entity, so nothing else would despawn them, and an entity holding an id whose
/// body is gone is polled by the transform syncs every tick.
#[test]
fn removing_the_spec_releases_bodies_and_parts() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = live_bodies(&mut app);
    let ragdoll_entity = spawn_ragdoll(&mut app);
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    let parts = part_entities(&app, ragdoll_entity);

    app.world_mut()
        .entity_mut(ragdoll_entity)
        .remove::<JoltRagdoll>();
    for _ in 0..10 {
        moving_tick(&mut app);
    }
    assert_eq!(
        live_bodies(&mut app),
        baseline_bodies,
        "removing JoltRagdoll leaked its part bodies"
    );
    for part_entity in parts {
        assert!(
            app.world().get_entity(part_entity).is_err(),
            "the parts should be despawned by the teardown"
        );
    }
    assert!(
        app.world().get_entity(ragdoll_entity).is_ok(),
        "only the spec component was removed, not the entity"
    );
}

/// Body ids get recycled. A fresh body that lands on an id a ragdoll used to own
/// must still be destroyable — nothing may remember that id as ragdoll-owned.
#[test]
fn recycled_body_ids_are_still_destroyable() {
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
        "a body was left behind: something still treated its id as ragdoll-owned"
    );
}
