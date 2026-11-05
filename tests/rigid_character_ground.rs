//! A rigid character's ground reading is opt-in, and the opt-out must not cost it its
//! pose: the component is what asks for the post-step ground query, while the pose
//! copy happens either way.
//!
//! Run single-threaded (`--test-threads=1`): parallel `JoltWorld`s crash on the
//! shared job plumbing (see joint_lifecycle.rs).
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltCharacterGround, JoltPhysicsWorld, JoltPlugin, JoltRigidCharacter,
    JoltRigidCharacterId, JoltShape,
};

/// Capsule centre above the entity origin, so the feet sit on the origin.
const CAPSULE_CENTER_HEIGHT: f32 = 0.8;

fn tick(app: &mut App) {
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
    app.world_mut().run_schedule(bevy_jolt::JoltStep);
}

/// A tick that actually advances simulation time: `tick` alone leaves the step delta
/// at ~0, so nothing moves.
fn moving_tick(app: &mut App) {
    let timestep = app.world().resource::<Time<Fixed>>().timestep();
    app.world_mut()
        .resource_mut::<Time<Fixed>>()
        .advance_by(timestep);
    tick(app);
}

fn app_with_floor() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    app.world_mut().spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(50.0, 0.1, 50.0)),
    ));
    app
}

/// The failure this opt-out must never cause: a character that asked for no ground
/// being skipped by the sync and left frozen on screen while its body falls.
#[test]
fn rigid_character_without_ground_component_still_syncs_its_pose() {
    let mut app = app_with_floor();
    let character = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            JoltRigidCharacter {
                capsule_center_height: CAPSULE_CENTER_HEIGHT,
                ..JoltRigidCharacter::new(0)
            },
        ))
        .id();

    for _ in 0..30 {
        moving_tick(&mut app);
    }

    assert!(
        app.world().get::<JoltRigidCharacterId>(character).is_some(),
        "rigid character should bake"
    );
    assert!(
        app.world().get::<JoltCharacterGround>(character).is_none(),
        "the bake handed out a ground component: every rigid character pays the query again"
    );
    let fallen_to = app
        .world()
        .get::<Transform>(character)
        .expect("the character keeps its Transform")
        .translation
        .y;
    assert!(
        fallen_to < 4.5,
        "pose sync skipped a character without the ground component: still at {fallen_to} m"
    );
}

/// The other half: carrying the component is what asks for the reading, and on a
/// floor it comes back supported.
#[test]
fn rigid_character_with_ground_component_reports_support() {
    let mut app = app_with_floor();
    let character = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 0.5, 0.0),
            JoltRigidCharacter {
                capsule_center_height: CAPSULE_CENTER_HEIGHT,
                ..JoltRigidCharacter::new(0)
            },
            JoltCharacterGround::default(),
        ))
        .id();

    for _ in 0..60 {
        moving_tick(&mut app);
    }

    let ground = app
        .world()
        .get::<JoltCharacterGround>(character)
        .expect("a character that asked for ground keeps the component");
    assert!(
        ground.is_supported,
        "a capsule resting on a floor reports no support: state {:?}, normal {:?}",
        ground.ground_state, ground.ground_normal
    );
}
