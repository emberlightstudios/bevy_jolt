// NOTE: run single-threaded (`--test-threads=1`). Parallel `JoltWorld`s
// crash each other today: every world builds its own ThreadPool and Jolt's
// global job plumbing doesn't survive two worlds on worker threads at once.
use bevy::prelude::*;
use bevy_jolt::{
    JoltPlugin, JoltTrackedDrive, JoltVehicleDrive, JoltVehicleId, JoltVehicleShift, VehicleSpec,
    VehicleTrack, VehicleWheel,
};

fn tick(app: &mut App) {
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
}

fn spawn_car(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((
            VehicleSpec::new(0),
            JoltVehicleDrive {
                forward: 0.5,
                steer: 0.0,
                brake: 0.0,
                hand_brake: 0.0,
            },
            Transform::from_xyz(0.0, 1.2, 0.0),
        ))
        .id()
}

#[test]
fn wheeled_vehicle_gains_id_and_drives() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let car = spawn_car(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    let vehicle_id = app
        .world()
        .get::<JoltVehicleId>(car)
        .expect("vehicle should own a Jolt body + constraint id");
    assert_ne!(vehicle_id.body_id_raw, 0, "Jolt rejected the chassis");
    assert_ne!(
        vehicle_id.constraint_id_raw, 0,
        "Jolt rejected the vehicle constraint"
    );
    // Drive + shift paths run without panicking on a live constraint.
    for _ in 0..10 {
        tick(&mut app);
    }
    app.world_mut().entity_mut(car).insert(JoltVehicleShift {
        gear: 1,
        clutch_friction: 1.0,
    });
    for _ in 0..10 {
        tick(&mut app);
    }
}

#[test]
fn tracked_vehicle_builds_two_tracks() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let mut spec = VehicleSpec::new(0);
    spec.wheels = (0..4)
        .map(|i| VehicleWheel {
            mount_position: Vec3::new(
                if i % 2 == 0 { 0.9 } else { -0.9 },
                -0.5,
                if i < 2 { 1.0 } else { -1.0 },
            ),
            wheel_radius: 0.3,
            wheel_width: 0.25,
            tracked: true,
            ..VehicleWheel::default()
        })
        .collect();
    spec.kind = bevy_jolt::VehicleKind::Tracked {
        tracks: [
            VehicleTrack {
                wheel_indices: vec![0, 2],
                driven_wheel: 1,
                ..VehicleTrack::default()
            },
            VehicleTrack {
                wheel_indices: vec![1, 3],
                driven_wheel: 1,
                ..VehicleTrack::default()
            },
        ],
    };
    let tank = app
        .world_mut()
        .spawn((
            spec,
            JoltTrackedDrive {
                forward: 0.5,
                left_ratio: 1.0,
                right_ratio: 0.5,
                brake: 0.0,
            },
            Transform::from_xyz(0.0, 1.2, 0.0),
        ))
        .id();
    for _ in 0..10 {
        tick(&mut app);
    }
    let vehicle_id = app
        .world()
        .get::<JoltVehicleId>(tank)
        .expect("tank should own a Jolt body + constraint id");
    assert_ne!(vehicle_id.constraint_id_raw, 0, "Jolt rejected the tank");
    for _ in 0..10 {
        tick(&mut app);
    }
}

#[test]
fn motorcycle_preset_builds() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let bike = app
        .world_mut()
        .spawn((
            VehicleSpec::motorcycle(0),
            JoltVehicleDrive {
                forward: 0.3,
                steer: 0.0,
                brake: 0.0,
                hand_brake: 0.0,
            },
            Transform::from_xyz(0.0, 1.0, 0.0),
        ))
        .id();
    for _ in 0..10 {
        tick(&mut app);
    }
    let vehicle_id = app
        .world()
        .get::<JoltVehicleId>(bike)
        .expect("bike should own a Jolt body + constraint id");
    assert_ne!(vehicle_id.constraint_id_raw, 0, "Jolt rejected the bike");
    for _ in 0..10 {
        tick(&mut app);
    }
}

#[test]
fn vehicle_despawn_destroys_chassis() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let car = spawn_car(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltVehicleId>(car).is_some(),
        "vehicle should exist before despawn"
    );
    app.world_mut().despawn(car);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get_entity(car).is_err(),
        "vehicle entity should be gone"
    );
}

// A driven vehicle owes Bevy its chassis pose: the post-step sync must see
// the entity, so `Transform` (and the interpolation anchor) track the
// moving chassis. Regression: the chassis was baked without `JoltSleeping`,
// which dropped the vehicle out of the sync query — render interpolation
// then blended the mesh between a spawn anchor frozen at bake and the live
// chassis, so the car teleported back toward its spawn point every frame
// while the debug wireframe drove away.
#[test]
fn driven_vehicle_syncs_chassis_pose_to_entity() {
    use bevy_jolt::{JoltBody, JoltPhysicsWorld, JoltShape, JoltStep, PreviousBodyTransform};
    use std::time::Duration;

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new().with_physics_hz(60.0)));
    // Flat ground slab so the drive is unambiguous and the car never falls
    // off the edge mid-run.
    app.world_mut().spawn((
        JoltBody::fixed(0),
        Transform::IDENTITY,
        JoltShape::box_shape(Vec3::new(200.0, 0.5, 200.0)),
    ));
    let car = spawn_car(&mut app);
    // Real fixed ticks: advance `Time<Fixed>` so the sim actually steps.
    for _ in 0..180 {
        app.world_mut()
            .resource_mut::<Time<Fixed>>()
            .advance_by(Duration::from_secs_f32(1.0 / 60.0));
        app.world_mut().run_schedule(FixedUpdate);
        app.world_mut().run_schedule(JoltStep);
    }
    let world = app.world();
    let vehicle_id = world
        .get::<JoltVehicleId>(car)
        .expect("car should own a Jolt body + constraint id");
    let (chassis_position, chassis_rotation) = world
        .resource::<JoltPhysicsWorld>()
        .body_full_transform(vehicle_id.body_id_raw);
    // The sim really moved the car: without this the pose match below could
    // pass vacuously on a frozen world.
    assert!(
        chassis_position.z > 1.0,
        "driven chassis should have driven forward, got {chassis_position}"
    );
    // The sync copied the chassis pose out to the entity transform.
    let car_transform = world
        .get::<Transform>(car)
        .expect("car entity should still exist");
    assert!(
        car_transform.translation.distance(chassis_position) < 0.05,
        "entity transform {:?} should match the chassis {chassis_position}",
        car_transform.translation
    );
    assert!(
        car_transform.rotation.angle_between(chassis_rotation) < 0.05,
        "entity rotation should match the chassis"
    );
    // The interpolation anchor trails the chassis by one tick, not by the
    // whole drive distance a frozen spawn anchor would leave behind.
    let anchor = world
        .get::<PreviousBodyTransform>(car)
        .expect("car entity should carry the interpolation anchor");
    assert!(
        anchor.previous_position.distance(chassis_position) < 0.5,
        "interpolation anchor {:?} should trail the chassis {chassis_position} by one tick",
        anchor.previous_position
    );
}

/// `VehicleSpec` owns the constraint *and* the chassis body creation built, so
/// removing the spec frees both — not only despawning the entity. Before this,
/// removal fired no observer: the vehicle kept driving with nothing referring
/// to it, and a re-add built a second chassis while overwriting the id.
#[test]
fn vehicle_spec_removal_frees_constraint_and_chassis() {
    use bevy_jolt::{JoltBodyId, JoltPhysicsWorld};

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, JoltPlugin::new()));
    let baseline_bodies = app
        .world_mut()
        .resource_mut::<JoltPhysicsWorld>()
        .body_stats()
        .0;
    let car = spawn_car(&mut app);
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltVehicleId>(car).is_some(),
        "vehicle should bake before the spec leaves"
    );

    app.world_mut().entity_mut(car).remove::<VehicleSpec>();
    for _ in 0..10 {
        tick(&mut app);
    }
    assert!(
        app.world().get::<JoltVehicleId>(car).is_none(),
        "the constraint id should leave with the spec that owns it"
    );
    assert!(
        app.world().get::<JoltBodyId>(car).is_none(),
        "the chassis id should leave with the spec that owns it"
    );
    assert_eq!(
        app.world_mut()
            .resource_mut::<JoltPhysicsWorld>()
            .body_stats()
            .0,
        baseline_bodies,
        "removing VehicleSpec leaked its chassis body"
    );
}
