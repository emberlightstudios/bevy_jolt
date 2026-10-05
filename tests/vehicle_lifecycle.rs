// NOTE: run single-threaded (`--test-threads=1`). Parallel `JoltWorld`s
// crash each other today: every world builds its own ThreadPool and Jolt's
// global job plumbing doesn't survive two worlds on worker threads at once.
use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltTrackedDrive, JoltVehicleDrive, JoltVehicleId, JoltVehicleShift,
    JoltPlugin, VehicleSpec, VehicleTrack, VehicleWheel,
};

fn tick(app: &mut App) {
    app.update();
    app.world_mut().run_schedule(FixedUpdate);
}

fn spawn_car(app: &mut App) -> Entity {
    app.world_mut()
        .spawn((
            VehicleSpec::new(CollisionLayers::MOVING),
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
    let mut spec = VehicleSpec::new(CollisionLayers::MOVING);
    spec.wheels = (0..4)
        .map(|i| VehicleWheel {
            mount_position: Vec3::new(if i % 2 == 0 { 0.9 } else { -0.9 }, -0.5, if i < 2 { 1.0 } else { -1.0 }),
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
            VehicleSpec::motorcycle(CollisionLayers::MOVING),
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
