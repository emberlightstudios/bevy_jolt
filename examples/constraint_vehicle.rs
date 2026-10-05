//! Vehicle: a four-wheel car with ray-cast wheels laps a paddock.
//! Gas held down, WASD steers from the keyboard. A held [`JoltVehicleDrive`]
//! steers back toward the paddock instead of driving away forever.

use bevy::prelude::*;
use bevy_jolt::{
    CollisionLayers, JoltBody, JoltDebugPlugin, JoltPlugin, JoltShape, JoltVehicle,
    JoltVehicleDrive, JoltVehicleId,
};

const CAR_SPAWN: Vec3 = Vec3::new(0.0, 1.2, 0.0);
const PADDOCK_HALF: f32 = 30.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, steer_car)
        .add_systems(PostUpdate, sync_car_mesh)
        .run();
}

#[derive(Resource)]
struct Demo {
    car: Entity,
    car_mesh: Entity,
}

fn spawn_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 18.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(300.0, 2.0, 300.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.35, 0.35, 0.38))),
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(150.0, 1.0, 150.0)),
    ));
    commands.spawn((
        Text::new("vehicle (WASD to steer)"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(16.0),
            ..default()
        },
    ));

    let car_mesh = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.8, 1.2, 4.4))),
            MeshMaterial3d(materials.add(Color::srgb(0.8, 0.15, 0.2))),
            Transform::from_translation(CAR_SPAWN),
        ))
        .id();
    let car = commands
        .spawn((
            JoltVehicle::new(CollisionLayers::MOVING),
            JoltVehicleDrive {
                forward: 0.6,
                steer: 0.0,
                brake: 0.0,
            },
            Transform::from_translation(CAR_SPAWN),
        ))
        .id();
    commands.insert_resource(Demo { car, car_mesh });
}

fn steer_car(
    demo: Res<Demo>,
    keyboard: Res<ButtonInput<KeyCode>>,
    vehicle_query: Query<&JoltVehicleId>,
    mut drive_query: Query<(&mut JoltVehicleDrive, &Transform)>,
) {
    // Creation bakes a flush after spawn: no id yet means not drivable.
    if vehicle_query.get(demo.car).is_err() {
        return;
    }
    let Ok((mut drive, car_transform)) = drive_query.get_mut(demo.car) else {
        return;
    };
    let forward = if keyboard.pressed(KeyCode::KeyW) {
        1.0
    } else if keyboard.pressed(KeyCode::KeyS) {
        -0.6
    } else {
        // Cruise so the demo moves on its own; keys override.
        0.6
    };
    let steer = if keyboard.pressed(KeyCode::KeyA) {
        -0.6
    } else if keyboard.pressed(KeyCode::KeyD) {
        0.6
    } else {
        0.0
    };
    let brake = if keyboard.pressed(KeyCode::Space) {
        1.0
    } else {
        0.0
    };
    // Steer back toward the paddock instead of teleporting: turn around when
    // near the edge so the car laps on its own.
    let car_position = car_transform.translation;
    let (drive_forward, drive_steer) = if car_position.x.abs() > PADDOCK_HALF - 6.0
        || car_position.z.abs() > PADDOCK_HALF - 6.0
    {
        (
            0.6,
            if car_position.x > car_position.z {
                0.6
            } else {
                -0.6
            },
        )
    } else {
        (forward, steer)
    };
    drive.forward = drive_forward;
    drive.steer = drive_steer;
    drive.brake = brake;
}

fn sync_car_mesh(
    demo: Res<Demo>,
    car_query: Query<&Transform, With<JoltVehicle>>,
    mut mesh_query: Query<&mut Transform, Without<JoltVehicle>>,
) {
    let Ok(car_pose) = car_query.get(demo.car) else {
        return;
    };
    let Ok(mut mesh_pose) = mesh_query.get_mut(demo.car_mesh) else {
        return;
    };
    *mesh_pose = *car_pose;
}
