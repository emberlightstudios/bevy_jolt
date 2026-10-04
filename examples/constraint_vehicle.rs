//! Vehicle: a four-wheel demo car with ray-cast wheels laps a paddock.
//! Gas held down, WASD steers from the keyboard. Teleports back inside the
//! paddock instead of driving away forever.

use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltBody, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin, JoltShape};

const CAR_SPAWN: Vec3 = Vec3::new(0.0, 1.2, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_scene)
        .add_systems(PreUpdate, create_car_once)
        .add_systems(PreUpdate, drive_car.after(create_car_once))
        .add_systems(FixedPostUpdate, report)
        .add_systems(PostUpdate, sync_car_mesh)
        .run();
}

#[derive(Resource)]
struct Demo {
    car_mesh: Entity,
    car_body: u32,
    car_joint: u32,
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
    commands.insert_resource(Demo {
        car_mesh,
        car_body: 0,
        car_joint: 0,
    });
}

fn create_car_once(
    mut demo: ResMut<Demo>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.car_joint != 0 {
        return;
    }
    let Some((body, joint)) = physics_world
        .create_demo_car(CollisionLayers::MOVING, CAR_SPAWN)
    else {
        panic!("car creation failed");
    };
    demo.car_body = body;
    demo.car_joint = joint;
    println!("car body {body}, vehicle joint id {joint}");
}

fn drive_car(
    demo: Res<Demo>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    if demo.car_joint == 0 {
        return;
    }
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
    let brake = if keyboard.pressed(KeyCode::Space) { 1.0 } else { 0.0 };
    physics_world
        .vehicle_drive(demo.car_joint, forward, steer, brake);
    if demo.car_body != 0 {
        physics_world.clamp_car_to_bounds(
            demo.car_body,
            Vec3::new(-30.0, 0.0, -30.0),
            Vec3::new(30.0, 0.0, 30.0),
        );
    }
}

fn report(mut tick: Local<u32>, demo: Res<Demo>, physics_world: Res<JoltPhysicsWorld>) {
    if demo.car_joint == 0 {
        return;
    }
    *tick += 1;
    if *tick % 300 != 0 {
        return;
    }
    let (position, _) = physics_world
        .body_full_transform(demo.car_body);
    println!(
        "tick {}: car at ({:.1}, {:.1}), height {:.2}.",
        *tick, position.x, position.z, position.y
    );
    if position.y <= 0.2 {
        println!("tick {}: car left its wheels.", *tick);
    }
}

fn sync_car_mesh(
    demo: Res<Demo>,
    physics_world: Res<JoltPhysicsWorld>,
    mut transform_query: Query<&mut Transform>,
) {
    if demo.car_body == 0 {
        return;
    }
    let Ok(mut mesh) = transform_query.get_mut(demo.car_mesh) else {
        return;
    };
    let (position, rotation) = physics_world
        .body_full_transform(demo.car_body);
    mesh.translation = position;
    mesh.rotation = rotation;
}