use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltBody, JoltShape, JoltBodyId, JoltDebugPlugin, JoltPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_physics_entities)
        .add_systems(FixedUpdate, watch_entity_bodies)
        .run();
}

#[derive(Resource)]
struct EntityDemo {
    box_entity: Entity,
    sphere_entity: Entity,
}

fn spawn_physics_entities(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-6.0, 5.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 3000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING), JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));

    let box_mesh = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    let box_material = materials.add(Color::srgb(0.2, 0.7, 0.3));
    let box_entity = commands
        .spawn((
            Mesh3d(box_mesh),
            MeshMaterial3d(box_material),
            Transform::from_xyz(-1.0, 3.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();

    let sphere_mesh = meshes.add(Sphere::new(0.5));
    let sphere_material = materials.add(Color::srgb(0.8, 0.3, 0.2));
    let sphere_entity = commands
        .spawn((
            Mesh3d(sphere_mesh),
            MeshMaterial3d(sphere_material),
            Transform::from_xyz(1.0, 4.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING), JoltShape::sphere(0.5),
        ))
        .id();

    commands.insert_resource(EntityDemo {
        box_entity,
        sphere_entity,
    });
}

fn watch_entity_bodies(
    mut tick_count: Local<u32>,
    demo_entities: Res<EntityDemo>,
    transform_query: Query<&Transform>,
    body_query: Query<&JoltBodyId>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < 300 {
        return;
    }

    for demo_entity in [demo_entities.box_entity, demo_entities.sphere_entity] {
        let entity_transform = transform_query
            .get(demo_entity)
            .expect("demo entity should still exist");
        let body_id = body_query
            .get(demo_entity)
            .expect("demo entity should own a Jolt body");
        println!(
            "Entity {:?} at y={:.3} owns body {}.",
            demo_entity, entity_transform.translation.y, body_id.body_id_raw
        );
    }

    let box_height = transform_query
        .get(demo_entities.box_entity)
        .expect("box entity should still exist")
        .translation
        .y;
    assert!(
        (box_height - 0.5).abs() < 0.05,
        "entity mesh should follow the physics body"
    );
    println!("Entity sync behaved: meshes follow physics bodies.");
    app_exit.write(AppExit::Success);
}
