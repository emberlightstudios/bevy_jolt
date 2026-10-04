use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltBody, JoltPlugin, JoltShape};

const GHOST_LAYER: u16 = 2;
const SETTLE_TICKS: u32 = 300;

fn main() {
    // Three teams: ground (0), normal bodies (1), ghosts (2). Ghosts hit
    // nothing, not even each other. Custom teams ride on the plugin: the
    // layer table is handed to Jolt once at world creation.
    let mut collision_layers = CollisionLayers::new(3);
    collision_layers.set_collide(GHOST_LAYER, CollisionLayers::NON_MOVING, false);
    collision_layers.set_collide(GHOST_LAYER, CollisionLayers::MOVING, false);
    collision_layers.set_collide(GHOST_LAYER, GHOST_LAYER, false);

    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(
            JoltPlugin::new()
                .with_collision_layers(collision_layers)
                .with_physics_hz(60.0),
        )
        .add_systems(Startup, spawn_layer_demo)
        .add_systems(FixedUpdate, watch_layer_demo)
        .run();
}

#[derive(Resource)]
struct LayerDemoBodies {
    normal_box: Entity,
    ghost_box: Entity,
}

fn spawn_layer_demo(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let normal_box = commands
        .spawn((
            Transform::from_xyz(-1.0, 3.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    // A ghost box at the same spot falls straight through the floor.
    let ghost_box = commands
        .spawn((
            Transform::from_xyz(1.0, 3.0, 0.0),
            JoltBody::dynamic(GHOST_LAYER),
            JoltShape::box_shape(Vec3::splat(0.5)),
        ))
        .id();
    commands.insert_resource(LayerDemoBodies {
        normal_box,
        ghost_box,
    });
}

fn watch_layer_demo(
    mut tick_count: Local<u32>,
    demo_bodies: Res<LayerDemoBodies>,
    transform_query: Query<&Transform>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    let Ok(normal_transform) = transform_query.get(demo_bodies.normal_box) else {
        return;
    };
    let Ok(ghost_transform) = transform_query.get(demo_bodies.ghost_box) else {
        return;
    };
    let box_height = normal_transform.translation.y;
    let ghost_height = ghost_transform.translation.y;
    println!(
        "Box rested at y={:.3}, ghost fell to y={:.3}.",
        box_height, ghost_height
    );
    assert!(
        (box_height - 0.5).abs() < 0.05,
        "normal box should rest on the floor"
    );
    assert!(ghost_height < -5.0, "ghost box should fall through the floor");
    println!("Custom layers behaved: ghosts ignore everything.");
    app_exit.write(AppExit::Success);
}
