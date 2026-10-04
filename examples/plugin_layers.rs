use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltMotion, JoltPhysicsWorld, JoltPlugin};

const GHOST_LAYER: u16 = 2;
const SETTLE_TICKS: u32 = 300;

fn main() {
    // Custom teams ride on the plugin: the layer table is handed to
    // Jolt once at world creation.
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
        .add_systems(Startup, spawn_physics_scene)
        .add_systems(FixedUpdate, watch_layer_demo)
        .run();
}

#[derive(Resource)]
struct LayerDemoBodies {
    box_body_id: u32,
    ghost_body_id: u32,
}

fn spawn_physics_scene(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    physics_world
        .create_floor(Vec3::new(100.0, 1.0, 100.0), -1.0);

    let box_body_id = physics_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(-1.0, 3.0, 0.0),
        CollisionLayers::MOVING,
        JoltMotion::Dynamic,
    );
    let ghost_body_id = physics_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(1.0, 3.0, 0.0),
        GHOST_LAYER,
        JoltMotion::Dynamic,
    );
    commands.insert_resource(LayerDemoBodies {
        box_body_id,
        ghost_body_id,
    });
}

fn watch_layer_demo(
    mut tick_count: Local<u32>,
    demo_bodies: Res<LayerDemoBodies>,
    physics_world: Res<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }

    let (box_position, _) = physics_world
        .body_full_transform(demo_bodies.box_body_id);
    let (ghost_position, _) = physics_world
        .body_full_transform(demo_bodies.ghost_body_id);
    println!(
        "Box rested at y={:.3}, ghost fell to y={:.3}.",
        box_position.y, ghost_position.y
    );
    assert!(
        (box_position.y - 0.5).abs() < 0.05,
        "normal box should rest on the floor"
    );
    assert!(
        ghost_position.y < -5.0,
        "ghost box should have fallen through the floor"
    );
    println!("Plugin layers behaved: ghosts ignore everything.");
    app_exit.write(AppExit::Success);
}
