use bevy_jolt::{CollisionLayers, JoltMotion, JoltWorld};
use bevy::prelude::Vec3;

const GHOST_LAYER: u16 = 2;

fn main() {
    // Three teams: ground (0), normal bodies (1), ghosts (2). Ghosts hit
    // nothing, not even each other.
    let mut collision_layers = CollisionLayers::new(3);
    collision_layers.set_collide(GHOST_LAYER, CollisionLayers::NON_MOVING, false);
    collision_layers.set_collide(GHOST_LAYER, CollisionLayers::MOVING, false);
    collision_layers.set_collide(GHOST_LAYER, GHOST_LAYER, false);

    let mut jolt_world = JoltWorld::with_layers(collision_layers);
    jolt_world.create_floor(Vec3::new(100.0, 1.0, 100.0), -1.0);

    // A normal box falls and rests on the floor.
    let box_body_id = jolt_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(-1.0, 3.0, 0.0),
        CollisionLayers::MOVING,
        JoltMotion::Dynamic,
    );

    // A ghost box at the same spot falls straight through the floor.
    let ghost_body_id = jolt_world.create_box(
        Vec3::splat(0.5),
        Vec3::new(1.0, 3.0, 0.0),
        GHOST_LAYER,
        JoltMotion::Dynamic,
    );

    let fixed_delta_time = 1.0 / 60.0;
    for _ in 0..300 {
        jolt_world.update(fixed_delta_time, 1);
    }

    let (box_position, _) = jolt_world.body_full_transform(box_body_id);
    let (ghost_position, _) = jolt_world.body_full_transform(ghost_body_id);
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

    for body_id in [box_body_id, ghost_body_id] {
        jolt_world.remove_and_destroy_body(body_id);
    }
    println!("Custom layers behaved: ghosts ignore everything.");
}
