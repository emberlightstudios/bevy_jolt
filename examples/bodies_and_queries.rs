use bevy_jolt::{CollisionLayers, JoltMotion, JoltWorld};
use bevy::prelude::Vec3;

fn main() {
    let mut jolt_world = JoltWorld::new();
    jolt_world.create_floor(Vec3::new(100.0, 1.0, 100.0), -1.0);

    // Dynamic box above the floor plus a capsule: both should fall and rest.
    let box_body_id = jolt_world.create_box(
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(-1.0, 3.0, 0.0),
        CollisionLayers::MOVING,
        JoltMotion::Dynamic,
    );
    let capsule_body_id = jolt_world.create_capsule(0.5, 0.3, Vec3::new(1.0, 4.0, 0.0), CollisionLayers::MOVING);

    let fixed_delta_time = 1.0 / 60.0;
    let mut physics_step = 0;
    while physics_step < 300
        && (jolt_world.body_is_active(box_body_id) || jolt_world.body_is_active(capsule_body_id))
    {
        physics_step += 1;
        jolt_world.update(fixed_delta_time, 1);
    }

    let (box_position, box_rotation) = jolt_world.body_full_transform(box_body_id);
    let (capsule_position, _capsule_rotation) = jolt_world.body_full_transform(capsule_body_id);
    println!(
        "Box rested at ({:.3}, {:.3}, {:.3}) rotation ({:.3}, {:.3}, {:.3}, {:.3}) after {} steps.",
        box_position.x,
        box_position.y,
        box_position.z,
        box_rotation.x,
        box_rotation.y,
        box_rotation.z,
        box_rotation.w,
        physics_step
    );
    println!(
        "Capsule rested at ({:.3}, {:.3}, {:.3}).",
        capsule_position.x, capsule_position.y, capsule_position.z
    );

    // A box is 1 unit tall sitting on a floor whose top is y=0, so its center
    // should rest near y=0.5. The capsule (half height 0.5 + radius 0.3 = 0.8
    // half extent) should rest near y=0.8.
    assert!(
        (box_position.y - 0.5).abs() < 0.05,
        "box rest height wrong: {}",
        box_position.y
    );
    assert!(
        (capsule_position.y - 0.8).abs() < 0.1,
        "capsule rest height wrong: {}",
        capsule_position.y
    );

    // Ray cast straight down from above: should hit one of the two bodies.
    let ray_hit = jolt_world
        .cast_ray(Vec3::new(-1.0, 10.0, 0.0), Vec3::new(0.0, -10.0, 0.0))
        .expect("downward ray should hit the box");
    println!(
        "Ray hit body {} at fraction {:.3}.",
        ray_hit.hit_body_id, ray_hit.hit_fraction
    );
    assert_eq!(ray_hit.hit_body_id, box_body_id, "ray should hit the box");
    assert!(
        (0.0..1.0).contains(&ray_hit.hit_fraction),
        "fraction should be inside the ray"
    );

    // A static body on NON_MOVING must not fall.
    let wall_body_id = jolt_world.create_box(
        Vec3::new(1.0, 1.0, 0.2),
        Vec3::new(0.0, 2.0, -3.0),
        CollisionLayers::NON_MOVING,
        JoltMotion::Static,
    );
    for _ in 0..60 {
        jolt_world.update(fixed_delta_time, 1);
    }
    let (wall_position, _) = jolt_world.body_full_transform(wall_body_id);
    assert!(
        (wall_position.y - 2.0).abs() < 0.001,
        "static body moved: {}",
        wall_position.y
    );

    for body_id in [box_body_id, capsule_body_id, wall_body_id] {
        jolt_world.remove_and_destroy_body(body_id);
    }
    println!("Bodies, transforms, layers, and ray cast all behaved.");
}
