use bevy_jolt::JoltWorld;
use bevy::prelude::Vec3;

fn main() {
    let mut jolt_world = JoltWorld::new();
    let floor_body_id = jolt_world.create_floor(Vec3::new(100.0, 1.0, 100.0), -1.0);
    let sphere_body_id = jolt_world.create_sphere(0.5, 2.0);

    let fixed_delta_time = 1.0 / 60.0;
    let mut physics_step = 0;
    while jolt_world.body_is_active(sphere_body_id) {
        physics_step += 1;
        let sphere_snapshot = jolt_world.body_snapshot(sphere_body_id);
        println!(
            "Step {}: Position = ({:.3}, {:.3}, {:.3}), Velocity = ({:.3}, {:.3}, {:.3})",
            physics_step,
            sphere_snapshot.body_position.x,
            sphere_snapshot.body_position.y,
            sphere_snapshot.body_position.z,
            sphere_snapshot.body_velocity.x,
            sphere_snapshot.body_velocity.y,
            sphere_snapshot.body_velocity.z
        );
        assert!(physics_step < 600, "sphere never went to sleep");
        jolt_world.update(fixed_delta_time, 1);
    }

    jolt_world.remove_and_destroy_body(sphere_body_id);
    jolt_world.remove_and_destroy_body(floor_body_id);
    println!("Sphere slept after {} steps.", physics_step);
}
