use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltPhysicsWorld, JoltPlugin};

const MAX_PHYSICS_STEPS: u32 = 600;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_systems(Startup, spawn_physics_scene)
        .add_systems(FixedUpdate, watch_falling_sphere)
        .run();
}

#[derive(Resource)]
struct FallingSphere {
    sphere_body_id: u32,
}

fn spawn_physics_scene(
    mut commands: Commands,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    physics_world
        .create_floor(Vec3::new(100.0, 1.0, 100.0), -1.0);
    let sphere_body_id = physics_world.create_sphere(
        0.5,
        Vec3::new(0.0, 2.0, 0.0),
        CollisionLayers::MOVING,
    );
    commands.insert_resource(FallingSphere { sphere_body_id });
}

fn watch_falling_sphere(
    mut sphere_step_count: Local<u32>,
    falling_sphere: Res<FallingSphere>,
    physics_world: Res<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *sphere_step_count += 1;
    let sphere_snapshot = physics_world
        .body_snapshot(falling_sphere.sphere_body_id);
    println!(
        "Step {}: Position = ({:.3}, {:.3}, {:.3})",
        *sphere_step_count,
        sphere_snapshot.body_position.x,
        sphere_snapshot.body_position.y,
        sphere_snapshot.body_position.z,
    );

    let sphere_sleeping = !physics_world
        .body_is_active(falling_sphere.sphere_body_id);
    if sphere_sleeping || *sphere_step_count >= MAX_PHYSICS_STEPS {
        println!("Sphere slept after {} steps.", *sphere_step_count);
        app_exit.write(AppExit::Success);
    }
}
