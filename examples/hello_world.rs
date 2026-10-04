use bevy::prelude::*;
use bevy_jolt::{CollisionLayers, JoltBody, JoltBodyId, JoltPhysicsWorld, JoltPlugin, JoltShape};

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
    sphere: Entity,
}

fn spawn_physics_scene(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(CollisionLayers::NON_MOVING),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let sphere = commands
        .spawn((
            Transform::from_xyz(0.0, 2.0, 0.0),
            JoltBody::dynamic(CollisionLayers::MOVING),
            JoltShape::sphere(0.5),
        ))
        .id();
    commands.insert_resource(FallingSphere { sphere });
}

fn watch_falling_sphere(
    mut sphere_step_count: Local<u32>,
    falling_sphere: Res<FallingSphere>,
    body_query: Query<(&Transform, &JoltBodyId)>,
    physics_world: Res<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok((sphere_transform, sphere_id)) = body_query.get(falling_sphere.sphere) else {
        return;
    };
    *sphere_step_count += 1;
    let sphere_position = sphere_transform.translation;
    println!(
        "Step {}: Position = ({:.3}, {:.3}, {:.3})",
        *sphere_step_count, sphere_position.x, sphere_position.y, sphere_position.z,
    );

    let sphere_sleeping = !physics_world.body_is_active(sphere_id.body_id_raw);
    if sphere_sleeping || *sphere_step_count >= MAX_PHYSICS_STEPS {
        println!("Sphere slept after {} steps.", *sphere_step_count);
        app_exit.write(AppExit::Success);
    }
}
