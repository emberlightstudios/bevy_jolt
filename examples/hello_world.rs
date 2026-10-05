use bevy::prelude::*;
use bevy_jolt::{JoltBody, JoltBodyId, JoltPhysicsWorld, JoltPlugin, JoltShape, JoltTeleport};

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
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let sphere = commands
        .spawn((
            Transform::from_xyz(0.0, 2.0, 0.0),
            JoltBody::dynamic(0),
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
    mut commands: Commands,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Ok((sphere_transform, sphere_id)) = body_query.get(falling_sphere.sphere) else {
        return;
    };
    *sphere_step_count += 1;
    if *sphere_step_count == 60 {
        commands.trigger(JoltTeleport {
            body_entity: falling_sphere.sphere,
            target_position: Vec3::new(3.0, 4.0, 0.0),
            target_rotation: Quat::IDENTITY,
        });
        println!("teleported the sphere to (3, 4, 0)");
    }
    if *sphere_step_count == 61 {
        assert!(
            (sphere_transform.translation - Vec3::new(3.0, 4.0, 0.0)).length() < 0.5,
            "teleport should land the sphere near (3, 4, 0), got {:?}",
            sphere_transform.translation
        );
        println!("teleport confirmed at {:?}", sphere_transform.translation);
    }
    let sphere_sleeping = !physics_world.body_is_active(sphere_id.body_id_raw);
    if sphere_sleeping || *sphere_step_count >= MAX_PHYSICS_STEPS {
        println!("Sphere slept after {} steps.", *sphere_step_count);
        app_exit.write(AppExit::Success);
    }
}
