//! Soft balloon: a pressurized sphere drops onto the floor, squashes, and
//! holds its shape. Proves the generic API: sphere shared settings +
//! pressure config + opt-in mesh, no cloth grid anywhere.

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltPhysicsWorld, JoltPlugin, JoltShape, JoltSoftBodyConfig, JoltSoftBodyId,
    JoltSoftBodyMesh, JoltSoftSharedSettings,
};
const BALLOON_RADIUS: f32 = 1.0;
const BALLOON_SPAWN: Vec3 = Vec3::new(0.0, 6.0, 0.0);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .insert_resource(BalloonDemo::default())
        .add_systems(Startup, spawn_balloon_scene)
        .add_systems(FixedUpdate, watch_balloon_scene)
        .run();
}

#[derive(Resource, Default)]
struct BalloonDemo {
    balloon: Option<Entity>,
    settled_ticks: u32,
}

fn spawn_balloon_scene(mut commands: Commands, mut demo: ResMut<BalloonDemo>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 10.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, -1.0, 0.0),
        JoltBody::fixed(0),
        JoltShape::box_shape(Vec3::new(100.0, 1.0, 100.0)),
    ));
    let balloon = commands
        .spawn((
            JoltSoftSharedSettings::sphere(BALLOON_RADIUS, 10, 20),
            JoltSoftBodyConfig {
                body_position: BALLOON_SPAWN,
                pressure: 2000.0,
                vertex_radius: 0.05,
                num_iterations: 10,
                ..default()
            },
            JoltSoftBodyMesh::colored(Color::srgb(0.2, 0.5, 0.8)),
        ))
        .id();
    demo.balloon = Some(balloon);
}

/// Waits for the fall, then asserts the balloon squashed but holds volume:
/// pressure keeps it inflated instead of collapsing flat.
fn watch_balloon_scene(
    mut tick_count: Local<u32>,
    mut demo: ResMut<BalloonDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let Some(balloon) = demo.balloon else {
        return;
    };
    let Ok(soft_id) = soft_query.get(balloon) else {
        return;
    };
    *tick_count += 1;
    if *tick_count < 300 {
        return;
    }
    let vertex_total = physics_world.soft_vertex_count(soft_id.body_id_raw) as usize;
    assert!(vertex_total > 0, "balloon should report vertices");
    let mut balloon_positions = vec![Vec3::ZERO; vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut balloon_positions);
    assert_eq!(
        written as usize, vertex_total,
        "balloon should report every vertex"
    );
    let lowest_y = balloon_positions
        .iter()
        .map(|vertex_position| vertex_position.y)
        .fold(f32::MAX, f32::min);
    let highest_y = balloon_positions
        .iter()
        .map(|vertex_position| vertex_position.y)
        .fold(f32::MIN, f32::max);
    let squashed_height = highest_y - lowest_y;
    println!("balloon rests at y={lowest_y:.3}, stands {squashed_height:.3} tall");
    // Squashed but not flat: pressure holds shape against the floor.
    assert!(
        lowest_y < 0.3,
        "balloon should rest on the floor, got lowest y={lowest_y:.3}"
    );
    assert!(
        squashed_height > 1.0,
        "pressure should hold the balloon up, got height={squashed_height:.3}"
    );
    let balloon_volume = physics_world.soft_volume(soft_id.body_id_raw);
    assert!(
        balloon_volume > 1.0,
        "balloon should hold volume, got {balloon_volume:.3}"
    );
    println!("Balloon squashes and holds; pressure works.");
    demo.balloon = None;
    demo.settled_ticks = *tick_count;
}
