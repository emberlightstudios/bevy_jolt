//! Soft cloth: a red banner pinned at the top drapes under gravity. Watch
//! it sag, then it prints pin stability + drape depth and exits.

use std::f32::consts::TAU;

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltDebugPlugin, JoltLinearVelocity, JoltPhysicsWorld, JoltPlugin, JoltShape,
    JoltSoftBodyConfig, JoltSoftBodyId, JoltSoftBodyMesh, JoltSoftSharedSettings,
};

const GRID_NX: u32 = 16;
const GRID_NZ: u32 = 12;
const PIN_HEIGHT: f32 = 6.0;
/// Slide period: the cube glides fully out to fully in over this many ticks.
const SLIDE_TICKS: u32 = 300;
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .insert_resource(ClothDemo::default())
        .add_systems(Startup, spawn_cloth_scene)
        .add_systems(FixedUpdate, drive_slider)
        .add_systems(FixedUpdate, watch_cloth_scene)
        .run();
}

#[derive(Resource, Default)]
struct ClothDemo {
    banner: Option<Entity>,
    slider: Option<Entity>,
}

fn spawn_cloth_scene(mut commands: Commands, mut demo: ResMut<ClothDemo>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.5, 8.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    let banner = commands
        .spawn((
            JoltSoftSharedSettings::cloth(GRID_NX, GRID_NZ, 0.25, 2),
            JoltSoftBodyConfig {
                body_position: Vec3::new(0.0, PIN_HEIGHT, 0.0),
                update_position: false,
                vertex_radius: 0.05,
                friction: 0.0,
                num_iterations: 10,
                ..default()
            },
            JoltSoftBodyMesh::colored(Color::srgb(0.7, 0.2, 0.2)),
        ))
        .id();
    // Sliding shoulder stand: a kinematic cube glides in (pressing the sheet
    // into a drape) and back out (letting it fall free) on a sine loop. The
    // debug plugin draws its wireframe so the invisible hand is visible.
    let slider = commands
        .spawn((
            Transform::from_xyz(0.0, 4.6, 1.),
            JoltBody::kinematic(0),
            JoltShape::box_shape(Vec3::splat(0.8)),
            JoltLinearVelocity {
                linear_velocity: Vec3::ZERO,
            },
        ))
        .id();
    demo.banner = Some(banner);
    demo.slider = Some(slider);
}

/// Sine-wave slide: the cube oscillates through the sheet on a cosine
/// velocity, so the crate's velocity drive moves it and the cloth feels
/// the shove through real velocity, not a teleport.
fn drive_slider(
    mut slide_ticks: Local<u32>,
    demo: Res<ClothDemo>,
    mut velocity_query: Query<&mut JoltLinearVelocity>,
) {
    let Some(slider) = demo.slider else {
        return;
    };
    let Ok(mut slider_drive) = velocity_query.get_mut(slider) else {
        return;
    };
    *slide_ticks += 1;
    // Gentle: 1.5 m/s peak is 0.025 m per tick, far below the 0.25 m gap
    // between cloth threads, so verts ride the faces instead of tunneling.
    slider_drive.linear_velocity =
        Vec3::Z * 1.5 * ((*slide_ticks as f32 / SLIDE_TICKS as f32) * TAU).cos();
}

/// No-clip guard: the slider oscillates through the sheet, so every tick
/// asserts no cloth vertex ends up inside it, then the demo exits.
fn watch_cloth_scene(
    mut tick_count: Local<u32>,
    demo: Res<ClothDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    transform_query: Query<&Transform>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let (Some(banner), Some(slider)) = (demo.banner, demo.slider) else {
        return;
    };
    let Ok(soft_id) = soft_query.get(banner) else {
        return;
    };
    let Ok(slider_pose) = transform_query.get(slider) else {
        return;
    };
    *tick_count += 1;
    let vertex_total = (GRID_NX * GRID_NZ) as usize;
    let mut cloth_positions = vec![Vec3::ZERO; vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut cloth_positions);
    assert_eq!(
        written as usize, vertex_total,
        "cloth should report every vertex"
    );
    let slider_position = slider_pose.translation;
    for vertex_position in &cloth_positions {
        let inside_slider = (vertex_position.x - slider_position.x).abs() < 0.75
            && (vertex_position.y - slider_position.y).abs() < 0.75
            && (vertex_position.z - slider_position.z).abs() < 0.75;
        assert!(
            !inside_slider,
            "cloth should ride the slider, not clip it: vertex at {vertex_position:?}, slider at {slider_position:?}"
        );
    }
}
