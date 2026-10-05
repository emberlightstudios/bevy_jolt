//! Soft cloth: a red banner pinned at the top drapes under gravity. Watch
//! it sag, then it prints pin stability + drape depth and exits.

use bevy::prelude::*;
use bevy_jolt::{JoltPhysicsWorld, JoltPlugin, JoltSoftBody, JoltSoftBodyId};

const SETTLE_TICKS: u32 = 400;
const GRID_NX: u32 = 16;
const GRID_NZ: u32 = 12;
const PIN_HEIGHT: f32 = 6.0;
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .insert_resource(ClothDemo::default())
        .add_systems(Startup, spawn_cloth_scene)
        .add_systems(FixedUpdate, watch_cloth_scene)
        .run();
}

#[derive(Resource, Default)]
struct ClothDemo {
    banner: Option<Entity>,
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
            Transform::from_xyz(0.0, PIN_HEIGHT, 0.0),
            JoltSoftBody {
                grid_nx: GRID_NX,
                grid_nz: GRID_NZ,
                spacing: 0.25,
                object_layer: 0,
            },
        ))
        .id();
    demo.banner = Some(banner);
}

fn watch_cloth_scene(
    mut tick_count: Local<u32>,
    demo: Res<ClothDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    let Some(banner) = demo.banner else {
        return;
    };
    let Ok(soft_id) = soft_query.get(banner) else {
        return;
    };
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let vertex_total = (GRID_NX * GRID_NZ) as usize;
    let mut cloth_positions = vec![Vec3::ZERO; vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut cloth_positions);
    assert_eq!(written as usize, vertex_total, "cloth should report every vertex");
    for x in 0..GRID_NX {
        let pin_height = cloth_positions[x as usize].y;
        assert!(
            (pin_height - PIN_HEIGHT).abs() < 0.05,
            "pin {x} should hold at y={PIN_HEIGHT}, got y={pin_height:.3}"
        );
    }
    let lowest_y = cloth_positions
        .iter()
        .map(|vertex_position| vertex_position.y)
        .fold(f32::MAX, f32::min);
    println!("cloth pins hold at y={PIN_HEIGHT}, free edge sags to y={lowest_y:.3}");
    assert!(
        PIN_HEIGHT - lowest_y > 1.0,
        "cloth should drape at least 1m below the pins, sagged {:.3}",
        PIN_HEIGHT - lowest_y
    );
    println!("Banner hangs: pins fixed, cloth draped.");
    app_exit.write(AppExit::Success);
}
