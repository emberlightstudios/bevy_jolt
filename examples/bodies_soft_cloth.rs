//! Soft cloth: a red banner pinned at the top drapes under gravity. Watch
//! it sag, then it prints pin stability + drape depth and exits.

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltBodyId, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin, JoltShape, JoltSoftBody,
    JoltSoftBodyId,
};

const GRID_NX: u32 = 16;
const GRID_NZ: u32 = 12;
const PIN_HEIGHT: f32 = 6.0;
/// Slide period: the cube glides fully out to fully in over this many ticks.
const SLIDE_TICKS: u32 = 800;
/// Cube z when clear of the sheet vs pressed into it.
const SLIDE_OUT_Z: f32 = -2.5;
const SLIDE_IN_Z: f32 = 0.35;
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .insert_resource(ClothDemo::default())
        .add_systems(Startup, spawn_cloth_scene)
        .add_systems(
            FixedUpdate,
            drive_slider.before(bevy_jolt::step_physics_world),
        )
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
            Transform::from_xyz(0.0, PIN_HEIGHT, 0.0),
            JoltSoftBody {
                grid_nx: GRID_NX,
                grid_nz: GRID_NZ,
                spacing: 0.25,
                object_layer: 0,
            },
        ))
        .id();
    // Sliding shoulder stand: a kinematic cube glides in (pressing the sheet
    // into a drape) and back out (letting it fall free) on a sine loop. The
    // debug plugin draws its wireframe so the invisible hand is visible.
    let slider = commands
        .spawn((
            Transform::from_xyz(0.0, 4.6, SLIDE_OUT_Z),
            JoltBody::kinematic(0),
            JoltShape::box_shape(Vec3::new(0.8, 0.8, 0.8)),
        ))
        .id();
    demo.banner = Some(banner);
    demo.slider = Some(slider);
}

/// Sine-wave slide: cube glides from clear of the sheet to pressed into it
/// and back. `MoveKinematic` derives velocity so the cloth feels the shove.
fn drive_slider(
    mut slide_ticks: Local<u32>,
    demo: Res<ClothDemo>,
    body_ids: Query<&JoltBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    fixed_time: Res<Time<Fixed>>,
) {
    let Some(slider) = demo.slider else {
        return;
    };
    let Ok(slider_id) = body_ids.get(slider) else {
        return;
    };
    *slide_ticks += 1;
    let slide_phase = (*slide_ticks as f32 / SLIDE_TICKS as f32) * std::f32::consts::TAU;
    // Cosine eases out and back: tick 0 out, half period fully in.
    let slide_blend = 0.5 - 0.5 * slide_phase.cos();
    let slider_z = SLIDE_OUT_Z + (SLIDE_IN_Z - SLIDE_OUT_Z) * slide_blend;
    physics_world.move_kinematic(
        slider_id.body_id_raw,
        Vec3::new(0.0, 4.6, slider_z),
        Quat::IDENTITY,
        fixed_time.delta().as_secs_f32(),
    );
}

fn watch_cloth_scene(
    mut tick_count: Local<u32>,
    demo: Res<ClothDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    transform_query: Query<&Transform>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
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
    // Gentle breeze so the sheet breathes; the slider does the real work.
    let breeze_phase = *tick_count as f32 * 0.15;
    physics_world.soft_push(
        soft_id.body_id_raw,
        Vec3::new(0.0, 0.0, 400.0 + 300.0 * breeze_phase.sin()),
    );
    *tick_count += 1;
    // Check twice per slide: fully in (drape) and fully out (free hang).
    let check_in = *tick_count == SLIDE_TICKS / 2;
    let check_out = *tick_count == SLIDE_TICKS;
    if !check_in && !check_out {
        return;
    }
    let vertex_total = (GRID_NX * GRID_NZ) as usize;
    let mut cloth_positions = vec![Vec3::ZERO; vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut cloth_positions);
    assert_eq!(written as usize, vertex_total, "cloth should report every vertex");
    for vertex_index in 0..GRID_NX {
        let pin_height = cloth_positions[vertex_index as usize].y;
        assert!(
            (pin_height - PIN_HEIGHT).abs() < 0.05,
            "pin {vertex_index} should hold at y={PIN_HEIGHT}, got y={pin_height:.3}"
        );
    }
    // No vertex may end up inside the slider, wherever it currently is.
    // Half extents 0.8 shrink-wrapped by the vertex radius margin.
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
    if check_in {
        // Drape proof: slider fully in, sheet lies on its top face.
        assert!(
            (slider_position.z - SLIDE_IN_Z).abs() < 0.15,
            "mid-run check should catch the slider fully in, got z={:.3}",
            slider_position.z
        );
        let rests_on_top = cloth_positions.iter().any(|vertex_position| {
            (vertex_position.x - slider_position.x).abs() < 0.8
                && (vertex_position.z - slider_position.z).abs() < 0.8
                && (vertex_position.y - (slider_position.y + 0.8)).abs() < 0.25
        });
        assert!(rests_on_top, "sheet should lie on the slider's top face");
        println!("tick {}: slider in, sheet drapes it, nothing clips.", *tick_count);
    }
    if check_out {
        // Free proof: slider fully out, sheet hangs clear of it.
        assert!(
            (slider_position.z - SLIDE_OUT_Z).abs() < 0.15,
            "final check should catch the slider fully out, got z={:.3}",
            slider_position.z
        );
        let clear_of_slider = cloth_positions.iter().all(|vertex_position| {
            (vertex_position.z - slider_position.z).abs() > 1.0
        });
        assert!(clear_of_slider, "sheet should hang clear once the slider leaves");
        println!("tick {}: slider out, sheet hangs free.", *tick_count);
        println!("Slider slides in and out; cloth drapes and releases.");
        app_exit.write(AppExit::Success);
    }
}
