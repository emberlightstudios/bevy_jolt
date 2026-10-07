//! Slow motion: a bouncy ball drops and keeps bouncing while the sim speed
//! cycles 1x -> 0.15x -> 1x. Press Space to toggle slow motion by hand; the
//! overlay shows the live scale. Exit with Esc.
//!
//! The fixed step never changes: `time_scale` only shrinks how much sim time
//! each tick advances. The render interpolation keeps the picture smooth.

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltDebugPlugin, JoltPhysicsWorld, JoltPlugin, JoltRestitution, JoltShape,
};

const NORMAL_SCALE: f32 = 1.0;
const SLOW_SCALE: f32 = 0.15;
/// Ticks between automatic toggles, so the demo shows both speeds alone.
const TICKS_PER_PHASE: u32 = 300;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, (spawn_slow_mo_scene, spawn_overlay))
        .add_systems(Update, (toggle_slow_mo, update_overlay))
        .run();
}

#[derive(Resource)]
struct SlowMoPhase {
    slow_motion: bool,
    phase_ticks: u32,
}

fn spawn_slow_mo_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 9.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
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
    commands.spawn((
        Transform::from_xyz(0.0, 5.0, 0.0),
        JoltBody::dynamic(0),
        JoltRestitution::new(0.9),
        JoltShape::sphere(0.5),
    ));
    commands.insert_resource(SlowMoPhase {
        slow_motion: false,
        phase_ticks: 0,
    });
}

fn toggle_slow_mo(
    input: Res<ButtonInput<KeyCode>>,
    mut phase: ResMut<SlowMoPhase>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    if input.just_pressed(KeyCode::Escape) {
        app_exit.write(AppExit::Success);
        return;
    }
    let mut slow_motion = phase.slow_motion;
    if input.just_pressed(KeyCode::Space) {
        // Manual override: flip now and restart the phase clock.
        slow_motion = !slow_motion;
        phase.phase_ticks = 0;
    } else {
        phase.phase_ticks += 1;
        if phase.phase_ticks >= TICKS_PER_PHASE {
            slow_motion = !slow_motion;
            phase.phase_ticks = 0;
        }
    }
    if slow_motion != phase.slow_motion {
        phase.slow_motion = slow_motion;
        physics_world.set_time_scale(if slow_motion {
            SLOW_SCALE
        } else {
            NORMAL_SCALE
        });
    }
}

#[derive(Component)]
struct SlowMoText;

fn spawn_overlay(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont::from_font_size(18.0),
        SlowMoText,
    ));
}

fn update_overlay(
    physics_world: Res<JoltPhysicsWorld>,
    mut overlay: Query<&mut Text, With<SlowMoText>>,
) {
    let Ok(mut overlay_text) = overlay.single_mut() else {
        return;
    };
    let current_scale = physics_world.time_scale();
    overlay_text.0 = format!("time scale: {current_scale:.2}x (Space toggles, Esc exits)");
}
