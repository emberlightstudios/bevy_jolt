//! Ragdoll: an 11-part humanoid drops onto the floor, crumples, and
//! settles. Bodies render as physics wireframes; the console still prints
//! the rest span, then it exits.

use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltBodyId, JoltDebugPlugin, JoltMotion, JoltPhysicsWorld, JoltPlugin, JoltRagdoll,
    JoltRagdollParts, JoltShape, RagdollJoint, RagdollPart, RagdollShape,
};
const SETTLE_TICKS: u32 = 400;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .add_systems(Startup, spawn_ragdoll_scene)
        .add_systems(FixedUpdate, watch_ragdoll_scene)
        .run();
}

#[derive(Resource)]
struct RagdollDemo {
    doll: Entity,
}
fn spawn_ragdoll_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 2.5, 6.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
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
    let doll = commands
        .spawn((Transform::from_xyz(0.0, 0.5, 0.0), demo_humanoid(0)))
        .id();
    commands.insert_resource(RagdollDemo { doll });
}

fn watch_ragdoll_scene(
    mut tick_count: Local<u32>,
    demo: Res<RagdollDemo>,
    ragdoll_query: Query<&JoltRagdollParts>,
    transform_query: Query<&Transform>,
    body_ids: Query<&JoltBodyId>,
    physics_world: Res<JoltPhysicsWorld>,
    mut app_exit: MessageWriter<AppExit>,
) {
    *tick_count += 1;
    if *tick_count < SETTLE_TICKS {
        return;
    }
    if *tick_count > SETTLE_TICKS {
        return;
    }
    let Ok(ragdoll_parts) = ragdoll_query.get(demo.doll) else {
        return;
    };
    assert_eq!(
        ragdoll_parts.part_entities.len(),
        11,
        "humanoid should bake 11 parts"
    );
    let mut lowest_y = f32::MAX;
    let mut highest_y = f32::MIN;
    for part_entity in &ragdoll_parts.part_entities {
        let Ok(part_pose) = transform_query.get(*part_entity) else {
            return;
        };
        lowest_y = lowest_y.min(part_pose.translation.y);
        highest_y = highest_y.max(part_pose.translation.y);
    }
    println!("ragdoll spans y={lowest_y:.3}..{highest_y:.3}");
    assert!(
        lowest_y < 0.3,
        "ragdoll should lie on the floor, lowest part at y={lowest_y:.3}"
    );
    assert!(
        highest_y - lowest_y < 1.2,
        "ragdoll should crumple flat, span is {:.3}",
        highest_y - lowest_y
    );
    let mut sleeping_parts = 0;
    for part_entity in &ragdoll_parts.part_entities {
        let Ok(part_id) = body_ids.get(*part_entity) else {
            return;
        };
        if !physics_world.body_is_active(part_id.body_id_raw) {
            sleeping_parts += 1;
        }
    }
    println!("ragdoll sleeps {sleeping_parts}/11 parts");
    assert_eq!(
        sleeping_parts, 11,
        "damping should settle every part to sleep"
    );
    println!("Ragdoll fell, folded, and slept.");
    app_exit.write(AppExit::Success);
}

/// Demo-only humanoid: hips root, torso, head, two arms (upper + lower), two
/// legs (upper + lower). Offsets hang down -y from the spawn; anchors are
/// midpoints between parent/child positions.
fn demo_humanoid(object_layer: u16) -> JoltRagdoll {
    let offsets = [
        Vec3::new(0.0, 0.9, 0.0),
        Vec3::new(0.0, 1.3, 0.0),
        Vec3::new(0.0, 1.7, 0.0),
        Vec3::new(-0.25, 1.35, 0.0),
        Vec3::new(-0.25, 1.05, 0.0),
        Vec3::new(0.25, 1.35, 0.0),
        Vec3::new(0.25, 1.05, 0.0),
        Vec3::new(-0.12, 0.5, 0.0),
        Vec3::new(-0.12, 0.1, 0.0),
        Vec3::new(0.12, 0.5, 0.0),
        Vec3::new(0.12, 0.1, 0.0),
    ];
    let parents = [
        None,
        Some(0),
        Some(1),
        Some(1),
        Some(3),
        Some(1),
        Some(5),
        Some(0),
        Some(7),
        Some(0),
        Some(9),
    ];
    let anchor_for = |part: usize| match parents[part] {
        Some(parent) => (offsets[parent] + offsets[part]) * 0.5,
        None => offsets[part],
    };
    let swing = |part: usize, cone: f32| RagdollJoint::SwingTwist {
        anchor: anchor_for(part),
        normal_half_cone: cone,
        plane_half_cone: cone,
        twist_min: -cone,
        twist_max: cone,
    };
    let capsule_part = |part: usize, half_height: f32, radius: f32, cone: f32| RagdollPart {
        shape: RagdollShape::Capsule {
            cylinder_half_height: half_height,
            radius,
        },
        part_position: offsets[part],
        part_rotation: Quat::IDENTITY,
        parent_part: parents[part],
        joint: swing(part, cone),
    };
    let limb_cone = 0.6;
    JoltRagdoll {
        parts: vec![
            capsule_part(0, 0.15, 0.15, 0.0),
            capsule_part(1, 0.2, 0.14, 0.5),
            capsule_part(2, 0.1, 0.12, 0.5),
            capsule_part(3, 0.12, 0.06, limb_cone),
            capsule_part(4, 0.12, 0.05, limb_cone),
            capsule_part(5, 0.12, 0.06, limb_cone),
            capsule_part(6, 0.12, 0.05, limb_cone),
            capsule_part(7, 0.2, 0.08, limb_cone),
            capsule_part(8, 0.2, 0.07, limb_cone),
            capsule_part(9, 0.2, 0.08, limb_cone),
            capsule_part(10, 0.2, 0.07, limb_cone),
        ],
        object_layer,
        density_kg_per_m3: 1000.0,
        motion: JoltMotion::Dynamic,
    }
}
