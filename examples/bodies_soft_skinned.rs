//! Skinned cloth: a low-res cloth sim drives flat skeleton joints, and a
//! high-res mesh rides the joints on the GPU. Same slider scene as the
//! cloth demo, but the mesh is never rewritten: per-tick work is a handful
//! of joint transform writes, and `bevy_pbr` bends the dense skin.
//!
//! Shows the pattern: cheap sim (few particles) -> expensive look (dense
//! mesh) via joint weights baked once at spawn.

use std::f32::consts::TAU;

use bevy::mesh::skinning::{SkinnedMesh, SkinnedMeshInverseBindposes};
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use bevy_jolt::{
    JoltBody, JoltDebugPlugin, JoltLinearVelocity, JoltPhysicsWorld, JoltPlugin, JoltShape,
    JoltSoftBodyConfig, JoltSoftBodyId, JoltSoftSharedSettings,
};
/// Sim grid: coarse particles Jolt actually simulates.
const SIM_NX: u32 = 5;
const SIM_NZ: u32 = 4;
/// Skin grid: dense visible mesh, bent on the GPU.
const SKIN_NX: u32 = 33;
const SKIN_NZ: u32 = 25;
const PIN_HEIGHT: f32 = 6.0;
/// Slide period in ticks: the cube glides through the sheet and back.
const SLIDE_TICKS: u32 = 300;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(JoltPlugin::new().with_physics_hz(60.0))
        .add_plugins(JoltDebugPlugin)
        .insert_resource(SkinDemo::default())
        .add_systems(Startup, spawn_skin_scene)
        .add_systems(FixedUpdate, drive_slider)
        .add_systems(FixedUpdate, drive_joints)
        .add_systems(FixedUpdate, watch_skin_scene)
        .run();
}

#[derive(Resource, Default)]
struct SkinDemo {
    sim_body: Option<Entity>,
    slider: Option<Entity>,
    /// Flat joints, one per sim particle, row-major like the sim grid.
    sim_joints: Vec<Entity>,
}

/// Skin vertex weights: the 4 surrounding sim particles + bilinear blend.
/// Baked once at spawn; never touched again.
#[derive(Debug, Clone)]
struct SkinWeights {
    joint_indices: Vec<[u16; 4]>,
    joint_weights: Vec<[f32; 4]>,
}

fn spawn_skin_scene(
    mut commands: Commands,
    mut demo: ResMut<SkinDemo>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut inverse_bindposes: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.5, 8.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(3.0, 8.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Low-res sim: physics only, no mesh marker. Jolt owns these particles.
    let sim_body = commands
        .spawn((
            JoltSoftSharedSettings::cloth(SIM_NX, SIM_NZ, 0.9, 1),
            JoltSoftBodyConfig {
                body_position: Vec3::new(0.0, PIN_HEIGHT, 0.0),
                update_position: false,
                vertex_radius: 0.05,
                friction: 0.0,
                num_iterations: 10,
                ..default()
            },
        ))
        .id();
    // Flat joints: one entity per sim particle, no hierarchy. Cloth has
    // no bones, so joints are just positioned from sim verts each tick.
    let sim_vertex_total = (SIM_NX * SIM_NZ) as usize;
    let mut sim_joints = Vec::with_capacity(sim_vertex_total);
    for _ in 0..sim_vertex_total {
        sim_joints.push(commands.spawn(Transform::IDENTITY).id());
    }
    // High-res skin: positions follow the sim shape at rest; the GPU bends
    // them every frame from the joint transforms. Built once, never rewritten.
    let skin_weights = skin_weights();
    let skin_vertex_total = (SKIN_NX * SKIN_NZ) as usize;
    let mut skin_positions = Vec::with_capacity(skin_vertex_total);
    let mut skin_uvs = Vec::with_capacity(skin_vertex_total);
    for skin_row in 0..SKIN_NZ {
        for skin_column in 0..SKIN_NX {
            let across = skin_column as f32 / (SKIN_NX - 1) as f32;
            let down = skin_row as f32 / (SKIN_NZ - 1) as f32;
            skin_positions.push([
                (across - 0.5) * (SIM_NX as f32 - 1.0) * 0.9,
                PIN_HEIGHT - down * (SIM_NZ as f32 - 1.0) * 0.9,
                0.0,
            ]);
            skin_uvs.push([across, down]);
        }
    }
    let mut skin_triangles = Vec::new();
    for skin_row in 0..SKIN_NZ - 1 {
        for skin_column in 0..SKIN_NX - 1 {
            let top_left = skin_column + skin_row * SKIN_NX;
            let top_right = skin_column + 1 + skin_row * SKIN_NX;
            let bottom_left = skin_column + (skin_row + 1) * SKIN_NX;
            let bottom_right = skin_column + 1 + (skin_row + 1) * SKIN_NX;
            skin_triangles.extend([top_left, bottom_left, bottom_right]);
            skin_triangles.extend([top_left, bottom_right, top_right]);
        }
    }
    let mut skin_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::MAIN_WORLD | bevy::asset::RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, skin_positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, skin_uvs)
    .with_inserted_indices(Indices::U32(skin_triangles));
    skin_mesh.insert_attribute(
        Mesh::ATTRIBUTE_JOINT_INDEX,
        VertexAttributeValues::Uint16x4(skin_weights.joint_indices.clone()),
    );
    skin_mesh.insert_attribute(
        Mesh::ATTRIBUTE_JOINT_WEIGHT,
        VertexAttributeValues::Float32x4(skin_weights.joint_weights.clone()),
    );
    skin_mesh.compute_normals();
    // At rest the joints sit exactly on the sim particles, so every inverse
    // bindpose is the inverse of a pure translation.
    let sim_rest = sim_rest_positions();
    let rest_bindposes: Vec<Mat4> = sim_rest
        .iter()
        .map(|joint_rest| Mat4::from_translation(*joint_rest).inverse())
        .collect();
    commands.spawn((
        Mesh3d(meshes.add(skin_mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.2, 0.2),
            cull_mode: None,
            ..default()
        })),
        SkinnedMesh {
            inverse_bindposes: inverse_bindposes.add(SkinnedMeshInverseBindposes::from(rest_bindposes)),
            joints: sim_joints.clone(),
        },
    ));
    // Same kinematic slider as the cloth demo: presses the sim sheet.
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
    demo.sim_body = Some(sim_body);
    demo.slider = Some(slider);
    demo.sim_joints = sim_joints;
}

/// Rest positions of the sim particles: cloth grid hanging below the pins.
fn sim_rest_positions() -> Vec<Vec3> {
    let mut sim_rest = Vec::with_capacity((SIM_NX * SIM_NZ) as usize);
    let offset_x = -0.5 * 0.9 * (SIM_NX as f32 - 1.0);
    for sim_row in 0..SIM_NZ {
        for sim_column in 0..SIM_NX {
            sim_rest.push(Vec3::new(
                offset_x + sim_column as f32 * 0.9,
                PIN_HEIGHT - sim_row as f32 * 0.9,
                0.0,
            ));
        }
    }
    sim_rest
}

/// Per-skin-vert blend of the 4 surrounding sim particles. Skin space maps
/// onto sim space by fraction, so weights are a plain bilinear blend.
fn skin_weights() -> SkinWeights {
    let mut joint_indices = Vec::with_capacity((SKIN_NX * SKIN_NZ) as usize);
    let mut joint_weights = Vec::with_capacity((SKIN_NX * SKIN_NZ) as usize);
    for skin_row in 0..SKIN_NZ {
        for skin_column in 0..SKIN_NX {
            let sim_x = skin_column as f32 / (SKIN_NX - 1) as f32 * (SIM_NX - 1) as f32;
            let sim_z = skin_row as f32 / (SKIN_NZ - 1) as f32 * (SIM_NZ - 1) as f32;
            let left = sim_x.floor() as u32;
            let top = sim_z.floor() as u32;
            let right = (left + 1).min(SIM_NX - 1);
            let bottom = (top + 1).min(SIM_NZ - 1);
            let blend_x = sim_x - left as f32;
            let blend_z = sim_z - top as f32;
            let top_left = left + top * SIM_NX;
            let top_right = right + top * SIM_NX;
            let bottom_left = left + bottom * SIM_NX;
            let bottom_right = right + bottom * SIM_NX;
            joint_indices.push([
                top_left as u16,
                top_right as u16,
                bottom_left as u16,
                bottom_right as u16,
            ]);
            joint_weights.push([
                (1.0 - blend_x) * (1.0 - blend_z),
                blend_x * (1.0 - blend_z),
                (1.0 - blend_x) * blend_z,
                blend_x * blend_z,
            ]);
        }
    }
    SkinWeights {
        joint_indices,
        joint_weights,
    }
}

/// Sine-wave slide: the cube oscillates through the sheet on a cosine
/// velocity, so the crate's velocity drive moves it and the cloth feels
/// the shove through real velocity, not a teleport.
fn drive_slider(
    mut slide_ticks: Local<u32>,
    demo: Res<SkinDemo>,
    mut velocity_query: Query<&mut JoltLinearVelocity>,
) {
    let Some(slider) = demo.slider else {
        return;
    };
    let Ok(mut slider_drive) = velocity_query.get_mut(slider) else {
        return;
    };
    *slide_ticks += 1;
    // Gentle: 1.5 m/s peak is 0.025 m per tick, far below the 0.9 m gap
    // between sim particles, so verts ride the faces instead of tunneling.
    slider_drive.linear_velocity =
        Vec3::Z * 1.5 * ((*slide_ticks as f32 / SLIDE_TICKS as f32) * TAU).cos();
}

/// Per-tick skinning on the CPU side: read sim positions, write joint
/// transforms. The dense mesh itself is never touched: the GPU bends it
/// from these joints.
fn drive_joints(
    demo: Res<SkinDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    mut joint_query: Query<&mut Transform>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let Some(sim_body) = demo.sim_body else {
        return;
    };
    let Ok(soft_id) = soft_query.get(sim_body) else {
        // Bake hasn't filed the id yet: joints wait at origin one tick.
        return;
    };
    let sim_vertex_total = (SIM_NX * SIM_NZ) as usize;
    let mut sim_positions = vec![Vec3::ZERO; sim_vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut sim_positions);
    if written as usize != sim_vertex_total {
        return;
    }
    for (joint_entity, sim_position) in demo.sim_joints.iter().zip(sim_positions.iter()) {
        let Ok(mut joint_pose) = joint_query.get_mut(*joint_entity) else {
            continue;
        };
        joint_pose.translation = *sim_position;
    }
}

/// No-clip guard: the slider oscillates through the sim sheet, so every
/// tick asserts no sim particle ends up inside it, then the demo exits.
/// Also draws the coarse sim particles as gizmo spheres over the shaded
/// skin, so the few drivers stay visible under the smooth result.
fn watch_skin_scene(
    mut tick_count: Local<u32>,
    demo: Res<SkinDemo>,
    soft_query: Query<&JoltSoftBodyId>,
    transform_query: Query<&Transform>,
    physics_world: Res<JoltPhysicsWorld>,
    mut gizmos: Gizmos,
) {
    let (Some(sim_body), Some(slider)) = (demo.sim_body, demo.slider) else {
        return;
    };
    let Ok(soft_id) = soft_query.get(sim_body) else {
        return;
    };
    let Ok(slider_pose) = transform_query.get(slider) else {
        return;
    };
    *tick_count += 1;
    let sim_vertex_total = (SIM_NX * SIM_NZ) as usize;
    let mut sim_positions = vec![Vec3::ZERO; sim_vertex_total];
    let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut sim_positions);
    if written as usize != sim_vertex_total {
        return;
    }
    let slider_position = slider_pose.translation;
    for sim_position in &sim_positions {
        let inside_slider = (sim_position.x - slider_position.x).abs() < 0.75
            && (sim_position.y - slider_position.y).abs() < 0.75
            && (sim_position.z - slider_position.z).abs() < 0.75;
        assert!(
            !inside_slider,
            "sim should ride the slider, not clip it: particle at {sim_position:?}, slider at {slider_position:?}"
        );
        gizmos.sphere(
            Isometry3d::new(*sim_position, Quat::IDENTITY),
            0.06,
            Color::srgb(0.2, 0.9, 0.3),
        );
    }
}
