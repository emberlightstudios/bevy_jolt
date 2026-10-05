//! Soft cloth banners: spawn spec, bake, mesh rebuild, despawn.
//!
//! Add [`JoltSoftBody`] (grid size + spacing + layer) with a `Transform`.
//! The plugin creates the Jolt cloth (top row pinned, hangs like a
//! curtain), builds a Bevy mesh with grid topology once, then rewrites
//! vertex positions from Jolt every Fixed tick after the step.

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};

use crate::plugin::JoltPhysicsWorld;

/// Soft cloth banner descriptor: grid resolution + spacing + layer. Top row
/// pins at the spawn position, the rest drapes under gravity.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltSoftBody {
    /// Vertices across (columns). 2 minimum.
    pub grid_nx: u32,
    /// Vertices down (rows). 2 minimum.
    pub grid_nz: u32,
    /// Rest distance between neighbors in meters.
    pub spacing: f32,
    /// Collision layer, same teams as rigid bodies.
    pub object_layer: u16,
}

/// Live soft body id, filed at bake. Marks mesh-holding entities.
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltSoftBodyId {
    pub body_id_raw: u32,
}

/// Creates the Jolt cloth when [`JoltSoftBody`] is added: builds the shared
/// settings (top row pinned), files the id, and spawns the mesh with grid
/// topology. Vertex positions fill in on the first sync after the step.
pub fn bake_jolt_soft_body(
    trigger: On<Add, JoltSoftBody>,
    mut commands: Commands,
    soft_query: Query<(&JoltSoftBody, &Transform)>,
    meshes: Option<ResMut<Assets<Mesh>>>,
) {
    let soft_entity = trigger.event().entity;
    let Ok((soft, soft_transform)) = soft_query.get(soft_entity) else {
        panic!("JoltSoftBody gone on {soft_entity:?} before bake ran");
    };
    assert!(
        soft.grid_nx >= 2 && soft.grid_nz >= 2,
        "cloth grid must be at least 2x2, got {}x{}",
        soft.grid_nx,
        soft.grid_nz
    );
    assert!(
        soft.spacing > 0.0 && soft.spacing.is_finite(),
        "cloth spacing must be positive, got {}",
        soft.spacing
    );
    let body_id_raw = physics_world.create_cloth(
        soft.grid_nx,
        soft.grid_nz,
        soft.spacing,
        soft_transform.translation,
        soft.object_layer,
    );
    assert_ne!(body_id_raw, 0, "Jolt rejected the cloth banner");
    // Headless apps (no AssetPlugin) get physics without a mesh: vertex
    // data stays readable through `soft_vertices` for tests and servers.
    let (Some(meshes), Some(materials)) = (meshes, materials) else {
        commands
            .entity(soft_entity)
            .insert(JoltSoftBodyId { body_id_raw });
        return;
    };
    let vertex_total = (soft.grid_nx * soft.grid_nz) as usize;
    let cloth_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[0.0, 0.0, 0.0]; vertex_total],
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, cloth_uvs(soft.grid_nx, soft.grid_nz))
    .with_inserted_indices(Indices::U32(cloth_indices(soft.grid_nx, soft.grid_nz)));
    commands.entity(soft_entity).insert((
        JoltSoftBodyId { body_id_raw },
        // World-space mesh: Jolt hands us world verts, so the entity sits
        // at the origin. (Leaving the spawn Transform here would offset
        // the banner twice and fling it off-camera.)
        Transform::IDENTITY,
        Mesh3d(meshes.into_inner().add(cloth_mesh)),
        MeshMaterial3d(materials.into_inner().add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.2, 0.2),
            cull_mode: None,
            ..default()
        })),
    ));
}
/// Grid triangulation, wound for an upward face: two triangles per quad,
/// same winding as the Jolt faces so the visible side matches.
fn cloth_indices(grid_nx: u32, grid_nz: u32) -> Vec<u32> {
    let mut cloth_indices = Vec::new();
    for z in 0..grid_nz - 1 {
        for x in 0..grid_nx - 1 {
            let top_left = x + z * grid_nx;
            let top_right = x + 1 + z * grid_nx;
            let bottom_left = x + (z + 1) * grid_nx;
            let bottom_right = x + 1 + (z + 1) * grid_nx;
            cloth_indices.extend([top_left, bottom_left, bottom_right]);
            cloth_indices.extend([top_left, bottom_right, top_right]);
        }
    }
    cloth_indices
}

/// Full-quad UVs so a texture (or solid color) spans the banner evenly.
fn cloth_uvs(grid_nx: u32, grid_nz: u32) -> Vec<[f32; 2]> {
    let mut cloth_uvs = Vec::new();
    for z in 0..grid_nz {
        for x in 0..grid_nx {
            cloth_uvs.push([
                x as f32 / (grid_nx - 1) as f32,
                z as f32 / (grid_nz - 1) as f32,
            ]);
        }
    }
    cloth_uvs
}

/// Rewrites every cloth mesh from Jolt vertex positions after the step.
/// Cloths missing their id (not baked yet) are skipped.
pub fn sync_soft_body_meshes(
    soft_query: Query<(&JoltSoftBody, &JoltSoftBodyId, &Mesh3d)>,
    mut meshes: Option<ResMut<Assets<Mesh>>>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let Some(mut meshes) = meshes else {
        return;
    };
    for (soft, soft_id, cloth_handle) in &soft_query {
        let vertex_total = (soft.grid_nx * soft.grid_nz) as usize;
        let mut cloth_positions = vec![Vec3::ZERO; vertex_total];
        let written = physics_world.soft_vertices(soft_id.body_id_raw, &mut cloth_positions);
        if written == 0 {
            continue;
        }
        let Some(mut cloth_mesh) = meshes.get_mut(cloth_handle) else {
            continue;
        };
        let flat_positions: Vec<[f32; 3]> = cloth_positions
            .iter()
            .map(|vertex_position| vertex_position.to_array())
            .collect();
        cloth_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, flat_positions);
        cloth_mesh.compute_normals();
    }
}

/// Destroys the Jolt cloth when its entity goes. Order-independent: the
/// soft registry never touches rigid bodies.
pub fn despawn_jolt_soft_body(
    trigger: On<Remove, JoltSoftBodyId>,
    soft_query: Query<&JoltSoftBodyId>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let trigger_entity = trigger.event().entity;
    let Ok(soft_id) = soft_query.get(trigger_entity) else {
        panic!("JoltSoftBodyId gone on {trigger_entity:?} before despawn ran");
    };
    physics_world.soft_destroy(soft_id.body_id_raw);
}
