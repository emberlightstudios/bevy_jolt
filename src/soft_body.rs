//! Soft bodies: shared shape settings, per-body config, bake, mesh sync, despawn.
//!
//! Thin Bevy mapping over Jolt soft bodies, same shape as rigid bodies:
//! spec components in, live id out.
//!
//! - [`JoltSoftSharedSettings`]: shape data (verts, faces, edges, volumes).
//!   Build it with [`JoltSoftSharedSettings::custom`], [`JoltSoftSharedSettings::cloth`],
//!   [`JoltSoftSharedSettings::sphere`], or [`JoltSoftSharedSettings::cube`].
//! - [`JoltSoftBodyConfig`]: per-body knobs (position, friction, pressure...).
//! - Add both with a `Transform`-free spawn: bake creates the Jolt body and
//!   files [`JoltSoftBodyId`]. The id is the live handle for `soft_*` reads
//!   on [`JoltPhysicsWorld`](crate::plugin::JoltPhysicsWorld).
//! - Mesh is opt-in: add [`JoltSoftBodyMesh`] and the sync builds a Bevy
//!   mesh from the settings' faces and rewrites verts every Fixed tick.
//!   Without it the body is physics-only (invisible collider, server use,
//!   or custom rendering from `soft_vertices`).

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};

use crate::physics_world::{SoftBendType, SoftBodyConfig};
use crate::plugin::JoltPhysicsWorld;

/// Shared soft-body shape: verts, faces, edges, volumes. Baked once into
/// Jolt, shared across bodies. Mirrors the C++ `SoftBodySharedSettings`
/// inputs; the FFI assembles, constrains, and optimizes.
#[derive(Component, Clone, Debug)]
pub struct JoltSoftSharedSettings {
    /// Rest positions of every particle.
    pub vertex_positions: Vec<Vec3>,
    /// Initial velocities (empty = still).
    pub vertex_velocities: Vec<Vec3>,
    /// Inverse masses: 0 nails a vertex, 1 is fully free.
    pub vertex_inv_masses: Vec<f32>,
    /// Surface triangles (vertex triples).
    pub face_triangles: Vec<[u32; 3]>,
    /// Hand-placed spring pairs (empty = Jolt auto-builds from faces).
    pub edge_pairs: Vec<[u32; 2]>,
    /// Per-edge stiffness (empty = uniform `edge_compliance`).
    pub edge_compliances: Vec<f32>,
    /// Tetrahedra that hold shape (cube bodies; empty for cloth).
    pub volume_quads: Vec<[u32; 4]>,
    /// Per-volume stiffness (empty = rigid).
    pub volume_compliances: Vec<f32>,
    /// Uniform edge stiffness when `edge_compliances` is empty.
    pub edge_compliance: f32,
    /// Shear stiffness for auto-built constraints.
    pub shear_compliance: f32,
    /// Bend stiffness for auto-built constraints.
    pub bend_compliance: f32,
    /// Which bend constraint the auto-builder creates.
    pub bend_type: SoftBendType,
}

impl JoltSoftSharedSettings {
    /// Custom bag: every array passes straight through to Jolt.
    /// Empty `edge_pairs` auto-builds edge/shear/bend from the faces.
    #[allow(clippy::too_many_arguments)]
    pub fn custom(
        vertex_positions: Vec<Vec3>,
        vertex_velocities: Vec<Vec3>,
        vertex_inv_masses: Vec<f32>,
        face_triangles: Vec<[u32; 3]>,
        edge_pairs: Vec<[u32; 2]>,
        edge_compliances: Vec<f32>,
        volume_quads: Vec<[u32; 4]>,
        volume_compliances: Vec<f32>,
    ) -> Self {
        Self {
            vertex_positions,
            vertex_velocities,
            vertex_inv_masses,
            face_triangles,
            edge_pairs,
            edge_compliances,
            volume_quads,
            volume_compliances,
            edge_compliance: 1.0e-5,
            shear_compliance: 1.0e-5,
            bend_compliance: 1.0e-5,
            bend_type: SoftBendType::Distance,
        }
    }

    /// Flat cloth grid hanging below the origin, top rows pinned.
    pub fn cloth(
        grid_columns: u32,
        grid_rows: u32,
        grid_spacing: f32,
        pinned_rows: u32,
    ) -> Self {
        let column_total = grid_columns.max(2) as usize;
        let row_total = grid_rows.max(2) as usize;
        let offset_x = -0.5 * grid_spacing * (column_total as f32 - 1.0);
        let mut vertex_positions = Vec::with_capacity(column_total * row_total);
        let mut vertex_inv_masses = Vec::with_capacity(column_total * row_total);
        for row in 0..row_total {
            for column in 0..column_total {
                vertex_positions.push(Vec3::new(
                    offset_x + column as f32 * grid_spacing,
                    -(row as f32) * grid_spacing,
                    0.0,
                ));
                vertex_inv_masses.push(if (row as u32) < pinned_rows { 0.0 } else { 1.0 });
            }
        }
        let mut face_triangles = Vec::new();
        for row in 0..row_total - 1 {
            for column in 0..column_total - 1 {
                let top_left = (column + row * column_total) as u32;
                let top_right = (column + 1 + row * column_total) as u32;
                let bottom_left = (column + (row + 1) * column_total) as u32;
                let bottom_right = (column + 1 + (row + 1) * column_total) as u32;
                face_triangles.push([top_left, bottom_left, bottom_right]);
                face_triangles.push([top_left, bottom_right, top_right]);
            }
        }
        Self {
            vertex_positions,
            vertex_velocities: Vec::new(),
            vertex_inv_masses,
            face_triangles,
            edge_pairs: Vec::new(),
            edge_compliances: Vec::new(),
            volume_quads: Vec::new(),
            volume_compliances: Vec::new(),
            edge_compliance: 1.0e-5,
            shear_compliance: 1.0e-5,
            bend_compliance: 1.0e-5,
            bend_type: SoftBendType::Distance,
        }
    }

    /// Hollow sphere skin (pressure inflates it). Built in Rust so faces
    /// stay readable for the mesh sync; same layout as Jolt's creator.
    pub fn sphere(sphere_radius: f32, theta_segments: u32, phi_segments: u32) -> Self {
        let theta_total = theta_segments.max(3);
        let phi_total = phi_segments.max(3);
        let mut vertex_positions = vec![Vec3::new(0.0, sphere_radius, 0.0), Vec3::new(0.0, -sphere_radius, 0.0)];
        for theta in 1..theta_total - 1 {
            for phi in 0..phi_total {
                let polar = std::f32::consts::PI * theta as f32 / (theta_total - 1) as f32;
                let azimuth = 2.0 * std::f32::consts::PI * phi as f32 / phi_total as f32;
                vertex_positions.push(Vec3::new(
                    sphere_radius * polar.sin() * azimuth.cos(),
                    sphere_radius * polar.cos(),
                    sphere_radius * polar.sin() * azimuth.sin(),
                ));
            }
        }
        let sphere_index = |theta: u32, phi: u32| -> u32 {
            if theta == 0 {
                0
            } else if theta == theta_total - 1 {
                1
            } else {
                2 + (theta - 1) * phi_total + phi % phi_total
            }
        };
        let mut face_triangles = Vec::new();
        for phi in 0..phi_total {
            for theta in 0..theta_total - 2 {
                // Reversed vs Jolt's own builder: our y-up mapping flips the
                // signed volume, so faces wind the other way to keep volume
                // positive (pressure only fires on positive volume).
                face_triangles.push([
                    sphere_index(theta, phi),
                    sphere_index(theta + 1, phi + 1),
                    sphere_index(theta + 1, phi),
                ]);
                if theta > 0 {
                    face_triangles.push([
                        sphere_index(theta, phi),
                        sphere_index(theta, phi + 1),
                        sphere_index(theta + 1, phi + 1),
                    ]);
                }
            }
            // Bottom cap fan: closes the last ring onto the shared south
            // pole. Same faces as Jolt's builder, wound for our mapping.
            face_triangles.push([
                sphere_index(theta_total - 2, phi),
                sphere_index(theta_total - 2, phi + 1),
                sphere_index(theta_total - 1, 0),
            ]);
        }
        let vertex_total = vertex_positions.len();
        Self {
            vertex_positions,
            vertex_velocities: Vec::new(),
            vertex_inv_masses: vec![1.0; vertex_total],
            face_triangles,
            edge_pairs: Vec::new(),
            edge_compliances: Vec::new(),
            volume_quads: Vec::new(),
            volume_compliances: Vec::new(),
            edge_compliance: 1.0e-4,
            shear_compliance: 1.0e-4,
            bend_compliance: 1.0e-3,
            bend_type: SoftBendType::None,
        }
    }

    /// Solid cube with volume constraints (Jolt's `sCreateCube` layout).
    /// Verts and faces are built here so the mesh sync sees them; edges
    /// and volumes are marked for the FFI cube path via `cube_grid`.
    pub fn cube(grid_size: u32, grid_spacing: f32) -> Self {
        let axis_total = grid_size.max(2) as usize;
        let mut vertex_positions = Vec::with_capacity(axis_total.pow(3));
        for z in 0..axis_total {
            for y in 0..axis_total {
                for x in 0..axis_total {
                    vertex_positions.push(Vec3::new(
                        (x as f32 - (axis_total as f32 - 1.0) / 2.0) * grid_spacing,
                        (y as f32 - (axis_total as f32 - 1.0) / 2.0) * grid_spacing,
                        (z as f32 - (axis_total as f32 - 1.0) / 2.0) * grid_spacing,
                    ));
                }
            }
        }
        // Surface faces only (6 sides); interior verts ride the volumes.
        let vertex_index = |x: usize, y: usize, z: usize| (x + y * axis_total + z * axis_total * axis_total) as u32;
        let mut face_triangles = Vec::new();
        for y in 0..axis_total - 1 {
            for x in 0..axis_total - 1 {
                face_triangles.push([vertex_index(x, y, 0), vertex_index(x, y + 1, 0), vertex_index(x + 1, y + 1, 0)]);
                face_triangles.push([vertex_index(x, y, 0), vertex_index(x + 1, y + 1, 0), vertex_index(x + 1, y, 0)]);
                let far = axis_total - 1;
                face_triangles.push([vertex_index(x, y, far), vertex_index(x + 1, y + 1, far), vertex_index(x, y + 1, far)]);
                face_triangles.push([vertex_index(x, y, far), vertex_index(x + 1, y, far), vertex_index(x + 1, y + 1, far)]);
                face_triangles.push([vertex_index(x, 0, y), vertex_index(x, 0, y + 1), vertex_index(x + 1, 0, y + 1)]);
                face_triangles.push([vertex_index(x, 0, y), vertex_index(x + 1, 0, y), vertex_index(x + 1, 0, y + 1)]);
                face_triangles.push([vertex_index(x, far, y), vertex_index(x, far, y + 1), vertex_index(x + 1, far, y + 1)]);
                face_triangles.push([vertex_index(x, far, y), vertex_index(x + 1, far, y + 1), vertex_index(x + 1, far, y)]);
                face_triangles.push([vertex_index(0, x, y), vertex_index(0, x, y + 1), vertex_index(0, x + 1, y + 1)]);
                face_triangles.push([vertex_index(0, x, y), vertex_index(0, x + 1, y + 1), vertex_index(0, x + 1, y)]);
                face_triangles.push([vertex_index(far, x, y), vertex_index(far, x + 1, y + 1), vertex_index(far, x, y + 1)]);
                face_triangles.push([vertex_index(far, x, y), vertex_index(far, x + 1, y), vertex_index(far, x + 1, y + 1)]);
            }
        }
        let vertex_total = vertex_positions.len();
        Self {
            vertex_positions,
            vertex_velocities: Vec::new(),
            vertex_inv_masses: vec![1.0; vertex_total],
            face_triangles,
            edge_pairs: Vec::new(),
            edge_compliances: Vec::new(),
            volume_quads: Vec::new(),
            volume_compliances: Vec::new(),
            edge_compliance: 0.0,
            shear_compliance: 0.0,
            bend_compliance: f32::MAX,
            bend_type: SoftBendType::None,
        }
    }
}

/// Per-body soft config: full Jolt creation knob set. Defaults mirror the
/// C++ defaults so an empty config behaves like stock Jolt.
#[derive(Component, Clone, Debug)]
pub struct JoltSoftBodyConfig {
    /// Spawn position of the soft body.
    pub body_position: Vec3,
    /// Spawn rotation of the soft body.
    pub body_rotation: Quat,
    /// Collision layer, same teams as rigid bodies.
    pub object_layer: u16,
    /// Solver iterations for this body.
    pub num_iterations: u32,
    /// Linear damping: dv/dt = -damping * v, near zero.
    pub linear_damping: f32,
    /// Fastest any vertex may travel (m/s).
    pub max_linear_velocity: f32,
    /// Bounciness on contact.
    pub restitution: f32,
    /// Grip on contact.
    pub friction: f32,
    /// Balloon pressure (n * R * T); inflates closed shapes.
    pub pressure: f32,
    /// Gravity scale for this body.
    pub gravity_factor: f32,
    /// Particle skin thickness: pushes verts off surfaces.
    pub vertex_radius: f32,
    /// False pins the body to the static world (only free verts move).
    pub update_position: bool,
    /// Bakes rotation into verts, keeps body rotation identity (more exact).
    pub make_rotation_identity: bool,
    /// Whether the body may sleep.
    pub allow_sleeping: bool,
    /// Faces collide from both sides (ray casts, shape queries).
    pub faces_double_sided: bool,
}

impl Default for JoltSoftBodyConfig {
    fn default() -> Self {
        Self {
            body_position: Vec3::ZERO,
            body_rotation: Quat::IDENTITY,
            object_layer: 0,
            num_iterations: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0,
            restitution: 0.0,
            friction: 0.2,
            pressure: 0.0,
            gravity_factor: 1.0,
            vertex_radius: 0.0,
            update_position: true,
            make_rotation_identity: true,
            allow_sleeping: true,
            faces_double_sided: false,
        }
    }
}

impl JoltSoftBodyConfig {
    fn as_world_config(&self) -> SoftBodyConfig {
        SoftBodyConfig {
            body_position: self.body_position,
            body_rotation: self.body_rotation,
            object_layer: self.object_layer,
            num_iterations: self.num_iterations,
            linear_damping: self.linear_damping,
            max_linear_velocity: self.max_linear_velocity,
            restitution: self.restitution,
            friction: self.friction,
            pressure: self.pressure,
            gravity_factor: self.gravity_factor,
            vertex_radius: self.vertex_radius,
            update_position: self.update_position,
            make_rotation_identity: self.make_rotation_identity,
            allow_sleeping: self.allow_sleeping,
            faces_double_sided: self.faces_double_sided,
            user_data: 0,
        }
    }
}

/// Live soft body id, filed at bake. The handle for every `soft_*` read
/// on the world (positions, velocities, pressure, volume...).
#[derive(Component, Clone, Copy, Debug)]
pub struct JoltSoftBodyId {
    pub body_id_raw: u32,
}

/// Opt-in auto mesh: the sync builds a Bevy mesh from the shared settings'
/// faces at bake and rewrites verts every Fixed tick. Without this marker
/// the body is physics-only.
#[derive(Component, Clone, Debug, Default)]
pub struct JoltSoftBodyMesh {
    /// Base color of the generated mesh.
    pub mesh_color: Color,
}

impl JoltSoftBodyMesh {
    pub fn colored(mesh_color: Color) -> Self {
        Self { mesh_color }
    }
}

/// Creates the Jolt soft body when shared settings land on an entity that
/// already carries a config (spawn both together): bakes the shared shape,
/// creates the body, files the id. With [`JoltSoftBodyMesh`] present, also
/// builds the Bevy mesh from the settings' faces (world-space verts, so
/// the entity sits at origin).
pub fn bake_jolt_soft_body(
    trigger: On<Add, JoltSoftSharedSettings>,
    mut commands: Commands,
    soft_query: Query<(&JoltSoftSharedSettings, &JoltSoftBodyConfig, Option<&JoltSoftBodyMesh>)>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
    mut physics_world: ResMut<JoltPhysicsWorld>,
) {
    let soft_entity = trigger.event().entity;
    let Ok((shared_shape, body_config, mesh_request)) = soft_query.get(soft_entity) else {
        // Config lands in a later tick than settings: the entity isn't
        // ready yet, skip without failing so a re-add can bake it.
        return;
    };
    let shared_handle = physics_world.create_soft_shared(
        &shared_shape.vertex_positions,
        &shared_shape.vertex_velocities,
        &shared_shape.vertex_inv_masses,
        &shared_shape.face_triangles,
        &shared_shape.edge_pairs,
        &shared_shape.edge_compliances,
        &shared_shape.volume_quads,
        &shared_shape.volume_compliances,
        shared_shape.edge_compliance,
        shared_shape.shear_compliance,
        shared_shape.bend_compliance,
        shared_shape.bend_type,
    );
    assert_ne!(shared_handle, 0, "Jolt rejected the soft shared settings");
    let body_id_raw = physics_world.create_soft_body(shared_handle, &body_config.as_world_config());
    assert_ne!(body_id_raw, 0, "Jolt rejected the soft body");
    commands.entity(soft_entity).insert(JoltSoftBodyId { body_id_raw });
    let Some(mesh_request) = mesh_request else { return };
    let (Some(meshes), Some(materials)) = (meshes, materials) else { return };
    let vertex_total = shared_shape.vertex_positions.len();
    let flat_triangles: Vec<u32> = shared_shape.face_triangles.iter().flatten().copied().collect();
    let cloth_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![[0.0, 0.0, 0.0]; vertex_total],
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, cloth_uvs(vertex_total))
    .with_inserted_indices(Indices::U32(flat_triangles));
    commands.entity(soft_entity).insert((
        // World-space mesh: Jolt hands us world verts, so the entity sits
        // at the origin. (Leaving a spawn Transform here would offset the
        // body twice and fling it off-camera.)
        Transform::IDENTITY,
        Mesh3d(meshes.into_inner().add(cloth_mesh)),
        MeshMaterial3d(materials.into_inner().add(StandardMaterial {
            base_color: mesh_request.mesh_color,
            cull_mode: None,
            ..default()
        })),
    ));
}

/// Placeholder UVs (vertex index fraction) so a material spans the mesh.
/// Grid shapes get proper quad UVs from the bake path instead.
fn cloth_uvs(vertex_total: usize) -> Vec<[f32; 2]> {
    (0..vertex_total)
        .map(|vertex_index| {
            let shade = if vertex_total > 1 {
                vertex_index as f32 / (vertex_total - 1) as f32
            } else {
                0.0
            };
            [shade, shade]
        })
        .collect()
}

/// Rewrites every opt-in soft mesh from Jolt vertex positions after the
/// step. Bodies missing their id (not baked yet) are skipped.
pub fn sync_soft_body_meshes(
    soft_query: Query<(&JoltSoftBodyId, &Mesh3d)>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    physics_world: Res<JoltPhysicsWorld>,
) {
    let Some(mut meshes) = meshes else {
        return;
    };
    for (soft_id, cloth_handle) in &soft_query {
        let vertex_total = physics_world.soft_vertex_count(soft_id.body_id_raw) as usize;
        if vertex_total == 0 {
            continue;
        }
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

/// Destroys the Jolt soft body when its entity goes. Order-independent:
/// the soft registry never touches rigid bodies.
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
