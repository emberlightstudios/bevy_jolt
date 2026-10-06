//! Optional gizmo visualizer: draws every known physics shape as a wireframe.
//!
//! Add [`JoltDebugPlugin`] after [`JoltPlugin`](crate::JoltPlugin). Each frame
//! it reads body transforms from the world and draws the matching outline:
//! boxes as cubes, spheres as spheres, capsules as a cylinder-plus-spheres
//! approximation, planes as grid quads.

use bevy::color::palettes::basic::{BLUE, GREEN, RED, YELLOW};
use bevy::math::Isometry3d;
use bevy::prelude::*;

use crate::physics_world::{CompoundGeometry, PhysicsShape};
use crate::plugin::JoltPhysicsWorld;

const DYNAMIC_BODY_COLOR: Color = Color::Srgba(GREEN);
const STATIC_BODY_COLOR: Color = Color::Srgba(BLUE);
const PLANE_GRID_COLOR: Color = Color::Srgba(YELLOW);
const PLANE_NORMAL_COLOR: Color = Color::Srgba(RED);
const PLANE_GRID_HALF_CELLS: i32 = 5;
const PLANE_GRID_CELL_SIZE: f32 = 1.0;

/// Adds wireframe drawing of every physics shape. Drawing starts as soon as
/// the plugin is added: no manual setup needed.
pub struct JoltDebugPlugin;

impl Plugin for JoltDebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, draw_physics_shapes);
    }
}

fn draw_physics_shapes(
    physics_world: Res<JoltPhysicsWorld>,
    character_query: Query<(&crate::character::JoltCharacter, &Transform)>,
    rigid_query: Query<(&crate::character::JoltRigidCharacterId, &Transform)>,
    mut gizmos: Gizmos,
) {

    for (body_id_raw, body_shape) in physics_world.body_shapes().iter() {
        let (body_position, body_rotation) = physics_world.body_full_transform(*body_id_raw);
        let body_active = physics_world.body_is_active(*body_id_raw);
        match body_shape {
            PhysicsShape::Box { half_extents } => {
                let debug_color = body_debug_color(body_active);
                let box_transform = Transform {
                    translation: body_position,
                    rotation: body_rotation,
                    scale: *half_extents * 2.0,
                };
                gizmos.cube(box_transform, debug_color);
            }
            PhysicsShape::Sphere { sphere_radius } => {
                let debug_color = body_debug_color(body_active);
                gizmos.sphere(
                    Isometry3d::new(body_position, body_rotation),
                    *sphere_radius,
                    debug_color,
                );
            }
            PhysicsShape::Capsule {
                capsule_half_height,
                capsule_radius,
            } => {
                let debug_color = body_debug_color(body_active);
                draw_capsule_outline(
                    &mut gizmos,
                    body_position,
                    body_rotation,
                    *capsule_half_height,
                    *capsule_radius,
                    debug_color,
                );
            }
            PhysicsShape::Cylinder {
                cylinder_half_height,
                cylinder_radius,
            } => {
                let debug_color = body_debug_color(body_active);
                draw_cylinder_outline(
                    &mut gizmos,
                    body_position,
                    body_rotation,
                    *cylinder_half_height,
                    *cylinder_radius,
                    debug_color,
                );
            }
            PhysicsShape::TaperedCylinder {
                tapered_half_height,
                top_radius,
                bottom_radius,
            } => {
                let debug_color = body_debug_color(body_active);
                draw_tapered_cylinder_outline(
                    &mut gizmos,
                    body_position,
                    body_rotation,
                    *tapered_half_height,
                    *top_radius,
                    *bottom_radius,
                    debug_color,
                );
            }
            PhysicsShape::TaperedCapsule {
                tapered_half_height,
                top_radius,
                bottom_radius,
            } => {
                let debug_color = body_debug_color(body_active);
                draw_tapered_capsule_outline(
                    &mut gizmos,
                    body_position,
                    body_rotation,
                    *tapered_half_height,
                    *top_radius,
                    *bottom_radius,
                    debug_color,
                );
            }
            PhysicsShape::Plane {
                surface_normal,
                plane_constant,
            } => {
                draw_plane_grid(
                    &mut gizmos,
                    *surface_normal,
                    *plane_constant,
                    body_position,
                    body_rotation,
                );
            }
            PhysicsShape::Compound { compound_parts } => {
                let debug_color = body_debug_color(body_active);
                for compound_part in compound_parts {
                    let part_pose = part_world_pose(
                        body_position,
                        body_rotation,
                        compound_part.part_offset,
                        compound_part.part_rotation,
                    );
                    draw_compound_part(&mut gizmos, part_pose, compound_part.part_geometry, debug_color);
                }
            }
        }
    }
    // Characters are not rigid bodies, so they never appear in the body
    // table. The pose already carries the capsule center (center-of-mass
    // position): draw here with no lift, same as the rigid side.
    for (character, character_pose) in &character_query {
        draw_capsule_outline(
            &mut gizmos,
            character_pose.translation,
            character_pose.rotation,
            character.capsule_half_height,
            character.capsule_radius,
            DYNAMIC_BODY_COLOR,
        );
    }
    // Rigid characters own a real simulated body, but it never enters the
    // body table — draw their filed capsule at the live entity pose.
    for (rigid_id, rigid_pose) in &rigid_query {
        let Some((capsule_half_height, capsule_radius)) = physics_world
            .rigid_character_shapes()
            .get(&rigid_id.character_id_raw)
        else {
            continue;
        };
        // Rigid GetPosition returns the body origin (capsule center), not
        // the feet like the virtual side: draw here with no lift.
        let capsule_center = rigid_pose.translation;
        draw_capsule_outline(
            &mut gizmos,
            capsule_center,
            rigid_pose.rotation,
            *capsule_half_height,
            *capsule_radius,
            DYNAMIC_BODY_COLOR,
        );
    }
}
fn body_debug_color(body_active: bool) -> Color {
    if body_active {
        DYNAMIC_BODY_COLOR
    } else {
        STATIC_BODY_COLOR
    }
}

/// World pose of one compound part: body pose composed with the part's local
/// offset + rotation.
fn part_world_pose(
    body_position: Vec3,
    body_rotation: Quat,
    part_offset: Vec3,
    part_rotation: Quat,
) -> Transform {
    Transform {
        translation: body_position + body_rotation * part_offset,
        rotation: body_rotation * part_rotation,
        scale: Vec3::ONE,
    }
}

/// Draws one compound part with the same outline style as its standalone
/// shape: boxes as cubes, spheres as spheres, capsules as line capsules.
fn draw_compound_part(
    gizmos: &mut Gizmos,
    part_pose: Transform,
    part_geometry: CompoundGeometry,
    debug_color: Color,
) {
    match part_geometry {
        CompoundGeometry::Box { part_half_extents } => {
            gizmos.cube(
                Transform {
                    scale: part_half_extents * 2.0,
                    ..part_pose
                },
                debug_color,
            );
        }
        CompoundGeometry::Sphere { part_radius } => {
            gizmos.sphere(
                Isometry3d::new(part_pose.translation, part_pose.rotation),
                part_radius,
                debug_color,
            );
        }
        CompoundGeometry::Capsule {
            part_half_height,
            part_radius,
        } => {
            draw_capsule_outline(
                gizmos,
                part_pose.translation,
                part_pose.rotation,
                part_half_height,
                part_radius,
                debug_color,
            );
        }
    }
}

fn draw_capsule_outline(
    gizmos: &mut Gizmos,
    capsule_center: Vec3,
    capsule_rotation: Quat,
    capsule_half_height: f32,
    capsule_radius: f32,
    debug_color: Color,
) {
    // One pill, not two balls: two silhouette rails per side plane plus
    // half-ring caps that continue the rails over each pole. Full rings
    // would read as separate spheres; arcs keep the single-capsule read.
    let up_direction = capsule_rotation * Vec3::Y;
    let top_cap_center = capsule_center + up_direction * capsule_half_height;
    let bottom_cap_center = capsule_center - up_direction * capsule_half_height;

    for side_plane in [0.0, std::f32::consts::FRAC_PI_2] {
        let plane_rotation = capsule_rotation * Quat::from_rotation_y(side_plane);
        let rail_direction = plane_rotation * Vec3::X;
        for rail_sign in [1.0, -1.0] {
            let rail_offset = rail_direction * (rail_sign * capsule_radius);
            gizmos.line(
                bottom_cap_center + rail_offset,
                top_cap_center + rail_offset,
                debug_color,
            );
            draw_cap_arc(
                gizmos,
                top_cap_center,
                up_direction,
                rail_offset,
                capsule_radius,
                debug_color,
            );
            draw_cap_arc(
                gizmos,
                bottom_cap_center,
                -up_direction,
                rail_offset,
                capsule_radius,
                debug_color,
            );
        }
    }
}

/// One quarter-to-half arc over a capsule pole: walks from a rail end
/// around to the pole tip so the cap continues the rail instead of
/// closing a separate ring.
fn draw_cap_arc(
    gizmos: &mut Gizmos,
    cap_center: Vec3,
    pole_direction: Vec3,
    rail_offset: Vec3,
    capsule_radius: f32,
    debug_color: Color,
) {
    const CAP_SEGMENTS: u32 = 8;
    let rail_direction = rail_offset.normalize_or_zero();
    let mut arc_previous = cap_center + rail_offset;
    for cap_step in 1..=CAP_SEGMENTS {
        let arc_angle =
            cap_step as f32 / CAP_SEGMENTS as f32 * std::f32::consts::FRAC_PI_2;
        let arc_point = cap_center
            + rail_direction * (arc_angle.cos() * capsule_radius)
            + pole_direction * (arc_angle.sin() * capsule_radius);
        gizmos.line(arc_previous, arc_point, debug_color);
        arc_previous = arc_point;
    }
}

fn draw_plane_grid(
    gizmos: &mut Gizmos,
    surface_normal: Vec3,
    plane_constant: f32,
    _body_position: Vec3,
    _body_rotation: Quat,
) {
    // A plane is infinite; draw a grid quad on the surface. The plane
    // equation is normal.dot(point) + constant = 0.
    let plane_up = surface_normal.normalize_or_zero();
    let grid_up = if plane_up.length_squared() < 0.5 {
        Vec3::Y
    } else {
        plane_up
    };
    let tangent_x = grid_up.cross(Vec3::Z).normalize_or_zero();
    let tangent_axis_x = if tangent_x.length_squared() < 0.5 {
        Vec3::X
    } else {
        tangent_x
    };
    let tangent_axis_z = grid_up.cross(tangent_axis_x).normalize();
    let grid_origin = grid_up * -plane_constant;
    let grid_extent = PLANE_GRID_HALF_CELLS as f32 * PLANE_GRID_CELL_SIZE;

    for grid_line in -PLANE_GRID_HALF_CELLS..=PLANE_GRID_HALF_CELLS {
        let line_offset = grid_line as f32 * PLANE_GRID_CELL_SIZE;
        let line_start_x = grid_origin + tangent_axis_z * line_offset - tangent_axis_x * grid_extent;
        let line_end_x = grid_origin + tangent_axis_z * line_offset + tangent_axis_x * grid_extent;
        gizmos.line(line_start_x, line_end_x, PLANE_GRID_COLOR);
        let line_start_z = grid_origin + tangent_axis_x * line_offset - tangent_axis_z * grid_extent;
        let line_end_z = grid_origin + tangent_axis_x * line_offset + tangent_axis_z * grid_extent;
        gizmos.line(line_start_z, line_end_z, PLANE_GRID_COLOR);
    }
    gizmos.ray(grid_origin, grid_up * 2.0, PLANE_NORMAL_COLOR);
}

fn draw_cylinder_outline(
    gizmos: &mut Gizmos,
    cylinder_center: Vec3,
    cylinder_rotation: Quat,
    cylinder_half_height: f32,
    cylinder_radius: f32,
    debug_color: Color,
) {
    // No cylinder primitive: two end-cap circles plus side rails.
    let up_direction = cylinder_rotation * Vec3::Y;
    let side_x = cylinder_rotation * Vec3::X;
    let side_z = cylinder_rotation * Vec3::Z;
    let top_cap_center = cylinder_center + up_direction * cylinder_half_height;
    let bottom_cap_center = cylinder_center - up_direction * cylinder_half_height;
    let cap_rotation = cylinder_rotation * Quat::from_rotation_x(core::f32::consts::FRAC_PI_2);
    gizmos.circle(
        Isometry3d::new(top_cap_center, cap_rotation),
        cylinder_radius,
        debug_color,
    );
    gizmos.circle(
        Isometry3d::new(bottom_cap_center, cap_rotation),
        cylinder_radius,
        debug_color,
    );
    for side_offset in [side_x, -side_x, side_z, -side_z] {
        let rail_bottom = bottom_cap_center + side_offset * cylinder_radius;
        let rail_top = top_cap_center + side_offset * cylinder_radius;
        gizmos.line(rail_bottom, rail_top, debug_color);
    }
}

fn draw_tapered_cylinder_outline(
    gizmos: &mut Gizmos,
    tapered_center: Vec3,
    tapered_rotation: Quat,
    tapered_half_height: f32,
    top_radius: f32,
    bottom_radius: f32,
    debug_color: Color,
) {
    // Frustum: different circle per face, rails slope between them.
    let up_direction = tapered_rotation * Vec3::Y;
    let side_x = tapered_rotation * Vec3::X;
    let side_z = tapered_rotation * Vec3::Z;
    let top_cap_center = tapered_center + up_direction * tapered_half_height;
    let bottom_cap_center = tapered_center - up_direction * tapered_half_height;
    let cap_rotation = tapered_rotation * Quat::from_rotation_x(core::f32::consts::FRAC_PI_2);
    gizmos.circle(
        Isometry3d::new(top_cap_center, cap_rotation),
        top_radius,
        debug_color,
    );
    gizmos.circle(
        Isometry3d::new(bottom_cap_center, cap_rotation),
        bottom_radius,
        debug_color,
    );
    for side_offset in [side_x, -side_x, side_z, -side_z] {
        let rail_bottom = bottom_cap_center + side_offset * bottom_radius;
        let rail_top = top_cap_center + side_offset * top_radius;
        gizmos.line(rail_bottom, rail_top, debug_color);
    }
}

fn draw_tapered_capsule_outline(
    gizmos: &mut Gizmos,
    tapered_center: Vec3,
    tapered_rotation: Quat,
    tapered_half_height: f32,
    top_radius: f32,
    bottom_radius: f32,
    debug_color: Color,
) {
    // Same pill read as the plain capsule: sloped rails plus cap arcs in
    // each end's own radius.
    let up_direction = tapered_rotation * Vec3::Y;
    let top_cap_center = tapered_center + up_direction * tapered_half_height;
    let bottom_cap_center = tapered_center - up_direction * tapered_half_height;

    for side_plane in [0.0, std::f32::consts::FRAC_PI_2] {
        let plane_rotation = tapered_rotation * Quat::from_rotation_y(side_plane);
        let rail_direction = plane_rotation * Vec3::X;
        for rail_sign in [1.0, -1.0] {
            let top_offset = rail_direction * (rail_sign * top_radius);
            let bottom_offset = rail_direction * (rail_sign * bottom_radius);
            gizmos.line(bottom_cap_center + bottom_offset, top_cap_center + top_offset, debug_color);
            draw_cap_arc(
                gizmos,
                top_cap_center,
                up_direction,
                top_offset,
                top_radius,
                debug_color,
            );
            draw_cap_arc(
                gizmos,
                bottom_cap_center,
                -up_direction,
                bottom_offset,
                bottom_radius,
                debug_color,
            );
        }
    }
}
