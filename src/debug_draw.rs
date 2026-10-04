//! Optional gizmo visualizer: draws every known physics shape as a wireframe.
//!
//! Add [`JoltDebugPlugin`] after [`JoltPlugin`](crate::JoltPlugin). Each frame
//! it reads body transforms from the world and draws the matching outline:
//! boxes as cubes, spheres as spheres, capsules as a cylinder-plus-spheres
//! approximation, planes as grid quads.

use bevy::color::palettes::basic::{BLUE, GREEN, RED, YELLOW};
use bevy::math::Isometry3d;
use bevy::prelude::*;

use crate::physics_world::PhysicsShape;
use crate::plugin::JoltPhysicsWorld;

const DYNAMIC_BODY_COLOR: Color = Color::Srgba(GREEN);
const STATIC_BODY_COLOR: Color = Color::Srgba(BLUE);
const PLANE_GRID_COLOR: Color = Color::Srgba(YELLOW);
const PLANE_NORMAL_COLOR: Color = Color::Srgba(RED);
const PLANE_GRID_HALF_CELLS: i32 = 5;
const PLANE_GRID_CELL_SIZE: f32 = 1.0;

/// Draws all physics shapes with Bevy gizmos. Off by default: add the plugin,
/// nothing appears until something inserts this resource.
#[derive(Resource, Default)]
pub struct JoltDebugDraw;

pub struct JoltDebugPlugin;

impl Plugin for JoltDebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<JoltDebugDraw>();
        app.add_systems(Update, draw_physics_shapes);
    }
}

fn draw_physics_shapes(
    _debug_draw: Res<JoltDebugDraw>,
    physics_world: Res<JoltPhysicsWorld>,
    mut gizmos: Gizmos,
) {

    for (body_id_raw, body_shape) in physics_world.physics_world.body_shapes().iter() {
        let (body_position, body_rotation) =
            physics_world.physics_world.body_full_transform(*body_id_raw);
        let body_active = physics_world.physics_world.body_is_active(*body_id_raw);
        match *body_shape {
            PhysicsShape::Box { half_extents } => {
                let debug_color = body_debug_color(body_active);
                let box_transform = Transform {
                    translation: body_position,
                    rotation: body_rotation,
                    scale: half_extents * 2.0,
                };
                gizmos.cube(box_transform, debug_color);
            }
            PhysicsShape::Sphere { sphere_radius } => {
                let debug_color = body_debug_color(body_active);
                gizmos.sphere(
                    Isometry3d::new(body_position, body_rotation),
                    sphere_radius,
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
                    capsule_half_height,
                    capsule_radius,
                    debug_color,
                );
            }
            PhysicsShape::Plane {
                surface_normal,
                plane_constant,
            } => {
                draw_plane_grid(
                    &mut gizmos,
                    surface_normal,
                    plane_constant,
                    body_position,
                    body_rotation,
                );
            }
        }
    }
}

fn body_debug_color(body_active: bool) -> Color {
    if body_active {
        DYNAMIC_BODY_COLOR
    } else {
        STATIC_BODY_COLOR
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
    // Gizmos has no capsule primitive, so build one from lines: two end-cap
    // spheres (cheap: three rings each) plus four side rails.
    let up_direction = capsule_rotation * Vec3::Y;
    let side_x = capsule_rotation * Vec3::X;
    let side_z = capsule_rotation * Vec3::Z;
    let top_cap_center = capsule_center + up_direction * capsule_half_height;
    let bottom_cap_center = capsule_center - up_direction * capsule_half_height;

    for ring_rotation in [
        capsule_rotation,
        capsule_rotation * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2),
        capsule_rotation * Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    ] {
        gizmos.circle(
            Isometry3d::new(top_cap_center, ring_rotation),
            capsule_radius,
            debug_color,
        );
        gizmos.circle(
            Isometry3d::new(bottom_cap_center, ring_rotation),
            capsule_radius,
            debug_color,
        );
    }

    for side_offset in [side_x, -side_x, side_z, -side_z] {
        let rail_bottom = bottom_cap_center + side_offset * capsule_radius;
        let rail_top = top_cap_center + side_offset * capsule_radius;
        gizmos.line(rail_bottom, rail_top, debug_color);
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
