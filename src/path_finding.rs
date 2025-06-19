use bevy::{
    ecs::system::{SystemParam, lifetimeless::Read},
    prelude::*,
};
use raven_bvh::prelude::TlasCast;

#[cfg(feature = "debug_draw")]
use crate::debug_draw::*;


///! Module for querying the nav-mesh.
use crate::{
    tile::{nav_mesh::TileNavMesh, Link, Tile }, nav::TileLookup, Nav
};

const HEURISTIC_SCALE: f32 = 0.999;

#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
enum NodeState {
    #[default]
    Unchecked,
    Open,
    Closed,
}

#[derive(Debug)]
struct NavMeshNode {
    position: Vec3,
    cost: f32,
    total_cost: f32,
    tile: UVec2,
    polygon: u16,
    state: NodeState,
    parent: Option<usize>,
}

/// Errors returned by [find_polygon_path]
#[derive(Debug)]
pub enum FindPolygonPathError {
    /// Nav-mesh couldn't be retrieved from lock.
    NavMeshUnavailable,
    /// No polygon found near ``start_pos``.
    NoValidStartPolygon,
    /// No polygon found near ``end_pos``.
    NoValidEndPolygon,
}

// Based on Bevy's MeshRayCast
#[derive(SystemParam)]
pub struct WaymapPath<'w, 's> {    
    #[cfg(feature = "debug_draw")]
    pub gizmos: Gizmos<'w, 's, RavenGizmos>,
    //#[doc(hidden)]
    //pub hits: Local<'s, Vec<(FloatOrd, (Entity, RayNavHit))>>,
    //#[doc(hidden)]
    //pub output: Local<'s, Vec<(Entity, RayNavHit)>>,
    //#[doc(hidden)]
    //pub culled_list: Local<'s, Vec<(FloatOrd, Entity)>>,
    #[doc(hidden)]
    pub waymap_query: Query<
        'w,
        's,
        (
            Entity,
            Read<Nav>,
            Read<TileLookup>,
            Read<GlobalTransform>,
        ),
        With<Nav>,
    >,
    #[doc(hidden)]
    pub tile_query: Query<'w, 's, (Read<TileNavMesh>, Read<GlobalTransform>), With<Tile>>,
    pub ray_cast: TlasCast<'w, 's>,
}

/// Performs A* pathfinding on the supplied nav-mesh.
/// Returning the polygons crossed as a [Vec] containing the tile coordinate ([UVec2]) & polygon index ([u16]) or [FindPathError]
///
/// * ``nav_mesh`` - Nav-mesh to pathfind across.
/// * ``nav_mesh_settings`` - Nav-mesh settings used to generate ``nav_mesh``.
/// * ``start_pos`` - Starting position for the path.
/// * ``end_pos`` - Destination position for the path, i.e where you want to go.
/// * ``position_search_radius`` - Radius to search for a start & end polygon in. In world units. If **``None``** is supplied a default value of ``5.0`` is used.
/// * ``area_cost_multipliers`` - Multipliers for area cost, use to prioritize or deprioritize taking certain paths. Values not present default to 1.0. Lesser value means the path costs less.
impl<'w, 's> WaymapPath<'w, 's> {
    
}

#[derive(Debug)]
pub enum StringPullingError {
    PathEmpty,
    MissingStartTile,
    MissingEndTile,
    MissingNodeTile,
    NoLinkBetweenPathPoints,
}

/// Performs "string pulling" on a path of polygons. Used to convert [find_path]'s result to a world space path.
///
/// Returns the path as `Vec<Vec3>` or [StringPullingError]
pub fn perform_string_pulling_on_path(
    //nav_mesh: &NavMeshTiles,
    start_pos: Vec3,
    end_pos: Vec3,
    path: &[(UVec2, u16)],
) -> Result<Vec<Vec3>, StringPullingError> {
    //if path.is_empty() {
        return Err(StringPullingError::PathEmpty);
    //}

    // let Some(start_tile) = nav_mesh.tiles.get(&path[0].0) else {
    //     return Err(StringPullingError::MissingStartTile);
    // };
    // let Some(end_tile) = nav_mesh.tiles.get(&path.last().unwrap().0) else {
    //     return Err(StringPullingError::MissingEndTile);
    // };

    // let start_pos = start_tile
    //     .get_closest_point_in_polygon(&start_tile.polygons[path[0].1 as usize], start_pos);
    // let end_pos = end_tile
    //     .get_closest_point_in_polygon(&end_tile.polygons[path.last().unwrap().1 as usize], end_pos);

    // let mut string_path = Vec::with_capacity(path.len() / 3 + 2);
    // string_path.push(start_pos);

    // if path.len() > 1 {
    //     let mut portal_apex = start_pos;
    //     let mut portal_left = start_pos;
    //     let mut portal_right = start_pos;

    //     let mut left_index = 0;
    //     let mut right_index = 0;

    //     let mut i = 0;
    //     while i < path.len() {
    //         let (left, right) = if let Some(next) = path.get(i + 1) {
    //             let current = &path[i];
    //             // Find link between this and next in path.
    //             let Some(node_tile) = nav_mesh.tiles.get(&current.0) else {
    //                 return Err(StringPullingError::MissingNodeTile);
    //             };
    //             let is_internal = current.0 == next.0;
    //             let Some(link) = node_tile.polygons[current.1 as usize]
    //                 .links
    //                 .iter()
    //                 .find(|link| {
    //                     // This is a mess :)))
    //                     match link {
    //                         Link::Internal {
    //                             neighbour_polygon, ..
    //                         } => is_internal && next.1 == *neighbour_polygon,
    //                         Link::External {
    //                             neighbour_polygon,
    //                             direction,
    //                             ..
    //                         } => {
    //                             direction.offset(current.0) == next.0
    //                                 && next.1 == *neighbour_polygon
    //                         }
    //                     }
    //                 })
    //             else {
    //                 return Err(StringPullingError::NoLinkBetweenPathPoints);
    //             };

    //             let indices = &node_tile.polygons[current.1 as usize].indices;
    //             match link {
    //                 Link::Internal { edge, .. } => {
    //                     let a = node_tile.vertices[indices[*edge as usize] as usize];
    //                     let b = node_tile.vertices
    //                         [indices[(*edge + 1) as usize % indices.len()] as usize];

    //                     (a, b)
    //                 }
    //                 Link::External {
    //                     edge,
    //                     bound_min,
    //                     bound_max,
    //                     ..
    //                 } => {
    //                     let a = node_tile.vertices[indices[*edge as usize] as usize];
    //                     let b = node_tile.vertices
    //                         [indices[(*edge + 1) as usize % indices.len()] as usize];

    //                     const S: f32 = 1.0 / 255.0;
    //                     let clamped_a = a.lerp(b, *bound_min as f32 * S);
    //                     let clamped_b = a.lerp(b, *bound_max as f32 * S);

    //                     (clamped_a, clamped_b)
    //                 }
    //             }
    //         } else {
    //             (end_pos, end_pos)
    //         };

    //         // Right vertex.
    //         if triangle_area_2d(portal_apex, portal_right, right) <= 0.0 {
    //             if portal_apex.distance_squared(portal_right) < (1.0 / 16384.0)
    //                 || triangle_area_2d(portal_apex, portal_left, right) > 0.0
    //             {
    //                 portal_right = right;
    //                 right_index = i;
    //             } else {
    //                 portal_apex = portal_left;

    //                 if *string_path.last().unwrap() != portal_apex {
    //                     string_path.push(portal_apex);
    //                 }

    //                 portal_left = portal_apex;
    //                 portal_right = portal_apex;
    //                 right_index = left_index;

    //                 i = left_index + 1;
    //                 continue;
    //             }
    //         }

    //         // Left vertex.
    //         if triangle_area_2d(portal_apex, portal_left, left) >= 0.0 {
    //             if portal_apex.distance_squared(portal_left) < (1.0 / 16384.0)
    //                 || triangle_area_2d(portal_apex, portal_right, left) < 0.0
    //             {
    //                 portal_left = left;
    //                 left_index = i;
    //             } else {
    //                 portal_apex = portal_right;

    //                 if *string_path.last().unwrap() != portal_apex {
    //                     string_path.push(portal_apex);
    //                 }

    //                 portal_left = portal_apex;
    //                 portal_right = portal_apex;
    //                 left_index = right_index;

    //                 i = right_index + 1;
    //                 continue;
    //             }
    //         }

    //         i += 1;
    //     }
    // }

    // if *string_path.last().unwrap() != end_pos {
    //     string_path.push(end_pos);
    // }

    // Ok(string_path)
}

#[derive(Debug)]
pub enum FindPathError {
    PolygonPath(FindPolygonPathError),
    StringPulling(StringPullingError),
}

/// Performs A* pathfinding and string pulling on the supplied nav-mesh.
/// Returns the path as `Vec<Vec3>` or [FindPathError]
///
/// * ``nav_mesh`` - Nav-mesh to pathfind across.
/// * ``nav_mesh_settings`` - Nav-mesh settings used to generate ``nav_mesh``.
/// * ``start_pos`` - Starting position for the path.
/// * ``end_pos`` - Destination position for the path, i.e where you want to go.
/// * ``position_search_radius`` - Radius to search for a start & end polygon in. In world units. If **``None``** is supplied a default value of ``5.0`` is used.
/// * ``area_cost_multipliers`` - Multipliers for area cost, use to prioritize or deprioritize taking certain paths. Values not present default to 1.0. Lesser value means the path costs less.
pub fn find_path(
    //nav_mesh: &NavMeshTiles,
    waymap: &Nav,
    start_pos: Vec3,
    end_pos: Vec3,
    position_search_radius: Option<f32>,
    area_cost_multipliers: Option<&[f32]>,
) -> Result<Vec<Vec3>, FindPathError> {
    // match find_polygon_path(
    //     nav_mesh,
    //     waymap,
    //     start_pos,
    //     end_pos,
    //     position_search_radius,
    //     area_cost_multipliers,
    // ) {
    //     Ok(path) => perform_string_pulling_on_path(nav_mesh, start_pos, end_pos, &path)
    //         .map_err(FindPathError::StringPulling),
    //        Err(error) => Err(FindPathError::PolygonPath(error))
    //}
    Ok(vec![])
}

fn triangle_area_2d(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let ab_x = b.x - a.x;
    let ab_z = b.z - a.z;

    let ac_x = c.x - a.x;
    let ac_z = c.z - a.z;

    ac_x * ab_z - ab_x * ac_z
}
