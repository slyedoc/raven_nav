use bevy::{
    color::palettes::tailwind,
    ecs::system::{SystemParam, lifetimeless::Read},
    prelude::*,
};
use raven_bvh::prelude::TlasCast;

#[cfg(feature = "debug_draw")]
use crate::debug_draw::*;

///! Module for querying the nav-mesh.
use crate::{
    Nav,
    nav::TileLookup,
    tile::{Link, Tile, nav_mesh::TileNavMesh},
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
pub enum PathError {
    /// Nav couldn't be found.
    NavNotFound,
    /// No polygon found near ``start_pos``.
    NoValidStartPolygon,
    /// No polygon found near ``end_pos``.
    NoValidEndPolygon,

    PathEmpty,
    MissingStartTile,
    MissingStartTileLookup,
    MissingEndTile,
    MissingEndTileLookup,
    MissingNodeTile,
    NoLinkBetweenPathPoints,
}

// Based on Bevy's MeshRayCast
#[derive(SystemParam)]
pub struct NavPath<'w, 's> {
    #[cfg(feature = "debug_draw")]
    pub gizmos: Gizmos<'w, 's, NavGizmos>,
    //#[doc(hidden)]
    //pub hits: Local<'s, Vec<(FloatOrd, (Entity, RayNavHit))>>,
    //#[doc(hidden)]
    //pub output: Local<'s, Vec<(Entity, RayNavHit)>>,
    //#[doc(hidden)]
    //pub culled_list: Local<'s, Vec<(FloatOrd, Entity)>>,
    #[doc(hidden)]
    pub nav_query:
        Query<'w, 's, (Entity, Read<Nav>, Read<TileLookup>, Read<GlobalTransform>), With<Nav>>,
    #[doc(hidden)]
    pub tile_query: Query<'w, 's, (Read<TileNavMesh>, Read<GlobalTransform>), With<Tile>>,
    pub tlas_cast: TlasCast<'w, 's>,
}

/// Performs A* pathfinding on the supplied nav
/// Returning the polygons crossed as a [Vec] containing the tile coordinate ([UVec2]) & polygon index ([u16]) or [FindPathError]
///
/// * ``nav_e`` - Nav to pathfind across.
/// * ``start_pos`` - Starting position for the path.
/// * ``end_pos`` - Destination position for the path, i.e where you want to go.
/// * ``position_search_radius`` - Radius to search for a start & end polygon in. In world units. If **``None``** is supplied a default value of ``5.0`` is used.
/// * ``area_cost_multipliers`` - Multipliers for area cost, use to prioritize or deprioritize taking certain paths. Values not present default to 1.0. Lesser value means the path costs less.
impl<'w, 's> NavPath<'w, 's> {
    /// Performs A* pathfinding on the supplied nav-mesh.

    ///
    /// * ``nav_e`` - Nav entity to pathfind across.    
    /// * ``start_pos`` - Starting position for the path.
    /// * ``end_pos`` - Destination position for the path, i.e where you want to go.
    /// * ``position_search_radius`` - Radius to search for a start & end polygon in. In world units. If **``None``** is supplied a default value of ``5.0`` is used.
    /// * ``area_cost_multipliers`` - Multipliers for area cost, use to prioritize or deprioritize taking certain paths. Values not present default to 1.0. Lesser value means the path costs less.
    pub fn find_path(
        &mut self,
        nav_e: Entity,
        start_pos: Vec3,
        end_pos: Vec3,
        position_search_radius: Option<f32>,
        area_cost_multipliers: Option<&[f32]>, // TODO: A slice might not be the best choice when there are many area types.
    ) -> Result<Vec<Vec3>, PathError> {
        let search_radius = position_search_radius.unwrap_or(50.0);

        let Ok((_e, _nav, lookup, _nav_trans)) = self.nav_query.get(nav_e) else {
            return Err(PathError::NavNotFound);
        };

        let Some((start_tile, start_poly, _start_pos)) =
            self.find_closest_polygon_in_box(nav_e, start_pos, search_radius)
        else {
            return Err(PathError::NoValidStartPolygon);
        };
        self.gizmos
            .line(start_pos, start_pos + Vec3::Y, tailwind::GREEN_400);

        let Some((end_tile, end_poly, _end_pos)) =
            self.find_closest_polygon_in_box(nav_e, end_pos, search_radius)
        else {
            return Err(PathError::NoValidEndPolygon);
        };
        self.gizmos
            .line(end_pos, end_pos + Vec3::Y, tailwind::GREEN_400);

        let path: Vec<(UVec2, u16)> = if start_tile == end_tile && start_poly == end_poly {
            // start and end are in the same tile, so we can just return the start tile and polygon
            vec![(start_tile, start_poly)]
        } else {
            // find path with A*, note that distance calculations are in world space

            // the search graph.
            let mut nodes = Vec::with_capacity(10);

            // node indices to explore, sorted by increasing total cost.
            let mut open_list = Vec::with_capacity(5);

            // Initialize the first node.
            nodes.push(NavMeshNode {
                position: start_pos,
                cost: 0.0,
                total_cost: start_pos.distance(end_pos) * HEURISTIC_SCALE,
                tile: start_tile,
                polygon: start_poly,
                state: NodeState::Open,
                parent: None,
            });
            open_list.push(0);

            let mut last_best_node = 0;
            let mut last_best_node_cost = nodes[0].total_cost;

            // The A* search loop.
            while let Some(best_node_index) = open_list.pop() {
                // Take the next best node (lowest total cost due to ordering)
                let (best_tile, best_polygon, best_position, best_cost, best_parent) = {
                    let node = &mut nodes[best_node_index];
                    node.state = NodeState::Closed;

                    // test if we reached the goal
                    if node.tile == end_tile && node.polygon == end_poly {
                        last_best_node = best_node_index;
                        break; // goal reached!
                    }
                    self.gizmos
                        .sphere(node.position, 0.1, tailwind::GRAY_700);
                    // Unpack key data for neighbor expansion
                    (
                        node.tile,
                        node.polygon,
                        node.position,
                        node.cost,
                        node.parent,
                    )
                };
                let tile_e = lookup.get(&best_tile).unwrap();
                let (tile, tile_trans) = self.tile_query.get(*tile_e).unwrap();

                // Find the best polygon in the tile
                for link in tile.polygons[best_polygon as usize].links.iter() {
                    let (link_tile, link_polygon) = match link {
                        Link::Internal {
                            neighbour_polygon, ..
                        } => (best_tile, *neighbour_polygon),
                        Link::External {
                            neighbour_polygon,
                            direction,
                            ..
                        } => {
                            // check if the direction is valid
                            if let Some(offset) = direction.offset(best_tile) {
                                (offset, *neighbour_polygon)
                            } else {
                                continue; // skip invalid directions
                            }
                        }
                    };

                    // Prevent going backward in the path (cycles)
                    if let Some(parent) = best_parent {
                        if nodes[parent].tile == link_tile && nodes[parent].polygon == link_polygon
                        {
                            continue;
                        }
                    }

                    // Node creation or lookup
                    let neighbour_node_index = if let Some(index) =
                        nodes.iter().position(|element| {
                            element.tile == link_tile && element.polygon == link_polygon
                        }) {
                        index
                    } else {
                        // Node hasn't been visited already, let's create it.
                        let position = match link {
                            Link::Internal { edge, .. } => {
                                // Just the midpoint of the current edge.
                                let indices = &tile.polygons[best_polygon as usize].indices;
                                let a = tile.vertices[indices[*edge as usize] as usize];
                                let b = tile.vertices[indices[(*edge + 1) as usize % indices.len()] as usize];

                                let local_mid = a.lerp(b, 0.5);
                                tile_trans.transform_point(local_mid)
                            }
                            Link::External {
                                edge,
                                bound_min,
                                bound_max,
                                ..
                            } => {
                                // The mid point of the current-edge sliced by bound_min & bound_max.
                                let indices = &tile.polygons[best_polygon as usize].indices;
                                let a = tile.vertices[indices[*edge as usize] as usize];
                                let b = tile.vertices
                                    [indices[(*edge + 1) as usize % indices.len()] as usize];
                                
                                let a = tile_trans.transform_point(a);
                                let b = tile_trans.transform_point(b);

                                
                                const S: f32 = 1.0 / 255.0;
                                let bound_min = *bound_min as f32 * S;
                                let bound_max = *bound_max as f32 * S;
                                let clamped_a = a.lerp(b, bound_min);
                                let clamped_b = a.lerp(b, bound_max);

                                clamped_a.lerp(clamped_b, 0.5)
                            }
                        };

                        nodes.push(NavMeshNode {
                            position,
                            cost: 0.0,
                            total_cost: 0.0,
                            tile: link_tile,
                            polygon: link_polygon,
                            state: NodeState::Unchecked,
                            parent: None,
                        });
                        nodes.len() - 1
                    };

                    // cost and heuristic evaluation
                    let (old_state, total_cost) = {
                        let neighbour_node = &mut nodes[neighbour_node_index];

                        // Optional area cost multipliers
                        // TODO: Ideally you want to be able to override this but for now we just go with the distance.
                        let node_cost_multiplier =
                            area_cost_multipliers.map_or(1.0, |multipliers| {
                                let node_tile_e = lookup.get(&best_tile).unwrap();
                                let (node_tile, _trans) =
                                    self.tile_query.get(*node_tile_e).unwrap();
                                let area = node_tile.areas[best_polygon as usize];

                                *multipliers.get(area.0 as usize).unwrap_or(&1.0)
                            });

                        let (cost, heuristic) = if end_tile == link_tile && end_poly == link_polygon
                        {
                            // Special case for the final node.
                            let current_cost = best_position.distance(neighbour_node.position)
                                * node_cost_multiplier;
                            let end_cost = neighbour_node.position.distance(end_pos);
                            let cost = best_cost + current_cost + end_cost;
                            (cost, 0.0)
                        } else {
                            let current_cost = best_position.distance(neighbour_node.position)
                                * node_cost_multiplier;
                            let cost = best_cost + current_cost;
                            let heuristic =
                                neighbour_node.position.distance(end_pos) * HEURISTIC_SCALE;
                            (cost, heuristic)
                        };
                        let total_cost = cost + heuristic;

                        // Don't update if already visited with better cost
                        if neighbour_node.state != NodeState::Unchecked
                            && total_cost >= neighbour_node.total_cost
                        {
                            continue;
                        }

                        // Update node data
                        let old_state = neighbour_node.state;
                        neighbour_node.parent = Some(best_node_index);
                        neighbour_node.state = NodeState::Open;
                        neighbour_node.cost = cost;
                        neighbour_node.total_cost = total_cost;

                        // Remember best node if heuristic is better
                        if heuristic < last_best_node_cost {
                            last_best_node_cost = heuristic;
                            last_best_node = neighbour_node_index;
                        }

                        (old_state, total_cost)
                    };

                    // Open list maintenance
                    if old_state == NodeState::Open {
                        // Node already exists. Let's remove it.
                        if let Some(existing_index) = open_list
                            .iter()
                            .position(|node| *node == neighbour_node_index)
                        {
                            open_list.remove(existing_index);
                        }
                    }

                    // Insert based on total cost ordering (descending)
                    if let Some(index) = open_list
                        .iter()
                        .position(|node_index| nodes[*node_index].total_cost < total_cost)
                    {
                        open_list.insert(index, neighbour_node_index);
                    } else {
                        // There is no entry with a lower total.
                        open_list.push(neighbour_node_index);
                    }
                }
            }

            // Path Reconstruction
            let path_count = {
                let mut count = 0;
                let mut parent = Some(last_best_node);
                while let Some(parent_index) = parent {
                    count += 1;
                    parent = nodes[parent_index].parent;
                }
                count
            };
            let mut path = Vec::with_capacity(path_count); // TODO: make local
            let mut parent = Some(last_best_node);
            while let Some(parent_index) = parent {
                let node = &nodes[parent_index];
                path.push((node.tile, node.polygon));
                parent = node.parent;
            }
            path.reverse(); // the path is constructed backwards
            path
        };

        

        // preform string pulling to get the actual path in world space
        if path.is_empty() {
            return Err(PathError::PathEmpty);
        }

        let Some(start_tile_e) = lookup.get(&path[0].0) else {
            return Err(PathError::MissingStartTileLookup);
        };
        let Ok((start_tile, start_trans)) = self.tile_query.get(*start_tile_e) else {
            return Err(PathError::MissingStartTile);
        };

        let Some(end_tile_e) = lookup.get(&path.last().unwrap().0) else {
            return Err(PathError::MissingEndTileLookup);
        };

        let Ok((end_tile, end_trans)) = self.tile_query.get(*end_tile_e) else {
            return Err(PathError::MissingEndTile);
        };

        let start_pos = start_tile.get_closest_point_in_polygon(&start_tile.polygons[path[0].1 as usize], start_pos, start_trans);
        let end_pos = end_tile.get_closest_point_in_polygon(&end_tile.polygons[path.last().unwrap().1 as usize], end_pos, end_trans);

        let mut string_path = Vec::with_capacity(path.len() / 3 + 2);
        string_path.push(start_pos);

        if path.len() > 1 {
            let mut portal_apex = start_pos;
            let mut portal_left = start_pos;
            let mut portal_right = start_pos;

            let mut left_index = 0;
            let mut right_index = 0;

            let mut i = 0;
            while i < path.len() {
                let (left, right) = if let Some(next) = path.get(i + 1) {
                    let current = &path[i];
                    // Find link between this and next in path.
                    let Some(tile_e) = lookup.get(&current.0) else {
                        return Err(PathError::MissingNodeTile);
                    };
                    let Ok((tile, tile_trans)) = self.tile_query.get(*tile_e) else {
                        return Err(PathError::MissingNodeTile);
                    };

                    let is_internal = current.0 == next.0;
                    let Some(link) = tile.polygons[current.1 as usize].links.iter().find(|link| {
                        // This is a mess :)))
                        match link {
                            Link::Internal {
                                neighbour_polygon, ..
                            } => is_internal && next.1 == *neighbour_polygon,
                            Link::External {
                                neighbour_polygon,
                                direction,
                                ..
                            } => match direction.offset(current.0) {
                                Some(d) => d == next.0 && next.1 == *neighbour_polygon,
                                None => false,
                            },
                        }
                    }) else {
                        return Err(PathError::NoLinkBetweenPathPoints);
                    };

                    let indices = &tile.polygons[current.1 as usize].indices;
                    match link {
                        Link::Internal { edge, .. } => {
                            let a = tile.vertices[indices[*edge as usize] as usize];
                            let b = tile.vertices
                                [indices[(*edge + 1) as usize % indices.len()] as usize];
                            let a = tile_trans.transform_point(a);
                            let b = tile_trans.transform_point(b);

                            self.gizmos.line(a, b, tailwind::LIME_400);
                            (a, b)
                        }
                        Link::External {
                            edge,
                            bound_min,
                            bound_max,
                            ..
                        } => {
                            let a = tile.vertices[indices[*edge as usize] as usize];
                            let b = tile.vertices[indices[(*edge + 1) as usize % indices.len()] as usize];    
                            const S: f32 = 1.0 / 255.0;
                            let a = a.lerp(b, *bound_min as f32 * S);
                            let b = a.lerp(b, *bound_max as f32 * S);

                            let a = tile_trans.transform_point(a);
                            let b = tile_trans.transform_point(b);
                            (a, b)

                        }
                    }
                } else {
                    (end_pos, end_pos)
                };

                // Right vertex.
                if triangle_area_2d(portal_apex, portal_right, right) <= 0.0 {
                    if portal_apex.distance_squared(portal_right) < (1.0 / 16384.0)
                        || triangle_area_2d(portal_apex, portal_left, right) > 0.0
                    {
                        portal_right = right;
                        right_index = i;
                    } else {
                        portal_apex = portal_left;

                        if *string_path.last().unwrap() != portal_apex {
                            string_path.push(portal_apex);
                        }

                        portal_left = portal_apex;
                        portal_right = portal_apex;
                        right_index = left_index;

                        i = left_index + 1;
                        continue;
                    }
                }

                // Left vertex.
                if triangle_area_2d(portal_apex, portal_left, left) >= 0.0 {
                    if portal_apex.distance_squared(portal_left) < (1.0 / 16384.0)
                        || triangle_area_2d(portal_apex, portal_right, left) < 0.0
                    {
                        portal_left = left;
                        left_index = i;
                    } else {
                        portal_apex = portal_right;

                        if *string_path.last().unwrap() != portal_apex {
                            string_path.push(portal_apex);
                        }

                        portal_left = portal_apex;
                        portal_right = portal_apex;
                        left_index = right_index;

                        i = right_index + 1;
                        continue;
                    }
                }

                i += 1;
            }
        }

        if *string_path.last().unwrap() != end_pos {
            string_path.push(end_pos);
        }

        self.gizmos.linestrip(string_path.clone(), tailwind::GRAY_100);        

        Ok(string_path)
    }

    pub fn find_closest_polygon_in_box(
        &self,
        nav: Entity,
        center: Vec3,
        half_extents: f32,
    ) -> Option<(UVec2, u16, Vec3)> {
        let (_e, nav, lookup, nav_trans) = self.nav_query.get(nav).unwrap();

        let local_center = nav_trans.affine().inverse().transform_point(center);        
        let min = local_center - half_extents;
        let max = local_center + half_extents;

        let min_tile = nav.get_tile_containing_position(min.xz(), nav_trans);
        let max_tile = nav.get_tile_containing_position(max.xz(), nav_trans);

        let mut out_polygon = None;
        let mut out_distance = f32::INFINITY;
        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coords = UVec2::new(x, y);
                let Some(tile_entity) = lookup.get(&tile_coords) else {
                    continue;
                };
                let Ok((tile, tile_trans)) = self.tile_query.get(*tile_entity) else {
                    continue;
                };
                for (poly_i, polygon) in tile.polygons.iter().enumerate() {
                    let closest_point = tile.get_closest_point_in_polygon(polygon, center, tile_trans);
                    let closest_distance = closest_point.distance_squared(center);
                    if closest_distance < out_distance {
                        out_distance = closest_distance;
                        out_polygon = Some((tile_coords, poly_i as u16, closest_point));
                    }
                }
            }
        }

        out_polygon
    }
}

fn triangle_area_2d(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let ab_x = b.x - a.x;
    let ab_z = b.z - a.z;

    let ac_x = c.x - a.x;
    let ac_z = c.z - a.z;

    ac_x * ab_z - ab_x * ac_z
}
