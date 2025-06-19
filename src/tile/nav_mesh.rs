use bevy::prelude::*;

use crate::{
    collider::Area,
    tile::{Link, NavPolygon, mesher::*},
    nav::Nav,
};

use super::mesher::VERTICES_IN_TRIANGLE;

// #[derive(Component, Clone, Debug, Deref, DerefMut, Reflect)]
// #[reflect(Component)]
// pub struct TileNavMesh(pub Handle<NavMesh>);

/// Set this up as an asset, but due to https://github.com/bevyengine/bevy/issues/16244
/// we cant access multiple assets in the same system, and since no need for reused, used it as a component instead
//#[derive(Asset, TypePath, Debug)]
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
pub struct TileNavMesh {
    /// The vertices that make up the polygons.
    pub(crate) vertices: Vec<Vec3>,
    /// The polygons of the mesh.
    pub(crate) polygons: Vec<NavPolygon>,
    pub areas: Vec<Area>,
    pub edges: Vec<[EdgeConnection; VERTICES_IN_TRIANGLE]>,
}
impl TileNavMesh {
    // /// Returns the closest point on ``polygon`` to ``position``.
    pub fn get_closest_point_in_polygon(&self, polygon: &NavPolygon, position: Vec3) -> Vec3 {
        let vertices: [Vec3; 3] = polygon.indices.map(|index| self.vertices[index as usize]);

        if let Some(height) = get_height_in_triangle(&vertices, position) {
            return Vec3::new(position.x, height, position.z);
        }

        closest_point_on_edges(&vertices, position)
    }

    pub fn remove_links_to_direction(&mut self, remove_direction: EdgeConnectionDirection) {
        for polygon in self.polygons.iter_mut() {
            polygon.links.retain(|link| match link {
                Link::Internal { .. } => true,
                Link::External { direction, .. } => *direction != remove_direction,
            });
        }
    }
}


fn get_height_in_triangle(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> Option<f32> {
    if !in_polygon(vertices, position) {
        return None;
    }

    if let Some(height) =
        closest_height_in_triangle(vertices[0], vertices[1], vertices[2], position)
    {
        return Some(height);
    }

    // We only hit this if we are ON an edge. Unlikely to happen.
    let closest = closest_point_on_edges(vertices, position);

    Some(closest.y)
}


fn closest_point_on_edges(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> Vec3 {
    let mut d_min = f32::INFINITY;
    let mut t_min = 0.0;

    let mut edge_min = Vec3::ZERO;
    let mut edge_max = Vec3::ZERO;

    for i in 0..vertices.len() {
        let prev = (vertices.len() + i - 1) % vertices.len();

        let (d, t) = distance_point_to_segment_2d(position, vertices[prev], vertices[i]);
        if d < d_min {
            d_min = d;
            t_min = t;
            edge_min = vertices[prev];
            edge_max = vertices[i];
        }
    }

    edge_min.lerp(edge_max, t_min)
}

fn distance_point_to_segment_2d(point: Vec3, seg_a: Vec3, seg_b: Vec3) -> (f32, f32) {
    let ba_x = seg_b.x - seg_a.x;
    let ba_z = seg_b.z - seg_a.z;

    let dx = point.x - seg_a.x;
    let dz = point.z - seg_a.z;

    let d = ba_x * ba_x + ba_z * ba_z;
    let mut t = ba_x * dx + ba_z * dz;
    if d > 0.0 {
        t /= d;
    }
    t = t.clamp(0.0, 1.0);

    let dx = seg_a.x + t * ba_x - point.x;
    let dz = seg_a.z + t * ba_z - point.z;

    (dx * dx + dz * dz, t)
}

fn in_polygon(vertices: &[Vec3; VERTICES_IN_TRIANGLE], position: Vec3) -> bool {
    let mut inside = false;

    for i in 0..vertices.len() {
        let prev = (vertices.len() + i - 1) % vertices.len();

        let a = vertices[i];
        let b = vertices[prev];
        if ((a.z > position.z) != (b.z > position.z))
            && (position.x < (b.x - a.x) * (position.z - a.z) / (b.z - a.z) + a.x)
        {
            inside = !inside;
        }
    }

    inside
}

fn closest_height_in_triangle(a: Vec3, b: Vec3, c: Vec3, position: Vec3) -> Option<f32> {
    let v0 = c - a;
    let v1 = b - a;
    let v2 = position - a;

    let mut denom = v0.x * v1.z - v0.z * v1.x;
    const EPS: f32 = 0.000001;
    if denom.abs() < EPS {
        return None;
    }

    let mut u = v1.z * v2.x - v1.x * v2.z;
    let mut v = v0.x * v2.z - v0.z * v2.x;

    if denom < 0.0 {
        denom = -denom;
        u = -u;
        v = -v;
    }

    if u >= 0.0 && v >= 0.0 && (u + v) <= denom {
        return Some(a.y + (v0.y * u + v1.y * v) / denom);
    }

    None
}

pub fn build_tile_nav_mesh(poly_mesh: PolyMesh, waymap: &Nav) -> TileNavMesh {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::build_nav_mesh").entered();

    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons: Vec<NavPolygon> = poly_mesh
        .polygons
        .into_iter()
        .zip(poly_mesh.edges.iter())
        .map(|(indices, edges)| {
            // Pre build internal links.
            let links = edges
                .iter()
                .enumerate()
                .filter_map(|(i, edge)| {
                    let EdgeConnection::Internal(neighbour_polygon) = edge else {
                        return None;
                    };
                    Some(Link::Internal {
                        edge: i as u8,
                        neighbour_polygon: *neighbour_polygon,
                    })
                })
                .collect();

            NavPolygon { links, indices }
        })
        .collect();

    let tile_origin = waymap.get_tile_minimum_bound_with_border();
    // Scale the vertices to tile space
    let vertices: Vec<Vec3> = poly_mesh
        .vertices
        .iter()
        .map(|vertex| {
            Vec3::new(
                tile_origin.x + vertex.x as f32 * waymap.cell_width,
                tile_origin.y + vertex.y as f32 * waymap.cell_height,
                tile_origin.z + vertex.z as f32 * waymap.cell_width,
            )
        })
        .collect();

    TileNavMesh {
        vertices,
        edges: poly_mesh.edges,
        polygons,
        areas: poly_mesh.areas,
    }
}

pub fn connect_external_links(
    tile: &mut TileNavMesh,
    tile_transform: &GlobalTransform,
    neighbour: &TileNavMesh,
    neighbour_transform: &GlobalTransform,
    neighbour_direction: EdgeConnectionDirection,
    neighbour_to_self_direction: EdgeConnectionDirection,
    remove_existing_links: bool,
    step_height: f32,
) {
    for (poly_index, polygon) in tile.polygons.iter_mut().enumerate() {
        if remove_existing_links {
            polygon.links.retain(|link| match link {
                Link::Internal { .. } => true,
                Link::External { direction, .. } => *direction != neighbour_direction,
            });
        }

        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            // filter to only external edges on the side we are interested in
            let EdgeConnection::External(edge_direction) = edge else {
                continue;
            };
            if *edge_direction != neighbour_direction {
                continue;
            }

            let vertex_a =
                tile_transform.transform_point(tile.vertices[polygon.indices[edge_index] as usize]);
            let vertex_b = tile_transform.transform_point(
                tile.vertices[polygon.indices[(edge_index + 1) % polygon.indices.len()] as usize],
            );

            let (connection_count, connected_polys, connection_areas) =
                find_connecting_polygons_in_tile(
                    &vertex_a,
                    &vertex_b,
                    neighbour,
                    neighbour_transform,
                    neighbour_to_self_direction,
                    step_height,
                );

            polygon.links.extend((0..connection_count).map(|i| {
                let neighbour_polygon = connected_polys[i];
                let area = connection_areas[i];

                let (mut bound_min, mut bound_max) = if neighbour_to_self_direction
                    == EdgeConnectionDirection::XNegative
                    || neighbour_to_self_direction == EdgeConnectionDirection::XPositive
                {
                    let min = (area.x - vertex_a.z) / (vertex_b.z - vertex_a.z);
                    let max = (area.y - vertex_a.z) / (vertex_b.z - vertex_a.z);

                    (min, max)
                } else {
                    let min = (area.x - vertex_a.x) / (vertex_b.x - vertex_a.x);
                    let max = (area.y - vertex_a.x) / (vertex_b.x - vertex_a.x);

                    (min, max)
                };

                if bound_min > bound_max {
                    std::mem::swap(&mut bound_min, &mut bound_max);
                }

                let min_byte = (bound_min.clamp(0.0, 1.0) * 255.0).round() as u8;
                let max_byte = (bound_max.clamp(0.0, 1.0) * 255.0).round() as u8;

                Link::External {
                    edge: edge_index as u8,
                    neighbour_polygon,
                    direction: neighbour_direction,
                    bound_min: min_byte,
                    bound_max: max_byte,
                }
            }));
            break; // We can only have one edge parallel to the direction in a triangle.
        }
    }
}

const MAX_CONNECTING_POLYGONS: usize = 8;

fn find_connecting_polygons_in_tile(
    vertex_a: &Vec3,
    vertex_b: &Vec3,
    tile: &TileNavMesh,
    tile_transform: &GlobalTransform,
    side: EdgeConnectionDirection,
    step_height: f32,
) -> (
    usize,
    [u16; MAX_CONNECTING_POLYGONS],
    [Vec2; MAX_CONNECTING_POLYGONS],
) {
    let mut connecting_polys = [0; MAX_CONNECTING_POLYGONS];
    let mut connection_area = [Vec2::ZERO; MAX_CONNECTING_POLYGONS];
    let mut count = 0;

    let (in_min, in_max) = calculate_slab_end_points(vertex_a, vertex_b, side);
    let in_pos = get_slab_position(vertex_a, side);

    for (poly_index, polygon) in tile.polygons.iter().enumerate() {
        for (edge_index, edge) in tile.edges[poly_index].iter().enumerate() {
            let EdgeConnection::External(direction) = edge else {
                continue;
            };
            if *direction != side {
                continue;
            }

            let vertex_c =
                tile_transform.transform_point(tile.vertices[polygon.indices[edge_index] as usize]);
            let vertex_d = tile_transform.transform_point(
                tile.vertices[polygon.indices[(edge_index + 1) % polygon.indices.len()] as usize],
            );

            let edge_pos = get_slab_position(&vertex_c, side);

            if (in_pos - edge_pos).abs() > 0.01 {
                continue;
            }
            let (edge_min, edge_max) = calculate_slab_end_points(&vertex_c, &vertex_d, side);

            if !check_slabs_overlap(in_min, in_max, edge_min, edge_max, 0.01, step_height) {
                continue;
            }

            if count < connecting_polys.len() {
                connecting_polys[count] = poly_index as u16;
                connection_area[count] =
                    Vec2::new(in_min.x.max(edge_min.x), in_max.x.min(edge_max.x));
                count += 1;
            }
            break;
        }
    }

    (count, connecting_polys, connection_area)
}

fn calculate_slab_end_points(
    vertex_a: &Vec3,
    vertex_b: &Vec3,
    side: EdgeConnectionDirection,
) -> (Vec2, Vec2) {
    if side == EdgeConnectionDirection::XNegative || side == EdgeConnectionDirection::XPositive {
        if vertex_a.z < vertex_b.z {
            let min = vertex_a.zy();
            let max = vertex_b.zy();

            (min, max)
        } else {
            let min = vertex_b.zy();
            let max = vertex_a.zy();

            (min, max)
        }
    } else if vertex_a.x < vertex_b.x {
        let min = vertex_a.xy();
        let max = vertex_b.xy();

        (min, max)
    } else {
        let min = vertex_b.xy();
        let max = vertex_a.xy();

        (min, max)
    }
}

fn get_slab_position(vertex: &Vec3, side: EdgeConnectionDirection) -> f32 {
    match side {
        EdgeConnectionDirection::XNegative => vertex.x,
        EdgeConnectionDirection::ZPositive => vertex.z,
        EdgeConnectionDirection::XPositive => vertex.x,
        EdgeConnectionDirection::ZNegative => vertex.z,
    }
}

fn check_slabs_overlap(
    a_min: Vec2,
    a_max: Vec2,
    b_min: Vec2,
    b_max: Vec2,
    edge_shrink: f32,
    allowed_step: f32,
) -> bool {
    let min_edge = (a_min.x + edge_shrink).max(b_min.x + edge_shrink);
    let max_edge = (a_max.x - edge_shrink).min(b_max.x - edge_shrink);
    if min_edge > max_edge {
        return false;
    }

    let a_d = (a_max.y - a_min.y) / (a_max.x - a_min.x);
    let a_k = a_min.y - a_d * a_min.x;

    let b_d = (b_max.y - b_min.y) / (b_max.x - b_min.x);
    let b_k = b_min.y - a_d * b_min.x;

    let a_min_y = a_d * min_edge + a_k;
    let a_max_y = a_d * max_edge + a_k;

    let b_min_y = b_d * min_edge + b_k;
    let b_max_y = b_d * max_edge + b_k;

    let delta_min = b_min_y - a_min_y;
    let delta_max = b_max_y - a_max_y;

    if delta_min * delta_max < 0.0 {
        return true;
    }

    let threshold = (allowed_step * 2.0).powi(2);

    delta_min * delta_min <= threshold || delta_max * delta_max <= threshold
}

// /// A reference to an edge on a navigation mesh.
// #[derive(PartialEq, Eq, Debug, Clone, Hash, Default, Reflect)]
// pub(crate) struct MeshEdgeRef {
//     /// The index of the polygon that this edge belongs to.
//     pub(crate) polygon_index: usize,
//     /// The index of the edge within the polygon.
//     pub(crate) edge_index: usize,
// }

// #[derive(PartialEq, Debug, Clone, Reflect)]
// pub(crate) struct Connectivity {
//     /// The index of the polygon that this edge leads to.
//     pub(crate) polygon_index: usize,
//     /// The distances of travelling across this connection. The first is the
//     /// distance travelled across the starting node, and the second is the
//     /// distance travelled across the destination node. These must be multiplied
//     /// by the actual node costs.
//     pub(crate) travel_distances: (f32, f32),
// }
