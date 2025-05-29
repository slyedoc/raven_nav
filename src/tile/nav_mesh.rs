use std::cmp::Ordering;

use bevy::{
    math::bounding::Aabb3d,
    platform::collections::{HashMap, HashSet},
    prelude::*,
};
use disjoint::DisjointSet;
use thiserror::Error;

use crate::{
    archipelago::Archipelago,
    tile::{
        Link, NavPolygon,
        mesher::{EdgeConnection, PolyMesh},
    },
    utils::Aabb3dExt,
};

/// A navigation mesh.
///

pub fn build_nav_mesh(
    poly_mesh: PolyMesh,
    archipelago: &Archipelago,
) -> Result<NavigationMesh, ValidationError> {
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

    let tile_origin = archipelago.get_tile_minimum_bound_with_border();
    // Scale the vertices to tile space
    let vertices: Vec<Vec3> = poly_mesh
        .vertices
        .iter()
        .map(|vertex| {
            Vec3::new(
                tile_origin.x + vertex.x as f32 * archipelago.cell_width,
                tile_origin.y + vertex.y as f32 * archipelago.cell_height,
                tile_origin.z + vertex.z as f32 * archipelago.cell_width,
            )
        })
        .collect();

    let polygon_type_indices = vec![0; polygons.len()];
    let mut region_sets = DisjointSet::with_len(polygons.len());
    let mut connectivity_set = HashMap::new();
    let polygons: Vec<Vec<usize>> = polygons
        .iter()
        .map(|polygon| {
            polygon
                .indices
                .iter()
                .copied()
                .map(|i| i as usize)
                .collect()
        })
        .collect();

    // build connectivity and validate mesh
    enum ConnectivityState {
        Disconnected,
        Boundary {
            polygon: usize,
            edge: usize,
        },
        Connected {
            polygon_1: usize,
            edge_1: usize,
            polygon_2: usize,
            edge_2: usize,
        },
    }

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        if polygon.len() < 3 {
            return Err(ValidationError::NotEnoughVerticesInPolygon(polygon_index));
        }

        for vertex_index in polygon {
            if *vertex_index >= vertices.len() {
                return Err(ValidationError::InvalidVertexIndexInPolygon(polygon_index));
            }
        }

        for i in 0..polygon.len() {
            let prev = polygon[if i == 0 { polygon.len() - 1 } else { i - 1 }];
            let center = polygon[i];
            let next = polygon[if i == polygon.len() - 1 { 0 } else { i + 1 }];

            // Check if the edge is degenerate.

            let edge = if center < next {
                (center, next)
            } else {
                (next, center)
            };
            if edge.0 == edge.1 {
                return Err(ValidationError::DegenerateEdgeInPolygon(polygon_index));
            }

            // Derive connectivity for the edge.

            let state = connectivity_set
                .entry(edge)
                .or_insert(ConnectivityState::Disconnected);
            match state {
                ConnectivityState::Disconnected => {
                    *state = ConnectivityState::Boundary {
                        polygon: polygon_index,
                        edge: i,
                    };
                }
                &mut ConnectivityState::Boundary {
                    polygon: polygon_1,
                    edge: edge_1,
                    ..
                } => {
                    *state = ConnectivityState::Connected {
                        polygon_1,
                        edge_1,
                        polygon_2: polygon_index,
                        edge_2: i,
                    };
                    region_sets.join(polygon_1, polygon_index);
                }
                ConnectivityState::Connected { .. } => {
                    return Err(ValidationError::DoublyConnectedEdge(edge.0, edge.1));
                }
            }

            // Check if the vertex is concave.
            let a = vertices[prev].xz();
            let b = vertices[center].xz();
            let c = vertices[next].xz();

            let left_edge = a - b;
            let right_edge = c - b;

            match right_edge.perp_dot(left_edge).partial_cmp(&0.0) {
                Some(Ordering::Less) => {} // convex
                // The right edge is parallel to the left edge, but they point in
                // opposite directions.
                Some(Ordering::Equal) if right_edge.dot(left_edge) < 0.0 => {}
                // right_edge is to the left of the left_edge (or they are parallel
                // and point in the same direciton), so the polygon is
                // concave.
                _ => return Err(ValidationError::ConcavePolygon(polygon_index)),
            }
        }
    }

    let mut region_to_normalized_region = HashMap::new();
    let mut polygons = polygons
        .iter()
        .enumerate()
        .map(|(polygon_index, polygon_vertices)| {
            ValidPolygon {
                bounds: Aabb3d::from_points(
                    polygon_vertices.iter().map(|&vertex| vertices[vertex]),
                )
                .expect("Polygon must have at least one vertex"),
                center: polygon_vertices.iter().map(|i| vertices[*i]).sum::<Vec3>()
                    / polygon_vertices.len() as f32,
                connectivity: vec![None; polygon_vertices.len()],
                vertices: polygon_vertices.clone(),
                region: {
                    let region = region_sets.root_of(polygon_index);
                    // Get around the borrow checker by deciding on the new normalized
                    // region beforehand.
                    let new_normalized_region = region_to_normalized_region.len();
                    // Either lookup the existing normalized region or insert the next
                    // unique index.
                    *region_to_normalized_region
                        .entry(region)
                        .or_insert_with(|| new_normalized_region)
                },
                type_index: polygon_type_indices[polygon_index],
            }
        })
        .collect::<Vec<_>>();

    let mut used_type_indices = HashSet::new();
    polygons.iter().for_each(|polygon| {
        used_type_indices.insert(polygon.type_index);
    });

    let mut boundary_edges = Vec::new();
    for connectivity_state in connectivity_set.values() {
        match connectivity_state {
            ConnectivityState::Disconnected => panic!("Value is never stored"),
            &ConnectivityState::Boundary { polygon, edge } => {
                boundary_edges.push(MeshEdgeRef {
                    edge_index: edge,
                    polygon_index: polygon,
                });
            }
            &ConnectivityState::Connected {
                polygon_1,
                edge_1,
                polygon_2,
                edge_2,
            } => {
                let edge = polygons[polygon_1].get_edge_indices(edge_1);
                let edge_center = (vertices[edge.0] + vertices[edge.1]) / 2.0;
                let travel_distances = (
                    polygons[polygon_1].center.distance(edge_center),
                    polygons[polygon_2].center.distance(edge_center),
                );
                polygons[polygon_1].connectivity[edge_1] = Some(Connectivity {
                    polygon_index: polygon_2,
                    travel_distances,
                });
                polygons[polygon_2].connectivity[edge_2] = Some(Connectivity {
                    polygon_index: polygon_1,
                    travel_distances: (travel_distances.1, travel_distances.0),
                });
            }
        }
    }

    Ok(NavigationMesh {
        polygons,
        vertices,
        boundary_edges,
        used_type_indices,
        //type_index_to_node_type: HashMap::new(),
    })
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Error)]
pub enum ValidationError {
    /// Stores the index of the polygon.
    #[error("The polygon at index {0} is concave or has edges in clockwise order.")]
    ConcavePolygon(usize),
    /// Stores the index of the polygon.
    #[error("The polygon at index {0} does not have at least 3 vertices.")]
    NotEnoughVerticesInPolygon(usize),
    /// Stores the index of the polygon.
    #[error("The polygon at index {0} references an out-of-bounds vertex.")]
    InvalidVertexIndexInPolygon(usize),
    /// Stores the index of the polygon.
    #[error("The polygon at index {0} contains a degenerate edge (an edge with zero length).")]
    DegenerateEdgeInPolygon(usize),
    /// Stores the indices of the two vertices that make up the edge.
    #[error("The edge made from vertices {0} and {1} is used by more than two polygons.")]
    DoublyConnectedEdge(usize, usize),
    #[error("There are no vertices in the polygon.")]
    NoVertices,
}

/// An asset holding a `landmass` nav mesh.
#[derive(Asset, TypePath, Debug)]
#[allow(dead_code)]
pub struct NavigationMesh {
    /// The vertices that make up the polygons.
    pub(crate) vertices: Vec<Vec3>,
    /// The polygons of the mesh.
    pub(crate) polygons: Vec<ValidPolygon>,
    /// The boundary edges in the navigation mesh. Edges are stored as pairs of
    /// vertices in a counter-clockwise direction. That is, moving along an edge
    /// (e.0, e.1) from e.0 to e.1 will move counter-clockwise along the
    /// boundary. The order of edges is undefined.
    pub(crate) boundary_edges: Vec<MeshEdgeRef>,
    /// The type indices used by this navigation mesh. This is a convenience for
    /// just iterating through every polygon and checking its type index. Note
    /// these don't correspond to [`crate::NodeType`]s yet. This occurs once
    /// assigned to an island.
    pub(crate) used_type_indices: HashSet<usize>,
    // /// The nav mesh data.
    // pub nav_mesh: Arc<ValidNavigationMesh>,
    // /// A map from the type indices used by [`Self::nav_mesh`] to the
    // /// [`NodeType`]s used in the [`crate::Archipelago`]. Type indices not
    // /// present in this map are implicitly assigned the "default" node type,
    // /// which always has a cost of 1.0.
    // pub type_index_to_node_type: HashMap<usize, NodeType>,
}

#[cfg(test)]
#[path = "nav_mesh_test.rs"]
mod test;

/// A reference to an edge on a navigation mesh.
#[derive(PartialEq, Eq, Debug, Clone, Hash, Default, Reflect)]
pub(crate) struct MeshEdgeRef {
    /// The index of the polygon that this edge belongs to.
    pub(crate) polygon_index: usize,
    /// The index of the edge within the polygon.
    pub(crate) edge_index: usize,
}

#[derive(PartialEq, Debug, Clone, Reflect)]
pub(crate) struct Connectivity {
    /// The index of the polygon that this edge leads to.
    pub(crate) polygon_index: usize,
    /// The distances of travelling across this connection. The first is the
    /// distance travelled across the starting node, and the second is the
    /// distance travelled across the destination node. These must be multiplied
    /// by the actual node costs.
    pub(crate) travel_distances: (f32, f32),
}

#[derive(Debug, Reflect, Clone, PartialEq)]

pub(crate) struct ValidPolygon {
    /// The vertices are indexes to the `vertices` Vec of the corresponding
    /// ValidNavigationMesh.
    pub(crate) vertices: Vec<usize>,
    /// The connectivity of each edge in the polygon. This is the same length as
    /// the number of edges (which is equivalent to `self.vertices.len()`).
    /// Entries that are `None` correspond to the boundary of the navigation
    /// mesh, while `Some` entries are connected to another node.
    pub(crate) connectivity: Vec<Option<Connectivity>>,
    /// The "region" that this polygon belongs to. Each region is disjoint from
    /// every other. A "direct" path only exists if the region matches between
    /// two nodes. An "indirect" path exists if regions are joined together
    /// through boundary links.
    pub(crate) region: usize,
    /// The "type" of this node. This is translated into a [`crate::NodeType`]
    /// once it is part of an island.
    pub(crate) type_index: usize,
    /// The bounding box of `vertices`.
    pub(crate) bounds: Aabb3d,
    /// The center of the polygon.
    pub(crate) center: Vec3,
}

impl ValidPolygon {
    /// Determines the vertices corresponding to `edge`.
    pub(crate) fn get_edge_indices(&self, edge: usize) -> (usize, usize) {
        (
            self.vertices[edge],
            self.vertices[if edge == self.vertices.len() - 1 {
                0
            } else {
                edge + 1
            }],
        )
    }
}
