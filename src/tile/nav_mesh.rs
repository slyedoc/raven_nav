use bevy::{
    math::bounding::Aabb3d,
    platform::collections::HashSet,
    prelude::*,
};
use thiserror::Error;

use crate::{archipelago::Archipelago, tile::{mesher::{EdgeConnection, PolyMesh}, Polygon, Link}};

/// A navigation mesh.
///
#[derive(Clone)]
pub struct PreNavigationMesh {
    /// The vertices that make up the polygons.
    pub vertices: Vec<Vec3>,
    /// The polygons of the mesh. Polygons are indices to the `vertices` that
    /// make up the polygon. Polygons must be convex, and oriented
    /// counterclockwise (using the right hand rule). Polygons are assumed to be
    /// not self-intersecting.
    pub polygons: Vec<Vec<usize>>,
    /// The type index of each polygon. This type index is translated into a real
    /// [`crate::NodeType`] when assigned to an [`crate::Archipelago`]. Must be
    /// the same length as [`Self::polygons`].
    pub polygon_type_indices: Vec<usize>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Error)]
pub enum ValidationError {
    /// Stores the number of polygons and the number of type indices.
    #[error(
        "The polygon type indices do not have the same length as the polygons. There are {0} polygons, but {1} type indices."
    )]
    TypeIndicesHaveWrongLength(usize, usize),
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



pub fn build_pre_nav_mesh_tile(poly_mesh: PolyMesh, archipelago: &Archipelago) -> PreNavigationMesh {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::create_nav_mesh_tile_from_poly_mesh").entered();

    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons = poly_mesh
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

            crate::tile::Polygon { links, indices }
        })
        .collect();

    let tile_origin = archipelago.get_tile_minimum_bound_with_border();
    let vertices = poly_mesh
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

    // let tile = NavMeshTile {
    //     vertices,
    //     edges: poly_mesh.edges,
    //     polygons,
    //     areas: poly_mesh.areas,
    // };


    PreNavigationMesh {
        vertices: vertices,
        polygons: polygons
            .iter()
            .map(|polygon| {
                polygon
                    .indices
                    .iter()
                    .copied()
                    .map(|i| i as usize)
                    .collect()
            })
            .collect(),
        polygon_type_indices: vec![0; polygons.len()],
    }
}
