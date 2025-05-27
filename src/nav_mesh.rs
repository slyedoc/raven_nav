#[cfg(feature = "debug_draw")]
use crate::debug_draw::aabb3d_transform;
#[cfg(feature = "debug_draw")]
use bevy::color::palettes::tailwind;
use bevy::{
    asset::RenderAssetUsages,
    math::bounding::{Aabb3d, BoundingVolume, IntersectsVolume},
    platform::collections::{HashMap, HashSet},
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};
use disjoint::DisjointSet;
use std::cmp::Ordering;
use thiserror::Error;

use crate::{
    AgentOptions,
    bounding_box::{Bounding, BoundingAabb3d},
};

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
    /// The bounds of the mesh data itself. This is a tight bounding box around
    /// the vertices of the navigation mesh.
    pub(crate) mesh_bounds: Aabb3d,
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

impl NavigationMesh {
    /// Finds the node nearest to (and within `distance_to_node` of) `point`.
    /// Returns the point on the nav mesh nearest to `point` and the index of the
    /// polygon.
    ///
    /// # Arguments
    /// * `point` - The point to sample. This is in relative coordinates, so it should be
    ///     transformed by the tile Global transform inverse before being passed in.
    /// * `agent_options` - The options for the agent. This is used to determine
    ///
    ///

    pub(crate) fn sample_point(
        &self,
        world_point: impl Into<Vec3A>,
        transform: &GlobalTransform,
        tile_bounding: &Bounding,
        agent_options: &AgentOptions,
        #[cfg(feature = "debug_draw")] gizmos: &mut Gizmos<crate::debug_draw::RavenGizmos>,
    ) -> Option<(Vec3, usize)> {
        let point: Vec3A = world_point.into();
        let p = transform.affine().inverse().transform_point3a(point);
        
        let sample_aabb = agent_options.point_sample_distance.aabb();
        // see if mesh bounds plus sample_aabb intersects
        let mesh_contains_aabb = Aabb3d {
            min: self.mesh_bounds.min + sample_aabb.min,
            max: self.mesh_bounds.max + sample_aabb.max,
        };
        let mesh_contains = mesh_contains_aabb.contains_point(p);
        if mesh_contains {
            #[cfg(feature = "debug_draw")]
            gizmos.cuboid(
                aabb3d_transform(&mesh_contains_aabb, transform),
                tailwind::BLUE_600,
            );
        } else {
            return None;
        }

        // test if point + sample_aabb intersects any polygon
        let sample_aabb_reversed = agent_options.point_sample_distance.aabb_reversed();
        let point_sample_aabb = Aabb3d {
            min: p + sample_aabb_reversed.min,
            max: p + sample_aabb_reversed.max,
        };
        gizmos.cuboid(
            aabb3d_transform(&point_sample_aabb, transform),
            tailwind::GREEN_500,
        );

        #[cfg(feature = "debug_draw")]
        let mut best_node = None;

        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            let poly_contains = point_sample_aabb.intersects(&polygon.bounds);

            if !poly_contains {
                continue;
            }
            gizmos.cuboid(
                aabb3d_transform(&polygon.bounds.grow(Vec3A::Y * 0.1), transform),
                match poly_contains {
                    true => tailwind::YELLOW_500,
                    false => tailwind::RED_500,
                },
            );

            for i in 2..polygon.vertices.len() {
                let v0 = self.vertices[polygon.vertices[0]];
                let v1 = self.vertices[polygon.vertices[i - 1]];
                let v2 = self.vertices[polygon.vertices[i]];

                let (proj, edge_type) = project_point_to_triangle(p.into(), v0, v1, v2);

                // horizontal distance
                let dh = p.xz().distance(proj.xz());
                // vertical distance
                let dv = proj.y - p.y;

                if dh < agent_options.point_sample_distance.horizontal_distance
                    && (-agent_options.point_sample_distance.distance_below
                        ..agent_options.point_sample_distance.distance_above)
                        .contains(&dv)
                {
                    let score = dh
                        * agent_options
                            .point_sample_distance
                            .vertical_preference_ratio
                        + dv.abs();

                    dbg!("Score", proj, dh, dv);
                    let better = best_node
                        .as_ref()
                        .map(|&(_, _, prev)| score < prev)
                        .unwrap_or(true);
                    if better {
                        best_node = Some((polygon_index, proj, score));
                    }
                } else {
                    info!("{edge_type} dh: {dh}, dv: {dv}, proj {proj}");

                    //dbg!("Miss", polygon_index, dh, dv);
                }
            }
        }
        if best_node.is_none() {
            dbg!("No Best");
        }

        best_node.map(|(polygon_index, projected_point, _)| {
            (transform.transform_point(projected_point), polygon_index)
        })
    }
}

fn project_point_to_triangle(point: Vec3, a: Vec3, b: Vec3, c: Vec3) -> (Vec3, u16) {
    // define the triangle edges
    let e0 = b - a;
    let e1 = c - b;
    let e2 = a - c;

    // grab the xz components of the triangle edges for 2D test, "flattening" the triangle
    let f0 = e0.xz();
    let f1 = e1.xz();
    let f2 = e2.xz();

    // for each edge we ask "is the point's XY projection outside the half-plane defined by this edge?"
    if let Some(value) = project_to_edge(a, point, e0, f0) {
        return (value, 1);
    }
    if let Some(value) = project_to_edge(b, point, e1, f1) {
        return (value, 2);
    }
    if let Some(value) = project_to_edge(c, point, e2, f2) {
        return (value, 3);
    }

    // Inside triangle → project vertically along Y
    let normal = -(e0.cross(e2)).normalize();
    let h = normal.dot(point - a) / normal.z;
    dbg!(h);
    (Vec3::new(point.x, point.y - h, point.z), 0)
}

#[inline]
// is the point's XY projection outside the half-plane defined by this edge
fn project_to_edge(vertex: Vec3, point: Vec3, edge: Vec3, edge2d: Vec2) -> Option<Vec3> {
    // grab the point projection relative to the vertex
    let p0 = point.xz() - vertex.xz();
    // is the point’s XY projection outside the half-plane defined by this edge?
    // if it’s negative, the point lies “to the right” of that edge’s outward normal → outside.
    if edge2d.perp_dot(p0) < 0.0 {
        // if so, we compute the closest point on that edge segment by
        let s = (edge2d.dot(p0) / edge2d.length_squared()).clamp(0.0, 1.0);
        // Then lift back into 3D along the original (non-flat) edge:
        return Some(edge * s + vertex);
    }
    None
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
