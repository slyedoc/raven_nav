use std::{cmp::Ordering};

use bevy::{
    math::bounding::{Aabb3d, IntersectsVolume},
    platform::collections::{HashMap, HashSet},
    prelude::*,
};
use disjoint::DisjointSet;
use thiserror::Error;

use crate::{bounding_box::BoundingAabb3d, prelude::AgentOptions};

/// A navigation mesh.
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

impl PreNavigationMesh {
    /// Ensures required invariants of the navigation mesh, and computes
    /// additional derived properties to produce and optimized and validated
    /// navigation mesh. Returns an error if the navigation mesh is invalid in
    /// some way.
    pub fn validate(mut self) -> Result<NavigationMesh, ValidationError> {
        if self.polygons.len() != self.polygon_type_indices.len() {
            return Err(ValidationError::TypeIndicesHaveWrongLength(
                self.polygons.len(),
                self.polygon_type_indices.len(),
            ));
        }

        // TODO: remove clone
        // build aabb
        let Ok(mesh_bounds) = Aabb3d::from_points(self.vertices.iter().map(|v| v.clone())) else {
            return Err(ValidationError::NoVertices);
        };

        // build connectivity
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

        let mut region_sets = DisjointSet::with_len(self.polygons.len());
        let mut connectivity_set = HashMap::new();
        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            if polygon.len() < 3 {
                return Err(ValidationError::NotEnoughVerticesInPolygon(polygon_index));
            }

            for vertex_index in polygon {
                if *vertex_index >= self.vertices.len() {
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
                let a = self.vertices[prev].xz();
                let b = self.vertices[center].xz();
                let c = self.vertices[next].xz();

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
        let mut used_type_indices = HashSet::new();

        let mut polygons = self
            .polygons
            .drain(..)
            .enumerate()
            .map(|(polygon_index, polygon_vertices)| {
                ValidPolygon {
                    bounds: Aabb3d::from_points(
                        polygon_vertices.iter().map(|&vertex| self.vertices[vertex]),
                    )
                    .expect("Polygon must have at least one vertex"),
                    center: polygon_vertices
                        .iter()
                        .map(|i| self.vertices[*i])
                        .sum::<Vec3>()
                        / polygon_vertices.len() as f32,
                    connectivity: vec![None; polygon_vertices.len()],
                    vertices: polygon_vertices,
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
                    type_index: self.polygon_type_indices[polygon_index],
                }
            })
            .inspect(|polygon| {
                used_type_indices.insert(polygon.type_index);
            })
            .collect::<Vec<_>>();

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
                    let edge_center = (self.vertices[edge.0] + self.vertices[edge.1]) / 2.0;
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
            mesh_bounds,
            polygons,
            vertices: self.vertices,
            boundary_edges,
            used_type_indices,
            //type_index_to_node_type: HashMap::new(),
        })
    }
}

// TODO: I dont understand what this is doing yet
//#[derive(Debug)]
//pub struct NodeType;

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
    pub(crate) fn sample_point(
        &self,
        point: impl Into<Vec3A>, 
        agent_options: &AgentOptions,
    ) -> Option<(Vec3, usize)> {
        let p: Vec3A = point.into();
        let psd = &agent_options.point_sample_distance;
        
        let sample_padding = Aabb3d {
            min: Vec3A::new(
                -psd.horizontal_distance,
                -psd.distance_below,
                -psd.horizontal_distance,                
            ),
            max: Vec3A::new(
                psd.horizontal_distance,
                psd.distance_above,
                psd.horizontal_distance,                
            ),
        };

        // test if the point is within the bounds of the mesh
        let mesh_expanded_bounds = Aabb3d {
            min: self.mesh_bounds.min + sample_padding.min,
            max: self.mesh_bounds.max + sample_padding.max,
        };

        if !mesh_expanded_bounds.contains_point(p){
            return None;
        }

        let sample_box = Aabb3d {
            min: p + sample_padding.min,
            max: p + sample_padding.max,
        };

        fn project_to_triangle((v0, v1, v2): (Vec3, Vec3, Vec3), point: Vec3) -> Vec3 {
            // edge deltas
            let d0 = v1 - v0;
            let d1 = v2 - v1;
            let d2 = v0 - v2;
            
            // flatten to xz
            let f0 = d0.xz();
            let f1 = d1.xz();
            let f2 = d2.xz();

            // test each edge for outside‐region in XZ
            let p0 = point.xz() - v0.xz();
            if f0.perp_dot(p0) < 0.0 {
                let s = (f0.dot(p0) / f0.length_squared()).clamp(0.0, 1.0);
                return d0 * s + v0;
            }
            let p1 = point.xz() - v1.xz();
            if f1.perp_dot(p1) < 0.0 {
                let s = (f1.dot(p1) / f1.length_squared()).clamp(0.0, 1.0);
                return d1 * s + v1;
            }
            let p2 = point.xz() - v2.xz();
            if f2.perp_dot(p2) < 0.0 {
                let s = (f2.dot(p2) / f2.length_squared()).clamp(0.0, 1.0);
                return d2 * s + v2;
            }

            // inside triangle → project vertically along Y
            let normal = -(d0.cross(d2)).normalize();
            let h = normal.dot(point - v0) / normal.y;
            Vec3::new(point.x, point.y - h, point.z)            
        }

        let mut best_node = None;

        for (polygon_index, polygon) in self.polygons.iter().enumerate() {
            if !sample_box.intersects(&polygon.bounds) {
                continue;
            }
            for i in 2..polygon.vertices.len() {
                let v0 = self.vertices[polygon.vertices[0]];
                let v1 = self.vertices[polygon.vertices[i - 1]];
                let v2 = self.vertices[polygon.vertices[i]];
                    
                let proj = project_to_triangle((v0, v1, v2), p.into());

                // horizontal distance
                let dh = p.xz().distance(proj.xz());
                // vertical distance
                let dv = proj.y - p.y;

                if dh < psd.horizontal_distance
                    && (-psd.distance_below..psd.distance_above)
                        .contains(&dv)
                {
                    let score = dh
                        * psd.vertical_preference_ratio
                        + dv.abs();
                    
                    let better = best_node
                        .as_ref()
                        .map(|&(_, _, prev)| score < prev)
                        .unwrap_or(true);
                    if better {
                        best_node = Some((polygon_index, proj, score));
                    }
                }
            }
        }

        best_node.map(|(polygon_index, projected_point, _)| (projected_point, polygon_index))
    }
}

// A navigation mesh which has been validated and derived data has been
/// computed.
// pub struct ValidNavigationMesh {

// }

// impl ValidNavigationMesh {
//     /// Returns the bounds of the navigation mesh.
//     pub(crate) fn get_bounds(&self) -> BoundingBox {
//         self.mesh_bounds
//     }

//     // Gets the points that make up the specified edge.
//     pub(crate) fn get_edge_points(&self, edge_ref: MeshEdgeRef) -> (Vec3, Vec3) {
//         let polygon = &self.polygons[edge_ref.polygon_index];
//         let left_vertex_index = polygon.vertices[edge_ref.edge_index];

//         let right_vertex_index = if edge_ref.edge_index == polygon.vertices.len() - 1 {
//             0
//         } else {
//             edge_ref.edge_index + 1
//         };
//         let right_vertex_index = polygon.vertices[right_vertex_index];

//         (
//             self.vertices[left_vertex_index],
//             self.vertices[right_vertex_index],
//         )
//     }

//     /// Finds the node nearest to (and within `distance_to_node` of) `point`.
//     /// Returns the point on the nav mesh nearest to `point` and the index of the
//     /// polygon.
//     pub(crate) fn sample_point(
//         &self,
//         point: Vec3,
//         point_sample_distance: &PointSampleDistance,
//     ) -> Option<(Vec3, usize)> {
//         let sample_box = BoundingBox::new_box(
//             point
//                 + Vec3::new(
//                     -point_sample_distance.horizontal_distance,
//                     -point_sample_distance.horizontal_distance,
//                     -point_sample_distance.distance_below,
//                 ),
//             point
//                 + Vec3::new(
//                     point_sample_distance.horizontal_distance,
//                     point_sample_distance.horizontal_distance,
//                     point_sample_distance.distance_above,
//                 ),
//         );

//         fn project_to_triangle(triangle: (Vec3, Vec3, Vec3), point: Vec3) -> Vec3 {
//             let triangle_deltas = (
//                 triangle.1 - triangle.0,
//                 triangle.2 - triangle.1,
//                 triangle.0 - triangle.2,
//             );
//             let triangle_deltas_flat = (
//                 triangle_deltas.0.xy(),
//                 triangle_deltas.1.xy(),
//                 triangle_deltas.2.xy(),
//             );

//             if triangle_deltas_flat
//                 .0
//                 .perp_dot(point.xy() - triangle.0.xy())
//                 < 0.0
//             {
//                 let s = triangle_deltas_flat.0.dot(point.xy() - triangle.0.xy())
//                     / triangle_deltas_flat.0.length_squared();
//                 return triangle_deltas.0 * s.clamp(0.0, 1.0) + triangle.0;
//             }
//             if triangle_deltas_flat
//                 .1
//                 .perp_dot(point.xy() - triangle.1.xy())
//                 < 0.0
//             {
//                 let s = triangle_deltas_flat.1.dot(point.xy() - triangle.1.xy())
//                     / triangle_deltas_flat.1.length_squared();
//                 return triangle_deltas.1 * s.clamp(0.0, 1.0) + triangle.1;
//             }
//             if triangle_deltas_flat
//                 .2
//                 .perp_dot(point.xy() - triangle.2.xy())
//                 < 0.0
//             {
//                 let s = triangle_deltas_flat.2.dot(point.xy() - triangle.2.xy())
//                     / triangle_deltas_flat.2.length_squared();
//                 return triangle_deltas.2 * s.clamp(0.0, 1.0) + triangle.2;
//             }

//             let normal = -triangle_deltas.0.cross(triangle_deltas.2).normalize();
//             let height = normal.dot(point - triangle.0) / normal.z;
//             Vec3::new(point.x, point.y, point.z - height)
//         }

//         let mut best_node = None;

//         for (polygon_index, polygon) in self.polygons.iter().enumerate() {
//             if !sample_box.intersects_bounds(&polygon.bounds) {
//                 continue;
//             }
//             for i in 2..polygon.vertices.len() {
//                 let triangle = (
//                     polygon.vertices[0],
//                     polygon.vertices[i - 1],
//                     polygon.vertices[i],
//                 );
//                 let triangle = (
//                     self.vertices[triangle.0],
//                     self.vertices[triangle.1],
//                     self.vertices[triangle.2],
//                 );
//                 let projected_point = project_to_triangle(triangle, point);

//                 let distance_to_triangle_horizontal = point.xy().distance(projected_point.xy());
//                 let distance_to_triangle_vertical = projected_point.z - point.z;
//                 if distance_to_triangle_horizontal < point_sample_distance.horizontal_distance
//                     && (-point_sample_distance.distance_below..point_sample_distance.distance_above)
//                         .contains(&distance_to_triangle_vertical)
//                 {
//                     let distance_to_triangle = distance_to_triangle_horizontal
//                         * point_sample_distance.vertical_preference_ratio
//                         + distance_to_triangle_vertical.abs();
//                     let replace = match best_node {
//                         None => true,
//                         Some((_, _, previous_best_distance))
//                             if distance_to_triangle < previous_best_distance =>
//                         {
//                             true
//                         }
//                         _ => false,
//                     };
//                     if replace {
//                         best_node = Some((polygon_index, projected_point, distance_to_triangle));
//                     }
//                 }
//             }
//         }

//         best_node.map(|(polygon_index, projected_point, _)| (projected_point, polygon_index))
//     }
// }

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

#[derive(Debug, Reflect)]
#[allow(dead_code)]
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
