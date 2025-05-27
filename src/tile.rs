use std::cmp::Ordering;

use bevy::{
    asset::RenderAssetUsages,
    math::bounding::Aabb3d,
    platform::collections::{HashMap, HashSet},
    prelude::*, render::mesh::Indices,
};
use disjoint::DisjointSet;

use crate::{
    Connectivity, MeshEdgeRef, ValidPolygon,
    archipelago::ArchipelagoTiles,
    bounding_box::BoundingAabb3d,
    build_contours,
    collider::*,
    heightfields::*,
    mesher::{EdgeConnection, PolyMesh, build_poly_mesh},
    nav_mesh::{NavigationMesh, PreNavigationMesh, ValidationError},
    prelude::Archipelago,
    regions::build_regions,
    tiles::{Link, NavMeshTile},
};

#[derive(Component, Reflect)]
#[require(Transform, TileAffectors)] //TileGeneration
pub struct Tile(pub UVec2);


/// Generation ticker for tiles.
/// Used only add newest nav_mesh on generation
// #[derive(Component, Reflect, Default, Deref, DerefMut)]
// pub struct TileGeneration(pub u64);

#[derive(Default, Component, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct TileAffectors(pub HashSet<Entity>);

/// Ref to Archipelago, added if not present when Character is added
#[derive(Component, Debug, Reflect)]
#[relationship(relationship_target = ArchipelagoTiles)]
pub struct TileArchipelago(pub Entity);

#[derive(Component, Clone, Debug, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct TileNavMesh(pub Handle<NavigationMesh>);

/// All Geometry collections are in local space of the tile
pub(crate) async fn build_tile(
    archipelago: Archipelago,
    geometry_collections: Vec<GeometryCollection>,
    heightfield_collections: Vec<HeightFieldCollection>,
) -> Result<(NavigationMesh, Mesh), ValidationError> {
    #[cfg(feature = "trace")]
    let _span = info_span!("Build Tile", name = "raven::build_tile").entered();

    let triangle_collection: Vec<TriangleCollection> = convert_geometry(geometry_collections);
    let voxelized_tile =
        build_heightfield_tile(&archipelago, triangle_collection, heightfield_collections);
    let mut open_tile = build_open_heightfield_tile(voxelized_tile, &archipelago);
    erode_walkable_area(&mut open_tile, &archipelago);
    calculate_distance_field(&mut open_tile, &archipelago);
    build_regions(&mut open_tile, &archipelago);
    let contour_set = build_contours(&open_tile, &archipelago);
    let poly_tile = build_poly_mesh(contour_set, &archipelago, &open_tile);
    let nav_mesh_tile = build_nav_mesh_tile(poly_tile.clone(), &archipelago);
    let pre_nav_mesh = build_pre_navigation_mesh(nav_mesh_tile);
    let nav_mesh = build_nav_mesh(&pre_nav_mesh)?;
    let mesh = build_mesh(poly_tile, &archipelago);

    Ok((nav_mesh, mesh))
}

pub fn build_nav_mesh_tile(poly_mesh: PolyMesh, archipelago: &Archipelago) -> NavMeshTile {
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

            crate::tiles::Polygon { links, indices }
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

    NavMeshTile {
        vertices,
        edges: poly_mesh.edges,
        polygons,
        areas: poly_mesh.areas,
    }
}

fn build_mesh(poly_mesh: PolyMesh, archipelago: &Archipelago) -> Mesh {
        #[cfg(feature = "trace")]
    let _span = info_span!("raven::build_mesh").entered();

    // Slight worry that the compiler won't optimize this but damn, it's cool.
    let polygons: Vec<crate::tiles::Polygon> = poly_mesh
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

            crate::tiles::Polygon { links, indices }
        })
        .collect();

    let tile_origin = archipelago.get_tile_minimum_bound_with_border();

    let positions: Vec<[f32; 3]> = poly_mesh
        .vertices
        .iter()
        .map(|vertex| {
            [
                tile_origin.x + vertex.x as f32 * archipelago.cell_width,
                tile_origin.y + vertex.y as f32 * archipelago.cell_height,
                tile_origin.z + vertex.z as f32 * archipelago.cell_width,
            ]
        })
        .collect();
    let indices: Vec<u32> = polygons
            .iter()
            .flat_map(|polygon| {
                polygon
                    .indices                                                         
                    
            })
            .collect();



    
    let mut mesh = Mesh::new(
        bevy::render::mesh::PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_indices(Indices::U32(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    //mesh.compute_flat_normals();
    mesh
}

fn build_pre_navigation_mesh(tile: NavMeshTile) -> PreNavigationMesh {
    PreNavigationMesh {
        vertices: tile.vertices,
        polygons: tile
            .polygons
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
        polygon_type_indices: vec![0; tile.polygons.len()],
    }
}

pub fn build_nav_mesh(mesh: &PreNavigationMesh) -> Result<NavigationMesh, ValidationError> {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::build_nav_mesh_tile").entered();

    /// Ensures required invariants of the navigation mesh, and computes
    /// additional derived properties to produce and optimized and validated
    /// navigation mesh. Returns an error if the navigation mesh is invalid in
    /// some way.
    if mesh.polygons.len() != mesh.polygon_type_indices.len() {
        return Err(ValidationError::TypeIndicesHaveWrongLength(
            mesh.polygons.len(),
            mesh.polygon_type_indices.len(),
        ));
    }

    // TODO: remove clone
    // build aabb
    let Ok(mesh_bounds) = Aabb3d::from_points(mesh.vertices.iter().map(|v| v.clone())) else {
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

    let mut region_sets = DisjointSet::with_len(mesh.polygons.len());
    let mut connectivity_set = HashMap::new();
    for (polygon_index, polygon) in mesh.polygons.iter().enumerate() {
        if polygon.len() < 3 {
            return Err(ValidationError::NotEnoughVerticesInPolygon(polygon_index));
        }

        for vertex_index in polygon {
            if *vertex_index >= mesh.vertices.len() {
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
            let a = mesh.vertices[prev].xz();
            let b = mesh.vertices[center].xz();
            let c = mesh.vertices[next].xz();

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

    let mut polygons = mesh
        .polygons
        .iter()
        .enumerate()
        .map(|(polygon_index, polygon_vertices)| {
            ValidPolygon {
                bounds: Aabb3d::from_points(
                    polygon_vertices.iter().map(|&vertex| mesh.vertices[vertex]),
                )
                .expect("Polygon must have at least one vertex"),
                center: polygon_vertices
                    .iter()
                    .map(|i| mesh.vertices[*i])
                    .sum::<Vec3>()
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
                type_index: mesh.polygon_type_indices[polygon_index],
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
                let edge_center = (mesh.vertices[edge.0] + mesh.vertices[edge.1]) / 2.0;
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
        vertices: mesh.vertices.clone(),
        boundary_edges,
        used_type_indices,
        //type_index_to_node_type: HashMap::new(),
    })
}

fn convert_geometry(geometry_collections: Vec<GeometryCollection>) -> Vec<TriangleCollection> {
    #[cfg(feature = "trace")]
    let _span = info_span!("raven::convert_geometry").entered();
    geometry_collections
        .into_iter()
        .map(|geometry_collection| TriangleCollection {
            transform: geometry_collection.transform,
            triangles: match geometry_collection.geometry_to_convert {
                GeometryToConvert::Collider(collider) => {
                    let triangles = Triangles::default();
                    rasterize_collider_inner(collider, triangles)
                }
                GeometryToConvert::ParryTriMesh(vertices, triangles) => {
                    Triangles::TriMesh(vertices.into_iter().map(Vec3::from).collect(), triangles)
                }
            },
            area: geometry_collection.area,
        })
        .collect()
}
