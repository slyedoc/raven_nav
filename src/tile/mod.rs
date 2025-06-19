pub mod contour;
pub mod detail_mesh;
pub mod mesher;
pub mod nav_mesh;
pub mod regions;
pub mod voxelization;

use bevy::{
    asset::RenderAssetUsages, math::bounding::Aabb3d, platform::collections::HashSet, prelude::*,
    render::mesh::Indices,
};
use raven_bvh::prelude::*;
use smallvec::SmallVec;

use crate::{collider::*, nav::*, tile::mesher::*, tile::nav_mesh::*, utils::Aabb3dExt};

#[derive(Component, Reflect, Deref, DerefMut)]
#[require(
    Transform,
    TileAffectors,
    Visibility // used for rendering mesh
)]
pub struct Tile(pub UVec2);

/// Tile bounds
#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct TileAabb(pub Aabb3d);

/// Tile Mesh Aaabb, generated from the nav mesh,
#[derive(Component, Reflect, Deref, DerefMut)]
#[reflect(Component)]
pub struct TileMeshAabb(pub Aabb3d);

/// Generation ticker for tiles.
/// Used only add newest nav_mesh on generation
// #[derive(Component, Reflect, Default, Deref, DerefMut)]
// pub struct TileGeneration(pub u64);

#[derive(Default, Component, Deref, DerefMut, Reflect)]
#[reflect(Component)]
pub struct TileAffectors(pub HashSet<Entity>);

/// Ref to Waymap, added if not present when Character is added
#[derive(Component, Debug, Reflect)]
#[relationship(relationship_target = WaymapTiles)]
pub struct TileWaymap(pub Entity);

/// Added to view mesh child of the tile, used for debug rendering
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
pub struct TileViewMesh;

pub(crate) type TileBuildResult = Option<(TileNavMesh, Aabb3d, Mesh, Bvh)>;

pub(crate) async fn build_tile(
    waymap: Nav,
    geometry_collections: Vec<GeometryCollection>,
    heightfield_collections: Vec<HeightFieldCollection>,
) -> TileBuildResult {
    #[cfg(feature = "trace")]
    let _span = info_span!("Build Tile", name = "raven::build_tile").entered();

    let triangle_collection: Vec<TriangleCollection> = convert_geometry(geometry_collections);
    let voxelized_tile =
        voxelization::build_heightfield_tile(&waymap, triangle_collection, heightfield_collections);
    let mut open_tile = voxelization::build_open_heightfield_tile(voxelized_tile, &waymap);
    voxelization::erode_walkable_area(&mut open_tile, &waymap);
    voxelization::calculate_distance_field(&mut open_tile, &waymap);
    regions::build_regions(&mut open_tile, &waymap);
    let contour_set = contour::build_contours(&open_tile, &waymap);
    let poly_tile = mesher::build_poly_mesh(contour_set, &waymap, &open_tile);
    let nav_mesh = nav_mesh::build_tile_nav_mesh(poly_tile.clone(), &waymap);
    let mesh = build_bevy_mesh(&poly_tile, &waymap);

    let Ok(aabb) = Aabb3d::from_points(nav_mesh.vertices.clone().into_iter()) else {
        // no vertices in nav mesh, can only happen when no walkable area is found
        return None;
    };
    let bvh = Bvh::from(&mesh);

    Some((nav_mesh, aabb, mesh, bvh))
}

fn build_bevy_mesh(poly_mesh: &PolyMesh, waymap: &Nav) -> Mesh {
    let tile_origin = waymap.get_tile_minimum_bound_with_border();
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(poly_mesh.vertices.len());
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(poly_mesh.vertices.len());
    let mut indices: Vec<u32> = Vec::with_capacity(poly_mesh.polygons.len() * 3);

    for v in poly_mesh.vertices.iter().map(|vertex| {
        Vec3::new(
            tile_origin.x + vertex.x as f32 * waymap.cell_width,
            tile_origin.y + vertex.y as f32 * waymap.cell_height,
            tile_origin.z + vertex.z as f32 * waymap.cell_width,
        )
    }) {
        positions.push([v.x, v.y, v.z]);
        // TODO, use area type to effect uv
        uvs.push([0.0, 0.0]);
    }
    for tri in &poly_mesh.polygons {
        indices.push(tri[0]);
        indices.push(tri[1]);
        indices.push(tri[2]);
    }

    let mut mesh = Mesh::new(
        bevy::render::mesh::PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    mesh.insert_indices(Indices::U32(indices));

    mesh.duplicate_vertices();
    mesh.compute_flat_normals();

    mesh
}

/// Representation of a link between different polygons either internal to the tile or external (crossing over to another tile).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
pub enum Link {
    Internal {
        /// Edge on self polygon.
        edge: u8,
        /// Index of polygon this polygon is linked to.
        neighbour_polygon: u16,
    },
    External {
        /// Edge on self polygon.
        edge: u8,
        /// Index of polygon this polygon is linked to.
        neighbour_polygon: u16,
        /// Direction toward the neighbour polygon's tile.
        direction: EdgeConnectionDirection,
        /// Min % of this edge that connects to the linked polygon.
        bound_min: u8, // % bound of edge that links to this.
        // MAx % of this edge that connects to the linked polygon.
        bound_max: u8, // For example: 10% -> 50% = the connected edge covers 10% from vertex A to B to 50%.
    },
}

/// A polygon within a nav-mesh tile.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Reflect)]
pub struct NavPolygon {
    pub indices: [u32; VERTICES_IN_TRIANGLE],
    pub links: SmallVec<[Link; VERTICES_IN_TRIANGLE]>, // This becomes a mess memory wise with a ton of different small objects around.
}

pub fn get_neighbour_index(tile_size: usize, index: usize, dir: usize) -> usize {
    match dir {
        0 => index - 1,
        1 => index + tile_size,
        2 => index + 1,
        3 => index - tile_size,
        _ => panic!("Not a valid direction"),
    }
}
