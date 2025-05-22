use bevy::{platform::collections::HashSet, prelude::*};

use crate::{
    archipelago::ArchipelagoTiles, build_contours, conversion::*,
    create_nav_mesh_tile_from_poly_mesh, heightfields::*, mesher::build_poly_mesh,
    nav_mesh::NavigationMesh, prelude::Archipelago, regions::build_regions, tiles::NavMeshTile,
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
) -> NavMeshTile {
    #[cfg(feature = "trace")]
    let _span = info_span!("Build Tile", name = "raven::build_tile").entered();

    let triangle_collection: Vec<TriangleCollection> = convert_geometry(geometry_collections);
    let voxelized_tile = build_heightfield_tile(&archipelago, &triangle_collection, &heightfield_collections);
    let mut open_tile = build_open_heightfield_tile(voxelized_tile, &archipelago);
    erode_walkable_area(&mut open_tile, &archipelago);    
    calculate_distance_field(&mut open_tile, &archipelago);
    build_regions(&mut open_tile, &archipelago);    
    let contour_set = build_contours(&open_tile, &archipelago);
    let poly_mesh = build_poly_mesh(contour_set, &archipelago, &open_tile);
    create_nav_mesh_tile_from_poly_mesh(poly_mesh, &archipelago)
}

fn convert_geometry(geometry_collections: Vec<GeometryCollection>) -> Vec<TriangleCollection> {
    #[cfg(feature = "trace")]
    let _span = info_span!("Convert Geometry Collections", name = "raven::").entered();
    geometry_collections
        .into_iter()
        .map(|geometry_collection| TriangleCollection {
            transform: geometry_collection.transform,
            triangles: match geometry_collection.geometry_to_convert {
                GeometryToConvert::Collider(collider) => {
                    let triangles = Triangles::default();
                    rasterize_collider_inner(collider, triangles)
                }
                GeometryToConvert::ParryTriMesh(vertices, triangles) =>                        
                    Triangles::TriMesh(vertices.into_iter().map(Vec3::from).collect(), triangles)
            },
            area: geometry_collection.area,
        })
        .collect()
}
