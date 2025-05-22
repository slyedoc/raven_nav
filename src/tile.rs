use std::sync::Arc;

use avian3d::parry::shape::HeightField;
use bevy::{platform::collections::HashSet, prelude::*};

use crate::{archipelago::ArchipelagoTiles, build_contours, build_heightfield_tile, conversion::{GeometryCollection, GeometryToConvert}, create_nav_mesh_tile_from_poly_mesh, heightfields::{build_open_heightfield_tile, calculate_distance_field, erode_walkable_area, HeightFieldCollection}, mesher::build_poly_mesh, nav_mesh::NavigationMesh, prelude::Archipelago, rasterize_collider_inner, regions::build_regions, tiles::NavMeshTile, TriangleCollection, Triangles };

#[derive(Component, Reflect)]
#[require(Transform, TileAffectors, TileGeneration)]
pub struct Tile(pub UVec2);

/// Generation ticker for tiles.
/// Used only add newest nav_mesh on generation
#[derive(Component, Reflect, Default, Deref, DerefMut)]
pub struct TileGeneration(pub u64);

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

    let triangle_collection: Vec<TriangleCollection> = {
        #[cfg(feature = "trace")]
        let _span =
            info_span!("Covert Geometrvey Collections", name = "raven::build_tile").entered();
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
                        let vertices = vertices.into_iter().map(Vec3::from).collect();
                        Triangles::TriMesh(vertices, triangles)
                    }
                },
                area: geometry_collection.area,
            })
            .collect()
    };

    let voxelized_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Heightfield Tile", name = "raven::build_tile").entered();
        build_heightfield_tile(
            &archipelago,
            &triangle_collection,
            &heightfield_collections,            
            //&arch_transform,
        )
    };

    let mut open_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Open Heightfield Tile", name = "raven::build_tile").entered();
        build_open_heightfield_tile(voxelized_tile, &archipelago)
    };

    // Remove areas that are too close to a wall.
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Erode walkable area", name = "raven::build_tile").entered();
        erode_walkable_area(&mut open_tile, &archipelago);
    }

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Calculate distance field", name = "raven::build_tile").entered();
        calculate_distance_field(&mut open_tile, &archipelago);
    }
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build regions", name = "raven::build_tile").entered();
        build_regions(&mut open_tile, &archipelago);
    }

    let contour_set = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build contours", name = "raven::build_tile").entered();
        build_contours(&open_tile, &archipelago)
    };

    let poly_mesh = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build poly mesh", name = "raven::build_tile").entered();
        build_poly_mesh(contour_set, &archipelago, &open_tile)
    };

    let nav_mesh_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!(
            "Create nav-mesh tile from poly mesh",
            name = "raven::build_tile"
        )
        .entered();
        create_nav_mesh_tile_from_poly_mesh(poly_mesh, &archipelago)
    };

    nav_mesh_tile
}