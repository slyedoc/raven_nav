#![allow(warnings)]

use std::{
    num::NonZeroU16,
    sync::{Arc, RwLock},
};

use collider::{GeometryResult, get_geometry_type};
use contour::build_contours;
use conversion::{GeometryCollection, convert_geometry_collections};
use heightfields::{
    HeightFieldCollection, build_heightfield_tile, build_open_heightfield_tile,
    calculate_distance_field, erode_walkable_area,
};
use mesher::build_poly_mesh;
use regions::build_regions;

mod settings;
use settings::*;

mod components;
use components::*;

mod resources;
use resources::*;

mod collider;
mod contour;
mod conversion;
#[cfg(feature = "debug_draw")]
pub mod debug_draw;
mod detail_mesh;
mod heightfields;
mod math;
mod mesher;
mod regions;
mod tiles;

use avian3d::{
    parry::{math::Isometry, na::Vector3, shape::TypedShape},
    prelude::*,
};
use bevy::{
    ecs::entity::EntityHashMap,
    platform::collections::{HashMap, HashSet},
    prelude::*,
    tasks::{AsyncComputeTaskPool, futures_lite::future},
};
use smallvec::SmallVec;
use tiles::{NavMeshTile, NavMeshTiles, create_nav_mesh_tile_from_poly_mesh};

pub mod prelude {
    #[cfg(feature = "debug_draw")]
    pub use crate::debug_draw::*;
    pub use crate::{components::NavMeshAffector, settings::*, *};
}

const FLAG_BORDER_VERTEX: u32 = 0x10000;
const MASK_CONTOUR_REGION: u32 = 0xffff; // Masks out the above value.

pub struct RavenPlugin {
    pub settings: NavMeshSettings,
}

impl Plugin for RavenPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.settings.clone())
            .init_resource::<TileAffectors>()
            .init_resource::<DirtyTiles>()
            .init_resource::<NavMesh>()
            .init_resource::<GenerationTicker>()
            .init_resource::<ActiveGenerationTasks>()
            .add_event::<TileGenerated>()
            .add_systems(
                Update,
                (
                    (remove_finished_tasks, update_navmesh_affectors),
                    send_tile_rebuild_tasks.run_if(can_generate_new_tiles),
                )
                    .chain(),
                //.in_set(OxidizedNavigation::Main),
            )
            ;
    }
}

/// Event containing the tile coordinate of a generated/regenerated tile.
///
/// Emitted when a tile has been updated.
#[derive(Event)]
pub struct TileGenerated(pub UVec2);

fn remove_finished_tasks(
    mut active_generation_tasks: ResMut<ActiveGenerationTasks>,
    mut event: EventWriter<TileGenerated>,
) {
    active_generation_tasks.0.retain_mut(|task| {
        if let Some(tile) = future::block_on(future::poll_once(task)) {
            if let Some(tile) = tile {
                event.write(TileGenerated(tile));
            }

            false
        } else {
            true
        }
    });
}

#[expect(clippy::type_complexity)]
fn update_navmesh_affectors(
    nav_mesh_settings: Res<NavMeshSettings>,
    mut tile_affectors: ResMut<TileAffectors>,
    mut dirty_tiles: ResMut<DirtyTiles>,
    mut query: Query<
        (
            Entity,
            &Collider,
            &GlobalTransform,
            &mut NavMeshAffectorRelations,
        ),
        (
            Or<(
                Changed<GlobalTransform>,
                Changed<Collider>,
                Changed<NavMeshAffector>,
            )>,
            With<NavMeshAffector>,
        ),
    >,
) {
    // Expand by 2 * walkable_radius to match with erode_walkable_area.
    let border_expansion =
        f32::from(nav_mesh_settings.walkable_radius * 2) * nav_mesh_settings.cell_width;

    for (e, collider, global_transform, mut relation) in query.iter_mut() {
        let transform = global_transform.compute_transform();
        let iso = Isometry::new(
            transform.translation.into(),
            transform.rotation.to_scaled_axis().into(),
        );
        let local_aabb = collider.shape_scaled().compute_local_aabb();
        let aabb = local_aabb
            .scaled(&Vector3::new(
                transform.scale.x,
                transform.scale.y,
                transform.scale.z,
            ))
            .transform_by(&iso);

        let min_vec = Vec2::new(
            aabb.mins.x - border_expansion,
            aabb.mins.z - border_expansion,
        );
        let min_tile = nav_mesh_settings.get_tile_containing_position(min_vec);

        let max_vec = Vec2::new(
            aabb.maxs.x + border_expansion,
            aabb.maxs.z + border_expansion,
        );
        let max_tile = nav_mesh_settings.get_tile_containing_position(max_vec);

        // Remove from previous relation
        for old_tile in relation.iter().filter(|tile_coord| {
            min_tile.x > tile_coord.x
                || min_tile.y > tile_coord.y
                || max_tile.x < tile_coord.x
                || max_tile.y < tile_coord.y
        }) {
            if let Some(affectors) = tile_affectors.get_mut(old_tile) {
                affectors.remove(&e);
                dirty_tiles.0.insert(*old_tile);
            }
        }
        relation.clear();

        for x in min_tile.x..=max_tile.x {
            for y in min_tile.y..=max_tile.y {
                let tile_coord = UVec2::new(x, y);

                let affectors = if let Some(affectors) = tile_affectors.get_mut(&tile_coord) {
                    affectors
                } else {
                    unsafe {
                        tile_affectors
                            .insert_unique_unchecked(tile_coord, HashSet::default())
                            .1
                    }
                };
                affectors.insert(e);

                relation.push(tile_coord);
                dirty_tiles.0.insert(tile_coord);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
#[expect(clippy::type_complexity)]
fn send_tile_rebuild_tasks(
    mut active_generation_tasks: ResMut<ActiveGenerationTasks>,
    mut generation_ticker: ResMut<GenerationTicker>,
    mut dirty_tiles: ResMut<DirtyTiles>,
    mut tiles_to_generate: Local<Vec<UVec2>>,
    mut heightfields: Local<EntityHashMap<Arc<HeightFieldCollection>>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
    tile_affectors: Res<TileAffectors>,
    collider_query: Query<
        (
            Entity,
            &Collider,
            &GlobalTransform,
            Option<&NavMeshAreaType>,
        ),
        With<NavMeshAffector>,
    >,
) {
    let thread_pool = AsyncComputeTaskPool::get();

    let max_task_count = (nav_mesh_settings
        .max_tile_generation_tasks
        .unwrap_or(NonZeroU16::MAX)
        .get() as usize)
        .saturating_sub(active_generation_tasks.0.len());
    tiles_to_generate.extend(dirty_tiles.0.iter().take(max_task_count));

    for tile_coord in tiles_to_generate.drain(..) {
        dirty_tiles.0.remove(&tile_coord);

        generation_ticker.0 += 1;

        let Some(affectors) = tile_affectors.get(&tile_coord) else {
            // Spawn task to remove tile.
            thread_pool
                .spawn(remove_tile(
                    generation_ticker.0,
                    tile_coord,
                    nav_mesh.0.clone(),
                ))
                .detach();
            continue;
        };
        if affectors.is_empty() {
            // Spawn task to remove tile.
            thread_pool
                .spawn(remove_tile(
                    generation_ticker.0,
                    tile_coord,
                    nav_mesh.0.clone(),
                ))
                .detach();
            continue;
        }

        // Step 1: Gather data.
        let mut geometry_collections = Vec::with_capacity(affectors.len());
        // Storing heightfields separately because they are massive.
        let mut heightfield_collections = Vec::new();

        let mut collider_iter = collider_query.iter_many(affectors.iter());
        while let Some((entity, collider, global_transform, nav_mesh_affector)) =
            collider_iter.fetch_next()
        {
            let area = nav_mesh_affector.map_or(Some(Area(0)), |area_type| area_type.0);

            let geometry_result = get_geometry_type(collider.shape_scaled().as_typed_shape());
            let transform = global_transform.compute_transform();
            handle_geometry_result(
                geometry_result,
                entity,
                transform,
                area,
                &mut geometry_collections,
                &mut heightfield_collections,
                &mut heightfields,
            );
        }

        // Step 2: Acquire nav_mesh lock
        let nav_mesh = nav_mesh.0.clone();

        // Step 3: Make it a task.
        let task = thread_pool.spawn(build_tile(
            generation_ticker.0,
            tile_coord,
            nav_mesh_settings.clone(),
            geometry_collections,
            heightfield_collections.into_boxed_slice(),
            nav_mesh,
        ));

        active_generation_tasks.0.push(task);
    }
    heightfields.clear();
}

async fn build_tile(
    generation: u64,
    tile_coord: UVec2,
    nav_mesh_settings: NavMeshSettings,
    geometry_collections: Vec<GeometryCollection>,
    heightfields: Box<[Arc<HeightFieldCollection>]>,
    nav_mesh: Arc<RwLock<NavMeshTiles>>,
) -> Option<UVec2> {
    #[cfg(feature = "trace")]
    let _span = info_span!("Async build Tile").entered();

    let nav_mesh_tile = build_tile_sync(
        geometry_collections,
        tile_coord,
        heightfields,
        &nav_mesh_settings,
    );

    let Ok(mut nav_mesh) = nav_mesh.write() else {
        error!("Nav-Mesh lock has been poisoned. Generation can no longer be continued.");
        return None;
    };

    if nav_mesh.tile_generations.get(&tile_coord).unwrap_or(&0) < &generation {
        nav_mesh.tile_generations.insert(tile_coord, generation);

        nav_mesh.add_tile(tile_coord, nav_mesh_tile, &nav_mesh_settings);

        Some(tile_coord)
    } else {
        None
    }
}

pub fn build_tile_sync(
    geometry_collections: Vec<GeometryCollection>,
    tile_coord: UVec2,
    heightfields: Box<[Arc<HeightFieldCollection>]>,
    nav_mesh_settings: &NavMeshSettings,
) -> NavMeshTile {
    let triangle_collection = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Convert Geometry Collections").entered();
        convert_geometry_collections(geometry_collections)
    };

    let voxelized_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Heightfield Tile").entered();
        build_heightfield_tile(
            tile_coord,
            &triangle_collection,
            &heightfields,
            nav_mesh_settings,
        )
    };

    let mut open_tile = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build Open Heightfield Tile").entered();
        build_open_heightfield_tile(voxelized_tile, nav_mesh_settings)
    };

    // Remove areas that are too close to a wall.
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Erode walkable area").entered();
        erode_walkable_area(&mut open_tile, nav_mesh_settings);
    }

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Calculate distance field").entered();
        calculate_distance_field(&mut open_tile, nav_mesh_settings);
    }
    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build regions").entered();
        build_regions(&mut open_tile, nav_mesh_settings);
    }

    let contour_set = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build contours").entered();
        build_contours(&open_tile, nav_mesh_settings)
    };

    let poly_mesh = {
        #[cfg(feature = "trace")]
        let _span = info_span!("Build poly mesh").entered();
        build_poly_mesh(contour_set, nav_mesh_settings, &open_tile)
    };

    {
        #[cfg(feature = "trace")]
        let _span = info_span!("Create nav-mesh tile from poly mesh").entered();
        create_nav_mesh_tile_from_poly_mesh(poly_mesh, tile_coord, nav_mesh_settings)
    }
}

fn handle_geometry_result(
    type_to_convert: GeometryResult,
    entity: Entity,
    global_transform: Transform,
    area: Option<Area>,
    geometry_collections: &mut Vec<GeometryCollection>,
    heightfield_collections: &mut Vec<Arc<HeightFieldCollection>>,
    heightfields: &mut EntityHashMap<Arc<HeightFieldCollection>>,
) {
    match type_to_convert {
        GeometryResult::GeometryToConvert(geometry_to_convert) => {
            geometry_collections.push(GeometryCollection {
                transform: global_transform,
                geometry_to_convert,
                area,
            });
        }
        GeometryResult::Heightfield(heightfield) => {
            // Deduplicate heightfields.
            let heightfield = if let Some(heightfield) = heightfields.get(&entity) {
                heightfield.clone()
            } else {
                let heightfield = Arc::new(HeightFieldCollection {
                    transform: global_transform,
                    heightfield: heightfield.clone(),
                    area,
                });

                heightfields.insert(entity, heightfield.clone());

                heightfield
            };

            heightfield_collections.push(heightfield);
        }
        GeometryResult::Compound(results) => {
            for (isometry, result) in results {
                let translation = Vec3::from(isometry.translation);
                let rotation = Quat::from(isometry.rotation);
                let mut transform = global_transform;
                transform.translation += translation;
                transform.rotation *= rotation;
                handle_geometry_result(
                    result,
                    entity,
                    transform,
                    area,
                    geometry_collections,
                    heightfield_collections,
                    heightfields,
                );
            }
        }
        GeometryResult::Unsupported => {}
    }
}

async fn remove_tile(
    generation: u64, // This is the max generation we remove. Should we somehow strangely be executing this after a new tile has arrived we won't remove it.
    tile_coord: UVec2,
    nav_mesh: Arc<RwLock<NavMeshTiles>>,
) {
    let Ok(mut nav_mesh) = nav_mesh.write() else {
        error!("Nav-Mesh lock has been poisoned. Generation can no longer be continued.");
        return;
    };

    if nav_mesh.tile_generations.get(&tile_coord).unwrap_or(&0) < &generation {
        nav_mesh.tile_generations.insert(tile_coord, generation);
        nav_mesh.remove_tile(tile_coord);
    }
}

fn can_generate_new_tiles(
    active_generation_tasks: Res<ActiveGenerationTasks>,
    dirty_tiles: Res<DirtyTiles>,
    nav_mesh_settings: Res<NavMeshSettings>,
) -> bool {
    nav_mesh_settings
        .max_tile_generation_tasks
        .is_none_or(|max_tile_generation_tasks| {
            active_generation_tasks.0.len() < max_tile_generation_tasks.get().into()
        })
        && !dirty_tiles.0.is_empty()
}

fn get_neighbour_index(tile_size: usize, index: usize, dir: usize) -> usize {
    match dir {
        0 => index - 1,
        1 => index + tile_size,
        2 => index + 1,
        3 => index - tile_size,
        _ => panic!("Not a valid direction"),
    }
}
