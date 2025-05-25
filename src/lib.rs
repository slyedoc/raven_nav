#![feature(test)]
extern crate test;

#[cfg(feature = "hot")]
use bevy_simple_subsecond_system::prelude::*;

mod agent;
mod archipelago;
mod bounding_box;
mod character;
mod collider;
mod contour;
#[cfg(feature = "debug_draw")]
pub mod debug_draw;
mod detail_mesh;
mod heightfields;
mod math;
mod mesher;
mod nav_mesh;
mod navigation;
mod regions;
mod tile;
mod tiles;


use agent::*;
use archipelago::*;

use bounding_box::Bounding;
use character::*;
use collider::*;
use contour::build_contours;
use nav_mesh::*;
use navigation::*;
use tile::*;

use std::sync::Arc;


use avian3d::{
    parry::{math::Isometry, na::Vector3, shape::HeightField},
    prelude::*,
};
use bevy::{
    ecs::entity::EntityHashMap, input::common_conditions::*, math::bounding::Aabb3d, platform::collections::HashSet, prelude::*, tasks::{futures_lite::future, AsyncComputeTaskPool}
};
use tiles::{NavMeshTile, create_nav_mesh_tile_from_poly_mesh};

pub mod prelude {
    #[cfg(feature = "debug_draw")]
    pub use crate::debug_draw::*;
    pub use crate::{RavenPlugin, agent::*, archipelago::*, character::*, collider::*};
}

const FLAG_BORDER_VERTEX: u32 = 0x10000;
const MASK_CONTOUR_REGION: u32 = 0xffff; // Masks out the above value.

pub struct RavenPlugin;

impl Plugin for RavenPlugin {
    fn build(&self, app: &mut App) {

        #[cfg(feature = "hot")]
        app.add_plugins(SimpleSubsecondPlugin::default());
        
        
        app.init_asset::<NavigationMesh>()
            .init_resource::<PathingResults>()
            .add_systems(
                Update,
                (
                    archipelago_changed,
                    handle_removed_affectors, //.in_set(OxidizedNavigation::Main),
                    update_tile_build_tasks,
                )
                    .chain(),
            )
            .add_systems(
                PostUpdate,
                (
                    update_navmesh_affectors,
                    //(add_agents_to_archipelago, add_characters_to_archipelago),
                    start_tile_build_tasks,
                    update_tile_build_tasks,
                    navigation::update_navigation.run_if(input_pressed(KeyCode::Space)),
                )
                    .chain()
                    .after(TransformSystem::TransformPropagate),
            )
            .register_type::<Agent>()
            .register_type::<AgentSettings>()
            .register_type::<AgentArchipelago>()
            .register_type::<Character>()
            .register_type::<CharacterSettings>()
            .register_type::<CharacterArchipelago>()
            .register_type::<TileAffectors>()
            .register_type::<Tile>()
            .register_type::<Handle<NavigationMesh>>()
            .register_type::<Bounding>()
            .register_type::<TileNavMesh>()
            .register_type::<Archipelago>()
            .register_type::<archipelago::TileLookup>()
            .register_type::<archipelago::ArchipelagoAgents>()
            .register_type::<archipelago::ArchipelagoCharacters>();
    }
}

// TODO: use arch bounding to find first available archipelago
/// Add archipelago refs to agents if not present
/// Only works when one archipelago exists, if not you need to set CharacterArchipelago yourself when spawning the character.
#[allow(dead_code)]
fn add_agents_to_archipelago(
    mut commands: Commands,
    mut query: Query<Entity, (Without<CharacterArchipelago>, Added<Agent>)>,
    archipelago: Single<Entity, With<Archipelago>>,
) {
    for e in query.iter_mut() {
        commands.entity(e).insert(AgentArchipelago(*archipelago));
    }
}

// TODO: use arch bounding to find first available archipelago
/// Add archipelago refs to characters if not present
/// Only works when one archipelago exists, if not you need to set CharacterArchipelago yourself when spawning the character.
#[allow(dead_code)]
fn add_characters_to_archipelago(
    mut commands: Commands,
    mut query: Query<Entity, (Without<CharacterArchipelago>, Added<Character>)>,
    archipelago: Single<Entity, With<Archipelago>>,
) {
    for e in query.iter_mut() {
        commands
            .entity(e)
            .insert(CharacterArchipelago(*archipelago));
    }
}

///
fn archipelago_changed(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &Archipelago,
            &ArchipelagoTiles,
            &mut ActiveGenerationTasks,
            &mut TileLookup,
            &mut DirtyTiles,
        ),
        (Or<(Changed<Transform>, Changed<Archipelago>)>,),
    >,
    mut collider_query: Query<
        Entity,
        (With<Collider>, With<GlobalTransform>, With<NavMeshAffector>),
    >,
) {
    for (e, arch, tiles, mut tasks, mut lookup, mut dirty) in query.iter_mut() {
        // clear any existing tiles
        for tile_e in tiles.iter() {
            commands.entity(tile_e).despawn();
        }
        tasks.clear();
        lookup.clear();
        dirty.clear();

        // setup bounding box
        commands
            .entity(e)
            .insert(Bounding(Aabb3d::new(Vec3A::ZERO, arch.world_half_extents)));

        // create islands
        let title_size = arch.get_tile_size();
        let half_tile_size = title_size * 0.5;

        let mut x_f = -arch.world_half_extents.x;
        let mut x_i = 0;
        let mut z_f = -arch.world_half_extents.z;
        let mut z_i = 0;

        while z_f < arch.world_half_extents.z {
            while x_f < arch.world_half_extents.x {
                let tile = UVec2::new(x_i, z_i);
                let translation = Vec3::new(x_f + half_tile_size, 0., z_f + half_tile_size);
                let tile_half_extends =
                    Vec3::new(half_tile_size, arch.world_half_extents.y, half_tile_size);

                let tile_id = commands
                    .spawn((
                        Name::new(format!("Tile ({},{})", tile.x, tile.y)),
                        Tile(tile),
                        Bounding(Aabb3d::new(Vec3::ZERO, tile_half_extends)),
                        Transform::from_translation(translation),
                        TileArchipelago(e),
                        ChildOf(e), // duplicated with TileArchipelago, but may have more children in the future
                    ))
                    .id();
                lookup.insert(tile, tile_id);
                dirty.insert(tile);

                x_f += title_size;
                x_i += 1;
            }
            x_f = -arch.world_half_extents.x;
            x_i = 0;

            z_f += title_size;
            z_i += 1;
        }

        // Hack: Since we need to update TileAffectors for any existing NavMeshAffectors, we add a marker component to all NavMeshAffectors
        // to trigger the update, would be nice mark Colliders as changed, but dont have easy way to do that
        for e in collider_query.iter_mut() {
            commands.entity(e).insert(UpdateTileAffectors);
        }
    }
}

// TODO: collapse these 2 to single struct
/// Converts the [`NavMeshTile`] into the corresponding [`PreNavigationMesh`].
fn tile_to_landmass_nav_mesh(tile: NavMeshTile) -> PreNavigationMesh {
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

#[expect(clippy::type_complexity)]
fn update_navmesh_affectors(
    mut commands: Commands,
    mut archipelago_query: Query<
        (&Archipelago, &GlobalTransform, &TileLookup, &mut DirtyTiles),
        Without<Tile>,
    >,
    mut tile_query: Query<&mut TileAffectors, With<Tile>>,
    mut collider_query: Query<
        (
            Entity,
            &Collider,
            &GlobalTransform,
            Has<UpdateTileAffectors>,
        ),
        (
            Or<(
                Or<(
                    Changed<GlobalTransform>,
                    Changed<Collider>,
                    Changed<NavMeshAffector>,
                )>,
                Added<UpdateTileAffectors>,
            )>,
            With<NavMeshAffector>,
        ),
    >,
) {
    for (archipelago, arch_transform, lookup, mut dirty_tiles) in archipelago_query.iter_mut() {
        // Expand by 2 * walkable_radius to match with erode_walkable_area.
        let border_expansion = f32::from(archipelago.walkable_radius * 2) * archipelago.cell_width;

        for (e, collider, global_transform, has_update) in collider_query.iter_mut() {
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
            let min_tile = archipelago.get_tile_containing_position(min_vec, arch_transform);

            let max_vec = Vec2::new(
                aabb.maxs.x + border_expansion,
                aabb.maxs.z + border_expansion,
            );
            let max_tile = archipelago.get_tile_containing_position(max_vec, arch_transform);

            // TODO: looping though all tiles for every collider not ideal,
            // maybe use sensors collisions? need to bench this vs old way way, but tile affectors and NavMeshAffectorRelations
            // where doing about the same thing
            for (tile_cord, tile) in lookup.iter() {
                let mut tile_affectors = tile_query.get_mut(*tile).unwrap();
                if (min_tile.x..=max_tile.x).contains(&tile_cord.x)
                    && (min_tile.y..=max_tile.y).contains(&tile_cord.y)
                {
                    tile_affectors.insert(e);
                    dirty_tiles.insert(*tile_cord);
                } else {
                    if tile_affectors.remove(&e) {
                        dirty_tiles.insert(*tile_cord);
                    }
                }
            }

            // remove change marker
            if has_update {
                commands.entity(e).remove::<UpdateTileAffectors>();
            }
        }
    }
}

/// Start the tile build tasks for the tiles that need to be generated.
#[allow(clippy::too_many_arguments)]
#[expect(clippy::type_complexity)]
fn start_tile_build_tasks(
    mut commands: Commands,
    mut tiles_to_generate: Local<Vec<UVec2>>,
    mut heightfields: Local<EntityHashMap<Arc<HeightField>>>,
    mut archipelago_query: Query<(
        &Archipelago,
        &TileLookup,
        &mut DirtyTiles,
        &mut ActiveGenerationTasks,
    )>,
    mut tile_query: Query<(&TileAffectors, &GlobalTransform), With<Tile>>,
    collider_query: Query<(Entity, &Collider, &GlobalTransform, &NavMeshAffector)>,
) {
    let thread_pool = AsyncComputeTaskPool::get();

    for (archipelago, tile_lookup, mut dirty_tiles, mut active_generation_tasks) in
        archipelago_query.iter_mut()
    {
        // see if we can start a new task
        let max = archipelago.max_tile_generation_tasks.get() as usize;
        let active = active_generation_tasks.0.len();
        if dirty_tiles.0.is_empty() || active >= max {
            continue;
        }

        let task_count = max.saturating_sub(active);
        tiles_to_generate.extend(dirty_tiles.0.iter().take(task_count));

        for tile_coord in tiles_to_generate.drain(..) {
            dirty_tiles.0.remove(&tile_coord);

            let tile_enity = tile_lookup.get(&tile_coord).unwrap();
            let (affectors, tile_transform) = tile_query.get_mut(*tile_enity).unwrap();

            // if tile has no affectors, remove it
            if affectors.is_empty() {
                // Spawn task to remove its nav mesh
                commands.entity(*tile_enity).remove::<TileNavMesh>();
                continue;
            }

            // Step 1: Gather Collider Geometry for for tile affectors relative to the tile
            let mut geometry_collections = Vec::with_capacity(affectors.len());
            let mut heightfield_collections = Vec::new(); // Storing heightfields separately because they are massive.

            for (entity, collider, collider_transform, nav_mesh_affector) in
                collider_query.iter_many(affectors.iter())
            {
                // Get the geometry type
                let geometry_result = get_geometry_type(collider.shape_scaled().as_typed_shape());

                // Convert the collider's transform to the tile's local space
                let transform = GlobalTransform::from(
                    tile_transform.affine().inverse() * collider_transform.affine(),
                );

                handle_geometry_result(
                    geometry_result,
                    entity,
                    transform,
                    Some(nav_mesh_affector.0),
                    &mut geometry_collections,
                    &mut heightfield_collections,
                    &mut heightfields,
                );
            }

            // Step 2: Clear any build tasks for this tile
            active_generation_tasks.retain(|job| job.entity != *tile_enity);

            // Step 3: Start Build Task.
            active_generation_tasks.0.push(NavMeshGenerationJob {
                entity: *tile_enity,
                //generation: tile_generation.0,
                task: thread_pool.spawn(build_tile(
                    archipelago.clone(),
                    geometry_collections,
                    heightfield_collections,
                )),
            });
        }
        heightfields.clear();
    }
}

/// Checks status of tile builds
fn update_tile_build_tasks(
    mut commands: Commands,
    mut archipelago_query: Query<&mut ActiveGenerationTasks, With<Archipelago>>,
    mut nav_meshes: ResMut<Assets<NavigationMesh>>,
) {
    for mut tasks in archipelago_query.iter_mut() {
        // check active tasks, canceling if not current
        tasks.0.retain_mut(|job| {
            if let Some(nav_mesh_tile) = future::block_on(future::poll_once(&mut job.task)) {
                // Generation complete
                let pre_nav_mesh = tile_to_landmass_nav_mesh(nav_mesh_tile);
                match pre_nav_mesh.validate() {
                    Ok(nav_mesh) => {
                        let nav_mesh_handle = nav_meshes.add(nav_mesh);
                        //info!("Tile {:?} generated {:?}", job.entity, nav_mesh_handle.id());
                        commands
                            .entity(job.entity)
                            .insert(TileNavMesh(nav_mesh_handle));
                    }
                    Err(err) => {
                        warn!("Failed to validate oxidized_navigation tile: {err:?}");
                        // remote existing nav mesh if any
                        commands.entity(job.entity).remove::<TileNavMesh>();
                    }
                };
                return false;
            }
            true
        });
    }
}

/// Update the tiles when the NavMeshAffector is removed.
fn handle_removed_affectors(
    mut removed_affectors: RemovedComponents<NavMeshAffector>,
    mut tile_query: Query<(&Tile, &mut TileAffectors, &TileArchipelago)>,
    mut archipelago_query: Query<&mut DirtyTiles, With<Archipelago>>,
    mut removed: Local<HashSet<Entity>>,
    mut intersection: Local<Vec<Entity>>,
) {
    removed.extend(removed_affectors.read());
    for (tile, mut tile_affectors, arch) in tile_query.iter_mut() {
        // see if any of the removed affectors are in the tile
        intersection.extend(tile_affectors.0.intersection(&removed));
        if intersection.is_empty() {
            continue;
        }
        // mark the tile as dirty
        let mut arch_dirty = archipelago_query.get_mut(arch.0).unwrap();
        arch_dirty.0.insert(tile.0);

        // remove from the tile
        for e in intersection.drain(..) {
            tile_affectors.remove(&e);
        }
    }
    removed.clear();
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
