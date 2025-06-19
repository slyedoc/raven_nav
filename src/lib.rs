#![allow(warnings)]
#![allow(unused_variables)]
#![feature(test)]
extern crate test;

mod agent;
mod character;
mod collider;
#[cfg(feature = "debug_draw")]
pub mod debug_draw;
mod math;
mod nav;
mod nav_ray_cast;
mod path_finding;
pub mod tile;
mod utils;

use crate::nav_ray_cast::NavLinks;
use agent::*;
use character::*;
use collider::*;
#[cfg(feature = "debug_draw")]
use debug_draw::*;
use nav::*;
use nav_ray_cast::*;
use raven_bvh::prelude::*;
use tile::{
    Tile, TileAabb, TileAffectors, TileMeshAabb, TileViewMesh, TileWaymap, build_tile,
    nav_mesh::TileNavMesh,
};

use avian3d::{
    parry::{math::Isometry, na::Vector3, shape::HeightField},
    prelude::*,
};
use bevy::{
    color::palettes::tailwind,
    ecs::entity::EntityHashMap,
    math::bounding::{Aabb3d, RayCast3d},
    pbr::NotShadowCaster,
    platform::collections::HashSet,
    prelude::*,
    tasks::{AsyncComputeTaskPool, futures_lite::future},
};
use std::sync::Arc;

pub mod prelude {
    #[cfg(feature = "debug_draw")]
    pub use crate::debug_draw::*;
    pub use crate::{
        NavPlugin, agent::*, character::*, collider::*, nav::*, nav_ray_cast::*, tile::*, utils::*,
    };
}

pub struct NavPlugin;

impl Plugin for NavPlugin {
    fn build(&self, app: &mut App) {
        if !app.is_plugin_added::<BvhPlugin>() {
            app.add_plugins(BvhPlugin);
        }

        app.add_systems(
            Update,
            (
                waymap_changed,
                handle_removed_affectors, //.in_set(OxidizedNavigation::Main),
            )
                .chain(),
        )
        .add_systems(
            PostUpdate,
            (
                update_navmesh_affectors,
                (add_agents_to_waymap, add_characters_to_waymap),
                start_tile_build_tasks,
                poll_tile_build_tasks,
                update_navigation, //navigation::update_navigation.run_if(input_pressed(KeyCode::Space))
            )
                .chain()
                .after(TransformSystem::TransformPropagate)
                .after(BvhSystems::Update),
        )
        .register_type::<Agent>()
        .register_type::<AgentSettings>()
        .register_type::<AgentWaymap>()
        .register_type::<Character>()
        .register_type::<CharacterSettings>()
        .register_type::<CharacterWaymap>()
        .register_type::<TileAffectors>()
        .register_type::<Tile>()
        .register_type::<TileNavMesh>()
        .register_type::<Nav>()
        .register_type::<TileLookup>()
        .register_type::<WaymapAgents>()
        .register_type::<WaymapCharacters>();
    }
}

pub fn update_navigation(
    mut arch_query: Query<(Entity, &WaymapAgents, &Tlas), With<Nav>>,
    agent_query: Query<(&GlobalTransform, &AgentState), With<Agent>>,
    // mut nav_mesh_cast: NavRayCast,
    tlas_cast: TlasCast,
    #[cfg(feature = "debug_draw")] mut gizmos: Gizmos<RavenGizmos>,
) {
    for (nav_entity, waymap_agents, tlas) in arch_query.iter_mut() {
        for agent_entity in waymap_agents.iter() {
            if let Ok((agent_transform, _agent_state)) = agent_query.get(agent_entity) {
                let point = agent_transform.translation();
                let ray = RayCast3d::new(point, Dir3::NEG_Y, 2.);

                #[cfg(feature = "debug_draw")]
                gizmos.line(point, ray.get_point(ray.max).into(), tailwind::YELLOW_400);
                if let Some((_entity, hit)) = tlas_cast.intersect_tlas(&ray, nav_entity) {
                    #[cfg(feature = "debug_draw")]
                    gizmos.line(
                        point,
                        ray.get_point(hit.distance).into(),
                        tailwind::FUCHSIA_500,
                    );
                }
            }
        }
    }
}

// TODO: use arch bounding to find first available waymap
/// Add waymap refs to agents if not present
/// Only works when one waymap exists, if not you need to set CharacterWaymap yourself when spawning the character.
#[allow(dead_code)]
fn add_agents_to_waymap(
    mut commands: Commands,
    mut query: Query<Entity, (Without<CharacterWaymap>, Added<Agent>)>,
    waymap: Single<Entity, With<Nav>>,
) {
    for e in query.iter_mut() {
        commands.entity(e).insert(AgentWaymap(*waymap));
    }
}

// TODO: use arch bounding to find first available waymap
/// Add waymap refs to characters if not present
/// Only works when one waymap exists, if not you need to set CharacterWaymap yourself when spawning the character.
#[allow(dead_code)]
fn add_characters_to_waymap(
    mut commands: Commands,
    mut query: Query<Entity, (Without<CharacterWaymap>, Added<Character>)>,
    waymap: Single<Entity, With<Nav>>,
) {
    for e in query.iter_mut() {
        commands.entity(e).insert(CharacterWaymap(*waymap));
    }
}

#[expect(clippy::type_complexity)]
fn update_navmesh_affectors(
    mut commands: Commands,
    mut waymap_query: Query<(&Nav, &GlobalTransform, &TileLookup, &mut DirtyTiles), Without<Tile>>,
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
    for (waymap, arch_transform, lookup, mut dirty_tiles) in waymap_query.iter_mut() {
        // Expand by 2 * walkable_radius to match with erode_walkable_area.
        let border_expansion = f32::from(waymap.walkable_radius * 2) * waymap.cell_width;

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
            let min_tile = waymap.get_tile_containing_position(min_vec, arch_transform);

            let max_vec = Vec2::new(
                aabb.maxs.x + border_expansion,
                aabb.maxs.z + border_expansion,
            );
            let max_tile = waymap.get_tile_containing_position(max_vec, arch_transform);

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

fn waymap_changed(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &Nav,
            &WaymapTiles,
            &mut WaymapGenerationTasks,
            &mut TileLookup,
            &mut DirtyTiles,
        ),
        (Or<(Changed<Transform>, Changed<Nav>)>,),
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
        commands.entity(e).insert(WaymapAabb(Aabb3d::new(
            Vec3A::ZERO,
            arch.world_half_extents,
        )));

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
                        TileAabb(Aabb3d::new(Vec3::ZERO, tile_half_extends)),
                        Transform::from_translation(translation),
                        TileWaymap(e),
                        ChildOf(e), // duplicated with TileWaymap, but may have more children in the future
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

/// Start the tile build tasks for the tiles that need to be generated.
#[allow(clippy::too_many_arguments)]
#[expect(clippy::type_complexity)]
fn start_tile_build_tasks(
    mut commands: Commands,
    mut tiles_to_generate: Local<Vec<UVec2>>,
    mut heightfields: Local<EntityHashMap<Arc<HeightField>>>,
    mut waymap_query: Query<(
        &Nav,
        &TileLookup,
        &mut DirtyTiles,
        &mut WaymapGenerationTasks,
    )>,
    mut tile_query: Query<(&TileAffectors, &GlobalTransform), With<Tile>>,
    collider_query: Query<(Entity, &Collider, &GlobalTransform, &NavMeshAffector)>,
) {
    let thread_pool = AsyncComputeTaskPool::get();

    for (waymap, tile_lookup, mut dirty_tiles, mut active_generation_tasks) in
        waymap_query.iter_mut()
    {
        // see if we can start a new task
        let max = waymap.max_tile_generation_tasks.get() as usize;
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
                task: thread_pool.spawn(build_tile(
                    waymap.clone(),
                    geometry_collections,
                    heightfield_collections,
                )),
            });
        }
        heightfields.clear();
    }
}

/// Checks status of tile builds
fn poll_tile_build_tasks(
    mut commands: Commands,
    mut waymap_query: Query<
        (Entity, &mut WaymapGenerationTasks, &mut TlasRebuildStrategy),
        With<Nav>,
    >,
    mut meshes: ResMut<Assets<Mesh>>,
    mut bvhs: ResMut<Assets<Bvh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut nav_edit_links: NavLinks,
    #[cfg(feature = "debug_draw")] store: Res<GizmoConfigStore>,
) {
    #[cfg(feature = "debug_draw")]
    let config = store.config::<RavenGizmos>().1;
    for (e, mut tasks, mut strat) in waymap_query.iter_mut() {
        // check active tasks, canceling if not current
        tasks.0.retain_mut(|job| {
            if let Some(result) = future::block_on(future::poll_once(&mut job.task)) {
                let tile_entity = job.entity;
                match result {
                    //
                    Some((mut tile_nav_mesh, aabb, mesh, bvh)) => {
                        // Update links to neighbours,
                        nav_edit_links.update_tile_links(tile_entity, &mut tile_nav_mesh);

                        // Update the tile nav mesh
                        commands.entity(job.entity).insert((
                            tile_nav_mesh,
                            MeshBvh(bvhs.add(bvh)),
                            TlasTarget(e),
                            TileMeshAabb(aabb),
                        ));

                        // adding view mesh as child so we can use Tranform to offset
                        #[cfg(feature = "debug_draw")]
                        commands.spawn((
                            ChildOf(job.entity),
                            TileViewMesh,
                            Mesh3d(meshes.add(mesh)),
                            MeshMaterial3d(materials.add(StandardMaterial {
                                base_color: config.view_mesh_color.into(),
                                unlit: true,
                                alpha_mode: AlphaMode::Blend,
                                ..default()
                            })),
                            NotShadowCaster,
                            Pickable::IGNORE,
                            Transform::from_translation(config.view_mesh_offset),
                            match config.show_view_mesh {
                                true => Visibility::Visible,
                                false => Visibility::Hidden,
                            },
                        ));
                    }
                    None => {
                        // Remove any links to this tile
                        nav_edit_links.remove_tile(tile_entity);

                        commands
                            .entity(tile_entity)
                            .remove::<TileNavMesh>()
                            .remove::<TileMeshAabb>()
                            .remove::<Children>(); // should delete view mesh
                    }
                }
                // trigger a rebuild of the tlas
                *strat = TlasRebuildStrategy::Mannual(true);

                return false;
            }
            true
        });
    }
}

/// Update the tiles when the NavMeshAffector is removed.
fn handle_removed_affectors(
    mut removed_affectors: RemovedComponents<NavMeshAffector>,
    mut tile_query: Query<(&Tile, &mut TileAffectors, &TileWaymap)>,
    mut waymap_query: Query<&mut DirtyTiles, With<Nav>>,
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
        let mut arch_dirty = waymap_query.get_mut(arch.0).unwrap();
        arch_dirty.0.insert(tile.0);

        // remove from the tile
        for e in intersection.drain(..) {
            tile_affectors.remove(&e);
        }
    }
    removed.clear();
}
