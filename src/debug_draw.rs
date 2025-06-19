use bevy::{
    color::palettes::tailwind,
    gizmos::{AppGizmoBuilder, config::GizmoConfigGroup},
    math::bounding::{Aabb3d, BoundingVolume},
    prelude::*,
    reflect::Reflect,
    render::view::RenderLayers,
};

use crate::{
    nav::*,
    tile::{nav_mesh::*, *},
};

#[derive(Default)]
pub struct RavenDebugPlugin {
    //pub depth_bias: f32,
    pub render_layer: RenderLayers,
}

impl Plugin for RavenDebugPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            RavenGizmos::default(),
            GizmoConfig {
                depth_bias: -0.01,
                render_layers: self.render_layer.clone(),
                ..Default::default()
            },
        );

        app.add_systems(
            Update,
            (
                draw_arhipelago_bounds,
                draw_tile_bounds,
                draw_tiles,
                //draw_path.run_if(any_with_component::<DrawPath>),
            )
                .run_if(|store: Res<GizmoConfigStore>| store.config::<RavenGizmos>().0.enabled),
        );
    }
}

#[derive(Reflect, GizmoConfigGroup)]
pub struct RavenGizmos {
    pub waymap_bounds: Option<Color>,
    pub tile_bounds: Option<Color>,
    pub tile_polygons: Option<Color>,
    pub tile_internal_links: Option<Color>,
    pub tile_external_links: Option<Color>,

    pub show_view_mesh: bool,
    pub view_mesh_color: Color,
    pub view_mesh_offset: Vec3,
}

impl Default for RavenGizmos {
    fn default() -> Self {
        Self {
            waymap_bounds: Some(tailwind::GRAY_300.with_alpha(0.5).into()),
            tile_bounds: Some(tailwind::RED_500.with_alpha(0.5).into()),
            tile_polygons: Some(tailwind::BLUE_500.into()),
            tile_internal_links: None, // Some(tailwind::YELLOW_500.into()),
            tile_external_links: Some(tailwind::YELLOW_500.into()),

            show_view_mesh: true,
            view_mesh_color: tailwind::BLUE_300.with_alpha(0.5).into(),
            view_mesh_offset: Vec3::new(0.0, 0.1, 0.0),
        }
    }
}

// Helper function to draw a path for the timer's duration.
// fn draw_path(
//     mut commands: Commands,
//     mut path_query: Query<(Entity, &mut DrawPath)>,
//     time: Res<Time>,
//     mut gizmos: Gizmos<RavenGizmos>,
// ) {
//     path_query.iter_mut().for_each(|(entity, mut draw_path)| {
//         if draw_path
//             .timer
//             .as_mut()
//             .is_some_and(|timer| timer.tick(time.delta()).just_finished())
//         {
//             commands.entity(entity).despawn();
//         } else {
//             gizmos.linestrip(draw_path.pulled_path.clone(), draw_path.color);
//         }
//     });
// }

fn draw_arhipelago_bounds(
    waymap_query: Query<(&GlobalTransform, &WaymapAabb), With<Nav>>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;
    if let Some(color) = config.waymap_bounds {
        for (&trans, bounding) in waymap_query.iter() {
            gizmos.cuboid(aabb3d_transform(&bounding.0, &trans), color);
        }
    }
}

fn draw_tile_bounds(
    island_query: Query<(&GlobalTransform, &TileMeshAabb), With<Tile>>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;
    if let Some(color) = config.tile_bounds {
        for (trans, bounding) in island_query.iter() {
            gizmos.cuboid(aabb3d_transform(&bounding.0, trans), color);
        }
    }
}

fn draw_tiles(
    tile_query: Query<(&TileNavMesh, &GlobalTransform)>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;
    for (tile, trans) in tile_query.iter() {
        for poly in tile.polygons.iter() {
            if let Some(color) = config.tile_polygons {
                let indices = &poly.indices;
                for i in 0..indices.len() {
                    let a = tile.vertices[indices[i] as usize];
                    let b = tile.vertices[indices[(i + 1) % indices.len()] as usize];
                    gizmos.line(trans.transform_point(a), trans.transform_point(b), color);
                }
            }

            for link in &poly.links {
                // Only draw links that connect to a polygon with a greater index
                // to avoid drawing the same link multiple times.
                match link {
                    Link::Internal {
                        edge,
                        neighbour_polygon: _,
                    } => {
                        if let Some(color) = config.tile_internal_links {
                            let a = tile.vertices[poly.indices[*edge as usize] as usize];
                            gizmos.line(
                                trans.transform_point(a),
                                trans.transform_point(a + Vec3::Y * 0.2),
                                color,
                            );
                        }
                    }
                    Link::External {
                        edge,
                        neighbour_polygon: _,
                        direction: _,
                        bound_min: _,
                        bound_max: _,
                    } => {
                        if let Some(color) = config.tile_external_links {
                            let a = tile.vertices[poly.indices[*edge as usize] as usize];
                            gizmos.line(
                                trans.transform_point(a),
                                trans.transform_point(a + Vec3::Y * 0.2),
                                color,
                            );
                        }
                    }
                }
            }
        }
    }
}

pub(crate) fn aabb3d_transform(bounding: &Aabb3d, transform: &GlobalTransform) -> GlobalTransform {
    *transform
        * GlobalTransform::from(
            Transform::from_translation(bounding.center().into())
                .with_scale((bounding.max - bounding.min).into()),
        )
}

pub(crate) fn aabb3d_global(bounding: &Aabb3d) -> GlobalTransform {
    GlobalTransform::from(
        Transform::from_translation(bounding.center().into())
            .with_scale((bounding.max - bounding.min).into()),
    )
}

// #[derive(Component)]
// /// Path drawing helper component. Each instance of this component will draw a path for until ``timer`` passed before being despawned.
// pub struct DrawPath {
//     /// Timer for how long to display path before it is despawned.
//     ///
//     /// If ``None`` the DrawPath entity will not be automatically despawned
//     pub timer: Option<Timer>,
//     /// Path to display.
//     pub pulled_path: Vec<Vec3>,
//     /// Color to display path as.
//     pub color: Color,
// }

// /// The type of debug points.
// #[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
// pub enum PointType {
//     /// The position of an agent.
//     AgentPosition(Entity),
//     /// The target of an agent.
//     TargetPosition(Entity),
//     /// The waypoint of an agent.
//     Waypoint(Entity),
// }

/// The type of debug lines.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum LineType {
    /// An edge of a node that is the boundary of a nav mesh.
    BoundaryEdge,
    /// An edge of a node that is connected to another node.
    ConnectivityEdge,
    /// A link between two islands along their boundary edge.
    BoundaryLink,
    /// Part of an agent's current path. The corridor follows the path along
    /// nodes, not the actual path the agent will travel.
    AgentCorridor(Entity),
    /// Line from an agent to its target.
    Target(Entity),
    /// Line to the waypoint of an agent.
    Waypoint(Entity),
}

/// The type of debug triangles.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum TriangleType {
    /// Part of a node/polygon in a nav mesh.
    Node,
}
