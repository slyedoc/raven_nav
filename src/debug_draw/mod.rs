use bevy::{
    color::{
        self,
        palettes::{css, tailwind},
    },
    gizmos::{AppGizmoBuilder, config::GizmoConfigGroup},
    math::bounding::BoundingVolume,
    prelude::*,
    reflect::Reflect,
    render::{primitives::Aabb, view::RenderLayers},
    time::{Time, Timer},
};

use crate::{
    Bounding,
    archipelago::{self, *},
    nav_mesh::{self, *},
    tile::*,
};

#[derive(Default)]
pub struct RavenDebugPlugin {
    pub render_layer: RenderLayers,
}

impl Plugin for RavenDebugPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            RavenGizmos::default(),
            GizmoConfig {
                //depth_bias: -1.0,
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
                //.after(PhysicsSet::StepSimulation)
                .run_if(|store: Res<GizmoConfigStore>| store.config::<RavenGizmos>().0.enabled),
        );
    }
}

#[derive(Reflect, GizmoConfigGroup)]
pub struct RavenGizmos {
    pub archipelago_bounds: Option<Color>,
    pub tile_bounds: Option<Color>,
    pub tile_polygons: Option<Color>,
    pub tile_edges: Option<Color>,
}

impl Default for RavenGizmos {
    fn default() -> Self {
        Self {
            archipelago_bounds: Some(tailwind::RED_100.into()),
            tile_bounds: Some(css::RED.into()),
            tile_polygons: Some(css::GREEN.into()),
            tile_edges: Some(css::BLUE.into()),
        }
    }
}

// Helper function to draw a path for the timer's duration.
fn draw_path(
    mut commands: Commands,
    mut path_query: Query<(Entity, &mut DrawPath)>,
    time: Res<Time>,
    mut gizmos: Gizmos<RavenGizmos>,
) {
    path_query.iter_mut().for_each(|(entity, mut draw_path)| {
        if draw_path
            .timer
            .as_mut()
            .is_some_and(|timer| timer.tick(time.delta()).just_finished())
        {
            commands.entity(entity).despawn();
        } else {
            gizmos.linestrip(draw_path.pulled_path.clone(), draw_path.color);
        }
    });
}

fn draw_arhipelago_bounds(
    archipelago_query: Query<(&GlobalTransform, &Archipelago, &Bounding)>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;
    if let Some(color) = config.archipelago_bounds {
        for (&trans, arch, bounding) in archipelago_query.iter() {
            gizmos.cuboid(bounding_transform(&bounding, &trans), color);
        }
    }
}

fn draw_tile_bounds(
    island_query: Query<(&GlobalTransform, &Bounding), With<Tile>>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;
    if let Some(color) = config.tile_bounds {
        for (trans, bounding) in island_query.iter() {
            gizmos.cuboid(bounding_transform(&bounding, trans), color);
        }
    }
}

fn draw_tiles(
    tile_query: Query<(&TileNavMesh, &GlobalTransform)>,
    nav_meshs: Res<Assets<NavigationMesh>>,
    mut gizmos: Gizmos<RavenGizmos>,
    store: Res<GizmoConfigStore>,
) {
    let config = store.config::<RavenGizmos>().1;

    for (navmesh_handle, trans) in tile_query.iter() {
        let navmesh = nav_meshs.get(&navmesh_handle.0).unwrap();        
        for (polygon_index, polygon) in navmesh.polygons.iter().enumerate() {
            // draw polygon triangles
            if let Some(color) = config.tile_polygons {
                let center_point = trans.transform_point(polygon.center);
                for i in 0..polygon.vertices.len() {
                    let j = (i + 1) % polygon.vertices.len();

                    let i = polygon.vertices[i];
                    let j = polygon.vertices[j];

                    gizmos.linestrip(
                        [
                            trans.transform_point(navmesh.vertices[i]),
                            trans.transform_point(navmesh.vertices[j]),
                            center_point,
                        ],
                        color,
                    );
                }
            }

            // draw island edges
            if let Some(color) = config.tile_edges {
                for (edge_index, connection) in polygon.connectivity.iter().enumerate() {
                    let line_type = match connection.as_ref() {
                        None => LineType::BoundaryEdge,
                        Some(connection) => {
                            // Ignore connections where the connected polygon has a greater
                            // index. This prevents drawing the same edge multiple
                            // times by picking one of the edges to draw.
                            if polygon_index > connection.polygon_index {
                                continue;
                            }
                            LineType::ConnectivityEdge
                        }
                    };

                    let i = edge_index;
                    let j = (i + 1) % polygon.vertices.len();

                    let i = polygon.vertices[i];
                    let j = polygon.vertices[j];

                    gizmos.line(
                        trans.transform_point(navmesh.vertices[i]),
                        trans.transform_point(navmesh.vertices[j]),
                        color,
                    );
                }
            }

            // let node_ref = NodeRef { island_id, polygon_index };
            // if let Some(boundary_link_ids) =
            //     archipelago.nav_data.node_to_boundary_link_ids.get(&node_ref)
            // {
            //     for &boundary_link_id in boundary_link_ids.iter() {
            //     let boundary_link = archipelago
            //         .nav_data
            //         .boundary_links
            //         .get(boundary_link_id)
            //         .expect("Boundary links are present.");
            //     // Ignore links where the connected node has a greater node_ref. This
            //     // prevents drawing the same link multiple times by picking one of the
            //     // links to draw.
            //     if node_ref > boundary_link.destination_node {
            //         continue;
            //     }

            //     debug_drawer.add_line(
            //         LineType::BoundaryLink,
            //         [
            //         CS::from_landmass(&boundary_link.portal.0),
            //         CS::from_landmass(&boundary_link.portal.1),
            //         ],
            //     );
            //     }
            // }
        }
    }
}

fn bounding_transform(bounding: &Bounding, transform: &GlobalTransform) -> GlobalTransform {
    *transform
        * GlobalTransform::from(
            Transform::from_translation(bounding.0.center().into())
                .with_scale((bounding.0.max - bounding.0.min).into()),
        )
}

#[derive(Component)]
/// Path drawing helper component. Each instance of this component will draw a path for until ``timer`` passed before being despawned.
pub struct DrawPath {
    /// Timer for how long to display path before it is despawned.
    ///
    /// If ``None`` the DrawPath entity will not be automatically despawned
    pub timer: Option<Timer>,
    /// Path to display.
    pub pulled_path: Vec<Vec3>,
    /// Color to display path as.
    pub color: Color,
}

/// The type of debug points.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum PointType {
    /// The position of an agent.
    AgentPosition(Entity),
    /// The target of an agent.
    TargetPosition(Entity),
    /// The waypoint of an agent.
    Waypoint(Entity),
}

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
