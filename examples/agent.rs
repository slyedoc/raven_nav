mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::{
    color::palettes::tailwind, math::bounding::RayCast3d, prelude::*, window::WindowResolution,
};
use raven_bvh::prelude::*;
use raven_nav::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Raven: Simple".to_string(),
                    resolution: WindowResolution::new(1920.0, 1080.0),
                    ..default()
                }),
                ..default()
            }),
            PhysicsPlugins::default(),
            NavPlugin,
            RavenDebugPlugin::default(),
            ExampleCommonPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, ray_cast)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    agent_spawner: Res<AgentSpawner>,
) {
    // spawn default waymap for now
    let nav = commands
        .spawn((
            Name::new("Nav"),
            // This creates the waymap which will generate a nav-mesh
            Nav::new(0.5, 1.9, Vec3::splat(60.0)).with_traversible_slope(40.0_f32),
            NavMovement, // helper to move the waymap around with Arrow Keys to see regeneration
        ))
        .id();

    commands.spawn((
        CameraFree, // Helper to move the camera around with WASD and mouse look with right mouse button
        BvhCamera::new(512, 512, nav), // debug camera for bvh
        Camera3d::default(),
        Camera {
            hdr: true,
            ..default()
        },
        Transform::from_xyz(0.0, 2.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    commands.spawn((
        Name::new("Ground"),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, vec2(30.0, 30.0)))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::default(),
        ColliderConstructor::TrimeshFromMesh,
        RigidBody::Static,
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    let gray = materials.add(StandardMaterial {
        base_color: tailwind::GRAY_300.into(),
        metallic: 0.0,
        perceptual_roughness: 0.5,
        ..default()
    });

    {
        // TODO: delete this nonsense
        // to construct a walkway with ramps
        // anything more than this and going to blender
        let walkway_width = 25.0f32;
        let walkway_depth = 8.0f32;
        let walkway_height = 5.0f32;
        let ramp_length = 10.0f32;
        let ramp_height = walkway_height;
        let ramp_width = 5.0f32;
        let ramp_thickness = 0.2f32;
        let ramp_angle = (ramp_height / ramp_length).atan();

        let ramp_center_y =
            walkway_height - ramp_thickness / 2.0 - (ramp_length / 2.0) * ramp_angle.sin();
        commands
            .spawn((
                Name::new("Walkway"),
                Mesh3d(meshes.add(Plane3d::new(Vec3::Y, vec2(walkway_width, walkway_depth)))),
                MeshMaterial3d(gray.clone()),
                Transform::from_xyz(0.0, walkway_height - 0.2, 0.0),
                ColliderConstructor::TrimeshFromMesh,
                RigidBody::Static,
                NavMeshAffector::default(),
            ))
            .with_children(|parent| {
                parent.spawn((
                    Name::new("Ramp Right"),
                    Mesh3d(meshes.add(Cuboid::new(ramp_width, ramp_thickness, ramp_length))),
                    MeshMaterial3d(gray.clone()),
                    Transform {
                        translation: Vec3::new(
                            10.0,
                            -ramp_center_y + 0.3,
                            3.3 + walkway_depth / 2. + ramp_length / 2.0,
                        ),
                        rotation: Quat::from_rotation_x(ramp_angle),
                        ..Default::default()
                    },
                    ColliderConstructor::TrimeshFromMesh,
                    RigidBody::Static,
                    NavMeshAffector::default(),
                ));

                parent.spawn((
                    Name::new("Ramp Left"),
                    Mesh3d(meshes.add(Cuboid::new(ramp_width, ramp_thickness, ramp_length))),
                    MeshMaterial3d(gray.clone()),
                    Transform {
                        translation: Vec3::new(
                            -10.0,
                            -ramp_center_y + 0.3,
                            -3.3 - walkway_depth / 2. - ramp_length / 2.0,
                        ),
                        rotation: Quat::from_rotation_x(-ramp_angle),
                        ..Default::default()
                    },
                    ColliderConstructor::TrimeshFromMesh,
                    RigidBody::Static,
                    NavMeshAffector::default(),
                ));
            });
    }

    commands.spawn((
        Name::new("Cube"),
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.5, 0.3, 0.3))),
        Transform::from_xyz(0.0, 3.0, 0.0),
        Collider::cuboid(1.0, 1.0, 1.0),
        RigidBody::Dynamic,
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    commands.spawn((
        Name::new("Agent 1"),
        agent_spawner.spawn(),
        Agent,
        Transform::from_xyz(0.0, 2.0, 2.0),
    ));
}

fn ray_cast(
    agent: Single<&GlobalTransform, With<Agent>>,
    camera_query: Single<(&Camera, &GlobalTransform)>,
    window: Single<&Window>,
    tlas_query: Single<Entity, (With<Nav>, With<Tlas>)>,
    tlas: TlasCast,
    mut gizmos: Gizmos,
    mut nav_path: NavPath,
    input: Res<ButtonInput<MouseButton>>,
    mut start_pos: Local<Vec3>,
    mut end_pos: Local<Vec3>,
) {
    let (camera, camera_transform) = *camera_query;
    let tlas_entity = *tlas_query;
    
    // Use Right mouse buttons to set start 
    if input.pressed(MouseButton::Right) {        
        let Some(cursor_position) = window.cursor_position() else {
            return;
        };
        let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
            return;
        };

        let ray_cast = RayCast3d::from_ray(ray, 100.0);
        if let Some((_e, hit)) = tlas.intersect_tlas(&ray_cast, tlas_entity) {        
            *start_pos = ray.get_point(hit.distance);
        }
    }

    // Use Left mouse button to set end
    if input.pressed(MouseButton::Left) {        
        let Some(cursor_position) = window.cursor_position() else {
            return;
        };
        let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
            return;
        };
        let ray_cast = RayCast3d::from_ray(ray, 100.0);
        if let Some((_e, hit)) = tlas.intersect_tlas(&ray_cast, tlas_entity) {        
            *end_pos = ray.get_point(hit.distance);
        }
    }
    
    gizmos.sphere(*start_pos, 0.1, tailwind::GREEN_400);      
    gizmos.sphere(*end_pos, 0.1, tailwind::RED_400);                
    gizmos.line(*start_pos, *end_pos, tailwind::YELLOW_400);    

    // Run pathfinding to get a polygon path.
    match nav_path.find_path(tlas_entity, *start_pos, *end_pos, None, Some(&[1.0, 0.5])) {
        Ok(path) => gizmos.linestrip(path, tailwind::BLUE_300),
        Err(error) => error!("Error with pathfinding: {:?}", error),
    }    
}