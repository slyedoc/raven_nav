mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::{color::palettes::tailwind, prelude::*, window::WindowResolution};
use raven::prelude::*;

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
            RavenPlugin,
            RavenDebugPlugin::default(),
            ExampleCommonPlugin,
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    agent_spawner: Res<AgentSpawner>,
) {
    commands.spawn((
        CameraFree, // Helper to move the camera around with WASD and mouse look with right mouse button
        Camera3d::default(),
        Camera {
            hdr: true,
            ..default()
        },
        Transform::from_xyz(0.0, 2.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    // spawn default archipelago for now
    let a1 = commands
        .spawn((
            Name::new("Archipelago"),
            // This creates the archipelago which will generate a nav-mesh
            Archipelago::new(0.5, 1.9, Vec3::splat(60.0))
                .with_traversible_slope(40.0_f32),
            ArchipelagoMovement, // helper to move the archipelago around with Arrow Keys to see regeneration
        ))
        .id();

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
        // construct a walkway with ramps
        // anything more than this and going to blender
        let walkway_width = 25.0f32;
        let walkway_depth = 8.0f32;
        let walkway_height = 5.0f32;
        let ramp_length = 10.0f32;
        let ramp_height = walkway_height;
        let ramp_width = 5.0f32;
        let ramp_thickness = 0.2f32;
        let ramp_angle = (ramp_height / ramp_length).atan();

        let ramp_center_y = walkway_height - ramp_thickness / 2.0 - (ramp_length / 2.0) * ramp_angle.sin();
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
                            3.3 +  walkway_depth / 2.  + ramp_length / 2.0,
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
