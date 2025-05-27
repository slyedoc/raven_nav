mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::{prelude::*, window::WindowResolution};
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
        
            // physics
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
        Name::new("Camera"),
        CameraFree,
        Camera3d::default(),
        Camera {
            hdr: true,
            ..default()
        },
        Transform::from_xyz(0.0, 20.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Name::new("Directional Light"),
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    // spawn default archipelago for now
    commands.spawn((
        Name::new("Archipelago"),
        Archipelago::new(0.5, 1.9, Vec3::splat(200.0)),
        ArchipelagoMovement, // helper to move the archipelago around with Arrow Keys to see regeneration
    ));

    // heightfield
    let resolution = 250;
    let oct = 10.0;
    let height_scale = 8.0;
    let scale = Vec3::new(resolution as f32, height_scale, resolution as f32);

    let mut heightfield = vec![vec![0.0; resolution]; resolution];
    for x in 0..resolution {
        for y in 0..resolution {
            heightfield[x][y] = (x as f32 / oct).sin() + (y as f32 / oct).cos();
        }
    }

    commands.spawn((
        Name::new("Heightfield"),
        Transform::from_xyz(0., -15.0, 0.),
        Mesh3d(meshes.add(generate_mesh_from_heightfield(&heightfield, scale, true))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
        Collider::heightfield(heightfield, scale),
        RigidBody::Static,
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    commands.spawn((
        Name::new("Agent 1"),
        agent_spawner.spawn(),
        Agent,
        Transform::from_xyz(0.0, 2.0, 2.0),
    ));

    // commands.spawn((
    //     Name::new("Cube"),
    //     Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
    //     MeshMaterial3d(materials.add(Color::srgb(0.5, 0.3, 0.3))),
    //     Transform::from_xyz(0.0, 10.0, 0.0),
    //     Collider::cuboid(1.0, 1.0, 1.0),
    //     RigidBody::Dynamic,
    //     NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    // ));

    commands.spawn((
        Name::new("Text"),
        Text::new("M: Toggle NavMesh Debug Draw"),
        TextFont {
            font_size: 10.0,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(20.0),
            right: Val::Px(5.0),
            ..default()
        },
    ));
}
