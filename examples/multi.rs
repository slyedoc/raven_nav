mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::prelude::*;
use raven::prelude::*;
use sly_editor::IsEditorCamera;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Raven: Simple".to_string(),
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
) {
    commands.spawn((
        Name::new("Camera"),
        IsEditorCamera,
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

    // first archipelago
    generate_area(
        &mut commands,
        &mut meshes,
        &mut materials,
        1,
        Vec3::new(-40.0, 0.0, 0.0),
        50.0,
    );

    // second archipelago
    generate_area(
        &mut commands,
        &mut meshes,
        &mut materials,
        2,
        Vec3::new(40.0, 0.0, 0.0),
        50.0,
    );

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

fn generate_area(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    index: usize,
    translation: Vec3,
    size: f32,
) {
    commands.spawn((
        Name::new(format!("Archipelago {}", index)),
        Archipelago::new(0.5, 1.9, Vec3::splat(size)),
        Transform::from_translation(translation),
    ));

    // heightfield
    let resolution = 30;
    let oct = 10.0;
    let height_scale = 8.0;
    let scale = Vec3::new(size, height_scale, size);

    let mut heightfield = vec![vec![0.0; resolution]; resolution];
    for x in 0..resolution {
        for y in 0..resolution {
            heightfield[x][y] = (x as f32 / oct).sin() + (y as f32 / oct).cos();
        }
    }

    commands.spawn((
        Name::new(format!("Heightfield {}", index)),
        Transform::from_translation(translation),
        Mesh3d(meshes.add(generate_mesh_from_heightfield(&heightfield, scale, true))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
        Collider::heightfield(heightfield, scale),
        RigidBody::Static,
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));
}