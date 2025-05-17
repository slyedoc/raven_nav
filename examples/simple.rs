mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::{input::common_conditions::input_toggle_active, prelude::*};
use raven::prelude::*;

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
            ExampleCommonPlugin,
            PhysicsPlugins::default(),
            RavenPlugin {
                settings: NavMeshSettings::from_agent_and_bounds(0.5, 1.9, 250.0, -1.0),
            },
            RavenDebugPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            toggle_nav_mesh_debug_draw.run_if(input_toggle_active(true, KeyCode::KeyM)),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn((
        CameraFree,
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

    // ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(10.0, 0.2, 10.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::default(),
        Collider::cuboid(10.0, 0.2, 10.0),
        RigidBody::Static,
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    // cube
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.5, 0.3, 0.3))),
        Transform::from_xyz(0.0, 3.0, 0.0),
        Collider::cuboid(1.0, 1.0, 1.0),
        RigidBody::Dynamic,
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

    commands.spawn((
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

fn toggle_nav_mesh_debug_draw(mut show_navmesh: ResMut<DrawNavMesh>) {
    show_navmesh.0 = !show_navmesh.0;
}
