mod common; // helper functions
use common::*;

use avian3d::prelude::*;
use bevy::{
    asset::RenderAssetUsages,
    input::common_conditions::input_toggle_active,
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};
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
            // physics
            PhysicsPlugins::default(),
            RavenPlugin {
                settings: NavMeshSettings::from_agent_and_bounds(0.5, 1.9, 250.0, -30.0)
                    .with_traversible_slope(30.0_f32.to_radians()),
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
        Archipelago::from_agent_radius(0.5),
    ));

    // heightfield
    let resolution = 150;
    let oct = 10.0;
    let height_scale = 5.0;
    let scale = Vec3::new(resolution as f32, height_scale, resolution as f32);

    let mut heightfield = vec![vec![0.0; resolution]; resolution];
    for x in 0..resolution {
        for y in 0..resolution {
            heightfield[x][y] = (x as f32 / oct).sin() + (y as f32 / oct).cos();
        }
    }

    commands.spawn((
        Name::new("Heightfield"),
        Transform::IDENTITY,
        Mesh3d(meshes.add(generate_mesh_from_heightfield(&heightfield, scale, true))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
        Collider::heightfield(heightfield, scale),
        RigidBody::Static,
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
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

/// Generates a mesh from a heightfield
/// Assumes the heightfield is square
pub fn generate_mesh_from_heightfield(
    heightfield: &Vec<Vec<f32>>,
    scale: Vec3,
    smooth: bool,
) -> Mesh {
    let size = heightfield.len();

    let num_vertices = size * size;
    let num_indices = (size - 1) * (size - 1) * 6;

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(num_vertices);

    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(num_vertices);
    let mut indices: Vec<u32> = Vec::with_capacity(num_indices);

    let half_size = (size as f32 - 1.0) / 2.0;
    for y in 0..size {
        for x in 0..size {
            let i = (y * size) + x;
            // find the position of the vertex and center, with height_multiplier
            let pos = [
                (x as f32 - half_size) * scale.x / (size as f32 - 1.0),
                (heightfield[x][y] * scale.y) as f32,
                (y as f32 - half_size) * scale.z / (size as f32 - 1.0),
            ];

            positions.push(pos);
            uvs.push([x as f32 / (size as f32 - 1.0), y as f32 / (size as f32 - 1.0)]);

            if x < size - 1 && y < size - 1 {
                let a = i;
                let b = i + size;
                let c = i + size + 1;
                let d = i + 1;

                indices.push(a as u32);
                indices.push(b as u32);
                indices.push(c as u32);

                indices.push(c as u32);
                indices.push(d as u32);
                indices.push(a as u32);
            }
        }
    }

    // build our mesh
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_indices(Indices::U32(indices));
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    // compute normals and add positions
    match smooth {
        true => {
            let mut normals: Vec<[f32; 3]> = Vec::with_capacity(num_vertices);
            for y in 0..size {
                for x in 0..size {
                    let pos: Vec3 = positions[(y * size + x) as usize].into();
                    if x < size - 1 && y < size - 1 {
                        let pos_right: Vec3 = positions[(y * size + x + 1) as usize].into();
                        let pos_up: Vec3 = positions[((y + 1) * size + x) as usize].into();
                        let tangent1 = pos_right - pos;
                        let tangent2 = pos_up - pos;
                        let normal = tangent2.cross(tangent1);
                        normals.push(normal.normalize().into());
                    } else {
                        normals.push(Vec3::Y.into());
                    }
                }
            }
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        }
        false => {
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
            mesh.duplicate_vertices();
            mesh.compute_flat_normals();
        }
    }

    mesh
}

fn toggle_nav_mesh_debug_draw(mut show_navmesh: ResMut<DrawNavMesh>) {
    show_navmesh.0 = !show_navmesh.0;
}
