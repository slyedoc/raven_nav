#![allow(dead_code)]

mod camera_free;
use bevy_enhanced_input::EnhancedInputPlugin;
pub use camera_free::*;
use raven::prelude::RavenGizmos;
pub use sly_editor::*;

mod archipelago_movement;
pub use archipelago_movement::*;

mod agent;
pub use agent::*;

use bevy::{
    asset::RenderAssetUsages,
    input::common_conditions::input_just_pressed,
    prelude::*,
    render::mesh::{Indices, PrimitiveTopology},
};

/// A plugin that adds common functionality used by examples,
/// such as physics diagnostics UI and the ability to pause and step the simulation.
pub struct ExampleCommonPlugin;

impl Plugin for ExampleCommonPlugin {
    fn build(&self, app: &mut App) {
        // Add diagnostics.
        app.add_plugins((
            SlyEditorPlugin::default(), // custom bevy_egui_inspector
            EnhancedInputPlugin,
            CameraFreePlugin,          // camera movement
            ArchipelagoMovementPlugin, // arch movement, used to test rebuilds
        ))
        .init_resource::<AgentSpawner>()
        .add_systems(
            Update,
            toggle_debug_draw.run_if(input_just_pressed(KeyCode::KeyM)),
        )
        .add_systems(Startup, setup_key_instructions);
    }
}

fn toggle_debug_draw(mut store: ResMut<GizmoConfigStore>) {
    let config = store.config_mut::<RavenGizmos>().0;
    config.enabled = !config.enabled;
}

fn setup_key_instructions(mut commands: Commands) {
    commands.spawn((
        Text::new("Backquote: Egui, U: Avian Diag | P: Pause/Unpause | Enter: Step | F1: Toggle Colliders, Right Mouse: Look Around, WASD: Move"),
        TextFont {
            font_size: 10.0,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        },
    ));
}

/// Generates a mesh from a heightfield
/// Assumes the heightfield is square
#[allow(dead_code)]
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

    let half_size = (size - 1) as f32 / 2.0;
    for y in 0..size {
        for x in 0..size {
            let i = (y * size) + x;
            // find the position of the vertex and center, with height_multiplier
            let pos = [
                (x as f32 - half_size) * scale.x / (size - 1) as f32,
                (heightfield[x][y] * scale.y) as f32,
                (y as f32 - half_size) * scale.z / (size - 1) as f32,
            ];

            positions.push(pos);
            uvs.push([x as f32 / (size - 1) as f32, y as f32 / (size - 1) as f32]);

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
