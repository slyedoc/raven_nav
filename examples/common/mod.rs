mod camera_free;
use bevy_enhanced_input::EnhancedInputPlugin;
pub use camera_free::*;

use bevy::prelude::*;

/// A plugin that adds common functionality used by examples,
/// such as physics diagnostics UI and the ability to pause and step the simulation.
pub struct ExampleCommonPlugin;

impl Plugin for ExampleCommonPlugin {
    fn build(&self, app: &mut App) {
        // Add diagnostics.
        app.add_plugins((
            sly_editor::SlyEditorPlugin::default(), // custom bevy_egui_inspector and avian editor
            EnhancedInputPlugin,
            CameraFreePlugin,
        ))
        .add_systems(Startup, setup_key_instructions);
    }
}

fn setup_key_instructions(mut commands: Commands) {
    commands.spawn((
        Text::new("U: Avian Diag | P: Pause/Unpause | Enter: Step | F1: Toggle Colliders, Backquote: Egui, Right Mouse: Look Around, WASD: Move, Arrow Keys: Rotate, Q/E: Ascend/Descend"),
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
