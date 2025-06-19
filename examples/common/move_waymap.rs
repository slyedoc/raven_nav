use bevy::prelude::*;
use bevy_enhanced_input::prelude::*;

pub(super) const DEFAULT_SPEED: f32 = 0.3;

/// Use Arrow keys to move waymap around for testing purposes.
pub struct MoveWaymapPlugin;

impl Plugin for MoveWaymapPlugin {
    fn build(&self, app: &mut App) {
        app.add_input_context::<NavInputContext>()
            .add_observer(default_binding)
            .add_observer(apply_movement);
    }
}

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Default, Reflect)]
#[require(    
    Actions::<NavInputContext>,
)]
pub struct NavMovement;

#[derive(Debug, InputContext, Default)]
pub struct NavInputContext;

fn default_binding(
    trigger: Trigger<Binding<NavInputContext>>,
    mut archs: Query<&mut Actions<NavInputContext>>,
) {
    let mut actions = archs.get_mut(trigger.target()).unwrap();

    // // TODO: Shift doesnt work with this, find out why, upgrade to latest bevy_enhanced_input first though?
    // actions.bind::<EnableSprint>().to(KeyCode::ShiftLeft);
    
    // Movement
    actions
        .bind::<Move>()
        .to((
            Cardinal::arrow_keys(),            
        ))
        // Don't trigger the action when the chord is active.
        .with_modifiers((
            //DeadZone::default(), // Apply non-uniform normalization to ensure consistent speed, otherwise diagonal movement will be faster.
            SmoothNudge::default(), // Make movement smooth and independent of the framerate. To only make it framerate-independent, use `DeltaScale`.
            Scale::splat(DEFAULT_SPEED), // Additionally multiply by a constant to achieve the desired speed.
        ));
}

fn apply_movement(
    trigger: Trigger<Fired<Move>>,
    mut query: Query<&mut Transform, With<NavMovement>>,
) {
    let mut trans = query.get_mut(trigger.target()).unwrap();
    let change = vec3(trigger.value.x, 0.0, -trigger.value.y);
    let rot = trans.rotation;
    trans.translation += rot * change;
}

#[derive(Debug, InputAction)]
#[input_action(output = Vec2)]
struct Move;

// #[derive(Debug, InputAction)]
// #[input_action(output = bool)]
// struct EnableSprint;
