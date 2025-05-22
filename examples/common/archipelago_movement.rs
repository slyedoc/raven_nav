use bevy::prelude::*;
use bevy_enhanced_input::prelude::*;

pub(super) const DEFAULT_SPEED: f32 = 0.1;

// Helper plugin to move an archipelago around for testing purposes.
pub struct ArchipelagoMovementPlugin;

impl Plugin for ArchipelagoMovementPlugin {
    fn build(&self, app: &mut App) {
        app.add_input_context::<ArchFreeInputContext>()
            .add_observer(default_binding)
            .add_observer(apply_movement);
    }
}

#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Default, Reflect)]
#[require(
    //Name = Name::new("CameraFree"),    
    Actions::<ArchFreeInputContext>,
)]
pub struct ArchipelagoMovement;

#[derive(Debug, InputContext, Default)]
pub struct ArchFreeInputContext;

fn default_binding(
    trigger: Trigger<Binding<ArchFreeInputContext>>,
    mut archs: Query<&mut Actions<ArchFreeInputContext>>,
) {
    let mut actions = archs.get_mut(trigger.target()).unwrap();

    // Movement
    actions.bind::<EnableSprint>().to(KeyCode::ShiftLeft);

    actions
        .bind::<Move>()
        .to((
            Cardinal::arrow_keys().with_conditions_each(BlockBy::<EnableSprint>::default()),
            Cardinal::arrow_keys()
                .with_conditions_each(Chord::<EnableSprint>::default())
                .with_modifiers_each(Scale::splat(10.0)),
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
    mut query: Query<&mut Transform, With<ArchipelagoMovement>>,
) {
    let mut trans = query.get_mut(trigger.target()).unwrap();
    let change = vec3(trigger.value.x, 0.0, -trigger.value.y);
    let rot = trans.rotation;
    trans.translation += rot * change;
}

#[derive(Debug, InputAction)]
#[input_action(output = Vec2)]
struct Move;

#[derive(Debug, InputAction)]
#[input_action(output = bool)]
struct EnableSprint;
