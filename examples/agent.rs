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
        .init_resource::<AgentSpawner>()
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
    let a1 = commands.spawn((
        Name::new("Archipelago"),
        // This creates the archipelago which will generate a nav-mesh
        Archipelago::new(0.5, 1.9, Vec3::splat(50.0)).with_traversible_slope(40.0_f32.to_radians()),
        ArchipelagoMovement, // helper to move the archipelago around with Arrow Keys to see regeneration
    )).id();

    commands.spawn((
        Name::new("Ground"),
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, vec2(10.0, 10.0)))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        Transform::default(),
        ColliderConstructor::TrimeshFromMesh,
        RigidBody::Static,
        NavMeshAffector::default(), // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));

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
        Transform::from_xyz(2.0, 1.9, 0.0),
        Collider::capsule(0.5, 1.9),
        RigidBody::Dynamic,
        Agent,
        AgentArchipelago(a1),
    ));
}


#[derive(Resource)]
struct AgentSpawner {
  mesh: Handle<Mesh>,
  material: Handle<StandardMaterial>,
  //archipelago_entity: Entity,
  // target_entity: Entity,
  //fast_material: Handle<ColorMaterial>,
  //slow_node_type: NodeType,
}

impl AgentSpawner {
    fn spawn(&self) -> impl Bundle {
        (
            Mesh3d(self.mesh.clone()),
            MeshMaterial3d(self.material.clone())
        )
    }
}

impl FromWorld for AgentSpawner {
    fn from_world(world: &mut World) -> Self {
        
        let mut meshes = world.resource_mut::<Assets<Mesh>>();
        let mesh = meshes.add(Capsule3d::new(0.5, 1.9));

        let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
        let material = materials.add(StandardMaterial {
            base_color: tailwind::GREEN_500.into(),
            ..default()
        });  

        Self {
            mesh,
            material,            
        }
    }
}


