use avian3d::prelude::*;
use bevy::{color::palettes::tailwind, prelude::*};


#[derive(Resource)]
pub struct AgentSpawner {
  pub mesh: Handle<Mesh>,
  pub material: Handle<StandardMaterial>,
  //archipelago_entity: Entity,
  // target_entity: Entity,
  //fast_material: Handle<ColorMaterial>,
  //slow_node_type: NodeType,
}

impl AgentSpawner {
    pub fn spawn(&self) -> impl Bundle {
        (
            Mesh3d(self.mesh.clone()),
            MeshMaterial3d(self.material.clone()),
            Collider::capsule(0.5, 1.9),
            RigidBody::Dynamic,           
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
