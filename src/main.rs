use bevy::{
    prelude::{*, shape::Cube}, 
    render::{
        render_resource::{AsBindGroup, RenderPipelineDescriptor, SpecializedMeshPipelineError}, 
    mesh::MeshVertexBufferLayout
    }, 
    reflect::{TypePath, TypeUuid}, 
    pbr::{MaterialPipeline, MaterialPipelineKey}, math::vec2
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam};
use utils::LogFramesPlugin;

mod utils;

fn main() {
    let mut app = App::new();

    app
        .add_plugins((
            DefaultPlugins,
            LogFramesPlugin::default(),
            MaterialPlugin::<CustomMaterial>::default(),
        ))
        .add_systems(Startup, setup);

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<CustomMaterial>>,
) {
    // cube
    commands.spawn(MaterialMeshBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        material: materials.add(CustomMaterial {
            resolution: vec2(500., 500.),
            thickness: 0.3,
            alpha_mode: AlphaMode::Blend,
        }),
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: Mesh::from(Cube::new(1)),
        material: 
    });

    // camera
    commands.spawn((
        Camera3dBundle {
            // transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            transform: Transform::from_xyz(0., 0., -1.).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        FlyCam
        )
    );
}

impl Material for CustomMaterial {
    fn fragment_shader() -> bevy::render::render_resource::ShaderRef {
        "shaders/custom_material.frag".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        self.alpha_mode
    }

    // Bevy assumes by default that vertex shaders use the "vertex" entry point
    // and fragment shaders use the "fragment" entry point (for WGSL shaders).
    // GLSL uses "main" as the entry point, so we must override the defaults here
    fn specialize(
        _pipeline: &MaterialPipeline<Self>,
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayout,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        descriptor.vertex.entry_point = "main".into();
        descriptor.fragment.as_mut().unwrap().entry_point = "main".into();
        Ok(())
    }
}

#[derive(TypePath, AsBindGroup, Debug, Clone, TypeUuid)]
#[uuid = "1f73101b-0d10-4c34-8aa9-f0e2b3b77ec2"]
pub struct CustomMaterial {
    #[uniform(0)]
    resolution: Vec2,
    #[uniform(1)]
    thickness: f32,
    alpha_mode: AlphaMode
}
