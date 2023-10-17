use bevy::{
    prelude::{*}, 
    render::{
        render_resource::{AsBindGroup, RenderPipelineDescriptor, SpecializedMeshPipelineError}, 
    mesh::MeshVertexBufferLayout
    }, 
    reflect::{TypePath, TypeUuid}, 
    pbr::{MaterialPipeline, MaterialPipelineKey}, math::{vec2, vec4}
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
            NoCameraPlayerPlugin,
        ))
        .add_systems(Startup, setup);

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<CustomMaterial>>,
    mut color_materials: ResMut<Assets<StandardMaterial>>,
) {
    // cube
    commands.spawn(MaterialMeshBundle {
        mesh: meshes.add(shape::Cube::new(1.).into()).into(),
        material: materials.add(CustomMaterial {
            color: vec4(1., 0., 0., 1.),
        }),
        transform: Transform::from_xyz(0.0, 2., 0.0),
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Cube::new(1.).into()).into(),
        material: color_materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        transform: Transform::from_xyz(0., 0., 0.),
        ..default()
    });

    // camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 0., -2.).looking_at(Vec3::ZERO, Vec3::Y),
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
    color: Vec4,
}
