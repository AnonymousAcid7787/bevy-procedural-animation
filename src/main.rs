use bevy::{prelude::*, DefaultPlugins, render::render_resource::AsBindGroup, reflect::{TypeUuid, TypePath}};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam};
use utils::LogFramesPlugin;


mod utils;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LogFramesPlugin::default(),
            MaterialPlugin::<TestMaterial>::default(),
            NoCameraPlayerPlugin
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<TestMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    //cube with test material
    commands.spawn(MaterialMeshBundle {
        material: materials.add(TestMaterial { color: Color::PURPLE }),
        mesh: meshes.add(Mesh::from(shape::Cube::new(1.))),
        transform: Transform::from_xyz(0., 0., 0.),
        ..Default::default()
    });

    //flycam
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(1., 1., 1.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam
    ));
}

impl Material for TestMaterial {
    fn fragment_shader() -> bevy::render::render_resource::ShaderRef {
        "shaders/test_material.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Blend
    }
}

#[derive(AsBindGroup, TypeUuid, TypePath, Debug, Clone)]
#[uuid = "e87f94fd-bffe-4e57-9501-262295cf7bdf"]
pub struct TestMaterial {
    #[uniform(0)]
    color: Color
}
