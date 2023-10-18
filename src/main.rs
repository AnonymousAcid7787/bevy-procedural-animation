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
        .add_systems(Update, billboarding)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<TestMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>
) {
    //quad with test material
    commands.spawn((
        MaterialMeshBundle {
            material: materials.add(TestMaterial { color: Color::PURPLE }),
            mesh: meshes.add(
                // shape::Quad::new(Vec2::new(1., 1.)).into()
                shape::Cube::new(1.).into()
            ),
            transform: Transform::from_xyz(0., 0., 0.),
            ..Default::default()
        },
        Billboard
    ));

    //flycam
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(1., 1., 1.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam
    ));
}

fn billboarding(
    mut billboard_entities: Query<&mut Transform, With<Billboard>>,
    camera_query: Query<&Transform, (With<Camera>, Without<Billboard>)>
) {
    let cam_pos = camera_query.get_single().unwrap().translation;
    for mut billboard_transform in billboard_entities.iter_mut() {
        billboard_transform.look_at(cam_pos, Vec3::Y);
    }
}

#[derive(Component)]
pub struct Billboard;

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
