use bevy::{prelude::*, DefaultPlugins, render::{render_resource::AsBindGroup, RenderPlugin, settings::{WgpuSettings, Backends}}, reflect::{TypeUuid, TypePath}};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam};
use utils::LogFramesPlugin;


mod utils;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(
                RenderPlugin {
                    wgpu_settings: WgpuSettings {
                        power_preference: bevy::render::settings::PowerPreference::LowPower,
                        backends: Some(Backends::DX12),
                        ..Default::default()
                    }
                }
            ),
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
    mut meshes: ResMut<Assets<Mesh>>,
    asset_server: Res<AssetServer>
) {
    //quad with test material
    commands.spawn((
        MaterialMeshBundle {
            material: materials.add(TestMaterial {
                 color: Color::PURPLE,
                 color_texture: Some(asset_server.load("images/icon.png")), 
                 alpha_mode: AlphaMode::Blend
            }),
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
        self.alpha_mode
    }
}

// This is the struct that will be passed to your shader
#[derive(TypePath, AsBindGroup, Debug, Clone, TypeUuid)]
#[uuid = "893a605b-ebb3-4dc1-8eb0-0b788a4bc91d"]
pub struct TestMaterial {
    #[uniform(0)]
    color: Color,
    #[texture(1)]
    #[sampler(2)]
    color_texture: Option<Handle<Image>>,
    alpha_mode: AlphaMode,
}
