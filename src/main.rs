

use bevy::{
    prelude::{*}, 
    DefaultPlugins, 
    render::{
        render_resource::{PolygonMode, AsBindGroup}, 
        RenderPlugin, 
        settings::{WgpuSettings, Backends}
    }, 
    pbr::wireframe::WireframePlugin, reflect::{TypePath, TypeUuid}
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use utils::LogFramesPlugin;


mod utils;

fn main() {
    let mut app = App::new();
    app.add_plugins((
            DefaultPlugins.set(
                RenderPlugin {
                    wgpu_settings: WgpuSettings {
                        backends: Some(Backends::DX12),
                        ..Default::default()
                    }
                }
            ),
            WireframePlugin::default(),
            WorldInspectorPlugin::default(),
            LogFramesPlugin::default(),
            MaterialPlugin::<TestMaterial>::default(),
            NoCameraPlayerPlugin
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, test_update)
        .register_type::<TestComponent>();


    app.run();

}

macro_rules! spawn_mesh {
    ($mesh:expr, $x:expr, $y:expr, $z:expr, $commands:expr, $mats:expr, $meshes:expr) => {
        unsafe {
            $commands.spawn((
                PbrBundle {
                    material: $mats.add(Color::PURPLE.into()),
                    mesh: $meshes.add($mesh.clone().into()),
                    transform: Transform::from_xyz($x, $y, $z),
                    ..Default::default()
                },
                std::mem::transmute::<shape::Capsule, TestComponent>($mesh)
            ));
        }
    };
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<TestMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>
) {
    let material = materials.add(TestMaterial {
        color: Color::PURPLE,
        color_texture: Some(asset_server.load("images/icon.png")), 
        alpha_mode: AlphaMode::Blend
    });

    let spine = shape::Capsule {
        radius: 0.03,
        depth: 0.54,
        ..Default::default()
    };

    let arm = shape::Capsule {
        radius: 0.03,
        depth: 0.48,
        ..Default::default()
    };

    let leg = shape::Capsule {
        radius: 0.03,
        depth: 0.6,
        ..Default::default()
    };

    spawn_mesh!(arm, 0., 0., 0., commands, standard_materials, meshes);

    unsafe {
        commands.spawn((
            PbrBundle {
                material: standard_materials.add(Color::PURPLE.into()),
                mesh: meshes.add(spine.clone().into()),
                transform: Transform::from_xyz(0., 0., 0.),
                ..Default::default()
            },
            std::mem::transmute::<shape::Capsule, TestComponent>(spine)
        ));
    }

    //flycam
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 0., 2.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam
    ));
}

fn test_update(
    mut meshes: ResMut<Assets<Mesh>>,
    mut capsules: Query<(&TestComponent, &mut Handle<Mesh>)>
) {
    for (capsule, mut handle) in capsules.iter_mut() {
        meshes.remove(handle.clone());
        unsafe {
            let new_handle = meshes.add(std::mem::transmute_copy::<TestComponent, shape::Capsule>(capsule).into());
            let _ = std::mem::replace(handle.as_mut(), new_handle);
        }
    }
}

#[derive(Component, Reflect)]
pub struct TestComponent {
    /// Radius on the `XZ` plane.
    pub radius: f32,
    /// Number of sections in cylinder between hemispheres.
    pub rings: usize,
    /// Height of the middle cylinder on the `Y` axis, excluding the hemispheres.
    pub depth: f32,
    /// Number of latitudes, distributed by inclination. Must be even.
    pub latitudes: usize,
    /// Number of longitudes, or meridians, distributed by azimuth.
    pub longitudes: usize,
    
    uv_profile: i8,
}

impl Material for TestMaterial {
    fn fragment_shader() -> bevy::render::render_resource::ShaderRef {
        "shaders/test_material.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        self.alpha_mode
    }

    fn specialize(
            _: &bevy::pbr::MaterialPipeline<Self>,
            descriptor: &mut bevy::render::render_resource::RenderPipelineDescriptor,
            _: &bevy::render::mesh::MeshVertexBufferLayout,
            _: bevy::pbr::MaterialPipelineKey<Self>,
        ) -> Result<(), bevy::render::render_resource::SpecializedMeshPipelineError> {
        descriptor.primitive.polygon_mode = PolygonMode::Line;
        return Ok(());
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
