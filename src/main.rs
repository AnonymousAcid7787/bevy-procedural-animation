use bevy::{prelude::*, render::render_resource::AsBindGroup, reflect::{TypePath, TypeUuid}, utils::Uuid};
use utils::LogFramesPlugin;

mod utils;

fn main() {
    let mut app = App::new();

    app.add_plugins((
        DefaultPlugins,
        LogFramesPlugin::default(),
        MaterialPlugin::<CustomMaterial>::default()
    ));

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<CustomMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // cube
    commands.spawn(MaterialMeshBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        material: materials.add(CustomMaterial {
            color: Color::BLUE,
            alpha_mode: AlphaMode::Blend,
        }),
        ..default()
    });

    // camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

impl Material for CustomMaterial {
    fn fragment_shader() -> bevy::render::render_resource::ShaderRef {
        "./shaders/custom_material.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        self.alpha_mode
    }
}

#[derive(TypePath, AsBindGroup, Debug, Clone, TypeUuid)]
#[uuid = "1f73101b-0d10-4c34-8aa9-f0e2b3b77ec2"]
pub struct CustomMaterial {
    #[uniform(0)]
    color: Color,
    alpha_mode: AlphaMode
}
