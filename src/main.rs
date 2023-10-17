use bevy::{prelude::{App, Commands, ResMut, Assets, StandardMaterial, Mesh, PbrBundle, Color, shape::Cube, Transform, Camera3dBundle, Vec3, PointLight, PointLightBundle, Startup, Projection, PerspectiveProjection}, DefaultPlugins};
use bevy_flycam::{FlyCam, NoCameraPlayerPlugin};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            NoCameraPlayerPlugin
        ))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(PbrBundle {
        material: standard_materials.add(Color::PURPLE.into()),
        mesh: meshes.add(Mesh::from(Cube::new(1.))),
        transform: Transform::from_xyz(0., 0., 0.),
        ..Default::default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(1., 1., 1.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam
    ));

}