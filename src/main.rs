use bevy::{
    prelude::{*}, 
    DefaultPlugins, 
    render::{
        RenderPlugin, 
        settings::{WgpuSettings, PowerPreference, Backends}
    }, 
    pbr::wireframe::WireframePlugin
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam, MovementSettings};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::{prelude::*, render::RapierDebugRenderPlugin};
use stickman::{StickmanArm, SegmentInfo, ArmMotorParams};
use systems::{stickman_body_setup, test_update, update_joint_handles};


mod utils;
mod stickman;
mod systems;

fn main() {
    let mut app = App::new();

    let default_plugins = 
        DefaultPlugins.set(
            RenderPlugin {
                wgpu_settings: WgpuSettings {
                    backends: Some(Backends::DX12),
                    power_preference: PowerPreference::HighPerformance,
                    ..Default::default()
                },
            }
        );
    

    app.add_plugins((
            default_plugins,
            WireframePlugin::default(),
            WorldInspectorPlugin::default(),
            // LogFramesPlugin::default(),
            NoCameraPlayerPlugin,

            RapierPhysicsPlugin::<()>::default(),
        ))
        .add_systems(Startup, ( 
            stickman_body_setup,
            scene_setup
        ))
        .add_systems(Update, (
            test_update,
            test_system,
        ))
        .add_systems(PostUpdate, update_joint_handles.after(PhysicsSet::Writeback))
        .register_type::<TestComponent>();
    
    #[cfg(debug_assertions)]
    app.add_plugins(RapierDebugRenderPlugin {
        ..Default::default()
    });


    app.run();

}

pub fn test_system(
    mut commands: Commands,
    camera: Query<&Transform, With<FlyCam>>,
    keys: Res<Input<KeyCode>>,
) {
    let cam_transform = camera.get_single().unwrap();
    if keys.just_pressed(KeyCode::J) {
        commands.spawn((
            RigidBody::Dynamic,
            Collider::cuboid(0.1, 0.1, 0.1),
            SpatialBundle {
                transform: Transform::from_xyz(cam_transform.translation.x, cam_transform.translation.y, cam_transform.translation.z),
                ..Default::default()
            }
        ));
    }
}


pub fn scene_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
    mut flycam_settings: ResMut<MovementSettings>
) {
    //flycam
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 0., 2.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam,
        Collider::cuboid(0.1, 0.1, 0.1)
    ));

    flycam_settings.speed = 2.;

    //ground
    commands.spawn((
        PbrBundle {
            material: standard_materials.add(Color::WHITE.into()),
            mesh: meshes.add(shape::Box::new(100., 1., 100.).into()),
            transform: Transform::from_xyz(0., -5., 0.),
            ..Default::default()
        },
        Collider::cuboid(50., 0.5, 50.)
    ));
}


#[derive(Component, Reflect)]
pub struct TestComponent {
    
}
