use std::f32::consts::PI;

use bevy::{
    prelude::{*}, 
    DefaultPlugins, 
    render::{
        RenderPlugin, 
        settings::{WgpuSettings, PowerPreference, RenderCreation}
    }, 
    pbr::wireframe::WireframePlugin
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam, MovementSettings};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::{prelude::*, render::RapierDebugRenderPlugin};
use systems::{control_axes, point_at_camera, spawn_cubes, stickman_setup, UpperArm};

mod utils;
mod stickman;
mod systems;
 
fn main() {
    let mut app = App::new();

    let default_plugins = 
        DefaultPlugins.set(
            RenderPlugin {
                render_creation: RenderCreation::Automatic(
                    WgpuSettings {
                        power_preference: PowerPreference::HighPerformance,
                        ..Default::default()
                    },
                )
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
            // stickman_setup,
            scene_setup
        ))
        .add_systems(Update, (
            // control_axes,
            point_at_camera,
            // spawn_cubes
        ))
        .register_type::<TestComponent>();
    
    #[cfg(debug_assertions)]
    app.add_plugins(RapierDebugRenderPlugin {
        ..Default::default()
    });


    app.run();

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
        // Collider::cuboid(0.1, 0.1, 0.1)
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


    let seg1 = commands.spawn(
        RigidBody::Fixed,
    ).id();

    let mut seg2 = commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule_y(0.4, 0.1),
        UpperArm,
        Sleeping::default()
    ));

    let mut joint = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(1., 0., 0.))
        .local_anchor2(Vec3::new(0., -1., 0.))
        .motor(JointAxis::AngX, 0., 30_f32.to_radians(), 1., 0.03)
        .motor(JointAxis::AngY, 0., 30_f32.to_radians(), 1., 0.03)
        .motor(JointAxis::AngZ, 0., 30_f32.to_radians(), 1., 0.03)
        .build();
        joint.set_contacts_enabled(false);
    
    seg2
        .set_parent(seg1)
        .insert(MultibodyJoint::new(seg1, joint));

}


#[derive(Component, Reflect)]
pub struct TestComponent;
