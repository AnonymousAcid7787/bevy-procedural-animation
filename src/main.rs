

use bevy::{
    prelude::{*}, 
    DefaultPlugins, 
    render::{
        render_resource::{PolygonMode, AsBindGroup}, 
        RenderPlugin, 
        settings::WgpuSettings
    }, 
    pbr::wireframe::WireframePlugin, reflect::{TypePath, TypeUuid}
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::{prelude::{RapierPhysicsPlugin, NoUserData, Collider, RigidBody}, render::RapierDebugRenderPlugin};


mod utils;

fn main() {
    let mut app = App::new();
    app.add_plugins((
            DefaultPlugins.set(
                RenderPlugin {
                    wgpu_settings: WgpuSettings {
                        // backends: Some(Backends::DX12),
                        ..Default::default()
                    }
                }
            ),
            WireframePlugin::default(),
            WorldInspectorPlugin::default(),
            // LogFramesPlugin::default(),
            MaterialPlugin::<TestMaterial>::default(),
            NoCameraPlayerPlugin,

            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, ( 
            stickman_mesh_setup,
            scene_setup
        ))
        .add_systems(Update, test_update)
        .register_type::<TestComponent>();


    app.run();

}

fn scene_setup(
    mut commands: Commands
) {
    //flycam
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 0., 2.).looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        },
        FlyCam
    ));
}


macro_rules! spawn_body_part {
    ($mesh:expr, $commands:expr, $material:expr, $transform:expr, $capsule_depth:expr, $radius:expr) => {
        {
            $commands.spawn((
                PbrBundle {
                    mesh: $mesh, 
                    material: $material,
                    transform: $transform,
                    ..Default::default()
                },
                RigidBody::Fixed,
                Collider::capsule(
                    Vec3::Y * $capsule_depth/2.,
                    Vec3::Y * -$capsule_depth/2.,
                    $radius
                )
            )).id()
        }
    };
}

macro_rules! add_child {
    ($commands:expr, $parent:expr, $child:expr) => {
        $commands.add(AddChild {
            parent: $parent,
            child: $child,
        });
    };
}

fn stickman_mesh_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
) {
    let material = standard_materials.add(Color::PURPLE.into());

    let scale = 1.;

    let radius = 0.03_f32 * scale;
    let torso_depth = 0.6_f32 * scale;
    let torso_len = torso_depth + radius*2.;
    let arm_depth = torso_depth;
    let arm_len = arm_depth + radius*2.;
    let leg_depth = 0.75 * scale;
    let leg_len = leg_depth + radius*2.;
    let latitudes = 8;
    let longitudes = 16;

    //shapes
    let torso = shape::Capsule {
        radius,
        depth: torso_depth,
        latitudes,
        longitudes,
        ..Default::default()
    };

    let arm = shape::Capsule {
        radius,
        depth: arm_depth,
        latitudes,
        longitudes,
        ..Default::default()
    };

    let leg = shape::Capsule {
        radius,
        depth: leg_depth,
        latitudes,
        longitudes,
        ..Default::default()
    };

    //transforms
    let mut arm1_transform = Transform::from_xyz(0., 0., 0.);
        arm1_transform.rotate_around(
            Vec3::new(0., arm_depth/2., 0.), 
            Quat::from_axis_angle(Vec3::Z, 45_f32.to_radians())
        );
    
    let mut arm2_transform = Transform::from_xyz(0., 0., 0.);
        arm2_transform.rotate_around(
            Vec3::new(0., arm_depth/2., 0.), 
            Quat::from_axis_angle(Vec3::Z, -45_f32.to_radians())
        );

    let torso_transform = Transform::from_xyz(0., 0., 0.);

    let mut leg1_transform = Transform::from_xyz(0., radius - torso_len - (leg_len - torso_len)/2., 0.);
        leg1_transform.rotate_around(
            Vec3::new(0., -torso_len/2., 0.), 
            Quat::from_axis_angle(Vec3::Z, 30_f32.to_radians())
        );
    let mut leg2_transform = Transform::from_xyz(0., radius - torso_len - (leg_len - torso_len)/2., 0.);
        leg2_transform.rotate_around(
            Vec3::new(0., -torso_len/2., 0.), 
            Quat::from_axis_angle(Vec3::Z, -30_f32.to_radians())
        );
    
    //spawning entities
    let body_mesh_entity = commands.spawn(StickmanMeshParentBundle::default()).id();
    let arm1_entity = spawn_body_part!(
        meshes.add(arm.into()),
        commands, 
        material.clone(),
        arm1_transform,
        arm_depth,
        radius
    );
    let arm2_entity = spawn_body_part!(
        meshes.add(arm.into()),
        commands, 
        material.clone(),
        arm2_transform,
        arm_depth,
        radius
    );
 
    let torso_entity = spawn_body_part!(
        meshes.add(torso.into()), 
        commands, 
        material.clone(),
        torso_transform,
        torso_depth,
        radius
    );
 
    let leg1_entity = spawn_body_part!(
        meshes.add(leg.into()), 
        commands, 
        material.clone(),
        leg1_transform,
        leg_depth,
        radius
    );
    let leg2_entity = spawn_body_part!(
        meshes.add(leg.into()), 
        commands, 
        material.clone(),
        leg2_transform,
        leg_depth,
        radius
    );

    //parent heirarchy stuff
    add_child!(commands, body_mesh_entity, torso_entity);
    add_child!(commands, body_mesh_entity, arm1_entity);
    add_child!(commands, body_mesh_entity, arm2_entity);
    add_child!(commands, body_mesh_entity, leg1_entity);
    add_child!(commands, body_mesh_entity, leg2_entity);

    //colliders
    // let _ = commands.spawn((//remove the "let _ ="
    //     PbrBundle {
    //         mesh: meshes.add(arm.into()), 
    //         material: material.clone(),
    //         transform: arm1_transform.clone(),//remove this .clone() after done
    //         ..Default::default()
    //     },
    //     RigidBody::Dynamic,
    //     Collider::capsule(arm1_transform.local_y()+arm_len/2., arm1_transform.local_y()-arm_len/2., radius)
    // )).id();
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

#[derive(Default, Bundle)]
pub struct StickmanMeshParentBundle {
    pub visiblity: Visibility,
    pub computed_visibility: ComputedVisibility,
    pub transform_bundle: TransformBundle,
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
