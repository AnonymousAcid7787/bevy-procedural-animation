

use bevy::{
    prelude::{*}, 
    DefaultPlugins, 
    render::{
        render_resource::{PolygonMode, AsBindGroup}, 
        RenderPlugin, 
        settings::{WgpuSettings, Backends, PowerPreference}
    }, 
    pbr::wireframe::WireframePlugin, reflect::{TypePath, TypeUuid}
};
use bevy_flycam::{NoCameraPlayerPlugin, FlyCam, MovementSettings};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::{prelude::*, render::RapierDebugRenderPlugin};


mod utils;

fn main() {
    let mut app = App::new();

    let default_plugins = 
        DefaultPlugins.set(
            RenderPlugin {
                wgpu_settings: WgpuSettings {
                    // backends: Some(Backends::DX12),
                    power_preference: PowerPreference::HighPerformance,
                    ..Default::default()
                }
            }
        );

    app.add_plugins((
            default_plugins,
            WireframePlugin::default(),
            WorldInspectorPlugin::default(),
            // LogFramesPlugin::default(),
            MaterialPlugin::<TestMaterial>::default(),
            NoCameraPlayerPlugin,

            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .add_systems(Startup, ( 
            stickman_body_setup,
            scene_setup
        ))
        .add_systems(Update, test_update)
        .register_type::<TestComponent>();
    
    #[cfg(debug_assertions)]
    app.add_plugins(RapierDebugRenderPlugin {
        ..Default::default()
    });


    app.run();

}

fn scene_setup(
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
                ),
                StickmanBodyPart::new($radius,$capsule_depth),
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

fn stickman_body_setup(
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
    let arm_segment_depth = arm_depth/2.;
    let arm_segment_len = arm_segment_depth+radius;

    let arm_segment = shape::Capsule {
        depth: arm_segment_depth,
        radius,
        latitudes,
        longitudes,
        ..Default::default()
    };
    
    //commented for now
    {

    // //shapes
    // let torso = shape::Capsule {
    //     radius,
    //     depth: torso_depth,
    //     latitudes,
    //     longitudes,
    //     ..Default::default()
    // };

    // let arm = shape::Capsule {
    //     radius,
    //     depth: arm_depth,
    //     latitudes,
    //     longitudes,
    //     ..Default::default()
    // };

    // let leg = shape::Capsule {
    //     radius,
    //     depth: leg_depth,
    //     latitudes,
    //     longitudes,
    //     ..Default::default()
    // };

    // let arm_segment = shape::Capsule {
    //     depth: arm_segment_depth,
    //     radius,
    //     latitudes,
    //     longitudes,
    //     ..Default::default()
    // };

    // //transforms
    // let mut arm1_transform = Transform::from_xyz(0., 0., 0.);
    //     arm1_transform.rotate_around(
    //         Vec3::new(0., arm_len/2., 0.), 
    //         Quat::from_axis_angle(Vec3::Z, 45_f32.to_radians())
    //     );
    
    // let mut arm2_transform = Transform::from_xyz(0., 0., 0.);
    //     arm2_transform.rotate_around(
    //         Vec3::new(0., arm_len/2., 0.), 
    //         Quat::from_axis_angle(Vec3::Z, -45_f32.to_radians())
    //     );

    // let torso_transform = Transform::from_xyz(0., 0., 0.);

    // let mut leg1_transform = Transform::from_xyz(0., -torso_len - (leg_len - torso_len)/2., 0.);
    //     leg1_transform.rotate_around(
    //         Vec3::new(0., -torso_len/2., 0.), 
    //         Quat::from_axis_angle(Vec3::Z, 30_f32.to_radians())
    //     );
    // let mut leg2_transform = Transform::from_xyz(0., -torso_len - (leg_len - torso_len)/2., 0.);
    //     leg2_transform.rotate_around(
    //         Vec3::new(0., -torso_len/2., 0.), 
    //         Quat::from_axis_angle(Vec3::Z, -30_f32.to_radians())
    //     );
    
    // //spawning entities
    // let body_mesh_entity = commands.spawn((
    //     StickmanMeshParentBundle::default(),
    //     StickmanBody
    // )).id();
    // let arm1_entity = spawn_body_part!(
    //     meshes.add(arm.into()),
    //     commands, 
    //     material.clone(),
    //     arm1_transform,
    //     arm_depth,
    //     radius
    // );

    // let arm2_entity = spawn_body_part!(
    //     meshes.add(arm.into()),
    //     commands, 
    //     material.clone(),
    //     arm2_transform,
    //     arm_depth,
    //     radius
    // );
 
    // let torso_entity = spawn_body_part!(
    //     meshes.add(torso.into()), 
    //     commands, 
    //     material.clone(),
    //     torso_transform,
    //     torso_depth,
    //     radius
    // );
 
    // let leg1_entity = spawn_body_part!(
    //     meshes.add(leg.into()), 
    //     commands, 
    //     material.clone(),
    //     leg1_transform,
    //     leg_depth,
    //     radius
    // );
    // let leg2_entity = spawn_body_part!(
    //     meshes.add(leg.into()), 
    //     commands, 
    //     material.clone(),
    //     leg2_transform,
    //     leg_depth,
    //     radius
    // );

    // //parent heirarchy stuff
    // add_child!(commands, body_mesh_entity, torso_entity);
    // add_child!(commands, body_mesh_entity, arm1_entity);
    // add_child!(commands, body_mesh_entity, arm2_entity);
    // add_child!(commands, body_mesh_entity, leg1_entity);
    // add_child!(commands, body_mesh_entity, leg2_entity);
    }

    //joints
    let joint_gap_size = radius*1.25;
    let joint_offset = (arm_segment_len+joint_gap_size)/2.;
    let joint = RevoluteJointBuilder::new(Vec3::Z)
        .local_anchor1(Vec3::new(joint_offset, 0., 0.))
        .local_anchor2(Vec3::new(0., -joint_offset, 0.))
        .motor(
            0_f32.to_radians(),
            30_f32.to_radians(),
            0.5,
            0.5
        )
        ;

    let par_entity = commands.spawn((
        SpatialBundle::default(),
        RigidBody::Fixed,
        Collider::capsule(Vec3::X * (-arm_segment_depth/2.), Vec3::X * (arm_segment_depth/2.), radius),
    )).id();
    

    commands.spawn_empty()
        .set_parent(par_entity)
        .insert((
            RigidBody::Dynamic,
            MultibodyJoint::new(par_entity, joint),
            Collider::capsule(Vec3::Y * (-arm_segment_depth/2.), Vec3::Y * (arm_segment_depth/2.), radius),

            PbrBundle {
                mesh: meshes.add(arm_segment.into()),
                material: standard_materials.add(Color::BLUE.into()),
                transform: Transform::from_xyz(arm_segment_len+joint_gap_size, 0., 0.),
                    // .with_rotation(Quat::from_euler(EulerRot::XYZ, 0., 0., 90_f32.to_radians())),
                ..Default::default()
            },
            Sleeping::default()
        ));

}

fn test_update(
    mut multibody_joints: Query<(&mut MultibodyJoint, &mut Sleeping)>,
    keys: Res<Input<KeyCode>>,
) {
    let mut dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(1.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-1.) }
        else { f32::to_radians(0.) };
    dir *= 10.;

    for (mut multibody_joint, mut sleeping) in multibody_joints.iter_mut() {
        let joint =  multibody_joint.data.as_revolute_mut().unwrap();
        let current_target_pos = joint.motor().unwrap().target_pos;

        if dir != 0. {
            sleeping.sleeping = false;
        }

        joint.set_motor(
            current_target_pos + dir,
            f32::to_radians(30.),
            0.1,
            0.1
        );
    }

}

#[derive(Component)]
pub struct StickmanBody;

#[derive(Component)]
pub struct StickmanBodyPart {
    /// Radius of capsule
    pub radius: f32,
    /// Depth of capsule
    pub depth: f32,
}

impl StickmanBodyPart {
    #[inline(always)]
    pub fn new(radius: f32, depth: f32) -> Self {
        Self { radius, depth }
    }

    #[inline(always)]
    pub fn length(&self) -> f32 {
        self.depth + self.radius
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
