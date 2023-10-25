use bevy::{prelude::*, utils::Uuid};
use bevy_rapier3d::{prelude::*, rapier::prelude::MotorModel};

use crate::stickman::StickmanArmSegment;

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

pub fn stickman_body_setup(
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
        .limits([f32::to_radians(-90.), f32::to_radians(0.)])
        .motor(
            -90_f32.to_radians(),
            30_f32.to_radians(),
            0.5,
            0.
        )
        .motor_model(MotorModel::ForceBased)
        ;

    let arm_cmp = StickmanArmSegment::new(Uuid::new_v4());

    let par_entity = commands.spawn((
        SpatialBundle::default(),
        RigidBody::Fixed,
        Collider::capsule(Vec3::X * (-arm_segment_depth/2.), Vec3::X * (arm_segment_depth/2.), radius),
        ActiveHooks::FILTER_INTERSECTION_PAIR,

        arm_cmp,
    )).id();

    commands.spawn_empty()
        .set_parent(par_entity)
        .insert((
            RigidBody::Dynamic,
            MultibodyJoint::new(par_entity, joint),
            Collider::capsule(Vec3::Y * (-arm_segment_depth/2.), Vec3::Y * (arm_segment_depth/2.), radius),
            ActiveHooks::FILTER_CONTACT_PAIRS,

            PbrBundle {
                mesh: meshes.add(arm_segment.into()),
                material: standard_materials.add(Color::BLUE.into()),
                transform: Transform::from_xyz(arm_segment_len+joint_gap_size, 0., 0.),
                ..Default::default()
            },
            Sleeping::default(),

            arm_cmp,
        ));

}

pub fn set_arm_uuids(
    mut rapier_context: ResMut<RapierContext>,
    stickman_arm_segments: Query<(&StickmanArmSegment, &RapierRigidBodyHandle)>
) {
    for (arm_segment, rigid_body_handle) in stickman_arm_segments.iter() {
        let body = rapier_context.bodies.get_mut(rigid_body_handle.0).unwrap();
        body.user_data = unsafe {
            std::mem::transmute::<Uuid, u128>(arm_segment.get_arm_uuid())
        }
    }
}

pub fn test_update(
    mut multibody_joints: Query<(&mut MultibodyJoint, &mut Sleeping)>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };
    
    let damping = 0.03;
    let stiffness = 1.;

    for (mut multibody_joint, mut sleeping) in multibody_joints.iter_mut() {
        let joint =  multibody_joint.data.as_revolute_mut().unwrap();
        let current_target_pos = joint.motor().unwrap().target_pos;
        let limits = joint.limits().unwrap();

        let new_target_pos = f32::clamp(
            current_target_pos + dir,
            limits.min * (1. + damping),
            limits.max
        );

        if dir != 0. {
            sleeping.sleeping = false;
        }

        joint.set_motor(
            new_target_pos,
            f32::to_radians(60.),
            stiffness,
            damping
        );
    }

}