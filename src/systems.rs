use bevy::prelude::*;
use bevy_rapier3d::{prelude::*, rapier::prelude::{JointLimits, MotorModel, MultibodyJointHandle}};
use smallvec::SmallVec;

use crate::{stickman::{StickmanArmSegment, StickmanArm}, TestComponent};

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

    
    let material = standard_materials.add(Color::PURPLE.into());
    let upper_arm_material = standard_materials.add(Color::ORANGE.into());
    
    //body assembly
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

        // let upper_arm = commands.spawn((
        //     PbrBundle {
        //         mesh: meshes.add(arm_segment.into()),
        //         material: upper_arm_material,
        //         ..Default::default()
        //     },
        //     RigidBody::Fixed,
        //     Collider::capsule(
        //         Vec3::Y * (arm_segment_depth/2.), 
        //         Vec3::Y * (-arm_segment.depth/2.),
        //         radius
        //     ),
        // ));

        //older code
        {
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
            //     SpatialBundle::default(),
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
    }

    
    //joints
    {
        let joint_gap_size = 0.;
        let joint_offset = (arm_segment_len+joint_gap_size)/2.;

        //motor params
        let target_pos = f32::to_radians(0.);
        let target_vel = f32::to_radians(30.);
        let stiffness = 1.;
        let damping = 0.03;
        
        let mut joint = 
            // GenericJointBuilder::new(JointAxesMask::LIN_AXES | JointAxesMask::ANG_X)
            // SphericalJointBuilder::new()
            RevoluteJointBuilder::new(Vec3::Z)
                .local_anchor1(Vec3::new(0., -joint_offset, 0.))
                .local_anchor2(Vec3::new(0., joint_offset , -0.))
                .limits([0., 150_f32.to_radians()])
                // .limits(JointAxis::AngX, [0., 0.])
                // .limits(JointAxis::AngY, [0., 270_f32.to_radians()])
                // .limits(JointAxis::AngZ, [f32::to_radians(-90.), f32::to_radians(60.)])
                // .motor(
                //     JointAxis::AngX,
                //     0.,
                //     target_vel,
                //     stiffness,
                //     damping
                // )
                // .motor(
                //     JointAxis::AngY,
                //     0.,
                //     target_vel,
                //     stiffness,
                //     damping
                // )
                .motor(
                    // JointAxis::AngZ,
                    target_pos,
                    target_vel,
                    stiffness,
                    damping
                )
                .motor_model(MotorModel::ForceBased)
                // .motor_model(JointAxis::AngZ, MotorModel::ForceBased)
                .build()
                ;
        joint.set_contacts_enabled(false);

        let arm_segment_mesh = meshes.add(arm_segment.into());

        let upper_arm = commands.spawn((
            RigidBody::Fixed,
            Collider::capsule(Vec3::Y * (-arm_segment_depth/2.), Vec3::Y * (arm_segment_depth/2.), radius),
            PbrBundle {
                mesh: arm_segment_mesh.clone(),
                material: upper_arm_material,
                transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, 0., 0., 90_f32.to_radians())),
                ..Default::default()
            },
            Sleeping::default(),
            // ActiveHooks::FILTER_CONTACT_PAIRS,
        )).id();
        
        let lower_arm = commands.spawn_empty()
            .set_parent(upper_arm)
            .insert((
                RigidBody::Dynamic,
                MultibodyJoint::new(upper_arm, joint),
                Collider::capsule(Vec3::Y * (-arm_segment_depth/2.), Vec3::Y * (arm_segment_depth/2.), radius),

                PbrBundle {
                    mesh: arm_segment_mesh,
                    material: material,
                    transform: Transform::from_xyz(arm_segment_len+joint_gap_size, 0., 0.),
                    ..Default::default()
                },
                Sleeping::default(),
                StickmanArmSegment::with_upper_arm(upper_arm),
            )).id();
        
        commands.get_entity(upper_arm).unwrap()
            .insert(StickmanArmSegment::with_lower_arm(lower_arm));

        commands.spawn(
            StickmanArm::new(SmallVec::from([upper_arm, lower_arm]))
        );
    }

}

pub fn test_update(
    mut multibody_joints: Query<(&mut MultibodyJoint, &mut Sleeping), With<StickmanArmSegment>>,
    keys: Res<Input<KeyCode>>,
) {
    let dir = 
        if keys.pressed(KeyCode::Up) { f32::to_radians(5.) }
        else if keys.pressed(KeyCode::Down) { f32::to_radians(-5.) }
        else { f32::to_radians(0.) };
        

    let default_limits = JointLimits::default();

    for (mut multibody_joint, mut sleeping) in multibody_joints.iter_mut() {
        let joint = &mut  multibody_joint.data;
        
        //revolute joint
        if joint.as_revolute().is_some() {
            let joint = unsafe { joint.as_revolute_mut().unwrap_unchecked() };
            let motor = joint.motor().unwrap();
            let target_vel = motor.target_vel;
            let current_target_pos = motor.target_pos;
            let stiffness = motor.stiffness;
            let damping = motor.damping;
            let limits = joint.limits().unwrap_or(&default_limits);
            
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
                target_vel,
                stiffness,
                damping
            );
        }

    }

}

pub fn update_joint_handles(
    mut arms: Query<&mut StickmanArm, Changed<StickmanArm>>,
    joint_handles: Query<&RapierMultibodyJointHandle>,
) {
    for mut arm in arms.iter_mut() {
        arm.joints.clear();

        for i in 0..arm.arm_segments.len() {
            let segment = arm.arm_segments[i];
            if let Ok(handle) = joint_handles.get(segment) {
                arm.joints.push(handle.clone());
            }
        }

    }
}
